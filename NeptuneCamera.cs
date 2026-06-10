using System.Runtime.InteropServices;
using OpenCvSharp;

namespace BoltPixelDetectorApp;

public sealed class NeptuneCamera : IDisposable
{
    private const int NEPTUNE_ERR_SUCCESS = 0;
    private const int NEPTUNE_DEV_ACCESS_EXCLUSIVE = 0;
    private const int NEPTUNE_BOOL_TRUE = 1;
    private const int NEPTUNE_BOOL_FALSE = 0;
    private const int NEPTUNE_ACQ_CONTINUOUS = 0;
    private const int NEPTUNE_GRAB_RGB = 1;
    private const int NEPTUNE_GRAB_RGB32 = 2;
    private const int NEPTUNE_CAMERALISTOPT_ALL = 1;

    private IntPtr _handle = IntPtr.Zero;
    private bool _initialized;
    private bool _disposed;

    public bool IsOpened => _handle != IntPtr.Zero;

    public bool Open()
    {
        if (IsOpened) return true;
        if (!TryInitSdk()) return false;

        _ = ntcSetCameraListOpt(NEPTUNE_CAMERALISTOPT_ALL);

        var camId = GetFirstCameraId();
        if (string.IsNullOrWhiteSpace(camId)) return false;

        if (ntcOpen(camId, out _handle, NEPTUNE_DEV_ACCESS_EXCLUSIVE) != NEPTUNE_ERR_SUCCESS)
        {
            _handle = IntPtr.Zero;
            return false;
        }

        ntcSetAcquisitionMode(_handle, NEPTUNE_ACQ_CONTINUOUS, 1);
        ntcSetAcquisition(_handle, NEPTUNE_BOOL_TRUE);
        return true;
    }

    public void Close()
    {
        if (_handle == IntPtr.Zero) return;

        ntcSetAcquisition(_handle, NEPTUNE_BOOL_FALSE);
        ntcClose(_handle);
        _handle = IntPtr.Zero;
    }

    public void SetExposure(int microseconds)
    {
        if (_handle == IntPtr.Zero) return;
        ntcSetExposureTime(_handle, (uint)microseconds);
    }

    public Mat? GrabFrame(int timeoutMs = 1000)
    {
        if (_handle == IntPtr.Zero) return null;

        var image = new NEPTUNE_IMAGE();
        if (ntcGrab(_handle, ref image, NEPTUNE_GRAB_RGB, (uint)timeoutMs) == NEPTUNE_ERR_SUCCESS && image.pData != IntPtr.Zero)
        {
            var mat = BuildMatFromGrab(image, isRgb32: false);
            if (mat != null) return mat;
        }

        image = new NEPTUNE_IMAGE();
        if (ntcGrab(_handle, ref image, NEPTUNE_GRAB_RGB32, (uint)timeoutMs) == NEPTUNE_ERR_SUCCESS && image.pData != IntPtr.Zero)
        {
            var mat = BuildMatFromGrab(image, isRgb32: true);
            if (mat != null) return mat;
        }

        var size = new NEPTUNE_IMAGE_SIZE();
        if (ntcGetImageSize(_handle, ref size) != NEPTUNE_ERR_SUCCESS || size.nSizeX <= 0 || size.nSizeY <= 0)
            return null;

        int width = size.nSizeX;
        int height = size.nSizeY;

        var rgb = new byte[width * height * 3];
        if (ntcGetRGBData(_handle, rgb, (uint)rgb.Length) == NEPTUNE_ERR_SUCCESS)
        {
            var mat = BuildMatFromBuffer(rgb, width, height, isRgb32: false);
            if (mat != null) return mat;
        }

        var rgb32 = new byte[width * height * 4];
        if (ntcGetRGB32Data(_handle, rgb32, (uint)rgb32.Length) == NEPTUNE_ERR_SUCCESS)
        {
            var mat = BuildMatFromBuffer(rgb32, width, height, isRgb32: true);
            if (mat != null) return mat;
        }

        return null;
    }

    private Mat? BuildMatFromGrab(NEPTUNE_IMAGE image, bool isRgb32)
    {
        int width = (int)image.uiWidth;
        int height = (int)image.uiHeight;
        int size = (int)image.uiSize;
        if (width <= 0 || height <= 0 || size <= 0) return null;

        int expected = isRgb32 ? width * height * 4 : width * height * 3;
        int copySize = Math.Min(expected, size);
        var buffer = new byte[copySize];
        Marshal.Copy(image.pData, buffer, 0, copySize);
        return BuildMatFromBuffer(buffer, width, height, isRgb32);
    }

    private Mat BuildMatFromBuffer(byte[] buffer, int width, int height, bool isRgb32)
    {
        if (!isRgb32)
        {
            var mat = new Mat(height, width, MatType.CV_8UC3);
            Marshal.Copy(buffer, 0, mat.Data, Math.Min(buffer.Length, width * height * 3));

            if (SystemConfig.SOFTWARE_GAIN > 1.0)
                mat.ConvertTo(mat, -1, SystemConfig.SOFTWARE_GAIN, 0);

            if (!SystemConfig.NEPTUNE_DATA_IS_BGR)
                Cv2.CvtColor(mat, mat, ColorConversionCodes.RGB2BGR);

            return ApplyBayerIfNeeded(mat);
        }

        var mat32 = new Mat(height, width, MatType.CV_8UC4);
        Marshal.Copy(buffer, 0, mat32.Data, Math.Min(buffer.Length, width * height * 4));

        if (SystemConfig.SOFTWARE_GAIN > 1.0)
            mat32.ConvertTo(mat32, -1, SystemConfig.SOFTWARE_GAIN, 0);

        var matBgr = new Mat();
        Cv2.CvtColor(mat32, matBgr, SystemConfig.NEPTUNE_DATA_IS_BGR ? ColorConversionCodes.BGRA2BGR : ColorConversionCodes.RGBA2BGR);
        mat32.Dispose();
        return ApplyBayerIfNeeded(matBgr);
    }

    private Mat ApplyBayerIfNeeded(Mat mat)
    {
        if (!SystemConfig.NEPTUNE_FORCE_BAYER) return mat;
        if (mat.Channels() == 1) return ConvertBayer(mat);
        return mat;
    }

    private Mat ConvertBayer(Mat mat)
    {
        ColorConversionCodes code = SystemConfig.NEPTUNE_BAYER_PATTERN.ToUpperInvariant() switch
        {
            "BG" => ColorConversionCodes.BayerBG2BGR,
            "GB" => ColorConversionCodes.BayerGB2BGR,
            "RG" => ColorConversionCodes.BayerRG2BGR,
            _ => ColorConversionCodes.BayerGR2BGR
        };
        var bgr = new Mat();
        Cv2.CvtColor(mat, bgr, code);
        mat.Dispose();
        return bgr;
    }

    private bool TryInitSdk()
    {
        if (_initialized) return true;
        if (!TrySetupDllPath()) return false;

        int err = ntcInit();
        if (err != NEPTUNE_ERR_SUCCESS) return false;

        _initialized = true;
        return true;
    }

    private static bool TrySetupDllPath()
    {
        string root = SystemConfig.NEPTUNE_SDK_ROOT;
        if (string.IsNullOrWhiteSpace(root) || !Directory.Exists(root)) return false;

        string runtimeDir = Path.Combine(root, "Runtime", Environment.Is64BitProcess ? "x64" : "Win32");
        if (!Directory.Exists(runtimeDir)) return false;
        SetDllDirectory(runtimeDir);

        string genicamDir = Path.Combine(root, "Runtime", "GenICam", "Bin", Environment.Is64BitProcess ? "Win64_x64" : "win32_i86", "genapi", "generic");
        if (Directory.Exists(genicamDir)) SetDllDirectory(genicamDir);

        string path = Environment.GetEnvironmentVariable("PATH") ?? "";
        Environment.SetEnvironmentVariable("GENICAM_GENTL64_PATH", runtimeDir);
        Environment.SetEnvironmentVariable("PATH", runtimeDir + Path.PathSeparator + genicamDir + Path.PathSeparator + path);
        return true;
    }

    private string? GetFirstCameraId()
    {
        uint count = 0;
        if (ntcGetCameraCount(ref count) != NEPTUNE_ERR_SUCCESS || count == 0) return null;

        var infos = new NEPTUNE_CAM_INFO[count];
        if (ntcGetCameraInfo(infos, count) != NEPTUNE_ERR_SUCCESS) return null;
        return ExtractString(infos[0].strCamID);
    }

    private static string ExtractString(byte[] bytes)
    {
        int len = Array.IndexOf(bytes, (byte)0);
        if (len < 0) len = bytes.Length;
        return System.Text.Encoding.ASCII.GetString(bytes, 0, len);
    }

    public void Dispose()
    {
        if (_disposed) return;

        Close();
        if (_initialized)
        {
            ntcUninit();
            _initialized = false;
        }

        _disposed = true;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct NEPTUNE_CAM_INFO
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)] public byte[] strVendor;
        public int emDevType;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 380)] public byte[] szReserved;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strModel;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strSerial;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strUserID;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strIP;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)] public byte[] strMAC;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strSubnet;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strGateway;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 512)] public byte[] strCamID;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct NEPTUNE_IMAGE
    {
        public uint uiWidth;
        public uint uiHeight;
        public uint uiBitDepth;
        public IntPtr pData;
        public uint uiSize;
        public uint uiIndex;
        public ulong uiTimestamp;
        public byte bFrameValid;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct NEPTUNE_IMAGE_SIZE
    {
        public int nStartX;
        public int nStartY;
        public int nSizeX;
        public int nSizeY;
    }

    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcInit();
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcUninit();
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcGetCameraCount(ref uint count);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcGetCameraInfo([Out] NEPTUNE_CAM_INFO[] infos, uint count);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcSetCameraListOpt(int opt);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)] private static extern int ntcOpen(string camId, out IntPtr handle, int access);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcClose(IntPtr handle);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcSetAcquisition(IntPtr handle, int enabled);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcSetAcquisitionMode(IntPtr handle, int mode, uint frames);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcGrab(IntPtr handle, ref NEPTUNE_IMAGE image, int format, uint timeoutMs);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcGetImageSize(IntPtr handle, ref NEPTUNE_IMAGE_SIZE size);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcGetRGBData(IntPtr handle, [Out] byte[] buffer, uint bufferSize);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcGetRGB32Data(IntPtr handle, [Out] byte[] buffer, uint bufferSize);
    [DllImport("NeptuneC_MD_VC141.dll", CallingConvention = CallingConvention.Cdecl)] private static extern int ntcSetExposureTime(IntPtr handle, uint microseconds);
    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)] private static extern bool SetDllDirectory(string? lpPathName);
}
