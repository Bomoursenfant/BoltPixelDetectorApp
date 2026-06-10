using System.Globalization;
using System.Linq;
using System.Text;
using OpenCvSharp;

namespace BoltPixelDetectorApp;

public static class CoordinateMapper
{
    private static readonly object HomographyLock = new();
    private static string _homographyKey = "";
    private static Mat? _homography;

    public static VisionResult ToVisionResult(DetectionResult detection, VisionSettings settings)
    {
        // PixelX/Y use the working coordinate system (origin bottom-left, +X up, +Y right).
        double robotX = detection.PixelX;
        double robotY = detection.PixelY;

        if (TryGetHomography(settings, out Mat? homography) &&
            homography is not null &&
            TryMapPixelToRobot(detection.PixelX, detection.PixelY, homography, out double mappedX, out double mappedY))
        {
            robotX = mappedX;
            robotY = mappedY;
        }

        return new VisionResult
        {
            Name = $"Object {detection.Id}",
            PixelX = detection.PixelX,
            PixelY = detection.PixelY,
            X = robotX,
            Y = robotY,
            Angle = detection.Angle,
            Score = detection.Confidence,
            Source = "BoltPixelDetectorApp Pixel"
        };
    }

    public static bool TryMapWorkPixelToRobot(double pixelX, double pixelY, VisionSettings settings, out double robotX, out double robotY)
    {
        robotX = pixelX;
        robotY = pixelY;
        if (!TryGetHomography(settings, out Mat? homography) || homography is null)
            return false;

        return TryMapPixelToRobot(pixelX, pixelY, homography, out robotX, out robotY);
    }

    /// <summary>Euclidean distance in robot plane (mm) between two working-frame pixel points.</summary>
    public static bool TryDistanceMm(
        double pixelX1, double pixelY1,
        double pixelX2, double pixelY2,
        VisionSettings settings,
        out double distanceMm)
    {
        distanceMm = 0;
        if (!TryMapWorkPixelToRobot(pixelX1, pixelY1, settings, out double r1x, out double r1y))
            return false;
        if (!TryMapWorkPixelToRobot(pixelX2, pixelY2, settings, out double r2x, out double r2y))
            return false;

        double dx = r2x - r1x;
        double dy = r2y - r1y;
        distanceMm = Math.Sqrt(dx * dx + dy * dy);
        return true;
    }

    private static bool TryGetHomography(VisionSettings settings, out Mat? homography)
    {
        homography = null;
        var pixelPoints = settings.PixelPoints;
        var robotPoints = settings.RobotPoints;
        if (pixelPoints.Count < 4 || pixelPoints.Count != robotPoints.Count)
            return false;

        string key = BuildHomographyKey(pixelPoints, robotPoints);

        lock (HomographyLock)
        {
            if (_homography is not null && !_homography.Empty() && key == _homographyKey)
            {
                homography = _homography;
                return true;
            }

            Point2f[] src = pixelPoints.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
            Point2f[] dst = robotPoints.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
            using var srcMat = InputArray.Create(src);
            using var dstMat = InputArray.Create(dst);
            using var inlierMask = new Mat();

            using var computed = Cv2.FindHomography(srcMat, dstMat, HomographyMethods.Ransac, 3.0, inlierMask);
            if (computed.Empty())
                return false;

            _homography?.Dispose();
            _homography = computed.Clone();
            _homographyKey = key;
            homography = _homography;
            return true;
        }
    }

    private static bool TryMapPixelToRobot(double pixelX, double pixelY, Mat homography, out double robotX, out double robotY)
    {
        robotX = 0;
        robotY = 0;

        using var src = new Mat(1, 1, MatType.CV_32FC2, new[] { new Point2f((float)pixelX, (float)pixelY) });
        using var dst = new Mat();
        Cv2.PerspectiveTransform(src, dst, homography);
        if (dst.Empty())
            return false;

        Point2f mapped = dst.Get<Point2f>(0, 0);
        robotX = mapped.X;
        robotY = mapped.Y;
        return true;
    }

    private static string BuildHomographyKey(IReadOnlyList<CalibrationPoint> pixelPoints, IReadOnlyList<CalibrationPoint> robotPoints)
    {
        var sb = new StringBuilder(256);
        var inv = CultureInfo.InvariantCulture;
        foreach (var p in pixelPoints)
            sb.Append(p.X.ToString("G", inv)).Append(',').Append(p.Y.ToString("G", inv)).Append(';');
        sb.Append('|');
        foreach (var p in robotPoints)
            sb.Append(p.X.ToString("G", inv)).Append(',').Append(p.Y.ToString("G", inv)).Append(';');
        return sb.ToString();
    }
}
