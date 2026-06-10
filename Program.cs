namespace BoltPixelDetectorApp;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        OpenCvSharp.Cv2.SetUseOptimized(true);
        OpenCvSharp.Cv2.SetNumThreads(GetOpenCvThreadCount());

        ApplicationConfiguration.Initialize();
        UiTheme.Initialize();
        Application.Run(new MainForm());
    }

    private static int GetOpenCvThreadCount()
    {
        string? configured = Environment.GetEnvironmentVariable("BOLT_OPENCV_THREADS");
        if (int.TryParse(configured, out int threads) && threads > 0)
            return Math.Min(threads, Math.Max(1, Environment.ProcessorCount));

        return Math.Max(1, Environment.ProcessorCount);
    }
}
