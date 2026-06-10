namespace BoltPixelDetectorApp;

public static class SystemConfig
{
    public static double SOFTWARE_GAIN = 1.0;
    public static double MIN_DETECTION_CONFIDENCE = 0.50;
    public static string YOLO_MODEL_PATH = Path.Combine(AppContext.BaseDirectory, "best2.onnx");
    public static string YOLO_PYTHON_EXE = "python";
    public static int YOLO_IMAGE_SIZE = 640;
    public static double YOLO_NMS_THRESHOLD = 0.45;
    /// <summary>Fused mask must be at least this fraction of the YOLO mask area to replace geometry (else YOLO-only contour is used).</summary>
    public static double FUSION_MIN_AREA_RATIO = 0.03;

    public static string EXPORT_CSV_DIRECTORY =
        @"F:\Dai_Hoc_Nam_5\DATN_5_5\RobotVisionApp\BoltPixelDetectorApp\exports\CSV";
    public static string EXPORT_IMAGES_DIRECTORY =
        @"F:\Dai_Hoc_Nam_5\DATN_5_5\RobotVisionApp\BoltPixelDetectorApp\exports\Images";

    public static bool USE_NEPTUNE_SDK = true;
    public static string NEPTUNE_SDK_ROOT = @"F:\IMI Tech\Neptune";
    public static int NEPTUNE_SHUTTER_US = 27300;
    public static int NEPTUNE_EXPOSURE_MIN = 1000;
    public static int NEPTUNE_EXPOSURE_MAX = 60000;
    public static bool NEPTUNE_DATA_IS_BGR = true;
    public static bool NEPTUNE_FORCE_BAYER = false;
    public static string NEPTUNE_BAYER_PATTERN = "BG";
}
