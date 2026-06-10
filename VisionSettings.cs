using System.Text.Json;

namespace BoltPixelDetectorApp;

public sealed class VisionSettings
{
    private const bool LockCalibrationPoints = true;
    public string SdkRoot { get; set; } = SystemConfig.NEPTUNE_SDK_ROOT;
    public int ExposureUs { get; set; } = SystemConfig.NEPTUNE_SHUTTER_US;
    public int DetectorMode { get; set; } = 2;
    public string YoloModelPath { get; set; } = SystemConfig.YOLO_MODEL_PATH;
    public string YoloPythonExe { get; set; } = SystemConfig.YOLO_PYTHON_EXE;
    public int YoloImageSize { get; set; } = SystemConfig.YOLO_IMAGE_SIZE;
    public double YoloNmsThreshold { get; set; } = SystemConfig.YOLO_NMS_THRESHOLD;
    public int DisplayWidth { get; set; }
    public int DisplayHeight { get; set; }
    public bool UseRoi { get; set; } = true;
    public int RoiX { get; set; } = 500;
    public int RoiY { get; set; } = 1060;
    public int RoiWidth { get; set; } = 3290;
    public int RoiHeight { get; set; } = 2440;
    public int Threshold { get; set; }
    public double MinArea { get; set; } = 1000;
    public double MaxArea { get; set; } = 100000;
    public double MinCircularity { get; set; } = 0.10;
    /// <summary>Real detection threshold (%). Objects below this are rejected during detect.</summary>
    public double MinConfidencePercent { get; set; } = SystemConfig.MIN_DETECTION_CONFIDENCE * 100.0;

    public int CrosshairLength { get; set; } = 55;

    public static double ToConfidencePercent(double confidence01) =>
        Math.Clamp(confidence01 * 100.0, 0, 100);
    public bool InvertBinary { get; set; }
    public bool ShowMask { get; set; } = true;

    /// <summary>When true, only detections within M8 width/length/area ranges are kept.</summary>
    public bool EnableM8SizeFilter { get; set; }

    /// <summary>Minimum bolt width (mm) for M8 — calibrate on your parts (hex across flats ~13 mm).</summary>
    public double M8HeadDiameterMinMm { get; set; } = 11.5;

    /// <summary>Maximum bolt width (mm) for M8.</summary>
    public double M8HeadDiameterMaxMm { get; set; } = 14.5;

    /// <summary>Minimum visible shank length (mm) for M8×15 — calibrate with Test Camera length column.</summary>
    public double M8LengthMinMm { get; set; } = 10.0;

    /// <summary>Maximum visible shank length (mm) for M8×15 (rejects longer e.g. M8×30).</summary>
    public double M8LengthMaxMm { get; set; } = 20.0;

    /// <summary>Minimum detected contour/mask area (pixel^2) for M8 filter.</summary>
    public double M8AreaMin { get; set; } = 1000;

    /// <summary>Maximum detected contour/mask area (pixel^2) for M8 filter.</summary>
    public double M8AreaMax { get; set; } = 100000;

    /// <summary>When true, detections that are too close/overlapping are not sent to robot/API/DB.</summary>
    public bool EnableRobotSafetyFilter { get; set; } = true;

    /// <summary>Minimum center-to-center spacing (mm) between any two detections (360° filter).</summary>
    public double RobotSafetyM8MinCenterSpacingMm { get; set; } = 15.0;

    /// <summary>Extra margin added to min center spacing (mm).</summary>
    public double RobotSafetyM8CenterSpacingMarginMm { get; set; } = 2.0;

    /// <summary>Fallback min center spacing (px); 0 = auto from mm + head width estimate.</summary>
    public double RobotSafetyM8MinCenterSpacingPx { get; set; } = 0;

    /// <summary>B1+B3: use full-scene OpenCV binary to block send when M8 masks touch or sit by undetected bolts.</summary>
    public bool EnableSceneProximityFilter { get; set; } = true;

    /// <summary>B3: dilation (px) when testing mask contact on scene binary.</summary>
    public int SceneProximityDilatePx { get; set; } = 12;

    /// <summary>B3: min area (px^2) for a B1 blob without YOLO to count as a blocking neighbor.</summary>
    public double SceneProximityMinNeighborBlobArea { get; set; } = 800;

    /// <summary>B3: if B1 component area exceeds this ratio × M8 mask area, treat as merged/touching.</summary>
    public double SceneProximityMergedComponentAreaRatio { get; set; } = 1.75;

    // Calibration points for homography (pixel -> robot). Must have >= 4 points and same length.
    // Pixel points use the working coordinate system (origin bottom-left, +X up, +Y right).
    public List<CalibrationPoint> PixelPoints { get; set; } = new();
    public List<CalibrationPoint> RobotPoints { get; set; } = new();

    private static string SettingsPath => Path.Combine(AppContext.BaseDirectory, "VisionSettings.json");

    public static VisionSettings Load()
    {
        try
        {
            if (File.Exists(SettingsPath))
            {
                var settings = JsonSerializer.Deserialize<VisionSettings>(File.ReadAllText(SettingsPath));
                if (settings is not null)
                {
                    settings.PixelPoints ??= new List<CalibrationPoint>();
                    settings.RobotPoints ??= new List<CalibrationPoint>();
                    if (LockCalibrationPoints || !HasValidCalibration(settings))
                    {
                        ApplyDefaultCalibration(settings);
                        TrySave(settings);
                    }
                    return settings;
                }
            }
        }
        catch
        {
            // Bad settings should not prevent the app from starting.
        }

        var fallback = new VisionSettings();
        ApplyDefaultCalibration(fallback);
        return fallback;
    }

    private static bool HasValidCalibration(VisionSettings settings)
    {
        if (settings.PixelPoints is null || settings.RobotPoints is null)
            return false;
        if (settings.PixelPoints.Count < 4 || settings.RobotPoints.Count < 4)
            return false;
        return settings.PixelPoints.Count == settings.RobotPoints.Count;
    }

    private static void ApplyDefaultCalibration(VisionSettings settings)
    {
        settings.PixelPoints = BuildDefaultPixelPoints();
        settings.RobotPoints = BuildDefaultRobotPoints();
    }

    private static void TrySave(VisionSettings settings)
    {
        try
        {
            settings.Save();
        }
        catch
        {
            // Ignore write failures; defaults are still in memory.
        }
    }

    private static List<CalibrationPoint> BuildDefaultPixelPoints()
    {
        return new List<CalibrationPoint>
        {
            new CalibrationPoint { X = 2568, Y = 1781 },
            new CalibrationPoint { X = 2390.794, Y = 821.179 },
            new CalibrationPoint { X = 2348.078, Y = 1270.992 },
            new CalibrationPoint { X = 2026.513, Y = 853.082 },
            new CalibrationPoint { X = 1898.357, Y = 1825.416 },
            new CalibrationPoint { X = 1804.431, Y = 1306.207 },
            new CalibrationPoint { X = 1235.051, Y = 1690.275 },
            new CalibrationPoint { X = 1141.159, Y = 803.439 },
            new CalibrationPoint { X = 1127.993, Y = 1202.219 },
            new CalibrationPoint { X = 2460.474, Y = 1449.549 },
            new CalibrationPoint { X = 2322.953, Y = 2094.926 },
            new CalibrationPoint { X = 2242.017, Y = 2487.279 },
            new CalibrationPoint { X = 1926.820, Y = 1736.907 },
            new CalibrationPoint { X = 1720.368, Y = 2589.119 },
            new CalibrationPoint { X = 1696.274, Y = 1316.711 },
            new CalibrationPoint { X = 1534.078, Y = 2153.408 },
            new CalibrationPoint { X = 943.765, Y = 1580.460 },
            new CalibrationPoint { X = 2692.657, Y = 1534.831 },
            new CalibrationPoint { X = 2254.243, Y = 1041.210 },
            new CalibrationPoint { X = 1932.930, Y = 2282.271 },
            new CalibrationPoint { X = 1825.640, Y = 1734.163 },
            new CalibrationPoint { X = 1436.117, Y = 1049.657 },
            new CalibrationPoint { X = 1211.529, Y = 1702.344 },
            new CalibrationPoint { X = 906.738, Y = 1153.840 },
            new CalibrationPoint { X = 2711.19, Y = 1250.688 },
            new CalibrationPoint { X = 2365.332, Y = 1707.682 },
            new CalibrationPoint { X = 2104.806, Y = 2199.317 },
            new CalibrationPoint { X = 2044.749, Y = 907.331 },
            new CalibrationPoint { X = 1660.295, Y = 1416.596 },
            new CalibrationPoint { X = 1545.11, Y = 2196.337 },
            new CalibrationPoint { X = 1076.942, Y = 749.299 },
            new CalibrationPoint { X = 982.138, Y = 1342.33 },
            new CalibrationPoint { X = 937.038, Y = 2137.427 }
        };
    }

    private static List<CalibrationPoint> BuildDefaultRobotPoints()
    {
        return new List<CalibrationPoint>
        {
            new CalibrationPoint { X = 476.38, Y = 82.37 },
            new CalibrationPoint { X = 454.20, Y = 200.3 },
            new CalibrationPoint { X = 448.72, Y = 145.36 },
            new CalibrationPoint { X = 407.59, Y = 197.30 },
            new CalibrationPoint { X = 395.08, Y = 76.01 },
            new CalibrationPoint { X = 382.68, Y = 140.22 },
            new CalibrationPoint { X = 314.96, Y = 90.56 },
            new CalibrationPoint { X = 299.88, Y = 201.92 },
            new CalibrationPoint { X = 299.06, Y = 151.46 },
            new CalibrationPoint { X = 462.53, Y = 123.56 },
            new CalibrationPoint { X = 447.45, Y = 44.88 },
            new CalibrationPoint { X = 439.67, Y = -3.48 },
            new CalibrationPoint { X = 399.34, Y = 87.25 },
            new CalibrationPoint { X = 377.68, Y = -18.41 },
            new CalibrationPoint { X = 369.38, Y = 140.3 },
            new CalibrationPoint { X = 352.34, Y = 34.66 },
            new CalibrationPoint { X = 279.13, Y = 104.28 },
            new CalibrationPoint { X = 491.97, Y = 114.87 },
            new CalibrationPoint { X = 437.27, Y = 174.90 },
            new CalibrationPoint { X = 402.42, Y = 19.52 },
            new CalibrationPoint { X = 388.30, Y = 86.80 },
            new CalibrationPoint { X = 339.67, Y = 170.35 },
            new CalibrationPoint { X = 312.39, Y = 88.49 },
            new CalibrationPoint { X = 273.16, Y = 156.29 },
            new CalibrationPoint { X = 493.66, Y = 149.91 },
            new CalibrationPoint { X = 455.56, Y = 91.67 },
            new CalibrationPoint { X = 422.68, Y = 30.89 },
            new CalibrationPoint { X = 412.12, Y = 188.20 },
            new CalibrationPoint { X = 368.12, Y = 124.48 },
            new CalibrationPoint { X = 354.46, Y = 27.32 },
            new CalibrationPoint { X = 291.29, Y = 207.75 },
            new CalibrationPoint { X = 283.23, Y = 132.56 },
            new CalibrationPoint { X = 279.13, Y = 36.29 }
        };
    }

    public void Save()
    {
        var json = JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true });
        File.WriteAllText(SettingsPath, json);
    }

    public void ApplyToSystemConfig()
    {
        SystemConfig.NEPTUNE_SDK_ROOT = SdkRoot;
        SystemConfig.NEPTUNE_SHUTTER_US = ExposureUs;
        SystemConfig.YOLO_MODEL_PATH = YoloModelPath;
        SystemConfig.YOLO_PYTHON_EXE = YoloPythonExe;
        SystemConfig.YOLO_IMAGE_SIZE = YoloImageSize;
        SystemConfig.YOLO_NMS_THRESHOLD = YoloNmsThreshold;
        SystemConfig.MIN_DETECTION_CONFIDENCE = MinConfidencePercent / 100.0;
    }
}

public sealed class CalibrationPoint
{
    public double X { get; set; }
    public double Y { get; set; }
}
