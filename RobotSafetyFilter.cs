namespace BoltPixelDetectorApp;

internal static class RobotSafetyFilter
{
    /// <summary>
    /// Filters robot-send candidates: 360° center spacing + optional B1 scene proximity (B3).
    /// </summary>
    public static List<DetectionResult> ApplyForRobotSend(
        IReadOnlyList<DetectionResult> allDetections,
        IReadOnlyList<DetectionResult> candidates,
        VisionSettings settings,
        SceneBinaryContext? sceneContext,
        out int rejectedCount)
    {
        rejectedCount = 0;
        if (!settings.EnableRobotSafetyFilter && !settings.EnableSceneProximityFilter)
            return candidates.ToList();

        var unsafeKeys = BuildUnsafeDetectionKeys(allDetections, candidates, settings, sceneContext);
        if (unsafeKeys.Count == 0)
            return candidates.ToList();

        var kept = new List<DetectionResult>();
        foreach (var detection in candidates)
        {
            if (unsafeKeys.Contains(BoltSizeFilter.RobotDetectionKey(detection)))
                rejectedCount++;
            else
                kept.Add(detection);
        }

        if (rejectedCount == 0)
            return candidates.ToList();

        for (int i = 0; i < kept.Count; i++)
            kept[i] = ReindexDetection(kept[i], i + 1);

        return kept;
    }

    public static List<DetectionResult> Apply(
        IReadOnlyList<DetectionResult> detections,
        VisionSettings settings,
        out int rejectedCount) =>
        ApplyForRobotSend(detections, detections, settings, sceneContext: null, out rejectedCount);

    public static HashSet<string> BuildUnsafeDetectionKeys(
        IReadOnlyList<DetectionResult> allDetections,
        VisionSettings settings,
        SceneBinaryContext? sceneContext) =>
        BuildUnsafeDetectionKeys(allDetections, allDetections, settings, sceneContext);

    private static HashSet<string> BuildUnsafeDetectionKeys(
        IReadOnlyList<DetectionResult> allDetections,
        IReadOnlyList<DetectionResult> subjects,
        VisionSettings settings,
        SceneBinaryContext? sceneContext)
    {
        var unsafeKeys = new HashSet<string>(StringComparer.Ordinal);
        var subjectKeys = new HashSet<string>(
            subjects.Select(BoltSizeFilter.RobotDetectionKey),
            StringComparer.Ordinal);

        if (settings.EnableRobotSafetyFilter && allDetections.Count >= 2)
        {
            int n = allDetections.Count;
            int dilatePx = Math.Max(1, settings.SceneProximityDilatePx);
            OpenCvSharp.Size? frameSize = sceneContext?.SceneBinary.Size();

            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    if (!IsNeighborCenterTooClose(allDetections[i], allDetections[j], settings))
                        continue;

                    // Centers close along head/tail (Y') — spacing rule does not block pick.
                    if (MaskChainContact.IsOtherAlongSubjectEndAxis(allDetections[i], allDetections[j]) ||
                        MaskChainContact.IsOtherAlongSubjectEndAxis(allDetections[j], allDetections[i]))
                    {
                        continue;
                    }

                    if (frameSize is not null &&
                        MaskChainContact.ShouldExemptPairForRobot(
                            allDetections[i], allDetections[j], frameSize.Value, dilatePx))
                    {
                        continue;
                    }

                    string keyA = BoltSizeFilter.RobotDetectionKey(allDetections[i]);
                    string keyB = BoltSizeFilter.RobotDetectionKey(allDetections[j]);
                    if (subjectKeys.Contains(keyA))
                    {
                        unsafeKeys.Add(keyA);
                        try { Console.WriteLine($"DEBUG: Mark unsafe (pairwise) {keyA} due to neighbor {BoltSizeFilter.RobotDetectionKey(allDetections[j])}"); } catch { }
                    }
                    if (subjectKeys.Contains(keyB))
                    {
                        unsafeKeys.Add(keyB);
                        try { Console.WriteLine($"DEBUG: Mark unsafe (pairwise) {keyB} due to neighbor {BoltSizeFilter.RobotDetectionKey(allDetections[i])}"); } catch { }
                    }
                }
            }
        }

        if (settings.EnableSceneProximityFilter && sceneContext is not null)
        {
            foreach (string key in sceneContext.BuildUnsafeDetectionKeys(allDetections, subjects, settings))
            {
                unsafeKeys.Add(key);
                try { Console.WriteLine($"DEBUG: Mark unsafe (scene) {key}"); } catch { }
            }
        }

        return unsafeKeys;
    }

    public static bool IsUnsafePair(DetectionResult a, DetectionResult b, VisionSettings settings)
    {
        if (!IsNeighborCenterTooClose(a, b, settings))
            return false;

        int dilatePx = Math.Max(1, settings.SceneProximityDilatePx);
        if (MaskChainContact.IsEndToEndChainContact(a, b, dilatePx))
            return false;

        return true;
    }

    /// <summary>
    /// True when two detection centers are closer than the configured spacing (360°, any object types).
    /// </summary>
    public static bool IsNeighborCenterTooClose(DetectionResult a, DetectionResult b, VisionSettings settings)
    {
        double requiredMm = settings.RobotSafetyM8MinCenterSpacingMm + settings.RobotSafetyM8CenterSpacingMarginMm;
        if (requiredMm > 0 &&
            CoordinateMapper.TryDistanceMm(a.PixelX, a.PixelY, b.PixelX, b.PixelY, settings, out double distMm))
        {
            return distMm < requiredMm;
        }

        double requiredPx = GetMinCenterSpacingPx(a, b, settings);
        return requiredPx > 0 && CenterDistancePx(a, b) < requiredPx;
    }

    public static double GetMinCenterSpacingPx(DetectionResult a, DetectionResult b, VisionSettings settings)
    {
        if (settings.RobotSafetyM8MinCenterSpacingPx > 0)
            return settings.RobotSafetyM8MinCenterSpacingPx;

        double requiredMm = settings.RobotSafetyM8MinCenterSpacingMm + settings.RobotSafetyM8CenterSpacingMarginMm;
        double pxPerMm = EstimatePxPerMm(a, settings);
        if (pxPerMm <= 0)
            pxPerMm = EstimatePxPerMm(b, settings);
        if (pxPerMm <= 0)
            return 0;

        return requiredMm * pxPerMm;
    }

    private static double EstimatePxPerMm(DetectionResult d, VisionSettings settings)
    {
        if (!BoltSizeFilter.TryMeasureHeadWidthMm(d, settings, out double widthMm) || widthMm <= 0.5)
            return 0;

        var size = d.RotatedBox.Size;
        if (size.Width <= 1 || size.Height <= 1)
            return 0;

        double headSpanPx = Math.Min(size.Width, size.Height);
        return headSpanPx / widthMm;
    }

    private static DetectionResult ReindexDetection(DetectionResult d, int id) => d.WithId(id);

    private static double CenterDistancePx(DetectionResult a, DetectionResult b)
    {
        double dx = a.Center.X - b.Center.X;
        double dy = a.Center.Y - b.Center.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

}
