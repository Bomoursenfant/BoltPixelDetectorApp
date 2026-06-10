using OpenCvSharp;

namespace BoltPixelDetectorApp;

/// <summary>
/// Bolt size in mm: width across head (ObjectXAxis) and length along shank (ObjectYAxis).
/// </summary>
public readonly record struct BoltSizeStats(
    double? WidthMinMm,
    double? WidthMaxMm,
    double? LengthMinMm,
    double? LengthMaxMm,
    double? AreaMin,
    double? AreaMax,
    int MeasurableCount);

public static class BoltSizeFilter
{
    public static BoltSizeStats ComputeBoltSizeStats(
        IReadOnlyList<DetectionResult> detections,
        VisionSettings settings)
    {
        double? widthMin = null;
        double? widthMax = null;
        double? lengthMin = null;
        double? lengthMax = null;
        double? areaMin = null;
        double? areaMax = null;
        int measurable = 0;

        foreach (var detection in detections)
        {
            if (!TryMeasureBoltSizeMm(detection, settings, out double widthMm, out double lengthMm))
                continue;
            measurable++;
            widthMin = widthMin.HasValue ? Math.Min(widthMin.Value, widthMm) : widthMm;
            widthMax = widthMax.HasValue ? Math.Max(widthMax.Value, widthMm) : widthMm;
            lengthMin = lengthMin.HasValue ? Math.Min(lengthMin.Value, lengthMm) : lengthMm;
            lengthMax = lengthMax.HasValue ? Math.Max(lengthMax.Value, lengthMm) : lengthMm;
            areaMin = areaMin.HasValue ? Math.Min(areaMin.Value, detection.Area) : detection.Area;
            areaMax = areaMax.HasValue ? Math.Max(areaMax.Value, detection.Area) : detection.Area;
        }

        return new BoltSizeStats(widthMin, widthMax, lengthMin, lengthMax, areaMin, areaMax, measurable);
    }

    /// <summary>Backward-compatible alias for width-only call sites.</summary>
    public static HeadDiameterStats ComputeHeadDiameterStats(
        IReadOnlyList<DetectionResult> detections,
        VisionSettings settings)
    {
        var s = ComputeBoltSizeStats(detections, settings);
        return new HeadDiameterStats(s.WidthMinMm, s.WidthMaxMm, s.MeasurableCount);
    }

    public static bool TryMeasureBoltSizeMm(
        DetectionResult detection,
        VisionSettings settings,
        out double widthMm,
        out double lengthMm)
    {
        widthMm = 0;
        lengthMm = 0;
        bool okWidth = TryMeasureHeadWidthMm(detection, settings, out widthMm);
        bool okLength = TryMeasureBoltLengthMm(detection, settings, out lengthMm);
        return okWidth && okLength;
    }

    /// <summary>Head width (mm) along <see cref="DetectionResult.ObjectXAxis"/> — shorter box side.</summary>
    public static bool TryMeasureHeadWidthMm(DetectionResult detection, VisionSettings settings, out double widthMm)
    {
        widthMm = 0;
        if (!TryGetWidthAxis(detection, out float axisX, out float axisY, out float halfPx))
            return false;
        return TryMeasureSpanMm(detection, settings, axisX, axisY, halfPx, out widthMm);
    }

    /// <summary>Shank length (mm) along <see cref="DetectionResult.ObjectYAxis"/> — longer box side.</summary>
    public static bool TryMeasureBoltLengthMm(DetectionResult detection, VisionSettings settings, out double lengthMm)
    {
        lengthMm = 0;
        if (!TryGetLengthAxis(detection, out float axisX, out float axisY, out float halfPx))
            return false;
        return TryMeasureSpanMm(detection, settings, axisX, axisY, halfPx, out lengthMm);
    }

    /// <summary>Alias: head diameter = width across bolt head.</summary>
    public static bool TryMeasureHeadDiameterMm(DetectionResult detection, VisionSettings settings, out double diameterMm)
        => TryMeasureHeadWidthMm(detection, settings, out diameterMm);

    public static bool IsWithinM8SizeRange(double widthMm, double lengthMm, double area, VisionSettings settings)
    {
        if (widthMm < settings.M8HeadDiameterMinMm || widthMm > settings.M8HeadDiameterMaxMm)
            return false;
        if (lengthMm < settings.M8LengthMinMm || lengthMm > settings.M8LengthMaxMm)
            return false;
        if (area < settings.M8AreaMin || area > settings.M8AreaMax)
            return false;
        return true;
    }

    public static bool IsWithinM8SizeRange(double widthMm, double lengthMm, VisionSettings settings)
    {
        return widthMm >= settings.M8HeadDiameterMinMm && widthMm <= settings.M8HeadDiameterMaxMm &&
               lengthMm >= settings.M8LengthMinMm && lengthMm <= settings.M8LengthMaxMm;
    }

    public static bool IsWithinM8HeadRange(double headDiameterMm, VisionSettings settings)
    {
        return headDiameterMm >= settings.M8HeadDiameterMinMm && headDiameterMm <= settings.M8HeadDiameterMaxMm;
    }

    public static List<DetectionResult> ApplyM8HeadFilter(IReadOnlyList<DetectionResult> detections, VisionSettings settings)
    {
        if (!settings.EnableM8SizeFilter)
            return detections.ToList();

        var kept = new List<DetectionResult>();
        foreach (var detection in detections)
        {
            if (!TryMeasureBoltSizeMm(detection, settings, out double widthMm, out double lengthMm))
                continue;
            if (!IsWithinM8SizeRange(widthMm, lengthMm, detection.Area, settings))
                continue;
            kept.Add(detection);
        }

        for (int i = 0; i < kept.Count; i++)
            kept[i] = kept[i].WithId(i + 1);

        return kept;
    }

    private static bool TryMeasureSpanMm(
        DetectionResult detection,
        VisionSettings settings,
        float axisImageX,
        float axisImageY,
        float halfSpanPx,
        out double spanMm)
    {
        spanMm = 0;
        double workX = detection.PixelX;
        double workY = detection.PixelY;

        double workX1 = workX + axisImageY * halfSpanPx;
        double workY1 = workY - axisImageX * halfSpanPx;
        double workX2 = workX - axisImageY * halfSpanPx;
        double workY2 = workY + axisImageX * halfSpanPx;

        if (!CoordinateMapper.TryDistanceMm(workX1, workY1, workX2, workY2, settings, out spanMm))
            return false;

        return spanMm > 0.5;
    }

    private static bool TryGetWidthAxis(
        DetectionResult detection,
        out float axisImageX,
        out float axisImageY,
        out float halfSpanPx)
    {
        if (!TryNormalizeAxis(detection.ObjectXAxis, detection.ObjectYAxis, perpendicular: true, out axisImageX, out axisImageY))
        {
            axisImageX = 0;
            axisImageY = 0;
            halfSpanPx = 0;
            return false;
        }

        return TryGetHalfSpanPx(detection, useLongSide: false, out halfSpanPx);
    }

    private static bool TryGetLengthAxis(
        DetectionResult detection,
        out float axisImageX,
        out float axisImageY,
        out float halfSpanPx)
    {
        if (!TryNormalizeAxis(detection.ObjectYAxis, detection.ObjectXAxis, perpendicular: false, out axisImageX, out axisImageY))
        {
            axisImageX = 0;
            axisImageY = 0;
            halfSpanPx = 0;
            return false;
        }

        return TryGetHalfSpanPx(detection, useLongSide: true, out halfSpanPx);
    }

    private static bool TryNormalizeAxis(
        Point2f primary,
        Point2f fallbackPerp,
        bool perpendicular,
        out float axisImageX,
        out float axisImageY)
    {
        float len = MathF.Sqrt(primary.X * primary.X + primary.Y * primary.Y);
        if (len >= 1e-4f)
        {
            axisImageX = primary.X / len;
            axisImageY = primary.Y / len;
            return true;
        }

        float flen = MathF.Sqrt(fallbackPerp.X * fallbackPerp.X + fallbackPerp.Y * fallbackPerp.Y);
        if (flen < 1e-4f)
        {
            axisImageX = 0;
            axisImageY = 0;
            return false;
        }

        if (perpendicular)
        {
            axisImageX = fallbackPerp.Y / flen;
            axisImageY = -fallbackPerp.X / flen;
        }
        else
        {
            axisImageX = fallbackPerp.X / flen;
            axisImageY = fallbackPerp.Y / flen;
        }

        return true;
    }

    private static bool TryGetHalfSpanPx(DetectionResult detection, bool useLongSide, out float halfSpanPx)
    {
        halfSpanPx = 0;
        var size = detection.RotatedBox.Size;
        if (size.Width > 1 && size.Height > 1)
        {
            halfSpanPx = (useLongSide ? Math.Max(size.Width, size.Height) : Math.Min(size.Width, size.Height)) * 0.5f;
            return halfSpanPx >= 0.5f;
        }

        if (detection.Radius >= 1)
        {
            halfSpanPx = (float)detection.Radius * (useLongSide ? 1.0f : 0.85f);
            return halfSpanPx >= 0.5f;
        }

        return false;
    }

    /// <summary>Stable key to match a detection before/after M8 and robot-safety reindexing.</summary>
    public static string RobotDetectionKey(DetectionResult d) =>
        FormattableString.Invariant(
            $"{d.PixelX:F3}|{d.PixelY:F3}|{d.Angle:F3}|{d.Area:F0}|{d.Center.X:F1}|{d.Center.Y:F1}");

    public static string FormatRobotSendText(bool wouldSend) => wouldSend ? "yes" : "no";
}

/// <summary>Legacy stats type (width only).</summary>
public readonly record struct HeadDiameterStats(double? MinMm, double? MaxMm, int MeasurableCount);
