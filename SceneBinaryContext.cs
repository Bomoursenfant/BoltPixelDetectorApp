using OpenCvSharp;
using CvPoint = OpenCvSharp.Point;

namespace BoltPixelDetectorApp;

/// <summary>
/// B1: full-scene OpenCV binary cached for one frame.
/// B3: compares YOLO+M8 masks (B2) against scene components to block robot send when bolts touch.
/// </summary>
public sealed class SceneBinaryContext : IDisposable
{
    public Mat SceneBinary { get; }
    public int ComponentCount { get; }

    private readonly Mat _labels;
    private readonly Dictionary<int, SceneComponentInfo> _components = new();

    private SceneBinaryContext(Mat sceneBinary, Mat labels, int componentCount)
    {
        SceneBinary = sceneBinary;
        _labels = labels;
        ComponentCount = componentCount;
    }

    public static SceneBinaryContext? TryCreate(Mat sceneBinary8u)
    {
        if (sceneBinary8u.Empty() || sceneBinary8u.Type() != MatType.CV_8UC1)
            return null;

        var labels = new Mat();
        int count = Cv2.ConnectedComponents(sceneBinary8u, labels, PixelConnectivity.Connectivity8, MatType.CV_32S);
        if (count <= 1)
        {
            labels.Dispose();
            return null;
        }

        return new SceneBinaryContext(sceneBinary8u.Clone(), labels, count);
    }

    public HashSet<string> BuildUnsafeDetectionKeys(
        IReadOnlyList<DetectionResult> detections,
        VisionSettings settings) =>
        BuildUnsafeDetectionKeys(detections, detections, settings);

    public HashSet<string> BuildUnsafeDetectionKeys(
        IReadOnlyList<DetectionResult> detections,
        IReadOnlyList<DetectionResult> subjects,
        VisionSettings settings)
    {
        var unsafeKeys = new HashSet<string>(StringComparer.Ordinal);
        if (detections.Count == 0 || subjects.Count == 0)
            return unsafeKeys;

        var subjectKeys = new HashSet<string>(
            subjects.Select(BoltSizeFilter.RobotDetectionKey),
            StringComparer.Ordinal);

        int dilatePx = Math.Max(1, settings.SceneProximityDilatePx);
        double minForeignArea = Math.Max(1, settings.SceneProximityMinNeighborBlobArea);
        var frameSize = SceneBinary.Size();

        var masks = new List<InstanceMaskEntry>();
        using (LightProfiler.Measure("Scene:BuildMasks"))
        {
            foreach (var detection in detections)
            {
                Mat? instanceMask = BuildInstanceMask(detection, frameSize);
                if (instanceMask is null)
                    continue;

                int primaryLabel = FindPrimaryLabel(instanceMask);
                masks.Add(new InstanceMaskEntry(detection, instanceMask, primaryLabel));
            }
        }

        if (masks.Count == 0)
            return unsafeKeys;

        MaskChainContact.DilationCacheClear();
        var claimedLabels = new HashSet<int>(masks.Select(m => m.PrimaryLabel).Where(l => l > 0));
        var maskRefs = masks
            .Select(m => new MaskChainContact.InstanceMaskRef(m.Detection, m.Mask))
            .ToList();

        // Precompute expensive subject probes only for robot-send candidates.
        var lateralProbesFromMask = new Mat?[masks.Count];
        var lateralProbesFromTight = new Mat?[masks.Count];
        for (int mi = 0; mi < masks.Count; mi++)
        {
            var m = masks[mi];
            if (!subjectKeys.Contains(BoltSizeFilter.RobotDetectionKey(m.Detection)))
                continue;

            using var tight = MaskChainContact.BuildTightB2Mask(m.Detection, frameSize);
            lateralProbesFromTight[mi] = MaskChainContact.BuildLateralFlankProbeMask(tight, m.Detection, MaskChainContact.LateralProbeOutwardPxForB3);
            lateralProbesFromMask[mi] = MaskChainContact.BuildLateralFlankProbeMask(m.Mask, m.Detection, MaskChainContact.LateralProbeOutwardPxForB3);
        }

        using (LightProfiler.Measure("Scene:MainLoop"))
        {
            object unsafeLock = new();
            Parallel.For(0, masks.Count, i =>
            {
                var a = masks[i];
                if (!subjectKeys.Contains(BoltSizeFilter.RobotDetectionKey(a.Detection)))
                    return;

                using (LightProfiler.Measure("Mask:PerDetection"))
                {
                    var localUnsafe = new List<string>();
                    string keyA = BoltSizeFilter.RobotDetectionKey(a.Detection);
                    bool relaxFlankNoise = MaskChainContact.ShouldRelaxB3FlankNoiseChecks(
                        a.Detection, maskRefs, frameSize, dilatePx);
                    bool clearFlanksVsDetections = MaskChainContact.HasClearFlanksVsAllDetections(
                        a.Detection, a.Mask, maskRefs, dilatePx);

                    // B3: B1 beside flank — tight B2 vs B1 (#1 + undetected neighbor), same-component blob, or unclaimed label.
                    if (MaskChainContact.HasB1ForeignOnSubjectFlank(
                            a.Detection, SceneBinary, a.Mask, maskRefs, frameSize, dilatePx) ||
                        (!relaxFlankNoise &&
                         lateralProbesFromTight[i] is not null &&
                         HasForeignNeighborOnSideBand(a, maskRefs, claimedLabels, frameSize, dilatePx, minForeignArea, lateralProbesFromTight[i]!)) ||
                        (!clearFlanksVsDetections && !relaxFlankNoise &&
                         lateralProbesFromMask[i] is not null &&
                         HasSharedB1SideIntrusion(a, maskRefs, frameSize, dilatePx, minForeignArea, lateralProbesFromMask[i]!)))
                    {
                        localUnsafe.Add(keyA);
                    }

                    for (int j = 0; j < masks.Count; j++)
                    {
                        if (j == i)
                            continue;

                        var b = masks[j];

                        double pairSpan = Math.Max(
                            MaskChainContact.GetApproxSpanPx(a.Detection),
                            MaskChainContact.GetApproxSpanPx(b.Detection));
                        if (MaskChainContact.CenterDistancePx(a.Detection, b.Detection) > pairSpan * 1.12 + 22)
                            continue;

                        // Block only bolts whose flank (X') touches another mask — head/tail (Y') may touch.
                        if (MaskChainContact.HasSubjectFlankContactWithOther(
                                a.Detection, b.Detection, a.Mask, b.Mask, dilatePx))
                        {
                            localUnsafe.Add(keyA);
                        }
                    }

                    if (localUnsafe.Count > 0)
                    {
                        lock (unsafeLock)
                        {
                            foreach (string key in localUnsafe)
                                unsafeKeys.Add(key);
                        }
                    }
                }
            });
        }

        // dispose precomputed probe mats
        foreach (var m in lateralProbesFromMask)
            m?.Dispose();
        foreach (var m in lateralProbesFromTight)
            m?.Dispose();

        foreach (var entry in masks)
            entry.Dispose();

        return unsafeKeys;
    }

    private bool TryGetComponentInfo(int label, out SceneComponentInfo info)
    {
        if (_components.TryGetValue(label, out var cached))
        {
            info = cached;
            return true;
        }

        if (label <= 0 || label >= ComponentCount)
        {
            info = null!;
            return false;
        }

        using var componentMask = new Mat();
        Cv2.InRange(_labels, new Scalar(label), new Scalar(label), componentMask);
        Cv2.FindContours(
            componentMask,
            out CvPoint[][] contours,
            out _,
            RetrievalModes.External,
            ContourApproximationModes.ApproxSimple);

        CvPoint[]? best = null;
        double bestArea = 0;
        foreach (var contour in contours)
        {
            double area = Cv2.ContourArea(contour);
            if (area > bestArea)
            {
                bestArea = area;
                best = contour;
            }
        }

        if (best is null || best.Length < 3)
        {
            info = null!;
            return false;
        }

        var moments = Cv2.Moments(best);
        double cx = moments.M10 / (moments.M00 + 1e-6);
        double cy = moments.M01 / (moments.M00 + 1e-6);
        info = new SceneComponentInfo(label, bestArea, new Point2f((float)cx, (float)cy));
        _components[label] = info;
        return true;
    }

    private static Rect GetMaskRoiRect(Mat mask, int padPx = 14)
    {
        using var nz = new Mat();
        Cv2.FindNonZero(mask, nz);
        if (nz.Empty())
            return new Rect(0, 0, mask.Width, mask.Height);

        Rect r = Cv2.BoundingRect(nz);
        int x0 = Math.Max(0, r.X - padPx);
        int y0 = Math.Max(0, r.Y - padPx);
        int x1 = Math.Min(mask.Width, r.X + r.Width + padPx);
        int y1 = Math.Min(mask.Height, r.Y + r.Height + padPx);
        return new Rect(x0, y0, Math.Max(1, x1 - x0), Math.Max(1, y1 - y0));
    }

    private static Mat? BuildInstanceMask(DetectionResult detection, OpenCvSharp.Size frameSize)
    {
        var mask = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        if (detection.MaskContour is { Length: >= 3 } contour)
        {
            Cv2.FillPoly(mask, new[] { contour }, Scalar.White);
            return mask;
        }

        if (detection.RotatedBox.Size.Width > 1 && detection.RotatedBox.Size.Height > 1)
        {
            var boxPoints = Cv2.BoxPoints(detection.RotatedBox);
            var pts = boxPoints
                .Select(p => new CvPoint((int)Math.Round(p.X), (int)Math.Round(p.Y)))
                .ToArray();
            Cv2.FillPoly(mask, new[] { pts }, Scalar.White);
            return mask;
        }

        if (detection.BoundingBox.Width > 0 && detection.BoundingBox.Height > 0)
        {
            Cv2.Rectangle(mask, detection.BoundingBox, Scalar.White, thickness: -1);
            return mask;
        }

        return null;
    }

    private int FindPrimaryLabel(Mat instanceMask)
    {
        using var maskedLabels = new Mat();
        _labels.CopyTo(maskedLabels, instanceMask);
        Rect roi = GetMaskRoiRect(instanceMask, 0);
        var counts = new Dictionary<int, int>();
        int bestLabel = 0;
        int bestCount = 0;
        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                int* row = (int*)maskedLabels.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    int label = row[x];
                    if (label <= 0)
                        continue;

                    int c = counts.TryGetValue(label, out int existing) ? existing + 1 : 1;
                    counts[label] = c;
                    if (c > bestCount)
                    {
                        bestCount = c;
                        bestLabel = label;
                    }
                }
            }
        }

        return bestLabel;
    }

    /// <summary>
    /// Same B1 component as the bolt but B1 area outside B2 mask touches the side band (e.g. horizontal bolt merged on B1).
    /// </summary>
    private bool HasSharedB1SideIntrusion(
        InstanceMaskEntry entry,
        IReadOnlyList<MaskChainContact.InstanceMaskRef> maskRefs,
        OpenCvSharp.Size frameSize,
        int dilatePx,
        double minForeignArea,
        Mat lateralProbeFromMask)
    {
        if (entry.PrimaryLabel <= 0)
            return false;

        using var componentMask = new Mat();
        Cv2.InRange(_labels, new Scalar(entry.PrimaryLabel), new Scalar(entry.PrimaryLabel), componentMask);

        if (!MaskChainContact.TryGetBoltAxes(entry.Detection, out var yAxis, out var xAxis))
            return false;

        using var tightOwn = MaskChainContact.BuildTightB2Mask(entry.Detection, frameSize);
        using var othersUnion = MaskChainContact.BuildOthersUnionMask(maskRefs, entry.Detection, frameSize);
        using var dilatedOthers = new Mat();
        using var dilatedOwn = new Mat();
        using var kernel = CreateDilateKernel(dilatePx + 2);
        Cv2.Dilate(othersUnion, dilatedOthers, kernel);
        Cv2.Dilate(tightOwn, dilatedOwn, kernel);

        using var explained = new Mat();
        Cv2.BitwiseOr(dilatedOthers, dilatedOwn, explained);
        using var inverted = new Mat();
        Cv2.BitwiseNot(explained, inverted);
        using var foreignBlob = new Mat();
        Cv2.BitwiseAnd(componentMask, inverted, foreignBlob);

        if (Cv2.CountNonZero(foreignBlob) < SideContactMinPixels)
            return false;

        using var dilatedSide = new Mat();
        Cv2.Dilate(lateralProbeFromMask, dilatedSide, kernel);

        using var sideBand = MaskChainContact.BuildSideBandMask(entry.Mask, entry.Detection);
        using var intrusion = new Mat();
        Cv2.BitwiseAnd(dilatedSide, foreignBlob, intrusion);
        using var intrusionOnFlank = new Mat();
        Cv2.BitwiseAnd(intrusion, sideBand, intrusionOnFlank);
        if (Cv2.CountNonZero(intrusionOnFlank) < SideContactMinPixels)
            return false;

        if (!MaskChainContact.IsMaskB1HitLateralToSubject(entry.Detection, intrusionOnFlank, xAxis, yAxis))
            return false;

        if (MaskChainContact.IsMaskB1HitTowardEndCap(entry.Detection, intrusionOnFlank, xAxis, yAxis))
            return false;

        if (MaskChainContact.IsB1RegionMostlyOnEndCapPartnerPublic(
                entry.Detection, intrusionOnFlank, maskRefs, dilatePx))
        {
            return false;
        }

        int pos = MaskChainContact.CountLateralPixelsOnSidePublic(intrusionOnFlank, entry.Detection.Center, xAxis, yAxis, +1);
        int neg = MaskChainContact.CountLateralPixelsOnSidePublic(intrusionOnFlank, entry.Detection.Center, xAxis, yAxis, -1);
        if (MaskChainContact.IsDominantLateralFlankSideBlock(pos, neg, SideContactMinPixels, MaskChainContact.B1FlankStrongBlockPixelsPublic))
            return true;

        return false;
    }

    private const int SideContactMinPixels = MaskChainContact.SideContactMinPixelsForB3;

    private static double GetLargestContourArea(Mat binaryMask)
    {
        Cv2.FindContours(
            binaryMask,
            out CvPoint[][] contours,
            out _,
            RetrievalModes.External,
            ContourApproximationModes.ApproxSimple);

        return contours.Length == 0 ? 0 : contours.Max(c => Cv2.ContourArea(c));
    }

    private bool HasForeignNeighborOnSideBand(
        InstanceMaskEntry entry,
        IReadOnlyList<MaskChainContact.InstanceMaskRef> maskRefs,
        HashSet<int> claimedLabels,
        OpenCvSharp.Size frameSize,
        int dilatePx,
        double minForeignArea,
        Mat lateralProbeFromTight)
    {
        if (!MaskChainContact.TryGetBoltAxes(entry.Detection, out var yAxis, out var xAxis))
            return false;

        using var tightOwn = MaskChainContact.BuildTightB2Mask(entry.Detection, frameSize);
        using var dilatedSide = new Mat();
        using var kernel = CreateDilateKernel(dilatePx);
        Cv2.Dilate(lateralProbeFromTight, dilatedSide, kernel);

        using var neighborhoodLabels = new Mat();
        _labels.CopyTo(neighborhoodLabels, dilatedSide);
        Rect roi = GetMaskRoiRect(dilatedSide, 0);

        var seen = new HashSet<int>();
        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                int* row = (int*)neighborhoodLabels.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    int label = row[x];
                    if (label <= 0 || !seen.Add(label))
                        continue;

                    if (claimedLabels.Contains(label))
                        continue;

                    if (!TryGetComponentInfo(label, out var foreign))
                        continue;

                    if (foreign.Area < minForeignArea)
                        continue;

                    if (!MaskChainContact.IsForeignCentroidNearFlankProbe(
                            entry.Detection,
                            foreign.Centroid,
                            MaskChainContact.LateralProbeOutwardPxForB3,
                            dilatePx))
                    {
                        continue;
                    }

                    using var foreignMask = new Mat();
                    Cv2.InRange(_labels, new Scalar(label), new Scalar(label), foreignMask);
                    using var overlap = new Mat();
                    Cv2.BitwiseAnd(dilatedSide, foreignMask, overlap);
                    if (Cv2.CountNonZero(overlap) < MaskChainContact.FlankBlockMinSideBandPxForB3)
                        continue;

                    int pos = MaskChainContact.CountLateralPixelsOnSidePublic(
                        overlap, entry.Detection.Center, xAxis, yAxis, +1);
                    int neg = MaskChainContact.CountLateralPixelsOnSidePublic(
                        overlap, entry.Detection.Center, xAxis, yAxis, -1);
                    int minBlock = MaskChainContact.FlankBlockMinSideBandPxForB3;
                    int strongBlock = MaskChainContact.B1FlankStrongBlockPixelsPublic;
                    if (MaskChainContact.HasNearbyDetectedBolt(entry.Detection, maskRefs))
                    {
                        minBlock = SideContactMinPixels;
                    }

                    if (MaskChainContact.IsDominantLateralFlankSideBlock(pos, neg, minBlock, strongBlock))
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    private static Mat CreateDilateKernel(int dilatePx)
    {
        int k = Math.Max(3, dilatePx * 2 + 1);
        return Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(k, k));
    }

    private sealed record SceneComponentInfo(int Label, double Area, Point2f Centroid);

    private sealed class InstanceMaskEntry : IDisposable
    {
        public DetectionResult Detection { get; }
        public Mat Mask { get; }
        public int PrimaryLabel { get; }

        public InstanceMaskEntry(DetectionResult detection, Mat mask, int primaryLabel)
        {
            Detection = detection;
            Mask = mask;
            PrimaryLabel = primaryLabel;
        }

        public void Dispose() => Mask.Dispose();
    }

    public void Dispose()
    {
        SceneBinary.Dispose();
        _labels.Dispose();
    }
}
