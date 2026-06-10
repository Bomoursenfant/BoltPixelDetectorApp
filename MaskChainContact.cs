using OpenCvSharp;
using CvPoint = OpenCvSharp.Point;

namespace BoltPixelDetectorApp;

/// <summary>
/// Detects end-to-end mask contact (bolts in a row: tail/head touch only).
/// Such pairs stay pickable despite 360° center spacing or B3 mask touch rules.
/// </summary>
internal static class MaskChainContact
{
    // Simple per-frame dilation cache keyed by (mask pointer hash, radius)
    private static readonly Dictionary<(int maskHash, int r), Mat> _dilateCache = new();
    private static readonly Dictionary<(int maskHash, string detectionKey), Mat> _sideBandCache = new();
    private static readonly object _cacheLock = new();

    public static void DilationCacheClear()
    {
        lock (_cacheLock)
        {
            foreach (var kv in _dilateCache)
                kv.Value.Dispose();
            _dilateCache.Clear();
            foreach (var kv in _sideBandCache)
                kv.Value.Dispose();
            _sideBandCache.Clear();
        }
    }

    private const double EndCapFraction = 0.30;
    private const int LateralProbeOutwardPx = 16;
    private const int B1FlankProbeOutwardPx = 26;
    private const int B1FlankMinBlockPixels = 4;
    private const int B1FlankStrongBlockPixels = 8;
    private const int B1FusionAnnexMinPixels = 5;
    private const int B1FusionAnnexDominanceWeakMin = 6;
    private const int B1FusedShellMinPixels = 4;
    private const double B1FlankDominanceRatio = 1.5;
    private const double B1HitLateralRatio = 0.48;
    private const double B1HitEndCapRatio = 0.78;
    private const float TightMaskWidthShrink = 0.70f;
    public const int LateralProbeOutwardPxForB3 = LateralProbeOutwardPx;
    private const double MinOverlapInEndCapsRatio = 0.40;
    private const double MaxPerpendicularSpreadRatio = 0.88;
    private const double LateralAngleMinDeg = 48;
    private const double LateralAngleMaxDeg = 132;
    private const int SideContactMinPixels = 10;
    /// <summary>YOLO mask overlap on subject X' side band to block send (slightly below SideContactMinPixels).</summary>
    private const int FlankBlockMinSideBandPx = 5;
    private const int FlankBlockExpandedSidePx = 5;
    private const int FlankClusterOverlapPx = 8;
    private const int FlankB1BlockMinPx = 2;
    private const int FlankB1OneSidedMinPx = 2;
    private const int FlankProbeExtraOutwardPx = 22;
    public const int SideContactMinPixelsForB3 = SideContactMinPixels;
    public const int FlankBlockMinSideBandPxForB3 = FlankBlockMinSideBandPx;
    public const int B1FlankMinBlockPixelsPublic = B1FlankMinBlockPixels;
    public const int B1FlankStrongBlockPixelsPublic = B1FlankStrongBlockPixels;
    public const double B1FlankDominanceRatioPublic = B1FlankDominanceRatio;

    /// <summary>
    /// Head/tail touch between two bolts is allowed (e.g. #1 above head of #2).
    /// </summary>
    public static bool ShouldExemptPairForRobot(
        DetectionResult a,
        DetectionResult b,
        OpenCvSharp.Size frameSize,
        int dilatePx)
    {
        using var maskA = BuildMaskFromDetection(a, frameSize);
        using var maskB = BuildMaskFromDetection(b, frameSize);
        if (maskA is null || maskB is null)
            return false;

        if (HasSubjectFlankContactWithOther(a, b, maskA, maskB, dilatePx) ||
            HasSubjectFlankContactWithOther(b, a, maskB, maskA, dilatePx))
        {
            return false;
        }

        return IsEndToEndChainContact(a, b, dilatePx) ||
               IsMaskContactOnlyAtEndCaps(a, b, maskA, maskB, dilatePx) ||
               IsOtherAlongSubjectEndAxis(a, b) ||
               IsOtherAlongSubjectEndAxis(b, a);
    }

    /// <summary>
    /// B3: reserved — undetected B1 on flank is handled in <see cref="SceneBinaryContext"/> (unclaimed labels + same-component intrusion).
    /// </summary>
    public static bool HasUndetectedSideContactOnB1(
        DetectionResult detection,
        Mat instanceMask,
        Mat sceneBinary,
        IReadOnlyList<InstanceMaskRef> allMasks,
        int contactDilatePx,
        double minForeignArea) => false;

    public static Mat BuildOthersUnionMask(
        IReadOnlyList<InstanceMaskRef> allMasks,
        DetectionResult subject,
        OpenCvSharp.Size frameSize)
    {
        var others = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            Cv2.BitwiseOr(others, entry.Mask, others);
        }

        return others;
    }

    /// <summary>
    /// Tight B2 for B3: aligned OBB only (fusion contour can swallow undetected B1 neighbors on the flank).
    /// </summary>
    public static Mat BuildTightB2Mask(DetectionResult detection, OpenCvSharp.Size frameSize)
    {
        var mask = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        var box = detection.RotatedBox;
        if (box.Size.Width > 1.5f && box.Size.Height > 1.5f)
        {
            CvPoint[] pts;
            if (TryGetBoltAxes(detection, out var yAxis, out var xAxis))
            {
                var corners = Cv2.BoxPoints(box);
                var center = box.Center;
                pts = corners
                    .Select(p =>
                    {
                        var v = new Point2f(p.X - center.X, p.Y - center.Y);
                        float alongX = (float)(Dot(v, xAxis) * TightMaskWidthShrink);
                        float alongY = (float)Dot(v, yAxis);
                        return new CvPoint(
                            (int)Math.Round(center.X + alongX * xAxis.X + alongY * yAxis.X),
                            (int)Math.Round(center.Y + alongX * xAxis.Y + alongY * yAxis.Y));
                    })
                    .ToArray();
            }
            else
            {
                pts = Cv2.BoxPoints(box)
                    .Select(p => new CvPoint((int)Math.Round(p.X), (int)Math.Round(p.Y)))
                    .ToArray();
            }

            Cv2.FillPoly(mask, new[] { pts }, Scalar.White);
            return mask;
        }

        if (detection.MaskContour is { Length: >= 3 } contour)
        {
            Cv2.FillPoly(mask, new[] { contour }, Scalar.White);
            using var eroded = new Mat();
            using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(5, 5));
            Cv2.Erode(mask, eroded, kernel);
            if (Cv2.CountNonZero(eroded) >= 12)
            {
                mask.Dispose();
                return eroded;
            }
        }

        return mask;
    }

    /// <summary>All B2 masks dilated; subject uses tight OBB so B1 beside flank stays visible.</summary>
    public static Mat BuildExplainedB2UnionForSubject(
        IReadOnlyList<InstanceMaskRef> allMasks,
        DetectionResult subject,
        OpenCvSharp.Size frameSize,
        int contactDilatePx)
    {
        using var union = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
            {
                using var tight = BuildTightB2Mask(entry.Detection, frameSize);
                Cv2.BitwiseOr(union, tight, union);
            }
            else
            {
                // Never dispose entry.Mask — it is owned by SceneBinaryContext.
                Cv2.BitwiseOr(union, entry.Mask, union);
            }
        }

        return DilateMask(union, contactDilatePx + 2);
    }

    /// <summary>B2 union + extra dilate for bolts stacked on subject head/tail (Y'), so B1 gaps do not block #3.</summary>
    public static Mat BuildExplainedForB1FlankCheck(
        IReadOnlyList<InstanceMaskRef> allMasks,
        DetectionResult subject,
        OpenCvSharp.Size frameSize,
        int contactDilatePx)
    {
        var explained = BuildExplainedB2UnionForSubject(allMasks, subject, frameSize, contactDilatePx);
        int partnerPad = contactDilatePx + 8;
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            if (!IsOtherAlongSubjectEndAxis(subject, entry.Detection))
                continue;

            using var partnerDilated = DilateMask(entry.Mask, partnerPad);
            Cv2.BitwiseOr(explained, partnerDilated, explained);
        }

        return explained;
    }

    /// <summary>
    /// B1 on flank not covered by B2. Uses fused mask for probe position, tight OBB for "explained" (#1 + neighbor).
    /// </summary>
    public static bool HasB1ForeignOnSubjectFlank(
        DetectionResult detection,
        Mat sceneBinary,
        Mat flankProbeSourceMask,
        IReadOnlyList<InstanceMaskRef> allMasks,
        OpenCvSharp.Size frameSize,
        int contactDilatePx)
    {
        if (!TryGetBoltAxes(detection, out var yAxis, out var xAxis))
            return false;

        int searchPx = Math.Max(4, contactDilatePx / 2 + 2);

        using var explained = BuildExplainedForB1FlankCheck(allMasks, detection, frameSize, contactDilatePx);
        using var inverted = new Mat();
        Cv2.BitwiseNot(explained, inverted);
        using var unexplainedB1 = new Mat();
        Cv2.BitwiseAnd(sceneBinary, inverted, unexplainedB1);

        // Only X' side band counts — B1 on head/tail (Y') does not block (#1).
        using var sideBand = BuildSideBandMask(flankProbeSourceMask, detection);
        using var unexplainedOnFlank = new Mat();
        Cv2.BitwiseAnd(unexplainedB1, sideBand, unexplainedOnFlank);

        // #2: undetected bolt beside one flank (wide probe, few B1 pixels).
        if (HasForeignB1OnOneFlankSide(detection, flankProbeSourceMask, unexplainedB1, xAxis, yAxis))
            return true;

        // #5 / #2: undetected B1 on X' flank (must run before end-cap-only skip below).
        if (HasSceneBinaryBesideSubjectFlank(
                detection, sceneBinary, flankProbeSourceMask, unexplainedB1, xAxis, yAxis))
        {
            return true;
        }

        if (IsLateralB1FlankObstruction(detection, flankProbeSourceMask, unexplainedOnFlank, xAxis, yAxis, searchPx, +1))
            return true;

        if (IsLateralB1FlankObstruction(detection, flankProbeSourceMask, unexplainedOnFlank, xAxis, yAxis, searchPx, -1))
            return true;

        if (HasUnexplainedB1OnSubjectFlank(detection, flankProbeSourceMask, unexplainedB1, xAxis, yAxis))
            return true;

        // #2/#3: no YOLO mask touch on X' side bands — skip shell/annex/tray B1 noise.
        if (HasClearFlanksVsAllDetections(detection, flankProbeSourceMask, allMasks, contactDilatePx))
        {
            // Horizontal + clear YOLO flanks — B1 probe noise only (#5 beside #4 still blocked in pairwise).
            if (ShouldRelaxB3FlankNoiseChecks(detection, allMasks, frameSize, contactDilatePx))
                return false;

            // #4: YOLO masks miss oblique/tail neighbors — B1 still on X' beside a close detection.
            if (HasNearbyDetectedBolt(detection, allMasks, maxSpanRatio: 1.10) &&
                Cv2.CountNonZero(unexplainedOnFlank) >= FlankB1OneSidedMinPx &&
                IsB1HitBlockingSubjectFlank(detection, unexplainedOnFlank, xAxis, yAxis, minPx: FlankB1OneSidedMinPx))
            {
                return true;
            }

            return false;
        }

        // YOLO on head/tail only — skip tray noise unless B1 also on X' flank.
        if (HasCloseNeighborContactOnSubjectEndCaps(detection, allMasks, frameSize, contactDilatePx))
        {
            if (Cv2.CountNonZero(unexplainedOnFlank) >= FlankB1OneSidedMinPx &&
                IsB1HitBlockingSubjectFlank(detection, unexplainedOnFlank, xAxis, yAxis))
            {
                return true;
            }

            return false;
        }

        // Isolated: still check fused shell; skip only generic annex noise.
        if (!HasNearbyDetectedBolt(detection, allMasks))
        {
            if (HasSceneB1InFusedShellOnFlank(
                    detection, flankProbeSourceMask, sceneBinary, allMasks, frameSize, contactDilatePx, searchPx, xAxis, yAxis))
            {
                return true;
            }

            return false;
        }

        // B1 inside fused contour but outside tight B2 (#1: horizontal bolt fully merged into fusion).
        if (HasSceneB1InFusedShellOnFlank(
                detection, flankProbeSourceMask, sceneBinary, allMasks, frameSize, contactDilatePx, searchPx, xAxis, yAxis))
        {
            return true;
        }

        // Fusion wider than tight OBB on flank (#1).
        return HasFusionAnnexOnFlank(
            detection, flankProbeSourceMask, allMasks, frameSize, contactDilatePx, searchPx, xAxis, yAxis);
    }

    /// <summary>
    /// #5: nearly horizontal bolt with another detection beside on X' (#4) — skip all B1-flank blocks for this subject.
    /// </summary>
    public static bool ShouldRelaxB3FlankNoiseChecks(
        DetectionResult subject,
        IReadOnlyList<InstanceMaskRef> allMasks,
        OpenCvSharp.Size frameSize,
        int dilatePx)
    {
        _ = frameSize;
        _ = dilatePx;
        if (!IsBoltNearlyHorizontal(subject))
            return false;

        if (!TryGetBoltAxes(subject, out var yAxis, out var xAxis))
            return false;

        double span = GetApproxSpanPx(subject);
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            if (IsNeighborBesideOnFlank(subject, entry.Detection, xAxis, yAxis, span))
                return true;
        }

        return false;
    }

    private static bool IsBoltNearlyHorizontal(DetectionResult detection, double maxDegFromHorizontal = 22)
    {
        double a = Math.Abs(detection.Angle % 180.0);
        if (a > 90)
            a = 180 - a;

        return a <= maxDegFromHorizontal;
    }

    private static bool IsNeighborBesideOnFlank(
        DetectionResult subject,
        DetectionResult other,
        Point2f xAxis,
        Point2f yAxis,
        double subjectSpanPx)
    {
        var v = new Point2f(other.Center.X - subject.Center.X, other.Center.Y - subject.Center.Y);
        double alongX = Math.Abs(Dot(v, xAxis));
        double alongY = Math.Abs(Dot(v, yAxis));
        double dist = Math.Sqrt(v.X * v.X + v.Y * v.Y);
        double span = Math.Max(subjectSpanPx, GetApproxSpanPx(other));

        return alongX >= alongY * 0.40 && dist <= span * 0.85 + 18;
    }

    /// <summary>Neighbor close along subject Y' (head/tail), not a far bolt along Y' (#1).</summary>
    public static bool IsCloseEndCapStackPair(DetectionResult subject, DetectionResult other)
    {
        if (!IsOtherAlongSubjectEndAxis(subject, other))
            return false;

        if (!IsPartnerWithinEndAxisStackRange(subject, other, maxAlongYRatio: 1.22))
            return false;

        double dist = CenterDistancePx(subject, other);
        double span = Math.Max(GetApproxSpanPx(subject), GetApproxSpanPx(other));
        return dist <= span * 0.55 + 8;
    }

    public static double CenterDistancePx(DetectionResult a, DetectionResult b)
    {
        double dx = a.Center.X - b.Center.X;
        double dy = a.Center.Y - b.Center.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// No other detection's mask overlaps this subject's flank probe on the side band (X').
    /// </summary>
    public static bool HasClearFlanksVsAllDetections(
        DetectionResult subject,
        Mat maskSubject,
        IReadOnlyList<InstanceMaskRef> allMasks,
        int contactDilatePx)
    {
        double subjectSpan = GetApproxSpanPx(subject);
        double maxDist = subjectSpan * 1.15 + 24;
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            if (CenterDistancePx(subject, entry.Detection) > maxDist)
                continue;

            if (HasRealFlankSideContactWithOther(subject, entry.Detection, maskSubject, entry.Mask, contactDilatePx))
                return false;
        }

        return true;
    }

    private static int CountOverlapOnSideBand(Mat overlap, Mat sideBand)
    {
        if (Cv2.CountNonZero(sideBand) < 4)
            return 0;

        using var onSide = new Mat();
        Cv2.BitwiseAnd(overlap, sideBand, onSide);
        return Cv2.CountNonZero(onSide);
    }

    private static int CountOverlapOnSubjectEndCap(
        Mat overlap,
        Mat maskSubject,
        DetectionResult subject)
    {
        using var endCap = BuildEndCapMask(maskSubject, subject);
        return CountOverlapOnSideBand(overlap, endCap);
    }

    private static int CountExpandedSideBandOverlap(
        DetectionResult subject,
        Mat maskSubject,
        Mat maskOther,
        int dilatePx)
    {
        using var expandedSide = BuildExpandedSideBandMask(maskSubject, subject, LateralProbeOutwardPx);
        if (Cv2.CountNonZero(expandedSide) < 4)
            return 0;

        using var dilatedOther = DilateMask(maskOther, dilatePx + 2);
        using var hit = new Mat();
        Cv2.BitwiseAnd(expandedSide, dilatedOther, hit);
        return Cv2.CountNonZero(hit);
    }

    private static bool IsPointInSubjectFlankCorridor(
        DetectionResult subject,
        Point2f point,
        Point2f yAxis,
        Point2f xAxis)
    {
        double span = GetApproxSpanPx(subject);
        double flankAlongYMax = span * (0.5 - EndCapFraction);
        var v = new Point2f(point.X - subject.Center.X, point.Y - subject.Center.Y);
        double alongY = Math.Abs(Dot(v, yAxis));
        double alongX = Math.Abs(Dot(v, xAxis));
        double minAlongX = Math.Max(4, span * 0.06);
        return alongY <= flankAlongYMax + 10 &&
               alongX <= span * 0.58 + 14 &&
               alongX >= minAlongX;
    }

    /// <summary>
    /// Partner OBB reaches subject X' flank (middle along Y') — fixes #5 vs #4 when centroids align on Y'.
    /// </summary>
    private static bool IsObbIntrusionOnSubjectFlank(DetectionResult subject, DetectionResult other)
    {
        if (!TryGetBoltAxes(subject, out var yAxis, out var xAxis))
            return false;

        var box = other.RotatedBox;
        if (box.Size.Width < 2)
            return false;

        foreach (var p in box.Points())
        {
            if (IsPointInSubjectFlankCorridor(subject, p, yAxis, xAxis))
                return true;
        }

        if (IsPointInSubjectFlankCorridor(subject, other.Center, yAxis, xAxis))
            return true;

        return false;
    }

    /// <summary>True when overlap on subject side band (X') — not head/tail only (#2/#3 yes, #5+#4 flank no).</summary>
    public static bool HasRealFlankSideContactWithOther(
        DetectionResult subject,
        DetectionResult other,
        Mat maskSubject,
        Mat maskOther,
        int dilatePx)
    {
        if (!TryGetBoltAxes(subject, out var yAxis, out var xAxis))
            return false;

        double span = Math.Max(GetApproxSpanPx(subject), GetApproxSpanPx(other));
        double dist = CenterDistancePx(subject, other);
        if (dist > span * 1.12 + 22)
            return false;

        using var dilatedSubject = DilateMask(maskSubject, dilatePx);
        using var dilatedOther = DilateMask(maskOther, dilatePx);
        using var overlap = new Mat();
        Cv2.BitwiseAnd(dilatedSubject, dilatedOther, overlap);
        int totalOverlap = Cv2.CountNonZero(overlap);
        if (totalOverlap < 4)
            return false;

        using var sideBandSubject = BuildSideBandMask(maskSubject, subject);
        int subjectBandPx = CountOverlapOnSideBand(overlap, sideBandSubject);
        int subjectEndCapPx = CountOverlapOnSubjectEndCap(overlap, maskSubject, subject);
        int expandedSubjectPx = CountExpandedSideBandOverlap(subject, maskSubject, maskOther, dilatePx);
        int subjectFlankPx = Math.Max(subjectBandPx, expandedSubjectPx);

        if (subjectFlankPx < FlankBlockMinSideBandPx - 1 &&
            totalOverlap < FlankClusterOverlapPx - 2 &&
            subjectEndCapPx < 5 &&
            dist > span * 0.78 + 16)
        {
            return false;
        }

        if (subjectBandPx >= FlankBlockMinSideBandPx)
            return true;

        // #4: tail/head pile-up plus any X' bytes (oblique neighbor on flank).
        if (dist <= span * 0.92 + 20 &&
            totalOverlap >= 10 &&
            subjectEndCapPx >= 5 &&
            subjectBandPx >= 2)
        {
            return true;
        }

        if (expandedSubjectPx >= FlankBlockExpandedSidePx && dist <= span * 0.88 + 18)
            return true;

        // OBB corner in flank corridor only when mask overlap supports it (#6 vs #7 gap).
        if (dist <= span * 0.86 + 16 &&
            IsObbIntrusionOnSubjectFlank(subject, other) &&
            (subjectBandPx >= 4 || expandedSubjectPx >= FlankBlockExpandedSidePx + 1))
        {
            return true;
        }

        // Head/tail only on Y' — allow (#2 + #1 on head, flanks clear).
        if (IsPairContactMainlyOnEndCaps(subject, maskSubject, other, maskOther, dilatePx) &&
            subjectBandPx <= 2)
        {
            return false;
        }

        return false;
    }

    /// <summary>True when another YOLO detection is close enough to affect flank B1 (not isolated like #3).</summary>
    public static bool HasNearbyDetectedBolt(
        DetectionResult subject,
        IReadOnlyList<InstanceMaskRef> allMasks,
        double maxSpanRatio = 1.45)
    {
        double span = GetApproxSpanPx(subject);
        double maxDist = span * maxSpanRatio + 16;
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            if (CenterDistancePx(subject, entry.Detection) <= maxDist)
                return true;
        }

        return false;
    }

    /// <summary>Unclaimed B1 must be near the flank probe, not a far tray blob (#3).</summary>
    public static bool IsForeignCentroidNearFlankProbe(
        DetectionResult subject,
        Point2f foreignCentroid,
        int probeOutwardPx,
        int dilatePx)
    {
        if (!IsPointOnSubjectFlank(subject, foreignCentroid))
            return false;

        double span = GetApproxSpanPx(subject);
        double dx = foreignCentroid.X - subject.Center.X;
        double dy = foreignCentroid.Y - subject.Center.Y;
        double dist = Math.Sqrt(dx * dx + dy * dy);
        return dist <= span * 1.05 + probeOutwardPx + dilatePx + 12;
    }

    /// <summary>
    /// Scene B1 in (fused − tight) on side band — catches neighbors swallowed by fusion with no annex/unexplained pixels.
    /// </summary>
    private static bool HasSceneB1InFusedShellOnFlank(
        DetectionResult subject,
        Mat fusedMask,
        Mat sceneBinary,
        IReadOnlyList<InstanceMaskRef> allMasks,
        OpenCvSharp.Size frameSize,
        int contactDilatePx,
        int searchPx,
        Point2f xAxis,
        Point2f yAxis)
    {
        using var tight = BuildTightB2Mask(subject, frameSize);
        using var invertedTight = new Mat();
        Cv2.BitwiseNot(tight, invertedTight);
        using var shellRegion = new Mat();
        Cv2.BitwiseAnd(fusedMask, invertedTight, shellRegion);

        using var b1Shell = new Mat();
        Cv2.BitwiseAnd(sceneBinary, shellRegion, b1Shell);
        if (Cv2.CountNonZero(b1Shell) < 3)
            return false;

        using var sideBand = BuildSideBandMask(fusedMask, subject);
        using var b1OnFlank = new Mat();
        Cv2.BitwiseAnd(b1Shell, sideBand, b1OnFlank);
        if (Cv2.CountNonZero(b1OnFlank) < 3)
            return false;

        int pos = MeasureB1ShellOnSide(subject, tight, b1OnFlank, xAxis, yAxis, searchPx, +1);
        int neg = MeasureB1ShellOnSide(subject, tight, b1OnFlank, xAxis, yAxis, searchPx, -1);

        // #1: clear one-sided lateral bulge in fused shell — do not treat as tray noise.
        if (HasStrongOneSidedLateralFlank(pos, neg, B1FusedShellMinPixels))
            return true;

        // #3: shell toward bolt stacked close on head/tail (Y'), not flank.
        if (IsB1RegionMostlyOnCloseEndCapPartner(subject, b1OnFlank, allMasks, contactDilatePx))
            return false;

        return IsDominantLateralFlankSideBlock(pos, neg, B1FusedShellMinPixels, B1FlankStrongBlockPixels);
    }

    private static int MeasureB1ShellOnSide(
        DetectionResult subject,
        Mat tightAnchor,
        Mat b1OnFlank,
        Point2f xAxis,
        Point2f yAxis,
        int searchPx,
        int sideSign)
    {
        using var hit = BuildB1FlankSideHitMask(tightAnchor, subject, b1OnFlank, searchPx, sideSign);
        int hitPx = Cv2.CountNonZero(hit);
        if (hitPx < 2)
            return 0;

        if (IsMaskB1HitTowardEndCap(subject, hit, xAxis, yAxis))
            return 0;

        int lateral = CountLateralPixelsOnSide(hit, subject.Center, xAxis, yAxis, sideSign);
        if (lateral >= 2)
            return lateral;

        return IsMaskB1HitLateralToSubject(subject, hit, xAxis, yAxis) ? hitPx : 0;
    }

    public static bool IsDominantLateralFlankSideBlock(int pos, int neg, int minPx, int strongPx)
    {
        if (ShouldSkipSymmetricTrayFlankNoise(pos, neg, minPx))
            return false;

        if (IsOneDominantLateralFlankSide(pos, neg, minPx, strongPx))
            return true;

        return IsOneDominantLateralFlankSide(neg, pos, minPx, strongPx);
    }

    /// <summary>#1 only: fused-shell lateral bulge on one X' (not tray noise on both sides).</summary>
    private static bool HasStrongOneSidedLateralFlank(int pos, int neg, int minPx)
    {
        int best = Math.Max(pos, neg);
        int weak = Math.Min(pos, neg);
        if (best < Math.Max(minPx + 2, 6))
            return false;

        if (ShouldSkipSymmetricTrayFlankNoise(pos, neg, minPx))
            return false;

        return best >= 11 || (weak <= 3 && best >= 9) || (weak <= 4 && best >= weak * 2.5 + 3);
    }

    /// <summary>#5: weak similar B1 on both flanks (tray), not a pick block.</summary>
    private static bool ShouldSkipSymmetricTrayFlankNoise(int pos, int neg, int minPx)
    {
        if (pos < minPx || neg < minPx)
            return false;

        int best = Math.Max(pos, neg);
        int weak = Math.Min(pos, neg);
        int diff = Math.Abs(pos - neg);
        if (diff <= 2 && best <= 6)
            return true;

        // Similar counts on both X' sides (e.g. #5 near tray edge).
        if (diff <= 3 && best <= 10 && weak >= 4)
            return true;

        return diff <= 4 && best <= 12 && weak >= 5;
    }

    private static bool IsOneDominantLateralFlankSide(int hitPx, int otherSide, int minPx, int strongPx)
    {
        if (hitPx < minPx)
            return false;

        if (hitPx >= strongPx)
            return true;

        int weak = Math.Min(hitPx, otherSide);
        int best = Math.Max(hitPx, otherSide);
        if (weak >= 5 && best < weak * 2.2 + 4)
            return false;

        if (weak >= 4 && best < weak * 2.0 + 2)
            return false;

        return hitPx >= minPx && hitPx >= otherSide * B1FlankDominanceRatio + 1;
    }

    /// <summary>#3: B1 shell/annex toward bolt on head/tail (Y') — not a flank block.</summary>
    private static bool IsFlankB1HitTowardEndCapPartner(
        DetectionResult subject,
        Mat anchorMask,
        Mat b1Region,
        IReadOnlyList<InstanceMaskRef> allMasks,
        int contactDilatePx,
        int searchPx,
        int sideSign)
    {
        using var hit = BuildB1FlankSideHitMask(anchorMask, subject, b1Region, searchPx, sideSign);
        return IsB1RegionMostlyOnCloseEndCapPartner(subject, hit, allMasks, contactDilatePx);
    }

    public static bool IsB1RegionMostlyOnEndCapPartnerPublic(
        DetectionResult subject,
        Mat b1Region,
        IReadOnlyList<InstanceMaskRef> allMasks,
        int contactDilatePx) =>
        IsB1RegionMostlyOnCloseEndCapPartner(subject, b1Region, allMasks, contactDilatePx);

    /// <summary>#3: B1 toward a bolt stacked close on Y' — not a distant bolt along Y' (#1).</summary>
    private static bool IsB1RegionMostlyOnCloseEndCapPartner(
        DetectionResult subject,
        Mat b1Region,
        IReadOnlyList<InstanceMaskRef> allMasks,
        int contactDilatePx)
    {
        int hitPx = Cv2.CountNonZero(b1Region);
        if (hitPx < 3)
            return false;

        int partnerPad = contactDilatePx + 8;
        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            if (!IsOtherAlongSubjectEndAxis(subject, entry.Detection))
                continue;

            if (!IsPartnerWithinEndAxisStackRange(subject, entry.Detection))
                continue;

            using var partnerDilated = DilateMask(entry.Mask, partnerPad);
            using var onPartner = new Mat();
            Cv2.BitwiseAnd(b1Region, partnerDilated, onPartner);
            if (Cv2.CountNonZero(onPartner) * 2 >= hitPx)
                return true;
        }

        return false;
    }

    private static bool IsPartnerWithinEndAxisStackRange(
        DetectionResult subject,
        DetectionResult other,
        double maxAlongYRatio = 1.28)
    {
        if (!TryGetBoltAxes(subject, out var yAxis, out var xAxis))
            return false;

        var toOther = new Point2f(other.Center.X - subject.Center.X, other.Center.Y - subject.Center.Y);
        double alongY = Math.Abs(Dot(toOther, yAxis));
        double alongX = Math.Abs(Dot(toOther, xAxis));
        if (alongY < alongX * 0.62)
            return false;

        double span = Math.Max(GetApproxSpanPx(subject), GetApproxSpanPx(other));
        return alongY <= span * maxAlongYRatio + 10;
    }

    public static double GetApproxSpanPx(DetectionResult detection)
    {
        var size = detection.RotatedBox.Size;
        if (size.Width > 1.5f && size.Height > 1.5f)
            return Math.Max(size.Width, size.Height);

        return Math.Max(detection.BoundingBox.Width, detection.BoundingBox.Height);
    }

    private static bool HasFusionAnnexOnFlank(
        DetectionResult subject,
        Mat fusedMask,
        IReadOnlyList<InstanceMaskRef> allMasks,
        OpenCvSharp.Size frameSize,
        int contactDilatePx,
        int searchPx,
        Point2f xAxis,
        Point2f yAxis)
    {
        using var tight = BuildTightB2Mask(subject, frameSize);
        using var annex = new Mat();
        Cv2.Subtract(fusedMask, tight, annex);
        if (Cv2.CountNonZero(annex) < B1FusionAnnexMinPixels)
            return false;

        using var sideBand = BuildSideBandMask(fusedMask, subject);
        using var annexOnFlank = new Mat();
        Cv2.BitwiseAnd(annex, sideBand, annexOnFlank);

        // Probe from tight body; side band first, then full annex (#1).
        int pos = Math.Max(
            MeasureAnnexOnSide(subject, tight, annexOnFlank, xAxis, yAxis, searchPx, +1),
            MeasureAnnexOnSide(subject, tight, annex, xAxis, yAxis, searchPx, +1));
        int neg = Math.Max(
            MeasureAnnexOnSide(subject, tight, annexOnFlank, xAxis, yAxis, searchPx, -1),
            MeasureAnnexOnSide(subject, tight, annex, xAxis, yAxis, searchPx, -1));

        if (HasStrongOneSidedLateralFlank(pos, neg, B1FusionAnnexMinPixels))
            return true;

        Mat annexCheck = Cv2.CountNonZero(annexOnFlank) >= B1FusionAnnexMinPixels ? annexOnFlank : annex;
        if (IsB1RegionMostlyOnCloseEndCapPartner(subject, annexCheck, allMasks, contactDilatePx))
            return false;

        return IsDominantLateralFlankSideBlock(pos, neg, B1FusionAnnexMinPixels, B1FlankStrongBlockPixels);
    }

    private static int MeasureAnnexOnSide(
        DetectionResult subject,
        Mat tightAnchor,
        Mat annexSource,
        Point2f xAxis,
        Point2f yAxis,
        int searchPx,
        int sideSign)
    {
        using var hit = BuildB1FlankSideHitMask(tightAnchor, subject, annexSource, searchPx, sideSign);
        if (Cv2.CountNonZero(hit) < 2)
            return 0;

        if (!IsMaskB1HitLateralToSubject(subject, hit, xAxis, yAxis))
            return 0;

        if (IsMaskB1HitTowardEndCap(subject, hit, xAxis, yAxis))
            return 0;

        return Cv2.CountNonZero(hit);
    }

    /// <summary>B1 blob blocks only when on subject X' flank, not head/tail (Y') — #4 yes, #5 no.</summary>
    private static bool IsB1HitBlockingSubjectFlank(
        DetectionResult subject,
        Mat hit,
        Point2f xAxis,
        Point2f yAxis,
        int minPx = FlankB1BlockMinPx)
    {
        int px = Cv2.CountNonZero(hit);
        if (px < minPx)
            return false;

        // #2: undetected bolt on one X' side — block even with few B1 pixels.
        int pos = CountLateralPixelsOnSidePublic(hit, subject.Center, xAxis, yAxis, +1);
        int neg = CountLateralPixelsOnSidePublic(hit, subject.Center, xAxis, yAxis, -1);
        if (Math.Max(pos, neg) >= FlankB1OneSidedMinPx && Math.Min(pos, neg) < FlankB1OneSidedMinPx)
            return true;

        if (IsDominantLateralFlankSideBlock(pos, neg, FlankB1BlockMinPx, FlankB1BlockMinPx))
            return true;

        // B1 at head/tail only with very few pixels — allow send.
        if (IsMaskB1HitTowardEndCap(subject, hit, xAxis, yAxis) && px <= FlankB1OneSidedMinPx)
            return false;

        if (IsMaskB1HitLateralToSubject(subject, hit, xAxis, yAxis))
            return true;

        return TryGetHitMaskCentroid(hit, out var centroid) &&
               IsPointInSubjectFlankCorridor(subject, centroid, yAxis, xAxis);
    }

    /// <summary>Foreign B1 within lateral probe on one flank side (#2 beside undetected bolt).</summary>
    private static bool HasForeignB1OnOneFlankSide(
        DetectionResult subject,
        Mat flankProbeSourceMask,
        Mat foreignB1,
        Point2f xAxis,
        Point2f yAxis)
    {
        int outward = LateralProbeOutwardPx + FlankProbeExtraOutwardPx + 6;
        foreach (int sideSign in new[] { +1, -1 })
        {
            using var probe = BuildLateralFlankProbeOneSide(
                flankProbeSourceMask, subject, outward, sideSign);
            using var hit = new Mat();
            Cv2.BitwiseAnd(probe, foreignB1, hit);
            int px = Cv2.CountNonZero(hit);
            if (px < FlankB1OneSidedMinPx)
                continue;

            int pos = CountLateralPixelsOnSidePublic(hit, subject.Center, xAxis, yAxis, +1);
            int neg = CountLateralPixelsOnSidePublic(hit, subject.Center, xAxis, yAxis, -1);
            if (Math.Max(pos, neg) >= FlankB1OneSidedMinPx && Math.Min(pos, neg) < FlankB1OneSidedMinPx)
                return true;
        }

        return false;
    }

    /// <summary>
    /// B1 foreground beside subject X' (undetected or same-component neighbor, e.g. #5).
    /// </summary>
    private static bool HasSceneBinaryBesideSubjectFlank(
        DetectionResult subject,
        Mat sceneBinary,
        Mat flankProbeSourceMask,
        Mat foreignB1,
        Point2f xAxis,
        Point2f yAxis)
    {
        if (HasForeignB1OnOneFlankSide(subject, flankProbeSourceMask, foreignB1, xAxis, yAxis))
            return true;

        int outward = LateralProbeOutwardPx + FlankProbeExtraOutwardPx;

        foreach (int sideSign in new[] { +1, -1 })
        {
            using var probe = BuildLateralFlankProbeOneSide(
                flankProbeSourceMask, subject, outward, sideSign);
            using var sideBand = BuildSideBandMask(flankProbeSourceMask, subject);
            using var flankZone = new Mat();
            Cv2.BitwiseAnd(probe, sideBand, flankZone);
            using var dilatedZone = DilateMask(flankZone, 5);
            using var hit = new Mat();
            Cv2.BitwiseAnd(dilatedZone, foreignB1, hit);

            if (IsB1HitBlockingSubjectFlank(subject, hit, xAxis, yAxis))
                return true;
        }

        return false;
    }

    /// <summary>Scene B1 on subject X' side band (undetected neighbor beside flank, e.g. #5).</summary>
    private static bool HasUnexplainedB1OnSubjectFlank(
        DetectionResult subject,
        Mat flankProbeSourceMask,
        Mat foreignB1,
        Point2f xAxis,
        Point2f yAxis)
    {
        using var expandedSide = BuildExpandedSideBandMask(
            flankProbeSourceMask, subject, LateralProbeOutwardPx + FlankProbeExtraOutwardPx);
        using var hit = new Mat();
        Cv2.BitwiseAnd(expandedSide, foreignB1, hit);
        return IsB1HitBlockingSubjectFlank(subject, hit, xAxis, yAxis, minPx: FlankB1BlockMinPx);
    }

    /// <summary>Block only when B1 on this flank is lateral (X'), not head/tail (Y') — #1 yes, #3 no.</summary>
    private static bool IsLateralB1FlankObstruction(
        DetectionResult subject,
        Mat flankProbeSourceMask,
        Mat unexplainedB1,
        Point2f xAxis,
        Point2f yAxis,
        int searchPx,
        int sideSign)
    {
        using var hit = BuildB1FlankSideHitMask(flankProbeSourceMask, subject, unexplainedB1, searchPx, sideSign);
        int hitPx = Cv2.CountNonZero(hit);
        if (hitPx < B1FlankMinBlockPixels)
            return false;

        if (!IsMaskB1HitLateralToSubject(subject, hit, xAxis, yAxis))
            return false;

        if (IsMaskB1HitTowardEndCap(subject, hit, xAxis, yAxis))
            return false;

        int otherSide = MeasureB1FlankSideHit(
            flankProbeSourceMask, subject, unexplainedB1, xAxis, yAxis, searchPx, -sideSign);

        // One-sided undetected neighbor (#5 flank).
        if (otherSide < 3 && hitPx >= B1FlankMinBlockPixels && IsMaskB1HitLateralToSubject(subject, hit, xAxis, yAxis))
            return true;

        int weak = Math.Min(hitPx, otherSide);
        int best = Math.Max(hitPx, otherSide);
        if (weak >= 3 && best < weak * 2.2 + 2)
            return false;

        if (hitPx >= B1FlankStrongBlockPixels)
            return true;

        return hitPx >= B1FlankMinBlockPixels &&
               hitPx >= otherSide * B1FlankDominanceRatio + 2;
    }

    private static Mat BuildB1FlankSideHitMask(
        Mat flankProbeSourceMask,
        DetectionResult detection,
        Mat unexplainedB1,
        int searchPx,
        int sideSign)
    {
        using var lateralProbe = BuildLateralFlankProbeOneSide(
            flankProbeSourceMask, detection, B1FlankProbeOutwardPx, sideSign);
        using var searchZone = DilateMask(lateralProbe, searchPx);
        var hit = new Mat();
        Cv2.BitwiseAnd(searchZone, unexplainedB1, hit);
        return hit;
    }

    /// <summary>Centroid of B1 hit lies on flank (X'), not along bolt length (Y').</summary>
    public static bool IsMaskB1HitLateralToSubject(
        DetectionResult subject,
        Mat hitMask,
        Point2f xAxis,
        Point2f yAxis)
    {
        if (!TryGetHitMaskCentroid(hitMask, out var centroid))
            return true;

        var v = centroid - subject.Center;
        double alongX = Math.Abs(Dot(v, xAxis));
        double alongY = Math.Abs(Dot(v, yAxis));
        return alongX >= alongY * B1HitLateralRatio;
    }

    /// <summary>B1 hit is toward head/tail (Y') — allow pick (#3 with #2 above).</summary>
    public static bool IsMaskB1HitTowardEndCap(
        DetectionResult subject,
        Mat hitMask,
        Point2f xAxis,
        Point2f yAxis)
    {
        if (!TryGetHitMaskCentroid(hitMask, out var centroid))
            return false;

        var v = centroid - subject.Center;
        double alongX = Math.Abs(Dot(v, xAxis));
        double alongY = Math.Abs(Dot(v, yAxis));
        return alongY >= alongX * B1HitEndCapRatio;
    }

    private static bool TryGetHitMaskCentroid(Mat hitMask, out Point2f centroid)
    {
        centroid = default;
        var moments = Cv2.Moments(hitMask, binaryImage: true);
        if (moments.M00 < 1e-3)
            return false;

        centroid = new Point2f((float)(moments.M10 / moments.M00), (float)(moments.M01 / moments.M00));
        return true;
    }

    private static int MeasureB1FlankSideHit(
        Mat flankProbeSourceMask,
        DetectionResult detection,
        Mat unexplainedB1,
        Point2f xAxis,
        Point2f yAxis,
        int searchPx,
        int sideSign)
    {
        using var lateralProbe = BuildLateralFlankProbeOneSide(
            flankProbeSourceMask, detection, B1FlankProbeOutwardPx, sideSign);
        if (Cv2.CountNonZero(lateralProbe) < 2)
            return 0;

        using var searchZone = DilateMask(lateralProbe, searchPx);
        using var hit = new Mat();
        Cv2.BitwiseAnd(searchZone, unexplainedB1, hit);

        int lateral = CountLateralPixelsOnSide(hit, detection.Center, xAxis, yAxis, sideSign);
        int raw = CountPixelsOnSignedSide(hit, detection.Center, xAxis, sideSign);
        return Math.Max(lateral, raw);
    }

    private static int CountPixelsOnSignedSide(Mat mask, Point2f center, Point2f xAxis, int sideSign)
    {
        int sign = sideSign >= 0 ? 1 : -1;
        int count = 0;
        Rect roi = GetMaskRoiRect(mask, 0);
        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                byte* row = (byte*)mask.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    if (row[x] == 0)
                        continue;

                    if (Dot(new Point2f(x, y) - center, xAxis) * sign >= 0.5)
                        count++;
                }
            }
        }

        return count;
    }

    /// <summary>True when a B1 blob centroid lies on subject flank (X'), not head/tail (Y').</summary>
    public static bool IsPointOnSubjectFlank(DetectionResult subject, Point2f point, double flankRatio = 0.68)
    {
        if (!TryGetBoltAxes(subject, out var yAxis, out var xAxis))
            return true;

        var v = new Point2f(point.X - subject.Center.X, point.Y - subject.Center.Y);
        double alongX = Math.Abs(Dot(v, xAxis));
        double alongY = Math.Abs(Dot(v, yAxis));
        return alongX >= alongY * flankRatio;
    }

    /// <summary>True when the other bolt lies mainly along subject Y' (head/tail), not on the flanks (X').</summary>
    public static bool IsOtherAlongSubjectEndAxis(DetectionResult subject, DetectionResult other, double endAxisRatio = 0.62)
    {
        if (!TryGetBoltAxes(subject, out var yAxis, out var xAxis))
            return false;

        var toOther = new Point2f(other.Center.X - subject.Center.X, other.Center.Y - subject.Center.Y);
        double alongY = Math.Abs(Dot(toOther, yAxis));
        double alongX = Math.Abs(Dot(toOther, xAxis));
        return alongY >= alongX * endAxisRatio;
    }

    public static int CountLateralPixelsOnSidePublic(
        Mat mask,
        Point2f center,
        Point2f xAxis,
        Point2f yAxis,
        int sideSign) =>
        CountLateralPixelsOnSide(mask, center, xAxis, yAxis, sideSign);

    private static int CountLateralPixelsOnSide(
        Mat mask,
        Point2f center,
        Point2f xAxis,
        Point2f yAxis,
        int sideSign)
    {
        int sign = sideSign >= 0 ? 1 : -1;
        int count = 0;
        Rect roi = GetMaskRoiRect(mask, 0);
        using (LightProfiler.Measure("Mask:CountLateralPixelsOnSide"))
        {
            unsafe
            {
                for (int y = roi.Y; y < roi.Y + roi.Height; y++)
                {
                    byte* row = (byte*)mask.Ptr(y);
                    for (int x = roi.X; x < roi.X + roi.Width; x++)
                    {
                        if (row[x] == 0)
                            continue;

                        var p = new Point2f(x, y) - center;
                        if (Dot(p, xAxis) * sign < 1.0)
                            continue;

                        double alongX = Math.Abs(Dot(p, xAxis));
                        double alongY = Math.Abs(Dot(p, yAxis));
                        if (alongX >= alongY * 0.40 && alongX >= 1.5)
                            count++;
                    }
                }
            }
        }

        return count;
    }

    /// <summary>
    /// Count B1 pixels on flank (X' direction), not along Y' head/tail.
    /// </summary>
    public static int CountLateralDirectionPixels(Mat mask, Point2f center, Point2f xAxis, Point2f yAxis)
    {
        int count = 0;
        Rect roi = GetMaskRoiRect(mask, 0);
        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                byte* row = (byte*)mask.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    if (row[x] == 0)
                        continue;

                    var p = new Point2f(x, y) - center;
                    double alongX = Math.Abs(Dot(p, xAxis));
                    double alongY = Math.Abs(Dot(p, yAxis));
                    if (alongX >= alongY * 0.42 && alongX >= 1.5)
                        count++;
                }
            }
        }

        return count;
    }

    /// <summary>
    /// Head/tail (Y') alignment: e.g. #2 above #3 head, #4 above #5 along Y'.
    /// </summary>
    public static bool IsEndCapAlignedPair(
        DetectionResult a,
        DetectionResult b,
        Mat maskA,
        Mat maskB,
        int dilatePx)
    {
        if (IsMaskContactOnlyAtEndCaps(a, b, maskA, maskB, dilatePx))
            return true;

        if (!TryGetBoltAxes(a, out var yA, out _) || !TryGetBoltAxes(b, out var yB, out _))
            return false;

        var ab = new Point2f(b.Center.X - a.Center.X, b.Center.Y - a.Center.Y);
        double dist = Math.Sqrt(ab.X * ab.X + ab.Y * ab.Y);
        if (dist < 4)
            return true;

        double spanA = GetMaskSpanAlongAxis(maskA, a.Center, yA);
        double spanB = GetMaskSpanAlongAxis(maskB, b.Center, yB);
        if (dist > Math.Max(spanA, spanB) * 1.35)
            return false;

        var abDir = new Point2f((float)(ab.X / dist), (float)(ab.Y / dist));
        if (AngleBetweenDeg(abDir, yA) <= 50 || AngleBetweenDeg(abDir, yB) <= 50)
            return true;

        return AngleBetweenDeg(yA, yB) <= 45 && dist <= Math.Max(spanA, spanB) * 1.2;
    }

    private static double GetMaskSpanAlongAxis(Mat mask, Point2f center, Point2f axis)
    {
        double min = double.MaxValue, max = double.MinValue;
        for (int y = 0; y < mask.Rows; y++)
        {
            for (int x = 0; x < mask.Cols; x++)
            {
                if (mask.At<byte>(y, x) == 0)
                    continue;
                double t = Dot(new Point2f(x, y) - center, axis);
                min = Math.Min(min, t);
                max = Math.Max(max, t);
            }
        }

        return Math.Max(1.0, max - min);
    }

    /// <summary>
    /// Expand side band only along X' (flanks), not along Y'.
    /// </summary>
    public static Mat BuildLateralFlankProbeMask(Mat instanceMask, DetectionResult detection, int outwardPx)
    {
        using var plus = BuildLateralFlankProbeOneSide(instanceMask, detection, outwardPx, +1);
        using var minus = BuildLateralFlankProbeOneSide(instanceMask, detection, outwardPx, -1);
        var both = new Mat();
        Cv2.BitwiseOr(plus, minus, both);
        return both;
    }

    /// <summary>Probe outward along +X' or -X' only (one flank side).</summary>
    public static Mat BuildLateralFlankProbeOneSide(Mat instanceMask, DetectionResult detection, int outwardPx, int sideSign)
    {
        int sign = sideSign >= 0 ? 1 : -1;
        using var sideBand = LightProfiler.Measure("Mask:BuildSideBand") != null ? BuildSideBandMask(instanceMask, detection) : BuildSideBandMask(instanceMask, detection);
        if (!TryGetBoltAxes(detection, out _, out var xAxis) || Cv2.CountNonZero(sideBand) < 4)
            return sideBand.Clone();

        int steps = Math.Max(1, outwardPx);
        // Build a linear structuring element along xAxis for k=1..steps on the requested side.
        int radius = steps;
        int kSize = radius * 2 + 1;
        var kernel = new Mat(new OpenCvSharp.Size(kSize, kSize), MatType.CV_8UC1, Scalar.Black);
        int center = radius;
        for (int k = 1; k <= steps; k++)
        {
            int ox = (int)Math.Round(xAxis.X * k * sign);
            int oy = (int)Math.Round(xAxis.Y * k * sign);
            int px = center + ox;
            int py = center + oy;
            if (px >= 0 && px < kSize && py >= 0 && py < kSize)
            {
                kernel.Set<byte>(py, px, 1);
            }
        }

        var probe = new Mat();
        using (LightProfiler.Measure("Mask:BuildLateralProbe"))
        {
            Cv2.Dilate(sideBand, probe, kernel);
        }

        kernel.Dispose();
        return probe;
    }

    /// <summary>
    /// Subject's flank (X') overlaps another mask — blocks only this subject (e.g. #2 yes, #3 no when #2 touches #3's side).
    /// </summary>
    public static bool HasSubjectFlankContactWithOther(
        DetectionResult subject,
        DetectionResult other,
        Mat maskSubject,
        Mat maskOther,
        int dilatePx) =>
        HasRealFlankSideContactWithOther(subject, other, maskSubject, maskOther, dilatePx);

    /// <summary>
    /// Close YOLO neighbor touches subject mainly on head/tail, not on X' flanks (#2 + #1 on head).
    /// </summary>
    public static bool HasCloseNeighborContactOnSubjectEndCaps(
        DetectionResult subject,
        IReadOnlyList<InstanceMaskRef> allMasks,
        OpenCvSharp.Size frameSize,
        int dilatePx)
    {
        using var maskSubject = BuildMaskFromDetection(subject, frameSize);
        if (maskSubject is null)
            return false;

        foreach (var entry in allMasks)
        {
            if (IsSameDetection(subject, entry.Detection))
                continue;

            double span = Math.Max(GetApproxSpanPx(subject), GetApproxSpanPx(entry.Detection));
            if (CenterDistancePx(subject, entry.Detection) > span * 0.55 + 14)
                continue;

            if (IsPairContactMainlyOnEndCaps(subject, maskSubject, entry.Detection, entry.Mask, dilatePx))
                return true;
        }

        return false;
    }

    /// <summary>
    /// Two masks overlap mainly on head/tail (Y') of either bolt — not on subject X' flanks (#4 tail + #5 head).
    /// </summary>
    public static bool IsPairContactMainlyOnEndCaps(
        DetectionResult subject,
        Mat maskSubject,
        DetectionResult other,
        Mat maskOther,
        int dilatePx)
    {
        using var dilatedSubject = DilateMask(maskSubject, dilatePx);
        using var dilatedOther = DilateMask(maskOther, dilatePx);
        using var overlap = new Mat();
        Cv2.BitwiseAnd(dilatedSubject, dilatedOther, overlap);

        int total = Cv2.CountNonZero(overlap);
        if (total < 4)
            return false;

        using var endCapSubject = BuildEndCapMask(maskSubject, subject);
        using var endCapOther = BuildEndCapMask(maskOther, other);
        using var sideBand = BuildSideBandMask(maskSubject, subject);

        using var onSubjectEnd = new Mat();
        using var onOtherEnd = new Mat();
        Cv2.BitwiseAnd(overlap, endCapSubject, onSubjectEnd);
        Cv2.BitwiseAnd(overlap, endCapOther, onOtherEnd);

        using var onAnyEnd = new Mat();
        Cv2.BitwiseOr(onSubjectEnd, onOtherEnd, onAnyEnd);

        using var onSide = new Mat();
        Cv2.BitwiseAnd(overlap, sideBand, onSide);

        using var sideBandOther = BuildSideBandMask(maskOther, other);
        using var onOtherSide = new Mat();
        Cv2.BitwiseAnd(overlap, sideBandOther, onOtherSide);

        int endPx = Cv2.CountNonZero(onAnyEnd);
        int sidePx = Cv2.CountNonZero(onSide);
        int sidePxOther = Cv2.CountNonZero(onOtherSide);
        int flankPx = Math.Max(sidePx, sidePxOther);

        if (flankPx >= 8)
            return false;

        return endPx >= 4 && endPx >= flankPx;
    }

    private static void GetAxisExtents(Mat mask, Point2f center, Point2f axis, out double minT, out double maxT)
    {
        minT = double.MaxValue;
        maxT = double.MinValue;
        for (int y = 0; y < mask.Rows; y++)
        {
            for (int x = 0; x < mask.Cols; x++)
            {
                if (mask.At<byte>(y, x) == 0)
                    continue;

                double t = Dot(new Point2f(x, y) - center, axis);
                minT = Math.Min(minT, t);
                maxT = Math.Max(maxT, t);
            }
        }

        if (maxT < minT)
        {
            minT = -1;
            maxT = 1;
        }
    }

    private static Mat ShiftMask(Mat source, int dx, int dy)
    {
        int w = source.Width;
        int h = source.Height;
        var shifted = new Mat(new OpenCvSharp.Size(w, h), MatType.CV_8UC1, Scalar.Black);

        // compute overlapping rectangles
        int srcX = Math.Max(0, -dx);
        int srcY = Math.Max(0, -dy);
        int dstX = Math.Max(0, dx);
        int dstY = Math.Max(0, dy);
        int copyW = Math.Min(w - srcX, w - dstX);
        int copyH = Math.Min(h - srcY, h - dstY);
        if (copyW <= 0 || copyH <= 0)
            return shifted;

        var srcR = new Rect(srcX, srcY, copyW, copyH);
        var dstR = new Rect(dstX, dstY, copyW, copyH);
        using var srcROI = new Mat(source, srcR);
        using var dstROI = new Mat(shifted, dstR);
        srcROI.CopyTo(dstROI);
        return shifted;
    }

    public static bool TryGetBoltAxes(DetectionResult d, out Point2f yAxis, out Point2f xAxis)
    {
        yAxis = new Point2f(0, 1);
        xAxis = new Point2f(1, 0);

        var rawY = d.ObjectYAxis;
        double lenY = Math.Sqrt(rawY.X * rawY.X + rawY.Y * rawY.Y);
        if (lenY < 1e-3)
        {
            if (!TryGetLongAxisUnitVector(d, out yAxis))
                return false;
        }
        else
        {
            yAxis = new Point2f((float)(rawY.X / lenY), (float)(rawY.Y / lenY));
        }

        var rawX = d.ObjectXAxis;
        double lenX = Math.Sqrt(rawX.X * rawX.X + rawX.Y * rawX.Y);
        if (lenX > 1e-3)
        {
            xAxis = new Point2f((float)(rawX.X / lenX), (float)(rawX.Y / lenX));
        }
        else
        {
            xAxis = new Point2f(-yAxis.Y, yAxis.X);
        }

        double dot = Math.Abs(Dot(xAxis, yAxis));
        if (dot > 0.25)
            xAxis = new Point2f(-yAxis.Y, yAxis.X);

        return true;
    }

    /// <summary>
    /// Mask overlap exists but only on head/tail zones of either bolt (not on flanks).
    /// </summary>
    public static bool IsMaskContactOnlyAtEndCaps(
        DetectionResult a,
        DetectionResult b,
        Mat maskA,
        Mat maskB,
        int dilatePx)
    {
        using var dilatedA = DilateMask(maskA, dilatePx);
        using var dilatedB = DilateMask(maskB, dilatePx);
        using var overlap = new Mat();
        Cv2.BitwiseAnd(dilatedA, dilatedB, overlap);

        int total = Cv2.CountNonZero(overlap);
        if (total < 4)
            return false;

        using var endCapA = BuildEndCapMask(maskA, a);
        using var endCapB = BuildEndCapMask(maskB, b);
        using var endZone = new Mat();
        Cv2.BitwiseOr(endCapA, endCapB, endZone);
        using var dilatedEnd = DilateMask(endZone, dilatePx + 2);

        using var overlapOnEnds = new Mat();
        Cv2.BitwiseAnd(overlap, dilatedEnd, overlapOnEnds);
        int onEnds = Cv2.CountNonZero(overlapOnEnds);

        return (double)onEnds / total >= 0.38;
    }

    public static Mat? BuildMaskFromDetection(DetectionResult detection, OpenCvSharp.Size frameSize)
    {
        var mask = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        if (detection.MaskContour is { Length: >= 3 } contour)
        {
            Cv2.FillPoly(mask, new[] { contour }, Scalar.White);
            return mask;
        }

        if (detection.RotatedBox.Size.Width > 1 && detection.RotatedBox.Size.Height > 1)
        {
            var pts = Cv2.BoxPoints(detection.RotatedBox)
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

        mask.Dispose();
        return null;
    }

    public static Mat BuildEndCapMask(Mat instanceMask, DetectionResult detection)
    {
        var endCap = new Mat(instanceMask.Size(), MatType.CV_8UC1, Scalar.Black);
        if (!TryGetBoltAxes(detection, out var longAxis, out _))
            return endCap;

        Rect roi = GetMaskRoiRect(instanceMask, 0);
        double minT = double.MaxValue, maxT = double.MinValue;
        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                byte* maskRow = (byte*)instanceMask.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    if (maskRow[x] == 0)
                        continue;

                    double t = Dot(new Point2f(x, y) - detection.Center, longAxis);
                    minT = Math.Min(minT, t);
                    maxT = Math.Max(maxT, t);
                }
            }
        }

        if (maxT <= minT)
            return endCap;

        double span = maxT - minT;
        double endMargin = EndCapFraction * span;

        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                byte* maskRow = (byte*)instanceMask.Ptr(y);
                byte* outRow = (byte*)endCap.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    if (maskRow[x] == 0)
                        continue;

                    double t = Dot(new Point2f(x, y) - detection.Center, longAxis);
                    if (t <= minT + endMargin || t >= maxT - endMargin)
                        outRow[x] = 255;
                }
            }
        }

        return endCap;
    }

    /// <summary>Side band + outward ring so B1 neighbors beside the hull are detected.</summary>
    public static Mat BuildExpandedSideBandMask(Mat instanceMask, DetectionResult detection, int outwardPx)
    {
        using var sideBand = BuildSideBandMask(instanceMask, detection);
        if (Cv2.CountNonZero(sideBand) < 4)
            return sideBand.Clone();

        using var dilated = DilateMask(sideBand, Math.Max(2, outwardPx));
        using var eroded = new Mat();
        using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(3, 3));
        Cv2.Erode(sideBand, eroded, kernel);
        var expanded = new Mat();
        Cv2.Subtract(dilated, eroded, expanded);
        Cv2.BitwiseOr(sideBand, expanded, expanded);
        return expanded;
    }

    public readonly struct InstanceMaskRef
    {
        public DetectionResult Detection { get; }
        public Mat Mask { get; }
        public InstanceMaskRef(DetectionResult detection, Mat mask)
        {
            Detection = detection;
            Mask = mask;
        }
    }

    /// <summary>
    /// B3: two detected masks touch on side bands (not end-to-end chain).
    /// </summary>
    public static bool HasDetectedSideContact(
        DetectionResult a,
        DetectionResult b,
        Mat maskA,
        Mat maskB,
        int dilatePx) =>
        HasSubjectFlankContactWithOther(a, b, maskA, maskB, dilatePx) ||
        HasSubjectFlankContactWithOther(b, a, maskB, maskA, dilatePx);

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

    /// <summary>
    /// Side bands = instance mask minus head/tail end caps along bolt long axis.
    /// </summary>
    public static Mat BuildSideBandMask(Mat instanceMask, DetectionResult detection)
    {
        using (LightProfiler.Measure("Mask:BuildSideBand"))
        {
        }
        var cacheKey = (instanceMask.GetHashCode(), BoltSizeFilter.RobotDetectionKey(detection));
        lock (_cacheLock)
        {
            if (_sideBandCache.TryGetValue(cacheKey, out var cached) && cached is not null && !cached.IsDisposed)
                return cached.Clone();
        }

        Mat sideBand;
        if (!TryGetBoltAxes(detection, out var longAxis, out _))
        {
            sideBand = BuildSideBandFromBox(instanceMask, detection);
            lock (_cacheLock)
                _sideBandCache[cacheKey] = sideBand.Clone();
            return sideBand;
        }

        sideBand = new Mat(instanceMask.Size(), MatType.CV_8UC1, Scalar.Black);

        Rect roi = GetMaskRoiRect(instanceMask);
        double minT = double.MaxValue, maxT = double.MinValue;
        int yEnd = roi.Y + roi.Height;
        int xEnd = roi.X + roi.Width;
        unsafe
        {
            for (int y = roi.Y; y < yEnd; y++)
            {
                byte* maskRow = (byte*)instanceMask.Ptr(y);
                for (int x = roi.X; x < xEnd; x++)
                {
                    if (maskRow[x] == 0)
                        continue;

                    double t = Dot(new Point2f(x, y) - detection.Center, longAxis);
                    minT = Math.Min(minT, t);
                    maxT = Math.Max(maxT, t);
                }
            }
        }

        if (maxT <= minT)
            return BuildSideBandFromBox(instanceMask, detection);

        double span = Math.Max(1.0, maxT - minT);
        double endMargin = EndCapFraction * span;

        unsafe
        {
            for (int y = roi.Y; y < yEnd; y++)
            {
                byte* maskRow = (byte*)instanceMask.Ptr(y);
                byte* outRow = (byte*)sideBand.Ptr(y);
                for (int x = roi.X; x < xEnd; x++)
                {
                    if (maskRow[x] == 0)
                        continue;

                    double t = Dot(new Point2f(x, y) - detection.Center, longAxis);
                    bool inEndCap = t <= minT + endMargin || t >= maxT - endMargin;
                    if (!inEndCap)
                        outRow[x] = 255;
                }
            }
        }

        if (Cv2.CountNonZero(sideBand) < 8)
        {
            sideBand.Dispose();
            sideBand = BuildSideBandFromBox(instanceMask, detection);
        }

        lock (_cacheLock)
            _sideBandCache[cacheKey] = sideBand.Clone();
        return sideBand;
    }

    private static Mat BuildSideBandFromBox(Mat instanceMask, DetectionResult detection)
    {
        var sideBand = new Mat(instanceMask.Size(), MatType.CV_8UC1, Scalar.Black);
        var box = detection.RotatedBox;
        if (box.Size.Width <= 1 || box.Size.Height <= 1)
            return sideBand;

        var corners = Cv2.BoxPoints(box);
        float minX = corners.Min(p => p.X);
        float maxX = corners.Max(p => p.X);
        float minY = corners.Min(p => p.Y);
        float maxY = corners.Max(p => p.Y);
        bool tall = box.Size.Height >= box.Size.Width;
        float endFrac = (float)EndCapFraction;
        Rect roi = GetMaskRoiRect(instanceMask, 0);

        unsafe
        {
            for (int y = roi.Y; y < roi.Y + roi.Height; y++)
            {
                byte* maskRow = (byte*)instanceMask.Ptr(y);
                byte* outRow = (byte*)sideBand.Ptr(y);
                for (int x = roi.X; x < roi.X + roi.Width; x++)
                {
                    if (maskRow[x] == 0)
                        continue;

                    bool inEndCap = tall
                        ? y <= minY + (maxY - minY) * endFrac || y >= maxY - (maxY - minY) * endFrac
                        : x <= minX + (maxX - minX) * endFrac || x >= maxX - (maxX - minX) * endFrac;

                    if (!inEndCap)
                        outRow[x] = 255;
                }
            }
        }

        return sideBand;
    }

    private static bool InstanceMasksOverlapRaw(Mat maskA, Mat maskB, int dilatePx)
    {
        using var dilatedA = DilateMask(maskA, dilatePx);
        using var dilatedB = DilateMask(maskB, dilatePx);
        using var overlap = new Mat();
        Cv2.BitwiseAnd(dilatedA, dilatedB, overlap);
        return Cv2.CountNonZero(overlap) > 0;
    }

    internal static Mat DilateMaskPublic(Mat mask, int dilatePx) => DilateMask(mask, dilatePx);

    internal static bool IsSameDetectionPublic(DetectionResult a, DetectionResult b) => IsSameDetection(a, b);

    private static Mat DilateMask(Mat mask, int dilatePx)
    {
        int key = mask.GetHashCode();
        var cacheKey = (key, dilatePx);
        lock (_cacheLock)
        {
            if (_dilateCache.TryGetValue(cacheKey, out var cached) && cached != null && !cached.IsDisposed)
                return cached.Clone();
        }

        using (LightProfiler.Measure("Mask:Dilate"))
        {
            // compute ROI around mask to avoid dilating full-frame when mask is small
            Rect roi = GetMaskRoiRect(mask, dilatePx + 2);
            using var srcROI = new Mat(mask, roi);
            var dilatedROI = new Mat();
            using var kernel = Cv2.GetStructuringElement(
                MorphShapes.Ellipse,
                new OpenCvSharp.Size(Math.Max(3, dilatePx * 2 + 1), Math.Max(3, dilatePx * 2 + 1)));
            Cv2.Dilate(srcROI, dilatedROI, kernel);

            var dilated = new Mat(mask.Size(), MatType.CV_8UC1, Scalar.Black);
            using var targetROI = new Mat(dilated, roi);
            dilatedROI.CopyTo(targetROI);

            // store canonical (not cloned) in cache; return a clone to caller to preserve ownership semantics
            lock (_cacheLock)
                _dilateCache[cacheKey] = dilated;
            return dilated.Clone();
        }
    }

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

    public static bool IsEndToEndChainContact(DetectionResult a, DetectionResult b, int dilatePx)
    {
        var contourA = GetContactContour(a);
        var contourB = GetContactContour(b);
        if (contourA is null || contourB is null)
            return false;

        if (!TryBuildOverlapMask(
                contourA,
                contourB,
                a.Center,
                b.Center,
                dilatePx,
                out var overlap,
                out var axis,
                out var localCenterA,
                out var localCenterB))
            return IsRowAlignedChainWithoutOverlap(a, b);

        try
        {
            int overlapCount = Cv2.CountNonZero(overlap);
            if (overlapCount < 4)
                return IsRowAlignedChainWithoutOverlap(a, b);

            if (AreAxesAlignedWithChain(a, b, axis) &&
                IsOverlapConfinedToMutualEndCaps(
                    contourA, contourB, a.Center, b.Center, localCenterA, localCenterB, axis, overlap, overlapCount))
            {
                return true;
            }

            if (IsAlignedRowChainFallback(
                    a, b, contourA, contourB, a.Center, b.Center, localCenterA, localCenterB, axis, overlap, overlapCount))
            {
                return true;
            }

            return IsRowAlignedChainWithoutOverlap(a, b);
        }
        finally
        {
            overlap.Dispose();
        }
    }

    /// <summary>
    /// Vertical/horizontal row: aligned long axes, centers on same line, spacing in bolt-length range.
    /// </summary>
    private static bool IsRowAlignedChainWithoutOverlap(DetectionResult a, DetectionResult b)
    {
        if (!TryGetLongAxisUnitVector(a, out var axisA) || !TryGetLongAxisUnitVector(b, out var axisB))
            return false;

        var ab = new Point2f(b.Center.X - a.Center.X, b.Center.Y - a.Center.Y);
        double centerDist = Math.Sqrt(ab.X * ab.X + ab.Y * ab.Y);
        if (centerDist < 6 || centerDist > 260)
            return false;

        var abDir = new Point2f((float)(ab.X / centerDist), (float)(ab.Y / centerDist));
        if (AngleBetweenDeg(axisA, abDir) > 38 || AngleBetweenDeg(axisB, abDir) > 38)
            return false;

        if (AngleBetweenDeg(axisA, axisB) > 35)
            return false;

        double spanA = Math.Max(a.RotatedBox.Size.Width, a.RotatedBox.Size.Height);
        double spanB = Math.Max(b.RotatedBox.Size.Width, b.RotatedBox.Size.Height);
        double maxSpan = Math.Max(spanA, spanB);
        double minSpan = Math.Min(spanA, spanB);
        if (maxSpan < 8)
            return false;

        return centerDist <= maxSpan * 1.55 && centerDist >= minSpan * 0.35;
    }

    private static bool IsAlignedRowChainFallback(
        DetectionResult a,
        DetectionResult b,
        CvPoint[] contourA,
        CvPoint[] contourB,
        Point2f imageCenterA,
        Point2f imageCenterB,
        Point2f localCenterA,
        Point2f localCenterB,
        Point2f axis,
        Mat overlap,
        int overlapCount)
    {
        if (!TryGetLongAxisUnitVector(a, out var axisA) || !TryGetLongAxisUnitVector(b, out var axisB))
            return false;

        if (AngleBetweenDeg(axisA, axis) > 40 || AngleBetweenDeg(axisB, axis) > 40)
            return false;

        return IsOverlapConfinedToMutualEndCaps(
            contourA, contourB, imageCenterA, imageCenterB, localCenterA, localCenterB, axis, overlap, overlapCount);
    }

    /// <summary>
    /// Centers are close but contact is lateral (e.g. #3 beside #4), not head-to-tail chain.
    /// </summary>
    public static bool IsLateralClosePair(
        DetectionResult a,
        DetectionResult b,
        VisionSettings settings,
        int dilatePx) =>
        RobotSafetyFilter.IsNeighborCenterTooClose(a, b, settings) &&
        !IsEndToEndChainContact(a, b, dilatePx);

    /// <summary>
    /// Chain member (#2–#4 row): ignore 360° block from lateral neighbor (#3).
    /// </summary>
    public static bool ShouldIgnoreLateralSpacingBlock(
        DetectionResult subject,
        DetectionResult neighbor,
        IReadOnlyList<DetectionResult> allDetections,
        VisionSettings settings,
        int dilatePx,
        OpenCvSharp.Size? frameSize = null)
    {
        if (!RobotSafetyFilter.IsNeighborCenterTooClose(subject, neighbor, settings))
            return false;

        if (frameSize is not null && ShouldExemptPairForRobot(subject, neighbor, frameSize.Value, dilatePx))
            return true;

        if (IsEndToEndChainContact(subject, neighbor, dilatePx))
            return true;

        if (!IsLateralClosePair(subject, neighbor, settings, dilatePx))
            return false;

        return HasEndToEndChainPartner(subject, allDetections, dilatePx, frameSize);
    }

    /// <summary>
    /// Foreign B1 pixels that lie only on chain end-caps toward a row partner are not blocking.
    /// </summary>
    public static int CountForeignPixelsAtChainEndCaps(
        DetectionResult subject,
        Mat foreignOnlyMask,
        IReadOnlyList<DetectionResult> allDetections,
        int dilatePx)
    {
        if (!TryGetLongAxisUnitVector(subject, out var longAxis))
            return 0;

        var contour = GetContactContour(subject);
        if (contour is null)
            return 0;

        ProjectContour(contour, subject.Center, longAxis, out double minT, out double maxT);
        double span = Math.Max(1.0, maxT - minT);
        int exempt = 0;

        foreach (var partner in allDetections)
        {
            if (IsSameDetection(subject, partner))
                continue;

            if (!IsEndToEndChainContact(subject, partner, dilatePx))
                continue;

            bool partnerIsPositive = Dot(partner.Center - subject.Center, longAxis) >= 0;
            double capMin = partnerIsPositive ? maxT - EndCapFraction * span : minT;
            double capMax = partnerIsPositive ? maxT : minT + EndCapFraction * span;

            for (int y = 0; y < foreignOnlyMask.Rows; y++)
            {
                for (int x = 0; x < foreignOnlyMask.Cols; x++)
                {
                    if (foreignOnlyMask.At<byte>(y, x) == 0)
                        continue;

                    var p = new Point2f(x, y);
                    double t = Dot(p - subject.Center, longAxis);
                    if (t >= Math.Min(capMin, capMax) && t <= Math.Max(capMin, capMax))
                        exempt++;
                }
            }
        }

        return exempt;
    }

    public static bool HasEndToEndChainPartner(
        DetectionResult subject,
        IReadOnlyList<DetectionResult> allDetections,
        int dilatePx,
        OpenCvSharp.Size? frameSize = null)
    {
        foreach (var other in allDetections)
        {
            if (IsSameDetection(subject, other))
                continue;

            if (IsEndToEndChainContact(subject, other, dilatePx))
                return true;

            if (frameSize is not null && ShouldExemptPairForRobot(subject, other, frameSize.Value, dilatePx))
                return true;
        }

        return false;
    }

    /// <summary>
    /// Foreign B1 blob blocks only when it touches the bolt body/sides, not chain end caps.
    /// </summary>
    public static bool IsForeignContactBlockingPick(
        DetectionResult detection,
        Point2f foreignCentroid,
        Mat foreignComponentMask,
        Mat instanceMask,
        int contactDilatePx)
    {
        var contour = GetContactContour(detection);
        if (contour is null)
            return true;

        if (!TryGetLongAxisUnitVector(detection, out var longAxis))
            return true;

        var toForeign = new Point2f(
            foreignCentroid.X - detection.Center.X,
            foreignCentroid.Y - detection.Center.Y);
        double dist = Math.Sqrt(toForeign.X * toForeign.X + toForeign.Y * toForeign.Y);
        if (dist < 1e-3)
            return true;

        var toForeignDir = new Point2f((float)(toForeign.X / dist), (float)(toForeign.Y / dist));
        double angleDeg = AngleBetweenDeg(longAxis, toForeignDir);
        bool foreignIsLateral = angleDeg >= LateralAngleMinDeg && angleDeg <= LateralAngleMaxDeg;

        using var dilated = new Mat();
        using var kernel = Cv2.GetStructuringElement(
            MorphShapes.Ellipse,
            new OpenCvSharp.Size(Math.Max(3, contactDilatePx * 2 + 1), Math.Max(3, contactDilatePx * 2 + 1)));
        Cv2.Dilate(instanceMask, dilated, kernel);

        using var overlap = new Mat();
        Cv2.BitwiseAnd(dilated, foreignComponentMask, overlap);
        int overlapCount = Cv2.CountNonZero(overlap);
        if (overlapCount < 3)
            return foreignIsLateral;

        return IsForeignOverlapOnBlockingRegion(
            contour,
            detection.Center,
            longAxis,
            overlap,
            overlapCount,
            foreignIsLateral);
    }

    public static bool IsLinearEndToEndChainGroup(IReadOnlyList<DetectionResult> detections, int dilatePx)
    {
        if (detections.Count < 2)
            return false;

        int n = detections.Count;
        var adjacency = new List<int>[n];
        for (int i = 0; i < n; i++)
            adjacency[i] = new List<int>();

        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (!IsEndToEndChainContact(detections[i], detections[j], dilatePx))
                    continue;

                adjacency[i].Add(j);
                adjacency[j].Add(i);
            }
        }

        // Simple open chain: connected, max degree 2, at most two nodes with degree 1.
        var visited = new bool[n];
        int components = 0;
        int degreeOne = 0;
        int degreeTwoPlus = 0;

        for (int start = 0; start < n; start++)
        {
            if (visited[start])
                continue;

            components++;
            var queue = new Queue<int>();
            queue.Enqueue(start);
            visited[start] = true;
            int nodes = 0;

            while (queue.Count > 0)
            {
                int u = queue.Dequeue();
                nodes++;
                int deg = adjacency[u].Count;
                if (deg == 1) degreeOne++;
                else if (deg > 2) return false;

                if (deg >= 2) degreeTwoPlus++;

                foreach (int v in adjacency[u])
                {
                    if (visited[v])
                        continue;
                    visited[v] = true;
                    queue.Enqueue(v);
                }
            }

            if (nodes >= 2 && degreeOne > 2)
                return false;
        }

        return components == 1 && degreeTwoPlus <= 1;
    }

    private static OpenCvSharp.Point[]? GetContactContour(DetectionResult d)
    {
        if (d.MaskContour is { Length: >= 3 } contour)
            return contour;

        if (d.RotatedBox.Size.Width <= 1 || d.RotatedBox.Size.Height <= 1)
            return null;

        var boxPoints = Cv2.BoxPoints(d.RotatedBox);
        return boxPoints
            .Select(p => new CvPoint((int)Math.Round(p.X), (int)Math.Round(p.Y)))
            .ToArray();
    }

    private static bool TryBuildOverlapMask(
        CvPoint[] contourA,
        CvPoint[] contourB,
        Point2f centerA,
        Point2f centerB,
        int dilatePx,
        out Mat overlap,
        out Point2f axis,
        out Point2f localCenterA,
        out Point2f localCenterB)
    {
        overlap = new Mat();
        axis = new Point2f(1, 0);
        localCenterA = centerA;
        localCenterB = centerB;

        var bounds = ComputeBounds(contourA, contourB);
        int pad = Math.Max(4, dilatePx + 2);
        int width = Math.Max(1, bounds.Right - bounds.Left + pad * 2);
        int height = Math.Max(1, bounds.Bottom - bounds.Top + pad * 2);
        var offset = new CvPoint(bounds.Left - pad, bounds.Top - pad);

        using var maskA = RenderMask(contourA, offset, width, height);
        using var maskB = RenderMask(contourB, offset, width, height);
        using var dilatedA = new Mat();
        using var dilatedB = new Mat();
        using var kernel = Cv2.GetStructuringElement(
            MorphShapes.Ellipse,
            new OpenCvSharp.Size(Math.Max(3, dilatePx * 2 + 1), Math.Max(3, dilatePx * 2 + 1)));

        Cv2.Dilate(maskA, dilatedA, kernel);
        Cv2.Dilate(maskB, dilatedB, kernel);
        Cv2.BitwiseAnd(dilatedA, dilatedB, overlap);

        localCenterA = new Point2f(centerA.X - offset.X, centerA.Y - offset.Y);
        localCenterB = new Point2f(centerB.X - offset.X, centerB.Y - offset.Y);
        var dir = new Point2f(localCenterB.X - localCenterA.X, localCenterB.Y - localCenterA.Y);
        double len = Math.Sqrt(dir.X * dir.X + dir.Y * dir.Y);
        if (len < 1e-3)
            return Cv2.CountNonZero(overlap) > 0;

        axis = new Point2f((float)(dir.X / len), (float)(dir.Y / len));
        return Cv2.CountNonZero(overlap) > 0;
    }

    private static bool IsSameDetection(DetectionResult a, DetectionResult b)
    {
        if (a.Id > 0 && b.Id > 0)
            return a.Id == b.Id;

        double dx = a.Center.X - b.Center.X;
        double dy = a.Center.Y - b.Center.Y;
        return dx * dx + dy * dy < 0.25;
    }

    private static bool IsOverlapConfinedToMutualEndCaps(
        CvPoint[] contourA,
        CvPoint[] contourB,
        Point2f imageCenterA,
        Point2f imageCenterB,
        Point2f localCenterA,
        Point2f localCenterB,
        Point2f axis,
        Mat overlap,
        int overlapCount)
    {
        ProjectContour(contourA, imageCenterA, axis, out double minA, out double maxA);
        ProjectContour(contourB, imageCenterB, axis, out double minB, out double maxB);

        double spanA = Math.Max(1.0, maxA - minA);
        double spanB = Math.Max(1.0, maxB - minB);

        bool bIsPositiveAlongAxis = Dot(imageCenterB - imageCenterA, axis) >= 0;
        double aCapTowardB = bIsPositiveAlongAxis ? maxA : minA;
        double bCapTowardA = bIsPositiveAlongAxis ? minB : maxB;

        double aBandMin = bIsPositiveAlongAxis ? maxA - EndCapFraction * spanA : minA;
        double aBandMax = bIsPositiveAlongAxis ? maxA : minA + EndCapFraction * spanA;
        double bBandMin = bIsPositiveAlongAxis ? minB : maxB - EndCapFraction * spanB;
        double bBandMax = bIsPositiveAlongAxis ? minB + EndCapFraction * spanB : maxB;

        var perp = new Point2f(-axis.Y, axis.X);
        int inEndCaps = 0;
        var alongVals = new List<double>(overlapCount);
        var perpVals = new List<double>(overlapCount);

        for (int y = 0; y < overlap.Rows; y++)
        {
            for (int x = 0; x < overlap.Cols; x++)
            {
                if (overlap.At<byte>(y, x) == 0)
                    continue;

                var p = new Point2f(x, y);
                double tA = Dot(p - localCenterA, axis);
                double tB = Dot(p - localCenterB, axis);
                alongVals.Add((tA + tB) * 0.5);
                perpVals.Add(Dot(p - localCenterA, perp));

                bool inAEnd = tA >= Math.Min(aBandMin, aBandMax) && tA <= Math.Max(aBandMin, aBandMax);
                bool inBEnd = tB >= Math.Min(bBandMin, bBandMax) && tB <= Math.Max(bBandMin, bBandMax);
                if (inAEnd && inBEnd)
                    inEndCaps++;
            }
        }

        if (alongVals.Count == 0)
            return false;

        double alongSpread = StdDev(alongVals);
        double perpSpread = StdDev(perpVals);
        if (perpSpread > alongSpread * MaxPerpendicularSpreadRatio)
            return false;

        return (double)inEndCaps / overlapCount >= MinOverlapInEndCapsRatio;
    }

    private static bool AreAxesAlignedWithChain(DetectionResult a, DetectionResult b, Point2f chainAxis)
    {
        if (!TryGetLongAxisUnitVector(a, out var axisA) || !TryGetLongAxisUnitVector(b, out var axisB))
            return true;

        double chainAngle = Math.Atan2(chainAxis.Y, chainAxis.X) * 180.0 / Math.PI;
        double axisADeg = Math.Atan2(axisA.Y, axisA.X) * 180.0 / Math.PI;
        double axisBDeg = Math.Atan2(axisB.Y, axisB.X) * 180.0 / Math.PI;
        if (!IsAngleAligned(axisADeg, chainAngle, 55) || !IsAngleAligned(axisBDeg, chainAngle, 55))
            return false;

        return IsAngleAligned(axisADeg, axisBDeg, 45);
    }

    private static bool TryGetLongAxisUnitVector(DetectionResult d, out Point2f axis)
    {
        axis = new Point2f(0, 1);
        var yAxis = d.ObjectYAxis;
        double len = Math.Sqrt(yAxis.X * yAxis.X + yAxis.Y * yAxis.Y);
        if (len > 1e-3)
        {
            axis = new Point2f((float)(yAxis.X / len), (float)(yAxis.Y / len));
            return true;
        }

        var size = d.RotatedBox.Size;
        if (size.Width <= 1 || size.Height <= 1)
            return false;

        double angleDeg = size.Width >= size.Height ? d.RotatedBox.Angle : d.RotatedBox.Angle + 90.0;
        double rad = angleDeg * Math.PI / 180.0;
        axis = new Point2f((float)Math.Cos(rad), (float)Math.Sin(rad));
        return true;
    }

    private static bool IsForeignOverlapOnBlockingRegion(
        CvPoint[] contour,
        Point2f center,
        Point2f longAxis,
        Mat overlap,
        int overlapCount,
        bool foreignIsLateral)
    {
        var perp = new Point2f(-longAxis.Y, longAxis.X);
        ProjectContour(contour, center, longAxis, out double minT, out double maxT);
        double span = Math.Max(1.0, maxT - minT);

        int onSide = 0;
        int onEndCap = 0;
        int nearCenter = 0;

        for (int y = 0; y < overlap.Rows; y++)
        {
            for (int x = 0; x < overlap.Cols; x++)
            {
                if (overlap.At<byte>(y, x) == 0)
                    continue;

                var p = new Point2f(x, y);
                double t = Dot(p - center, longAxis);
                double lateral = Math.Abs(Dot(p - center, perp));

                bool inEndCap = t <= minT + EndCapFraction * span || t >= maxT - EndCapFraction * span;
                bool inSideBand = lateral >= span * 0.22;
                bool nearPickCenter = lateral <= span * 0.20 &&
                    t >= minT + span * 0.25 && t <= maxT - span * 0.25;

                if (inEndCap)
                    onEndCap++;
                if (inSideBand)
                    onSide++;
                if (nearPickCenter)
                    nearCenter++;
            }
        }

        if (nearCenter > overlapCount * 0.12)
            return true;

        if (onSide > overlapCount * 0.22)
            return true;

        if (foreignIsLateral && onSide + nearCenter > overlapCount * 0.12)
            return true;

        if (foreignIsLateral)
            return onSide > 0 || nearCenter > 0;

        return onEndCap < overlapCount * 0.40;
    }

    private static double AngleBetweenDeg(Point2f a, Point2f b)
    {
        double dot = Math.Abs(Dot(a, b));
        dot = Math.Clamp(dot, -1.0, 1.0);
        return Math.Acos(dot) * 180.0 / Math.PI;
    }

    private static bool IsAngleAligned(double angleA, double angleB, double maxDeltaDeg)
    {
        double delta = Math.Abs(NormalizeAngle(angleA - angleB));
        if (delta > 90)
            delta = 180 - delta;
        return delta <= maxDeltaDeg;
    }

    private static double NormalizeAngle(double angle)
    {
        angle %= 180.0;
        if (angle < 0) angle += 180.0;
        return angle;
    }

    private static void ProjectContour(CvPoint[] contour, Point2f origin, Point2f axis, out double min, out double max)
    {
        min = double.MaxValue;
        max = double.MinValue;
        foreach (var p in contour)
        {
            var pt = new Point2f(p.X, p.Y);
            double t = Dot(pt - origin, axis);
            min = Math.Min(min, t);
            max = Math.Max(max, t);
        }
    }

    private static Mat RenderMask(CvPoint[] contour, CvPoint offset, int width, int height)
    {
        var shifted = contour
            .Select(p => new CvPoint(p.X - offset.X, p.Y - offset.Y))
            .ToArray();
        var mask = new Mat(height, width, MatType.CV_8UC1, Scalar.Black);
        Cv2.FillPoly(mask, new[] { shifted }, Scalar.White);
        return mask;
    }

    private static Rect ComputeBounds(CvPoint[] contourA, CvPoint[] contourB)
    {
        int minX = int.MaxValue, minY = int.MaxValue, maxX = int.MinValue, maxY = int.MinValue;
        foreach (var p in contourA.Concat(contourB))
        {
            minX = Math.Min(minX, p.X);
            minY = Math.Min(minY, p.Y);
            maxX = Math.Max(maxX, p.X);
            maxY = Math.Max(maxY, p.Y);
        }

        return new Rect(minX, minY, Math.Max(1, maxX - minX + 1), Math.Max(1, maxY - minY + 1));
    }

    private static double Dot(Point2f a, Point2f b) => a.X * b.X + a.Y * b.Y;

    private static double StdDev(IReadOnlyList<double> values)
    {
        if (values.Count == 0)
            return 0;
        double mean = values.Average();
        double var = values.Sum(v => (v - mean) * (v - mean)) / values.Count;
        return Math.Sqrt(var);
    }
}
