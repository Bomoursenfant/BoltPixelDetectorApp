using OpenCvSharp;
using CvPoint = OpenCvSharp.Point;
using CvSize = OpenCvSharp.Size;

namespace BoltPixelDetectorApp;

public sealed class BoltDetector
{
    private const int TemplateAngleSearchDegrees = 5;
    private const int PreprocessMedianBlurKsize = 31;
    private const int PreprocessTophatKsize = 15;
    private const int PreprocessBilateralDiameter = 9;
    private const double PreprocessBilateralSigmaColor = 75;
    private const double PreprocessBilateralSigmaSpace = 75;
    private const double PreprocessFlatWeight = 0.7;
    private const double PreprocessTophatWeight = 0.3;
    private const double PreprocessUnsharpSigma = 1.2;
    private const double PreprocessUnsharpAmount = 0.6;
    private const double PreprocessClaheClipLimit = 2.0;
    private const int PreprocessClaheTileGrid = 8;

    private static double Clamp01(double value) => Math.Max(0.0, Math.Min(1.0, value));

    private static double NormalizeDegrees(double angle)
    {
        angle %= 360.0;
        return angle < 0 ? angle + 360.0 : angle;
    }

    private static (Point2f center, Point2f yAxis) ComputePcaCenterAndYAxis(CvPoint[] contour)
    {
        int n = contour.Length;
        if (n == 0)
            return (new Point2f(0, 0), new Point2f(1, 0));

        double meanX = 0;
        double meanY = 0;
        foreach (var point in contour)
        {
            meanX += point.X;
            meanY += point.Y;
        }
        meanX /= n;
        meanY /= n;

        double sxx = 0;
        double syy = 0;
        double sxy = 0;
        foreach (var point in contour)
        {
            double dx = point.X - meanX;
            double dy = point.Y - meanY;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }
        sxx /= n;
        syy /= n;
        sxy /= n;

        double trace = sxx + syy;
        double determinant = sxx * syy - sxy * sxy;
        double root = Math.Sqrt(Math.Max(0.0, trace * trace / 4.0 - determinant));
        double largestEigenvalue = trace / 2.0 + root;

        double vx;
        double vy;
        if (Math.Abs(sxy) > 1e-12 || Math.Abs(largestEigenvalue - sxx) > 1e-12)
        {
            vx = sxy;
            vy = largestEigenvalue - sxx;
        }
        else
        {
            vx = 1;
            vy = 0;
        }

        double norm = Math.Sqrt(vx * vx + vy * vy);
        if (norm < 1e-12)
        {
            vx = 1;
            vy = 0;
            norm = 1;
        }

        vx /= norm;
        vy /= norm;

        // Keep Y' deterministic: prefer the direction pointing to global +Y (image right).
        if (vx < 0)
        {
            vx = -vx;
            vy = -vy;
        }

        return (new Point2f((float)meanX, (float)meanY), new Point2f((float)vx, (float)vy));
    }

    private static double CalculateClockwiseAngleFromGlobalY(Point2f yAxisImage)
    {
        // Global Y is the 3 o'clock direction. In image coordinates, clockwise rotation is +Y.
        return NormalizeDegrees(Math.Atan2(yAxisImage.Y, yAxisImage.X) * 180.0 / Math.PI);
    }

    private static (Point2f xAxis, Point2f yAxis) GetAxesFromClockwiseAngle(double angle)
    {
        double radians = angle * Math.PI / 180.0;
        var yAxis = new Point2f((float)Math.Cos(radians), (float)Math.Sin(radians));
        var xAxis = new Point2f(yAxis.Y, -yAxis.X);
        if (xAxis.Y > 0)
            xAxis = new Point2f(-xAxis.X, -xAxis.Y);
        return (xAxis, yAxis);
    }

    private static Point2f NormalizeVector(Point2f vector)
    {
        double length = Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
        if (length < 1e-9) return new Point2f(1, 0);
        return new Point2f((float)(vector.X / length), (float)(vector.Y / length));
    }

    private static (Point2f xAxis, Point2f yAxis) GetRotatedBoxAxes(RotatedRect rotatedBox)
    {
        var points = rotatedBox.Points();
        var edge01 = new Point2f(points[1].X - points[0].X, points[1].Y - points[0].Y);
        var edge12 = new Point2f(points[2].X - points[1].X, points[2].Y - points[1].Y);
        double length01 = edge01.X * edge01.X + edge01.Y * edge01.Y;
        double length12 = edge12.X * edge12.X + edge12.Y * edge12.Y;

        var yAxis = NormalizeVector(length01 >= length12 ? edge01 : edge12);
        if (yAxis.X < 0)
            yAxis = new Point2f(-yAxis.X, -yAxis.Y);

        // X' is 90 degrees counterclockwise from Y', so positive X' points toward the 12 o'clock side.
        var xAxis = new Point2f(yAxis.Y, -yAxis.X);
        if (xAxis.Y > 0)
            xAxis = new Point2f(-xAxis.X, -xAxis.Y);

        return (xAxis, yAxis);
    }

    private static Rect ClampRect(Rect rect, OpenCvSharp.Size size)
    {
        int x = Math.Clamp(rect.X, 0, Math.Max(0, size.Width - 1));
        int y = Math.Clamp(rect.Y, 0, Math.Max(0, size.Height - 1));
        int right = Math.Clamp(rect.X + rect.Width, x + 1, size.Width);
        int bottom = Math.Clamp(rect.Y + rect.Height, y + 1, size.Height);
        return new Rect(x, y, Math.Max(1, right - x), Math.Max(1, bottom - y));
    }

    private static Rect InflateRect(Rect rect, int margin, OpenCvSharp.Size size)
    {
        return ClampRect(new Rect(
            rect.X - margin,
            rect.Y - margin,
            rect.Width + margin * 2,
            rect.Height + margin * 2), size);
    }

    private static Mat RotateTemplate(Mat templateImage, double angle)
    {
        if (Math.Abs(angle) < 1e-9)
            return templateImage.Clone();

        var center = new Point2f((templateImage.Width - 1) / 2f, (templateImage.Height - 1) / 2f);
        using var rotation = Cv2.GetRotationMatrix2D(center, angle, 1.0);
        double cos = Math.Abs(rotation.At<double>(0, 0));
        double sin = Math.Abs(rotation.At<double>(0, 1));
        int newWidth = Math.Max(1, (int)Math.Round(templateImage.Height * sin + templateImage.Width * cos));
        int newHeight = Math.Max(1, (int)Math.Round(templateImage.Height * cos + templateImage.Width * sin));

        rotation.Set(0, 2, rotation.At<double>(0, 2) + newWidth / 2.0 - center.X);
        rotation.Set(1, 2, rotation.At<double>(1, 2) + newHeight / 2.0 - center.Y);

        var rotated = new Mat();
        Cv2.WarpAffine(
            templateImage,
            rotated,
            rotation,
            new CvSize(newWidth, newHeight),
            InterpolationFlags.Linear,
            BorderTypes.Constant,
            Scalar.Black);
        return rotated;
    }

    private static double MatchPattern(Mat sourceImage, Mat templateImage, Rect boundingBox, double angle)
    {
        using var rotatedTemplate = RotateTemplate(templateImage, angle);
        if (rotatedTemplate.Empty()) return double.NegativeInfinity;

        int margin = Math.Max(12, Math.Max(rotatedTemplate.Width, rotatedTemplate.Height) / 2);
        var paddedRoiRect = InflateRect(boundingBox, margin, sourceImage.Size());
        using var paddedRoi = new Mat(sourceImage, paddedRoiRect);
        if (paddedRoi.Width < rotatedTemplate.Width || paddedRoi.Height < rotatedTemplate.Height)
            return double.NegativeInfinity;

        using var correlation = new Mat();
        Cv2.MatchTemplate(paddedRoi, rotatedTemplate, correlation, TemplateMatchModes.CCoeffNormed);
        Cv2.MinMaxLoc(correlation, out _, out double maxValue, out _, out _);
        return double.IsNaN(maxValue) ? double.NegativeInfinity : maxValue;
    }

    private static (double angle, double score) RefineAngleByTemplateMatching(Mat grayImage, Rect boundingBox, double angle)
    {
        var templateRect = ClampRect(boundingBox, grayImage.Size());
        if (templateRect.Width < 2 || templateRect.Height < 2)
            return (angle, 0.0);

        using var templateImage = new Mat(grayImage, templateRect).Clone();
        double firstScore = MatchPattern(grayImage, templateImage, templateRect, 0);
        double bestScore = firstScore;
        double bestAngle = angle;

        for (int delta = 1; delta <= TemplateAngleSearchDegrees; delta++)
        {
            double minusScore = MatchPattern(grayImage, templateImage, templateRect, -delta);
            if (minusScore > bestScore)
            {
                bestScore = minusScore;
                bestAngle = NormalizeDegrees(angle - delta);
            }

            double plusScore = MatchPattern(grayImage, templateImage, templateRect, delta);
            if (plusScore > bestScore)
            {
                bestScore = plusScore;
                bestAngle = NormalizeDegrees(angle + delta);
            }
        }

        if (bestScore < firstScore)
            bestAngle = angle;

        return (bestAngle, bestScore);
    }

    private static Mat PreprocessForBinary(Mat frame)
    {
        using var gray = new Mat();
        Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);

        using var denoised = new Mat();
        Cv2.BilateralFilter(gray, denoised, PreprocessBilateralDiameter, PreprocessBilateralSigmaColor, PreprocessBilateralSigmaSpace);

        using var background = new Mat();
        Cv2.MedianBlur(denoised, background, PreprocessMedianBlurKsize);

        using var flat = new Mat();
        Cv2.Subtract(denoised, background, flat);
        Cv2.Normalize(flat, flat, 0, 255, NormTypes.MinMax);

        using var kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new CvSize(PreprocessTophatKsize, PreprocessTophatKsize));
        using var tophat = new Mat();
        Cv2.MorphologyEx(denoised, tophat, MorphTypes.TopHat, kernel);

        using var enhanced = new Mat();
        Cv2.AddWeighted(flat, PreprocessFlatWeight, tophat, PreprocessTophatWeight, 0, enhanced);

        using var clahe = Cv2.CreateCLAHE(PreprocessClaheClipLimit, new CvSize(PreprocessClaheTileGrid, PreprocessClaheTileGrid));
        using var claheOut = new Mat();
        clahe.Apply(enhanced, claheOut);

        using var blurred = new Mat();
        Cv2.GaussianBlur(claheOut, blurred, new CvSize(0, 0), PreprocessUnsharpSigma);

        using var sharpened = new Mat();
        Cv2.AddWeighted(claheOut, 1.0 + PreprocessUnsharpAmount, blurred, -PreprocessUnsharpAmount, 0, sharpened);
        Cv2.Normalize(sharpened, sharpened, 0, 255, NormTypes.MinMax);

        return sharpened.Clone();
    }

    public List<DetectionResult> Detect(
        Mat frame,
        int threshold,
        double minArea,
        double maxArea,
        double minCircularity,
        double minConfidence,
        bool invert,
        out Mat mask)
    {
        return Detect(frame, threshold, minArea, maxArea, minCircularity, minConfidence, invert, detectionRoi: null, out mask);
    }

    /// <summary>
    /// Same binary image used for contour detection (Otsu/fixed threshold, morphology, optional ROI copy).
    /// Used together with YOLO to intersect classical segmentation with neural mask for fused geometry.
    /// </summary>
    public Mat BuildBinaryMask(Mat frame, int threshold, bool invert, OpenCvSharp.Rect? detectionRoi)
    {
        using var preprocessed = PreprocessForBinary(frame);
        using var binary = new Mat();
        using var cleaned = new Mat();
        using var roiFiltered = new Mat();

        var thresholdType = invert ? ThresholdTypes.BinaryInv : ThresholdTypes.Binary;
        if (threshold <= 0)
        {
            Cv2.Threshold(preprocessed, binary, 0, 255, thresholdType | ThresholdTypes.Otsu);
        }
        else
        {
            Cv2.Threshold(preprocessed, binary, threshold, 255, thresholdType);
        }

        using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new CvSize(5, 5));
        Cv2.MorphologyEx(binary, cleaned, MorphTypes.Open, kernel, iterations: 1);
        Cv2.MorphologyEx(cleaned, cleaned, MorphTypes.Close, kernel, iterations: 2);

        Mat contourSource = cleaned;
        if (detectionRoi.HasValue)
        {
            roiFiltered.Create(cleaned.Size(), MatType.CV_8UC1);
            roiFiltered.SetTo(Scalar.Black);
            using var sourceRoi = new Mat(cleaned, detectionRoi.Value);
            using var destinationRoi = new Mat(roiFiltered, detectionRoi.Value);
            sourceRoi.CopyTo(destinationRoi);
            contourSource = roiFiltered;
        }

        return contourSource.Clone();
    }

    public List<DetectionResult> Detect(
        Mat frame,
        int threshold,
        double minArea,
        double maxArea,
        double minCircularity,
        double minConfidence,
        bool invert,
        OpenCvSharp.Rect? detectionRoi,
        out Mat mask)
    {
        using var gray = new Mat();
        Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
        using var contourSource = BuildBinaryMask(frame, threshold, invert, detectionRoi);

        Cv2.FindContours(
            contourSource,
            out CvPoint[][] contours,
            out _,
            RetrievalModes.External,
            ContourApproximationModes.ApproxSimple);

        var detections = new List<DetectionResult>();
        foreach (var contour in contours)
        {
            var hull = Cv2.ConvexHull(contour);
            CvPoint[] measureContour = hull.Length >= 3 ? hull : contour;

            var rotatedBox = Cv2.MinAreaRect(measureContour);
            double area = DetectionGeometry.ComputeHullMaskAreaFromContour(contour, contourSource.Size());
            if (area < minArea || area > maxArea) continue;

            double perimeter = Cv2.ArcLength(measureContour, true);
            if (perimeter <= 0) continue;

            double circularity = 4.0 * Math.PI * area / (perimeter * perimeter);
            if (circularity < minCircularity) continue;

            var center = rotatedBox.Center;
            var (objectXAxis, objectYAxis) = GetRotatedBoxAxes(rotatedBox);
            Cv2.MinEnclosingCircle(measureContour, out _, out float radius);
            var box = Cv2.BoundingRect(measureContour);
            double fillRatio = radius > 1e-9 ? area / (Math.PI * radius * radius) : 0.0;
            fillRatio = Clamp01(fillRatio);

            double hullArea = hull.Length >= 3 ? Cv2.ContourArea(hull) : area;
            double solidity = hullArea > 1e-9 ? area / hullArea : 0.0;
            solidity = Clamp01(solidity);

            double confidence =
                0.50 * Clamp01(circularity) +
                0.30 * fillRatio +
                0.20 * solidity;
            confidence = Clamp01(confidence);

            if (confidence <= minConfidence) continue;

            double angle = CalculateClockwiseAngleFromGlobalY(objectYAxis);
            var angleRefinement = RefineAngleByTemplateMatching(gray, box, angle);
            angle = angleRefinement.angle;
            (objectXAxis, objectYAxis) = GetAxesFromClockwiseAngle(angle);

            detections.Add(new DetectionResult
            {
                Center = center,
                Area = area,
                Radius = radius,
                Circularity = circularity,
                Confidence = confidence,
                Angle = angle,
                BoundingBox = box,
                RotatedBox = rotatedBox,
                ObjectXAxis = objectXAxis,
                ObjectYAxis = objectYAxis,
                SimilarityScore = angleRefinement.score,
                MaskContour = (CvPoint[])measureContour.Clone()
            });
        }

        mask = contourSource.Clone();

        return detections
            .OrderBy(d => d.Center.Y)
            .ThenBy(d => d.Center.X)
            .Select((d, index) => d.WithId(index + 1))
            .ToList();
    }
}
