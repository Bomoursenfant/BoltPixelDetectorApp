using System.Diagnostics;
using System.Globalization;
using System.Text.Json;
using OpenCvSharp;
using OpenCvSharp.Dnn;
using CvPoint = OpenCvSharp.Point;
using CvSize = OpenCvSharp.Size;

namespace BoltPixelDetectorApp;

public sealed class YoloSegmentationDetector
{
    private const double UnionAreaGainRatio = 1.20;
    private const double UnionBoxFillThreshold = 0.45;
    private const double UnionSearchExpandRatio = 0.50;
    private const int UnionSearchExpandMin = 8;

    // Cache the ONNX network so frame-by-frame inference reuses the loaded model.
    private readonly object _onnxCacheLock = new();
    private Net? _cachedOnnxNet;
    private string? _cachedOnnxModelPath;
    private DateTime _cachedOnnxModelWriteTimeUtc;
    private string[]? _cachedOnnxOutputNames;
    private int _cachedOnnxWarmupImageSize = -1;

    private sealed class Candidate
    {
        public Rect Box { get; init; }
        public float Confidence { get; init; }
        public float[] MaskCoefficients { get; init; } = Array.Empty<float>();
    }

    private sealed class PythonResponse
    {
        public string? Error { get; set; }
        public List<PythonObject> Objects { get; set; } = new();
    }

    private sealed class PythonObject
    {
        public double Confidence { get; set; }
        public List<double> Box { get; set; } = new();
        public List<List<double>> Points { get; set; } = new();
    }

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true
    };


    public List<DetectionResult> Detect(
        Mat frame,
        string modelPath,
        string pythonExe,
        int imageSize,
        double nmsThreshold,
        double minConfidence,
        OpenCvSharp.Rect? detectionRoi,
        out Mat mask)
    {
        string extension = Path.GetExtension(modelPath);
        return extension.Equals(".pt", StringComparison.OrdinalIgnoreCase)
            ? DetectPt(frame, modelPath, pythonExe, imageSize, nmsThreshold, minConfidence, detectionRoi, opencvBinaryForFusion: null, fusionMinContourArea: 0, fusionMinAreaRatio: 0, out mask)
            : DetectOnnx(frame, modelPath, imageSize, nmsThreshold, minConfidence, detectionRoi, opencvBinaryForFusion: null, fusionMinContourArea: 0, fusionMinAreaRatio: 0, out mask);
    }

    /// <summary>
    /// YOLO instance mask intersected with classical OpenCV binary (threshold/Otsu + morphology).
    /// MinAreaRect center/angle are computed from the fused contour when overlap is strong enough; otherwise YOLO contour is used.
    /// </summary>
    public List<DetectionResult> DetectFused(
        Mat frame,
        string modelPath,
        string pythonExe,
        int imageSize,
        double nmsThreshold,
        double minConfidence,
        OpenCvSharp.Rect? detectionRoi,
        Mat opencvBinaryMask,
        double fusionMinContourArea,
        double fusionMinAreaRatio,
        out Mat mask)
    {
        string extension = Path.GetExtension(modelPath);
        return extension.Equals(".pt", StringComparison.OrdinalIgnoreCase)
            ? DetectPt(frame, modelPath, pythonExe, imageSize, nmsThreshold, minConfidence, detectionRoi, opencvBinaryMask, fusionMinContourArea, fusionMinAreaRatio, out mask)
            : DetectOnnx(frame, modelPath, imageSize, nmsThreshold, minConfidence, detectionRoi, opencvBinaryMask, fusionMinContourArea, fusionMinAreaRatio, out mask);
    }

    private List<DetectionResult> DetectOnnx(
        Mat frame,
        string modelPath,
        int imageSize,
        double nmsThreshold,
        double minConfidence,
        OpenCvSharp.Rect? detectionRoi,
        Mat? opencvBinaryForFusion,
        double fusionMinContourArea,
        double fusionMinAreaRatio,
        out Mat mask)
    {
        mask = new Mat(frame.Size(), MatType.CV_8UC1, Scalar.Black);

        if (!File.Exists(modelPath))
            throw new FileNotFoundException("YOLO model file was not found.", modelPath);

        if (!Path.GetExtension(modelPath).Equals(".onnx", StringComparison.OrdinalIgnoreCase))
            throw new NotSupportedException("Only .onnx is supported by OpenCV DNN directly. Use .pt with Python/Ultralytics or export it to .onnx.");

        using var input = Letterbox(frame, imageSize, out double scale, out int padX, out int padY);
        using var blob = CvDnn.BlobFromImage(input, 1.0 / 255.0, new CvSize(imageSize, imageSize), Scalar.All(0), swapRB: true, crop: false);

        var outputs = new List<Mat>();

        try
        {
            using (LightProfiler.Measure("YOLO:Forward"))
            {
                lock (_onnxCacheLock)
                {
                    EnsureOnnxModelLoaded(modelPath, imageSize);
                    _cachedOnnxNet!.SetInput(blob);
                    string[] outputNames = _cachedOnnxOutputNames ?? Array.Empty<string>();
                    outputs = outputNames.Select(_ => new Mat()).ToList();
                    _cachedOnnxNet!.Forward(outputs, outputNames);
                }
            }

            if (outputs.Count == 0 || outputs[0].Empty())
                return new List<DetectionResult>();

            Mat prediction = outputs[0];
            Mat? prototype = outputs.Count > 1 && !outputs[1].Empty() ? outputs[1] : null;
            List<Candidate> candidates;
            List<Candidate> selected;
            using (LightProfiler.Measure("YOLO:ParseNms"))
            {
                candidates = ParseCandidates(prediction, prototype, frame.Size(), scale, padX, padY, minConfidence);
                selected = ApplyNms(candidates, (float)nmsThreshold);
            }

            var detections = new List<DetectionResult>();
            using (LightProfiler.Measure("YOLO:BuildMasks"))
            {
                foreach (var candidate in selected)
                {
                    CvPoint[] contour = prototype is not null && candidate.MaskCoefficients.Length > 0
                        ? BuildSegmentationContour(candidate, prototype, frame.Size(), imageSize, scale, padX, padY)
                        : BuildBoxContour(candidate.Box);

                    AddDetectionFromContour(detections, mask, contour, candidate.Box, candidate.Confidence, detectionRoi, opencvBinaryForFusion, fusionMinContourArea, fusionMinAreaRatio);
                }
            }

            return Reindex(detections);
        }
        finally
        {
            foreach (var output in outputs)
                output.Dispose();
        }
    }

    


    private static List<DetectionResult> DetectPt(
        Mat frame,
        string modelPath,
        string pythonExe,
        int imageSize,
        double nmsThreshold,
        double minConfidence,
        OpenCvSharp.Rect? detectionRoi,
        Mat? opencvBinaryForFusion,
        double fusionMinContourArea,
        double fusionMinAreaRatio,
        out Mat mask)
    {
        mask = new Mat(frame.Size(), MatType.CV_8UC1, Scalar.Black);

        if (!File.Exists(modelPath))
            throw new FileNotFoundException("YOLO model file was not found.", modelPath);

        string scriptPath = GetPythonScriptPath();
        if (!File.Exists(scriptPath))
            throw new FileNotFoundException("YOLO Python bridge script was not found.", scriptPath);

        Directory.CreateDirectory(Path.Combine(Path.GetTempPath(), "BoltPixelDetectorApp"));
        string imagePath = Path.Combine(Path.GetTempPath(), "BoltPixelDetectorApp", $"yolo_seg_{Guid.NewGuid():N}.png");

        try
        {
            frame.SaveImage(imagePath);
            PythonResponse response = RunPythonBridge(scriptPath, imagePath, modelPath, pythonExe, imageSize, nmsThreshold, minConfidence);
            var detections = new List<DetectionResult>();

            foreach (var item in response.Objects)
            {
                var contour = item.Points
                    .Where(point => point.Count >= 2)
                    .Select(point => new CvPoint(
                        Math.Clamp((int)Math.Round(point[0]), 0, frame.Width - 1),
                        Math.Clamp((int)Math.Round(point[1]), 0, frame.Height - 1)))
                    .Distinct()
                    .ToArray();

                Rect box = item.Box.Count >= 4
                    ? RectFromXyxy(item.Box, frame.Size())
                    : Cv2.BoundingRect(contour);

                AddDetectionFromContour(detections, mask, contour, box, item.Confidence, detectionRoi, opencvBinaryForFusion, fusionMinContourArea, fusionMinAreaRatio);
            }

            return Reindex(detections);
        }
        finally
        {
            try
            {
                if (File.Exists(imagePath))
                    File.Delete(imagePath);
            }
            catch
            {
                // Temp cleanup should not break detection.
            }
        }
    }

    private static void AddDetectionFromContour(
        List<DetectionResult> detections,
        Mat mask,
        CvPoint[] contour,
        Rect yoloBox,
        double confidence,
        OpenCvSharp.Rect? detectionRoi,
        Mat? opencvBinaryForFusion,
        double fusionMinContourArea,
        double fusionMinAreaRatio)
    {
        if (contour.Length < 3) return;

        if (opencvBinaryForFusion is not null &&
            (opencvBinaryForFusion.Width != mask.Width || opencvBinaryForFusion.Height != mask.Height))
            throw new ArgumentException("OpenCV binary mask size must match the frame.");

        double yoloContourArea = Cv2.ContourArea(contour);
        CvPoint[] geometryContour = contour;
        CvPoint[]? unionContour = null;

        if (opencvBinaryForFusion is not null)
        {
            using var instanceMask = new Mat(mask.Size(), MatType.CV_8UC1, Scalar.Black);
            Cv2.FillPoly(instanceMask, new[] { contour }, Scalar.White);
            using var fused = new Mat();
            Cv2.BitwiseAnd(instanceMask, opencvBinaryForFusion, fused);
            Cv2.FindContours(
                fused,
                out CvPoint[][] fusedContours,
                out _,
                RetrievalModes.External,
                ContourApproximationModes.ApproxSimple);

            CvPoint[]? bestContour = null;
            double bestArea = 0;
            foreach (var fc in fusedContours)
            {
                if (fc.Length < 3) continue;
                double a = Cv2.ContourArea(fc);
                if (a > bestArea)
                {
                    bestArea = a;
                    bestContour = fc;
                }
            }

            if (bestContour is not null &&
                bestArea >= fusionMinContourArea &&
                bestArea >= fusionMinAreaRatio * Math.Max(yoloContourArea, 1.0))
            {
                geometryContour = bestContour;
            }

            unionContour = BuildUnionContour(instanceMask, opencvBinaryForFusion, yoloBox, mask.Size());
        }

        double geometryContourArea = Cv2.ContourArea(geometryContour);
        CvPoint[] centerContour = geometryContour;
        if (unionContour is not null)
        {
            double unionArea = Cv2.ContourArea(unionContour);
            double yoloBoxArea = yoloBox.Width > 0 && yoloBox.Height > 0
                ? yoloBox.Width * yoloBox.Height
                : 0.0;
            double fillRatio = yoloBoxArea > 1e-6 ? geometryContourArea / yoloBoxArea : 1.0;

            if (unionArea >= Math.Max(geometryContourArea * UnionAreaGainRatio, fusionMinContourArea) ||
                (fillRatio < UnionBoxFillThreshold && unionArea > geometryContourArea))
            {
                centerContour = unionContour;
            }
        }

        var fallbackBox = Cv2.MinAreaRect(geometryContour);

        CvPoint[] orientationContour = geometryContour;
        var hull = Cv2.ConvexHull(geometryContour);
        if (hull.Length >= 3)
            orientationContour = hull;

        Point2f objectYAxis;
        Point2f objectXAxis;
        if (TryComputePcaAxis(orientationContour, out var pcaCenter, out var pcaYAxis))
        {
            objectYAxis = NormalizeVector(pcaYAxis);
            CvPoint[] directionContour = unionContour ?? centerContour;
            if (TryResolveAxisDirectionByWidth(directionContour, pcaCenter, objectYAxis, out var resolvedYAxis))
                objectYAxis = resolvedYAxis;

            objectXAxis = new Point2f(objectYAxis.Y, -objectYAxis.X);
            if (objectXAxis.Y > 0)
                objectXAxis = new Point2f(-objectXAxis.X, -objectXAxis.Y);
        }
        else
        {
            (objectXAxis, objectYAxis) = GetRotatedBoxAxes(fallbackBox);
        }
        double angle = CalculateClockwiseAngleFromGlobalY(objectYAxis);

        RotatedRect rotatedBox = TryBuildAlignedBox(centerContour, objectYAxis, out var alignedBox)
            ? alignedBox
            : fallbackBox;

        if (detectionRoi.HasValue && !detectionRoi.Value.Contains(new CvPoint((int)Math.Round(rotatedBox.Center.X), (int)Math.Round(rotatedBox.Center.Y))))
            return;

        // Area = YOLO instance mask pixels only (stable vs. angle; avoids OBB empty corners and binary union).
        double area = DetectionGeometry.ComputeMaskAreaFromContour(contour, mask.Size());
        Cv2.MinEnclosingCircle(centerContour, out _, out float radius);
        Cv2.FillPoly(mask, new[] { centerContour }, Scalar.White);

        detections.Add(new DetectionResult
        {
            Center = rotatedBox.Center,
            PixelX = rotatedBox.Center.Y,
            PixelY = rotatedBox.Center.X,
            Area = area,
            Radius = radius,
            Circularity = 0,
            Confidence = confidence,
            Angle = angle,
            BoundingBox = centerContour == geometryContour && yoloBox.Width > 0 && yoloBox.Height > 0
                ? yoloBox
                : Cv2.BoundingRect(centerContour),
            RotatedBox = rotatedBox,
            ObjectXAxis = objectXAxis,
            ObjectYAxis = objectYAxis,
            MaskContour = (CvPoint[])centerContour.Clone()
        });
    }

    private static CvPoint[]? BuildUnionContour(Mat instanceMask, Mat binaryMask, Rect yoloBox, OpenCvSharp.Size imageSize)
    {
        if (yoloBox.Width <= 0 || yoloBox.Height <= 0)
            return null;

        int expand = Math.Max(UnionSearchExpandMin, (int)Math.Round(Math.Max(yoloBox.Width, yoloBox.Height) * UnionSearchExpandRatio));
        Rect searchRect = ExpandAndClampRect(yoloBox, expand, imageSize);

        using var unionMask = instanceMask.Clone();
        using var unionRoi = new Mat(unionMask, searchRect);
        using var binaryRoi = new Mat(binaryMask, searchRect);
        Cv2.BitwiseOr(unionRoi, binaryRoi, unionRoi);

        Cv2.FindContours(unionMask, out CvPoint[][] contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
        return contours.OrderByDescending(c => Cv2.ContourArea(c)).FirstOrDefault();
    }

    private static Rect ExpandAndClampRect(Rect rect, int margin, OpenCvSharp.Size size)
    {
        int x = Math.Max(0, rect.X - margin);
        int y = Math.Max(0, rect.Y - margin);
        int right = Math.Min(size.Width, rect.X + rect.Width + margin);
        int bottom = Math.Min(size.Height, rect.Y + rect.Height + margin);
        int width = Math.Max(1, right - x);
        int height = Math.Max(1, bottom - y);
        return new Rect(x, y, width, height);
    }

    private static List<DetectionResult> Reindex(IEnumerable<DetectionResult> detections)
    {
        return detections
            .OrderBy(d => d.Center.Y)
            .ThenBy(d => d.Center.X)
            .Select((d, index) => d.WithId(index + 1))
            .ToList();
    }

    private static Mat Letterbox(Mat source, int imageSize, out double scale, out int padX, out int padY)
    {
        scale = Math.Min(imageSize / (double)source.Width, imageSize / (double)source.Height);
        int resizedWidth = Math.Max(1, (int)Math.Round(source.Width * scale));
        int resizedHeight = Math.Max(1, (int)Math.Round(source.Height * scale));
        padX = (imageSize - resizedWidth) / 2;
        padY = (imageSize - resizedHeight) / 2;

        var result = new Mat(new CvSize(imageSize, imageSize), MatType.CV_8UC3, new Scalar(114, 114, 114));
        using var resized = new Mat();
        Cv2.Resize(source, resized, new CvSize(resizedWidth, resizedHeight), 0, 0, InterpolationFlags.Linear);
        using var roi = new Mat(result, new Rect(padX, padY, resizedWidth, resizedHeight));
        resized.CopyTo(roi);
        return result;
    }

    private void EnsureOnnxModelLoaded(string modelPath, int imageSize)
    {
        DateTime modelWriteTimeUtc = File.GetLastWriteTimeUtc(modelPath);
        bool shouldReload = _cachedOnnxNet is null ||
            !string.Equals(_cachedOnnxModelPath, modelPath, StringComparison.OrdinalIgnoreCase) ||
            _cachedOnnxModelWriteTimeUtc != modelWriteTimeUtc;

        if (shouldReload)
        {
            _cachedOnnxNet?.Dispose();
            _cachedOnnxNet = CvDnn.ReadNetFromOnnx(modelPath)
                ?? throw new InvalidOperationException($"Cannot load ONNX model: {modelPath}");
            _cachedOnnxNet.SetPreferableBackend(Backend.OPENCV);
            _cachedOnnxNet.SetPreferableTarget(GetConfiguredDnnTarget());

            _cachedOnnxModelPath = modelPath;
            _cachedOnnxModelWriteTimeUtc = modelWriteTimeUtc;
            _cachedOnnxOutputNames = null;
            _cachedOnnxWarmupImageSize = -1;
        }

        if (_cachedOnnxOutputNames is null)
        {
            _cachedOnnxOutputNames = _cachedOnnxNet!
                .GetUnconnectedOutLayersNames()
                .Where(name => !string.IsNullOrWhiteSpace(name))
                .Select(name => name!)
                .ToArray();
        }

        if (_cachedOnnxWarmupImageSize != imageSize)
        {
            WarmupOnnxModel(imageSize);
            _cachedOnnxWarmupImageSize = imageSize;
        }
    }

    private void WarmupOnnxModel(int imageSize)
    {
        if (_cachedOnnxNet is null || _cachedOnnxOutputNames is null || _cachedOnnxOutputNames.Length == 0)
            return;

        using var warmupInput = new Mat(new CvSize(imageSize, imageSize), MatType.CV_8UC3, new Scalar(114, 114, 114));
        using var warmupBlob = CvDnn.BlobFromImage(warmupInput, 1.0 / 255.0, new CvSize(imageSize, imageSize), Scalar.All(0), swapRB: true, crop: false);
        _cachedOnnxNet.SetInput(warmupBlob);
        var warmupOutputs = _cachedOnnxOutputNames.Select(_ => new Mat()).ToList();
        try
        {
            _cachedOnnxNet.Forward(warmupOutputs, _cachedOnnxOutputNames);
        }
        finally
        {
            foreach (var output in warmupOutputs)
                output.Dispose();
        }
    }

    private static Target GetConfiguredDnnTarget()
    {
        string? configured = Environment.GetEnvironmentVariable("BOLT_DNN_TARGET");
        if (!string.IsNullOrWhiteSpace(configured) &&
            Enum.TryParse(configured, ignoreCase: true, out Target target))
        {
            return target;
        }

        return Target.CPU;
    }

    public void Warmup(string modelPath, int imageSize)
    {
        if (!Path.GetExtension(modelPath).Equals(".onnx", StringComparison.OrdinalIgnoreCase))
            return;

        if (!File.Exists(modelPath))
            return;

        lock (_onnxCacheLock)
        {
            EnsureOnnxModelLoaded(modelPath, imageSize);
        }
    }

    private static List<Candidate> ParseCandidates(
        Mat output,
        Mat? prototype,
        OpenCvSharp.Size frameSize,
        double scale,
        int padX,
        int padY,
        double minConfidence)
    {
        using Mat rows = ToPredictionRows(output);
        int maskDimensions = prototype is null ? 0 : prototype.Size(1);
        int dimensions = rows.Cols;
        int classCount = dimensions - 4 - maskDimensions;
        if (classCount <= 0) return new List<Candidate>();

        var candidates = new List<Candidate>();
        for (int row = 0; row < rows.Rows; row++)
        {
            float bestScore = 0;
            for (int cls = 0; cls < classCount; cls++)
            {
                float score = rows.At<float>(row, 4 + cls);
                if (score > bestScore)
                    bestScore = score;
            }

            if (bestScore < minConfidence) continue;

            float cx = rows.At<float>(row, 0);
            float cy = rows.At<float>(row, 1);
            float width = rows.At<float>(row, 2);
            float height = rows.At<float>(row, 3);

            int left = ToFrameX(cx - width / 2f, scale, padX, frameSize.Width);
            int top = ToFrameY(cy - height / 2f, scale, padY, frameSize.Height);
            int right = ToFrameX(cx + width / 2f, scale, padX, frameSize.Width);
            int bottom = ToFrameY(cy + height / 2f, scale, padY, frameSize.Height);

            var maskCoefficients = new float[maskDimensions];
            for (int i = 0; i < maskDimensions; i++)
                maskCoefficients[i] = rows.At<float>(row, 4 + classCount + i);

            candidates.Add(new Candidate
            {
                Box = new Rect(left, top, Math.Max(1, right - left), Math.Max(1, bottom - top)),
                Confidence = bestScore,
                MaskCoefficients = maskCoefficients
            });
        }

        return candidates;
    }

    private static Mat ToPredictionRows(Mat output)
    {
        if (output.Dims == 3)
        {
            int first = output.Size(1);
            int second = output.Size(2);
            using Mat reshaped = output.Reshape(1, first);
            if (first < second)
            {
                var transposed = new Mat();
                Cv2.Transpose(reshaped, transposed);
                return transposed;
            }

            return reshaped.Clone();
        }

        if (output.Dims == 2)
            return output.Clone();

        throw new NotSupportedException($"Unsupported YOLO output dimensions: {output.Dims}");
    }

    private static List<Candidate> ApplyNms(IReadOnlyList<Candidate> candidates, float nmsThreshold)
    {
        var ordered = candidates.OrderByDescending(c => c.Confidence).ToList();
        var selected = new List<Candidate>();

        while (ordered.Count > 0)
        {
            Candidate current = ordered[0];
            selected.Add(current);
            ordered.RemoveAt(0);
            ordered.RemoveAll(candidate => IoU(current.Box, candidate.Box) > nmsThreshold);
        }

        return selected;
    }

    private static double IoU(Rect a, Rect b)
    {
        int x1 = Math.Max(a.X, b.X);
        int y1 = Math.Max(a.Y, b.Y);
        int x2 = Math.Min(a.X + a.Width, b.X + b.Width);
        int y2 = Math.Min(a.Y + a.Height, b.Y + b.Height);
        int intersection = Math.Max(0, x2 - x1) * Math.Max(0, y2 - y1);
        int union = a.Width * a.Height + b.Width * b.Height - intersection;
        return union <= 0 ? 0 : intersection / (double)union;
    }

    private static CvPoint[] BuildSegmentationContour(
        Candidate candidate,
        Mat prototype,
        OpenCvSharp.Size frameSize,
        int imageSize,
        double scale,
        int padX,
        int padY)
    {
        int maskDimensions = prototype.Size(1);
        int maskHeight = prototype.Size(2);
        int maskWidth = prototype.Size(3);
        if (candidate.MaskCoefficients.Length != maskDimensions)
            return BuildBoxContour(candidate.Box);

        using var lowResMask = new Mat(new CvSize(maskWidth, maskHeight), MatType.CV_8UC1, Scalar.Black);
        for (int y = 0; y < maskHeight; y++)
        {
            for (int x = 0; x < maskWidth; x++)
            {
                double value = 0;
                for (int c = 0; c < maskDimensions; c++)
                    value += candidate.MaskCoefficients[c] * prototype.At<float>(new[] { 0, c, y, x });

                double probability = 1.0 / (1.0 + Math.Exp(-value));
                if (probability >= 0.5)
                    lowResMask.Set(y, x, (byte)255);
            }
        }

        using var modelMask = new Mat();
        Cv2.Resize(lowResMask, modelMask, new CvSize(imageSize, imageSize), 0, 0, InterpolationFlags.Linear);

        int unpaddedWidth = Math.Clamp((int)Math.Round(frameSize.Width * scale), 1, imageSize - padX);
        int unpaddedHeight = Math.Clamp((int)Math.Round(frameSize.Height * scale), 1, imageSize - padY);
        var validModelRect = new Rect(padX, padY, unpaddedWidth, unpaddedHeight);
        using var validModelMask = new Mat(modelMask, validModelRect);
        using var frameMask = new Mat();
        Cv2.Resize(validModelMask, frameMask, frameSize, 0, 0, InterpolationFlags.Nearest);

        using var boxMask = new Mat(frameSize, MatType.CV_8UC1, Scalar.Black);
        Cv2.Rectangle(boxMask, candidate.Box, Scalar.White, -1);
        Cv2.BitwiseAnd(frameMask, boxMask, frameMask);

        using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new CvSize(3, 3));
        Cv2.MorphologyEx(frameMask, frameMask, MorphTypes.Close, kernel, iterations: 1);

        Cv2.FindContours(frameMask, out CvPoint[][] contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
        return contours.OrderByDescending(contour => Cv2.ContourArea(contour)).FirstOrDefault() ?? BuildBoxContour(candidate.Box);
    }

    private static CvPoint[] BuildBoxContour(Rect box)
    {
        return new[]
        {
            new CvPoint(box.X, box.Y),
            new CvPoint(box.X + box.Width, box.Y),
            new CvPoint(box.X + box.Width, box.Y + box.Height),
            new CvPoint(box.X, box.Y + box.Height)
        };
    }

    private static Rect RectFromXyxy(IReadOnlyList<double> box, OpenCvSharp.Size frameSize)
    {
        int left = Math.Clamp((int)Math.Round(box[0]), 0, frameSize.Width - 1);
        int top = Math.Clamp((int)Math.Round(box[1]), 0, frameSize.Height - 1);
        int right = Math.Clamp((int)Math.Round(box[2]), left + 1, frameSize.Width);
        int bottom = Math.Clamp((int)Math.Round(box[3]), top + 1, frameSize.Height);
        return new Rect(left, top, right - left, bottom - top);
    }

    private static (Point2f xAxis, Point2f yAxis) GetRotatedBoxAxes(RotatedRect rotatedBox)
    {
        var points = rotatedBox.Points();
        var edge01 = new Point2f(points[1].X - points[0].X, points[1].Y - points[0].Y);
        var edge12 = new Point2f(points[2].X - points[1].X, points[2].Y - points[1].Y);
        double length01 = edge01.X * edge01.X + edge01.Y * edge01.Y;
        double length12 = edge12.X * edge12.X + edge12.Y * edge12.Y;
        var yAxis = NormalizeVector(length01 >= length12 ? edge01 : edge12);
        if (yAxis.X < 0) yAxis = new Point2f(-yAxis.X, -yAxis.Y);
        var xAxis = new Point2f(yAxis.Y, -yAxis.X);
        if (xAxis.Y > 0) xAxis = new Point2f(-xAxis.X, -xAxis.Y);
        return (xAxis, yAxis);
    }

    private static bool TryComputePcaAxis(CvPoint[] contour, out Point2f center, out Point2f yAxis)
    {
        center = new Point2f(0, 0);
        yAxis = new Point2f(1, 0);

        int n = contour.Length;
        if (n < 3) return false;

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
        if (norm < 1e-12) return false;

        vx /= norm;
        vy /= norm;

        // Prefer Y' pointing to global +Y (image right) for deterministic angle.
        if (vx < 0)
        {
            vx = -vx;
            vy = -vy;
        }

        center = new Point2f((float)meanX, (float)meanY);
        yAxis = new Point2f((float)vx, (float)vy);
        return true;
    }

    private static bool TryResolveAxisDirectionByWidth(
        CvPoint[] contour,
        Point2f center,
        Point2f yAxis,
        out Point2f resolvedYAxis)
    {
        resolvedYAxis = yAxis;
        var axis = NormalizeVector(yAxis);
        var perp = new Point2f(-axis.Y, axis.X);

        double minT = double.PositiveInfinity;
        double maxT = double.NegativeInfinity;
        foreach (var point in contour)
        {
            double dx = point.X - center.X;
            double dy = point.Y - center.Y;
            double t = dx * axis.X + dy * axis.Y;
            if (t < minT) minT = t;
            if (t > maxT) maxT = t;
        }

        double length = maxT - minT;
        if (length <= 1e-6)
            return false;

        double slice = length * 0.20;
        double minSum = 0;
        double maxSum = 0;
        int minCount = 0;
        int maxCount = 0;

        foreach (var point in contour)
        {
            double dx = point.X - center.X;
            double dy = point.Y - center.Y;
            double t = dx * axis.X + dy * axis.Y;
            double w = Math.Abs(dx * perp.X + dy * perp.Y);
            if (t <= minT + slice)
            {
                minSum += w;
                minCount++;
            }
            else if (t >= maxT - slice)
            {
                maxSum += w;
                maxCount++;
            }
        }

        if (minCount == 0 || maxCount == 0)
            return false;

        double minWidth = minSum / minCount;
        double maxWidth = maxSum / maxCount;
        const double flipRatio = 1.25;

        // Head is wider end; force Y' to point toward the wider side.
        if (maxWidth >= minWidth * flipRatio)
        {
            resolvedYAxis = axis;
            return true;
        }

        if (minWidth >= maxWidth * flipRatio)
        {
            resolvedYAxis = new Point2f(-axis.X, -axis.Y);
            return true;
        }

        return false;
    }

    private static bool TryBuildAlignedBox(CvPoint[] contour, Point2f axis, out RotatedRect box)
    {
        box = default;
        if (contour.Length < 3)
            return false;

        var unitAxis = NormalizeVector(axis);
        var perp = new Point2f(-unitAxis.Y, unitAxis.X);

        double minT = double.PositiveInfinity;
        double maxT = double.NegativeInfinity;
        double minS = double.PositiveInfinity;
        double maxS = double.NegativeInfinity;

        foreach (var point in contour)
        {
            double t = point.X * unitAxis.X + point.Y * unitAxis.Y;
            double s = point.X * perp.X + point.Y * perp.Y;
            if (t < minT) minT = t;
            if (t > maxT) maxT = t;
            if (s < minS) minS = s;
            if (s > maxS) maxS = s;
        }

        if (maxT <= minT || maxS <= minS)
            return false;

        double centerT = (minT + maxT) / 2.0;
        double centerS = (minS + maxS) / 2.0;
        var center = new Point2f(
            (float)(unitAxis.X * centerT + perp.X * centerS),
            (float)(unitAxis.Y * centerT + perp.Y * centerS));

        float width = (float)Math.Max(1.0, maxT - minT);
        float height = (float)Math.Max(1.0, maxS - minS);
        float angle = (float)(Math.Atan2(unitAxis.Y, unitAxis.X) * 180.0 / Math.PI);

        box = new RotatedRect(center, new Size2f(width, height), angle);
        return true;
    }

    private static Point2f NormalizeVector(Point2f vector)
    {
        double length = Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
        if (length < 1e-9) return new Point2f(1, 0);
        return new Point2f((float)(vector.X / length), (float)(vector.Y / length));
    }

    private static double CalculateClockwiseAngleFromGlobalY(Point2f yAxisImage)
    {
        double angle = Math.Atan2(yAxisImage.Y, yAxisImage.X) * 180.0 / Math.PI;
        angle %= 360.0;
        return angle < 0 ? angle + 360.0 : angle;
    }

    private static int ToFrameX(double modelX, double scale, int padX, int frameWidth)
    {
        return Math.Clamp((int)Math.Round((modelX - padX) / scale), 0, frameWidth - 1);
    }

    private static int ToFrameY(double modelY, double scale, int padY, int frameHeight)
    {
        return Math.Clamp((int)Math.Round((modelY - padY) / scale), 0, frameHeight - 1);
    }

    private static string GetPythonScriptPath()
    {
        string outputPath = Path.Combine(AppContext.BaseDirectory, "YoloSegmentationBridge.py");
        if (File.Exists(outputPath)) return outputPath;
        return Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "YoloSegmentationBridge.py"));
    }

    private static PythonResponse RunPythonBridge(
        string scriptPath,
        string imagePath,
        string modelPath,
        string pythonExe,
        int imageSize,
        double nmsThreshold,
        double minConfidence)
    {
        var startInfo = new ProcessStartInfo
        {
            FileName = string.IsNullOrWhiteSpace(pythonExe) ? "python" : pythonExe,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true
        };

        startInfo.ArgumentList.Add(scriptPath);
        startInfo.ArgumentList.Add("--model");
        startInfo.ArgumentList.Add(modelPath);
        startInfo.ArgumentList.Add("--image");
        startInfo.ArgumentList.Add(imagePath);
        startInfo.ArgumentList.Add("--conf");
        startInfo.ArgumentList.Add(minConfidence.ToString("G", CultureInfo.InvariantCulture));
        startInfo.ArgumentList.Add("--iou");
        startInfo.ArgumentList.Add(nmsThreshold.ToString("G", CultureInfo.InvariantCulture));
        startInfo.ArgumentList.Add("--imgsz");
        startInfo.ArgumentList.Add(imageSize.ToString(CultureInfo.InvariantCulture));

        using var process = Process.Start(startInfo) ?? throw new InvalidOperationException("Cannot start Python YOLO bridge.");
        string stdout = process.StandardOutput.ReadToEnd();
        string stderr = process.StandardError.ReadToEnd();
        process.WaitForExit();

        if (process.ExitCode != 0 && string.IsNullOrWhiteSpace(stdout))
            throw new InvalidOperationException($"YOLO Python bridge failed: {stderr}");

        var response = JsonSerializer.Deserialize<PythonResponse>(stdout, JsonOptions)
            ?? throw new InvalidOperationException($"YOLO Python bridge returned invalid JSON. stderr: {stderr}");

        if (!string.IsNullOrWhiteSpace(response.Error))
            throw new InvalidOperationException($"YOLO Python bridge failed: {response.Error}");

        return response;
    }

    public void Dispose()
    {
        lock (_onnxCacheLock)
        {
            _cachedOnnxNet?.Dispose();
            _cachedOnnxNet = null;
            _cachedOnnxModelPath = null;
            _cachedOnnxOutputNames = null;
            _cachedOnnxWarmupImageSize = -1;
        }
    }
}
