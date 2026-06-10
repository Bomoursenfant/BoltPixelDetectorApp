using OpenCvSharp;

namespace BoltPixelDetectorApp;

public sealed class DetectionResult
{
    public int Id { get; init; }
    public Point2f Center { get; init; }
    public double PixelX { get; init; }
    public double PixelY { get; init; }
    public double Area { get; init; }
    public double Radius { get; init; }
    public double Circularity { get; init; }
    public double Confidence { get; init; }
    public double Angle { get; init; }
    public Rect BoundingBox { get; init; }
    public RotatedRect RotatedBox { get; init; }
    public Point2f ObjectXAxis { get; init; }
    public Point2f ObjectYAxis { get; init; }
    public double SimilarityScore { get; init; }

    /// <summary>Fused/YOLO instance contour used for scene proximity (B3).</summary>
    public OpenCvSharp.Point[]? MaskContour { get; init; }

    public DetectionResult WithId(int id) => new()
    {
        Id = id,
        Center = Center,
        PixelX = PixelX,
        PixelY = PixelY,
        Area = Area,
        Radius = Radius,
        Circularity = Circularity,
        Confidence = Confidence,
        Angle = Angle,
        BoundingBox = BoundingBox,
        RotatedBox = RotatedBox,
        ObjectXAxis = ObjectXAxis,
        ObjectYAxis = ObjectYAxis,
        SimilarityScore = SimilarityScore,
        MaskContour = MaskContour
    };
}
