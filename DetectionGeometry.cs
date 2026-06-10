namespace BoltPixelDetectorApp;

using OpenCvSharp;
using CvPoint = OpenCvSharp.Point;

/// <summary>Geometry metrics for detections.</summary>
internal static class DetectionGeometry
{
    /// <summary>
    /// Area in px^2 = number of pixels inside the contour (filled mask).
    /// Same physical bolt at different angles yields similar values; not inflated like OBB W×H.
    /// </summary>
    public static double ComputeMaskAreaFromContour(Point[] contour, OpenCvSharp.Size imageSize)
    {
        if (contour.Length < 3 || imageSize.Width <= 0 || imageSize.Height <= 0)
            return 0;

        Rect bounds = Cv2.BoundingRect(contour);
        int x = Math.Clamp(bounds.X, 0, Math.Max(0, imageSize.Width - 1));
        int y = Math.Clamp(bounds.Y, 0, Math.Max(0, imageSize.Height - 1));
        int right = Math.Clamp(bounds.X + bounds.Width, x + 1, imageSize.Width);
        int bottom = Math.Clamp(bounds.Y + bounds.Height, y + 1, imageSize.Height);
        var roi = new Rect(x, y, Math.Max(1, right - x), Math.Max(1, bottom - y));

        var shifted = new Point[contour.Length];
        for (int i = 0; i < contour.Length; i++)
        {
            shifted[i] = new Point(
                Math.Clamp(contour[i].X - roi.X, 0, roi.Width - 1),
                Math.Clamp(contour[i].Y - roi.Y, 0, roi.Height - 1));
        }

        using var mask = new Mat(roi.Size, MatType.CV_8UC1, Scalar.Black);
        Cv2.FillPoly(mask, new[] { shifted }, Scalar.White);
        return CountMaskPixels(mask);
    }

    /// <summary>Convex-hull mask area — smoother than spiky threshold contours (OpenCV path).</summary>
    public static double ComputeHullMaskAreaFromContour(Point[] contour, OpenCvSharp.Size imageSize)
    {
        if (contour.Length < 3)
            return 0;

        var hull = Cv2.ConvexHull(contour);
        if (hull.Length < 3)
            return ComputeMaskAreaFromContour(contour, imageSize);

        return ComputeMaskAreaFromContour(hull, imageSize);
    }

    public static int CountMaskPixels(Mat binaryMask)
    {
        if (binaryMask.Empty())
            return 0;
        return Cv2.CountNonZero(binaryMask);
    }
}
