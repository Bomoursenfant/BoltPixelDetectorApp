namespace BoltPixelDetectorApp;

/// <summary>
/// Converts vision clockwise angle (0° at 3 o'clock)
/// to robot angle: 9h is 0; upper half (9h->3h) is negative, lower half (3h->9h) is positive.
/// </summary>
public static class RobotAngleConverter
{
    public static double FromVisionClockwiseAngle(double visionAngleDegrees)
    {
        double theta = visionAngleDegrees % 360.0;
        if (theta < 0)
            theta += 360.0;

        // Vision theta: 3h=0, 6h=90, 9h=180, 12h=270.
        // Robot angle: 9h=0, 12h=-90, 3h=+/-180, 6h=90.
        double robot = 180.0 - theta;
        if (robot < -180.0)
            robot += 360.0;
        return Math.Clamp(robot, -180.0, 180.0);
    }
}
