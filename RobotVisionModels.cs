using System.Text.Json.Serialization;

namespace BoltPixelDetectorApp;

public sealed class VisionResult
{
    public string Name { get; set; } = "DETECTED_OBJECT";
    public double PixelX { get; set; }
    public double PixelY { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Angle { get; set; }
    public double Score { get; set; }
    public string Source { get; set; } = "BoltPixelDetectorApp";
}

public sealed class RobotCoordinateBatch
{
    public List<VisionResult> Results { get; set; } = new();
    public int TotalBeforeSend { get; set; }
    public string Source { get; set; } = "Local Queue";
    public VisionCycleTiming? Timing { get; set; }
}

public sealed class VisionCycleTiming
{
    public string CycleId { get; set; } = Guid.NewGuid().ToString("N");
    public int SnapshotIndex { get; set; }
    public string TriggerSource { get; set; } = "Manual";
    public int? RequestCode { get; set; }
    public int DetectionCount { get; set; }
    public int RobotReadyCount { get; set; }
    public long DetectMs { get; set; }
    public long DbFlaskMs { get; set; }
    public long RobotFetchMs { get; set; }
    public long RobotReadMs { get; set; }
    public long TcpReplyMs { get; set; }
    public long TotalCycleMs { get; set; }
    public long RobotRequestTotalMs { get; set; }
    public string QueueSource { get; set; } = "";
    public string Notes { get; set; } = "";
}

public sealed class RobotDataEventArgs : EventArgs
{
    public string Status { get; set; } = "DISCONNECTED";
    public int PendingCount { get; set; }
    /// <summary>Last robot request byte (0–255), or -1 if no TCP request received yet.</summary>
    public int LastRequestCode { get; set; } = -1;
}

public sealed class PendingRobotCoordinate
{
    [JsonPropertyName("id")]
    public int Id { get; set; }

    [JsonPropertyName("x")]
    public double X { get; set; }

    [JsonPropertyName("y")]
    public double Y { get; set; }

    [JsonPropertyName("angle")]
    public double Angle { get; set; }

    [JsonPropertyName("object_name")]
    public string ObjectName { get; set; } = "DETECTED_OBJECT";

    public VisionResult ToVisionResult()
    {
        return new VisionResult
        {
            Name = string.IsNullOrWhiteSpace(ObjectName) ? "DETECTED_OBJECT" : ObjectName,
            PixelX = 0,
            PixelY = 0,
            X = X,
            Y = Y,
            Angle = Angle,
            Score = 1.0,
            Source = "Flask/DB"
        };
    }
}

public sealed class VisionStatistics
{
    [JsonPropertyName("total_results")]
    public int TotalResults { get; set; }

    [JsonPropertyName("today_results")]
    public int TodayResults { get; set; }

    [JsonPropertyName("average_confidence")]
    public double AverageConfidence { get; set; }

    [JsonPropertyName("pending_robot_coordinates")]
    public int PendingRobotCoordinates { get; set; }
}
