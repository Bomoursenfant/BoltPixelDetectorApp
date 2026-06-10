using System.Text;
using System.Text.Json;

namespace BoltPixelDetectorApp;

public sealed class FlaskApiClient : IDisposable
{
    private readonly HttpClient _httpClient;
    private readonly string _baseUrl;

    public event EventHandler<string>? OnStatusChanged;

    public FlaskApiClient(string baseUrl, int timeoutMs = 10000)
    {
        _baseUrl = baseUrl.TrimEnd('/');
        _httpClient = new HttpClient { Timeout = TimeSpan.FromMilliseconds(timeoutMs) };
    }

    public async Task<bool> CheckHealth()
    {
        try
        {
            using var response = await _httpClient.GetAsync($"{_baseUrl}/health").ConfigureAwait(false);
            bool ok = response.IsSuccessStatusCode;
            OnStatusChanged?.Invoke(this, ok ? "OK: Flask API ready" : "ERROR: Flask API not responding");
            return ok;
        }
        catch (Exception ex)
        {
            OnStatusChanged?.Invoke(this, $"ERROR: Flask connection failed: {ex.Message}");
            return false;
        }
    }

    public Task<bool> SendVisionResult(VisionResult result)
    {
        var payload = new
        {
            object_name = result.Name,
            x = result.X,
            y = result.Y,
            x_robot = result.X,
            y_robot = result.Y,
            angle = result.Angle,
            confidence = result.Score,
            source = result.Source,
            timestamp = DateTime.UtcNow
        };
        return PostJsonAsync("/api/vision/result", payload);
    }

    public async Task<int> ClearPendingRobotCoordinatesAsync()
    {
        try
        {
            using var content = new StringContent("{}", Encoding.UTF8, "application/json");
            using var response = await _httpClient
                .PostAsync($"{_baseUrl}/api/robot/coordinates/clear-pending", content)
                .ConfigureAwait(false);
            if (!response.IsSuccessStatusCode)
                return 0;

            string json = await response.Content.ReadAsStringAsync().ConfigureAwait(false);
            using var doc = JsonDocument.Parse(json);
            if (doc.RootElement.TryGetProperty("cleared", out var cleared))
                return cleared.GetInt32();
            return 0;
        }
        catch
        {
            return 0;
        }
    }

    public Task<bool> SendRobotCoordinates(double x, double y, double angle, string? objectName = null)
    {
        var payload = new
        {
            x_coordinate = x,
            y_coordinate = y,
            angle_degrees = angle,
            object_name = objectName ?? "DETECTED_OBJECT",
            timestamp = DateTime.UtcNow
        };
        return PostJsonAsync("/api/robot/coordinates", payload);
    }

    public async Task<PendingRobotCoordinate?> GetPendingRobotCoordinate(bool markSent)
    {
        try
        {
            string mark = markSent ? "1" : "0";
            using var response = await _httpClient.GetAsync($"{_baseUrl}/api/robot/coordinates/pending?mark_sent={mark}").ConfigureAwait(false);
            if ((int)response.StatusCode == 204 || !response.IsSuccessStatusCode)
                return null;

            string json = await response.Content.ReadAsStringAsync().ConfigureAwait(false);
            return JsonSerializer.Deserialize<PendingRobotCoordinate>(json, new JsonSerializerOptions { PropertyNameCaseInsensitive = true });
        }
        catch
        {
            return null;
        }
    }

    public async Task<VisionStatistics?> GetStatistics()
    {
        try
        {
            using var response = await _httpClient.GetAsync($"{_baseUrl}/api/statistics").ConfigureAwait(false);
            if (!response.IsSuccessStatusCode)
                return null;

            string json = await response.Content.ReadAsStringAsync().ConfigureAwait(false);
            return JsonSerializer.Deserialize<VisionStatistics>(json, new JsonSerializerOptions { PropertyNameCaseInsensitive = true });
        }
        catch
        {
            return null;
        }
    }

    public Task<bool> LogRobotEvent(string eventType, string? eventData = null)
    {
        return PostJsonAsync("/api/robot/event", new
        {
            event_type = eventType,
            event_data = eventData ?? "",
            timestamp = DateTime.UtcNow
        });
    }

    public Task<bool> SendCycleTiming(VisionCycleTiming timing)
    {
        var payload = new
        {
            cycle_id = timing.CycleId,
            snapshot_index = timing.SnapshotIndex,
            trigger_source = timing.TriggerSource,
            request_code = timing.RequestCode,
            detection_count = timing.DetectionCount,
            robot_ready_count = timing.RobotReadyCount,
            detect_ms = timing.DetectMs,
            db_flask_ms = timing.DbFlaskMs,
            robot_fetch_ms = timing.RobotFetchMs,
            robot_read_ms = timing.RobotReadMs,
            tcp_reply_ms = timing.TcpReplyMs,
            total_cycle_ms = timing.TotalCycleMs,
            robot_request_total_ms = timing.RobotRequestTotalMs,
            queue_source = timing.QueueSource,
            notes = timing.Notes,
            timestamp = DateTime.UtcNow
        };
        return PostJsonAsync("/api/vision/timing", payload);
    }

    private async Task<bool> PostJsonAsync(string endpoint, object payload)
    {
        try
        {
            string json = JsonSerializer.Serialize(payload);
            using var content = new StringContent(json, Encoding.UTF8, "application/json");
            using var response = await _httpClient.PostAsync($"{_baseUrl}{endpoint}", content).ConfigureAwait(false);
            return response.IsSuccessStatusCode;
        }
        catch
        {
            return false;
        }
    }

    public void Dispose()
    {
        _httpClient.Dispose();
    }
}
