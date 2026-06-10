using System.Globalization;

namespace BoltPixelDetectorApp;

/// <summary>Tracks robot TCP request bytes (0x00 / 0x01 / 0x02) and binary batch replies for UI diagnostics.</summary>
public sealed class RobotRequestMonitor
{
    private readonly object _lock = new();
    private int _requestCount;
    private int _binaryBatchSentCount;
    private DateTime? _lastRequestAt;
    private DateTime? _lastBinaryBatchSentAt;
    private string _lastRemoteIp = "";
    private int _lastRequestByte = -1;

    public int RequestCount
    {
        get { lock (_lock) return _requestCount; }
    }

    public int LastRequestByte
    {
        get { lock (_lock) return _lastRequestByte; }
    }

    public DateTime? LastRequestAt
    {
        get { lock (_lock) return _lastRequestAt; }
    }

    public string RecordRequestByte(string remoteIp, byte[] data, string requestType = "")
    {
        int requestByte = data.Length > 0 ? data[0] : -1;

        lock (_lock)
        {
            _requestCount++;
            _lastRequestAt = DateTime.Now;
            _lastRemoteIp = remoteIp;
            _lastRequestByte = requestByte;
        }

        string byteText = requestByte >= 0 ? $"0x{requestByte:X2} ({requestByte})" : "<none>";
        string typeText = string.IsNullOrWhiteSpace(requestType) ? "" : $", TYPE={requestType}";
        return $"CHECK RX REQUEST #{_requestCount} FROM {remoteIp}: {byteText}, BYTES={data.Length}{typeText}";
    }

    public string RecordBinaryBatchSent(
        string remoteIp,
        int rawRequestCode,
        string requestType,
        IReadOnlyList<VisionResult> batch,
        int totalBeforeSend,
        int headerBytes,
        int bodyBytes,
        bool isTestCoordinate)
    {
        lock (_lock)
        {
            _binaryBatchSentCount++;
            _lastBinaryBatchSentAt = DateTime.Now;
        }

        string mode = isTestCoordinate ? "TEST " : "";
        string coordinateSummary = FormatCoordinates(batch);
        return string.Format(
            CultureInfo.InvariantCulture,
            "CHECK TX BATCH #{0} TO {1}: RAW=0x{2:X2}, TYPE={3}, {4}COUNT={5}, TOTAL={6}, HDR={7}B, BODY={8}B{9}",
            _binaryBatchSentCount,
            remoteIp,
            rawRequestCode,
            requestType,
            mode,
            batch.Count,
            totalBeforeSend,
            headerBytes,
            bodyBytes,
            coordinateSummary);
    }

    public string BuildSummary()
    {
        lock (_lock)
        {
            string requestTime = _lastRequestAt.HasValue
                ? _lastRequestAt.Value.ToString("HH:mm:ss", CultureInfo.InvariantCulture)
                : "never";
            string sendTime = _lastBinaryBatchSentAt.HasValue
                ? _lastBinaryBatchSentAt.Value.ToString("HH:mm:ss", CultureInfo.InvariantCulture)
                : "never";
            string requestByte = _lastRequestByte >= 0
                ? $"0x{_lastRequestByte:X2} ({_lastRequestByte})"
                : "<none>";

            return $"CHECK SUMMARY: RX={_requestCount} last={requestTime} byte={requestByte} from={_lastRemoteIp}; TX_BATCH={_binaryBatchSentCount} last={sendTime}";
        }
    }

    private static string FormatCoordinates(IReadOnlyList<VisionResult> batch)
    {
        if (batch.Count == 0)
            return "";

        string coords = string.Join(
            " | ",
            batch.Take(3).Select((item, index) => string.Format(
                CultureInfo.InvariantCulture,
                "#{0}: X={1:F2}, Y={2:F2}, A={3:F2}",
                index + 1,
                item.X,
                item.Y,
                item.Angle)));

        if (batch.Count > 3)
            coords += $" | +{batch.Count - 3} more";

        return " | " + coords;
    }
}
