using System.Globalization;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace BoltPixelDetectorApp;

/// <summary>
/// TCP server/client for Nachi MZ04 — aligned with RobotNachi receive path (buffered read + timeout).
/// </summary>
public sealed class RobotComms : IDisposable
{
    private readonly RobotConnectionSettings _settings;
    private readonly RobotRequestMonitor _requestMonitor = new();
    private readonly object _queueLock = new();
    private readonly Queue<VisionResult> _pendingResults = new();
    private TcpListener? _listener;
    private CancellationTokenSource? _cts;
    private int _lastRobotRequestByte = -1;

    public event EventHandler<RobotDataEventArgs>? OnRobotDataReceived;
    public Func<int, Task<RobotCoordinateBatch?>>? FetchRobotCoordinateBatchAsync { get; set; }
    public Func<Task<int?>>? FetchPendingRobotCoordinateCountAsync { get; set; }
    public Func<VisionCycleTiming, Task>? SaveCycleTimingAsync { get; set; }

    public RobotRequestMonitor RequestMonitor => _requestMonitor;

    public bool IsConnected => _listener is not null;

    public int PendingCount
    {
        get
        {
            lock (_queueLock)
                return _pendingResults.Count;
        }
    }

    public RobotComms(RobotConnectionSettings settings)
    {
        _settings = settings;
    }

    public bool Connect()
    {
        Disconnect();
        if (!_settings.IsValidRobotConfig())
        {
            PublishStatus("INVALID ROBOT SOCKET SETTINGS");
            return false;
        }

        _cts = new CancellationTokenSource();
        return _settings.IsServerMode ? StartServer(_cts.Token) : true;
    }

    public void Disconnect()
    {
        try { _cts?.Cancel(); } catch { }
        try { _listener?.Stop(); } catch { }
        _listener = null;
        _cts?.Dispose();
        _cts = null;
    }

    public void SendData(VisionResult data)
    {
        if (_settings.IsServerMode)
        {
            lock (_queueLock)
                _pendingResults.Enqueue(data);
            PublishStatus($"QUEUED {data.Name}: X={data.X:F1}, Y={data.Y:F1}, A={data.Angle:F1}");
            return;
        }

        _ = Task.Run(() => SendClientPacketAsync(data));
    }

    private bool StartServer(CancellationToken token)
    {
        try
        {
            var ip = ParseListenAddress(_settings.PcListenIP);
            _listener = new TcpListener(ip, _settings.PcListenPort);
            _listener.Start();
            _ = Task.Run(() => AcceptLoopAsync(token), token);
            PublishStatus($"SERVER LISTENING {ip}:{_settings.PcListenPort}");
            return true;
        }
        catch (Exception ex)
        {
            PublishStatus($"SERVER START FAILED: {ex.Message}");
            _listener = null;
            return false;
        }
    }

    private async Task AcceptLoopAsync(CancellationToken token)
    {
        while (!token.IsCancellationRequested && _listener is not null)
        {
            try
            {
                TcpClient robotClient = await _listener.AcceptTcpClientAsync(token).ConfigureAwait(false);
                _ = Task.Run(() => HandleRobotRequestAsync(robotClient, token), token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception ex)
            {
                PublishStatus($"ACCEPT ERROR: {ex.Message}");
                await Task.Delay(500, token).ConfigureAwait(false);
            }
        }
    }

    private async Task HandleRobotRequestAsync(TcpClient robotClient, CancellationToken token)
    {
        string remoteIp = GetRemoteIp(robotClient);
        var requestSw = Stopwatch.StartNew();
        PublishStatus($"CHECK SERVER ACCEPT FROM {remoteIp}");

        using (robotClient)
        {
            if (_settings.RestrictToRobotIP &&
                !string.IsNullOrWhiteSpace(_settings.RobotIP) &&
                !string.Equals(remoteIp, _settings.RobotIP, StringComparison.OrdinalIgnoreCase))
            {
                PublishStatus($"REJECTED CLIENT {remoteIp}");
                return;
            }

            try
            {
                robotClient.NoDelay = true;
                using NetworkStream stream = robotClient.GetStream();

                var readSw = Stopwatch.StartNew();
                byte[] initialMessage = await ReadInitialRobotMessageAsync(stream, token).ConfigureAwait(false);
                readSw.Stop();
                long robotReadMs = readSw.ElapsedMilliseconds;
                if (initialMessage.Length == 0)
                {
                    PublishStatus($"CONNECTED FROM {remoteIp}: NO DATA");
                    return;
                }

                int requestCode = initialMessage[0];
                string requestType = _settings.UseBinaryBatchProtocol
                    ? DescribeNachiRequestByte(requestCode)
                    : "ASCII";
                PublishStatus(_requestMonitor.RecordRequestByte(remoteIp, initialMessage, requestType), requestCode);

                if (IsLikelyText(initialMessage) && !_settings.UseBinaryBatchProtocol)
                {
                    string text = Encoding.ASCII.GetString(initialMessage).Trim();
                    PublishStatus($"ROBOT TEXT FROM {remoteIp}: {text}");
                    await SendAsciiReplyAsync(stream, remoteIp, token).ConfigureAwait(false);
                    await ReadServerFollowUpMessagesAsync(stream, remoteIp, token).ConfigureAwait(false);
                    return;
                }

                if (_settings.UseBinaryBatchProtocol && IsSupportedNachiRequestByte(requestCode))
                {
                    await DispatchNachiRequestAsync(stream, remoteIp, requestCode, robotReadMs, requestSw, token).ConfigureAwait(false);
                    await ReadServerFollowUpMessagesAsync(stream, remoteIp, token).ConfigureAwait(false);
                    PublishStatus(_requestMonitor.BuildSummary(), requestCode);
                    return;
                }

                if (!_settings.UseBinaryBatchProtocol)
                {
                    await SendAsciiReplyAsync(stream, remoteIp, token).ConfigureAwait(false);
                    await ReadServerFollowUpMessagesAsync(stream, remoteIp, token).ConfigureAwait(false);
                    return;
                }

                PublishStatus($"UNKNOWN BINARY REQUEST 0x{requestCode:X2} FROM {remoteIp}", requestCode);
            }
            catch (Exception ex) when (ex is IOException or SocketException or TimeoutException or InvalidOperationException)
            {
                PublishStatus($"ROBOT REQUEST ERROR {remoteIp}: {ex.Message}");
            }
        }
    }

    private async Task<byte[]> ReadInitialRobotMessageAsync(NetworkStream stream, CancellationToken token)
    {
        using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(token);
        timeoutCts.CancelAfter(_settings.ConnectionTimeoutMs);

        byte[] buffer = new byte[1024];
        int read = await stream.ReadAsync(buffer.AsMemory(0, buffer.Length), timeoutCts.Token).ConfigureAwait(false);
        if (read <= 0)
            return Array.Empty<byte>();

        byte[] data = new byte[read];
        Buffer.BlockCopy(buffer, 0, data, 0, read);
        return data;
    }

    private async Task ReadServerFollowUpMessagesAsync(NetworkStream stream, string remoteIp, CancellationToken token)
    {
        using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(token);
        timeoutCts.CancelAfter(Math.Min(_settings.ConnectionTimeoutMs, 3000));

        byte[] buffer = new byte[1024];
        while (!timeoutCts.Token.IsCancellationRequested)
        {
            int read;
            try
            {
                read = await stream.ReadAsync(buffer.AsMemory(0, buffer.Length), timeoutCts.Token).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                break;
            }

            if (read <= 0)
                break;

            byte[] message = new byte[read];
            Buffer.BlockCopy(buffer, 0, message, 0, read);
            if (IsLikelyText(message))
                PublishStatus($"RX TEXT FROM {remoteIp}: {Encoding.ASCII.GetString(message).Trim()}");
            else
                PublishStatus($"RX BINARY FROM {remoteIp}: {FormatBinaryPreview(message)}");
        }
    }

    private async Task SendAsciiReplyAsync(NetworkStream stream, string remoteIp, CancellationToken token)
    {
        VisionResult? result = DequeueLocalResult();
        string line = result is null
            ? "NO_DATA\r\n"
            : string.Create(CultureInfo.InvariantCulture, $"{result.X:F3},{result.Y:F3},{result.Angle:F3}\r\n");
        byte[] bytes = Encoding.ASCII.GetBytes(line);
        await stream.WriteAsync(bytes, token).ConfigureAwait(false);
        PublishStatus($"ASCII REPLY TO {remoteIp}: {(result is null ? "NO_DATA" : line.Trim())}");
    }

    /// <summary>Primary Nachi request bytes (MZ04 / USERTASK-A): all three are first-class handlers.</summary>
    private static bool IsSupportedNachiRequestByte(int requestCode) => requestCode is 0 or 1 or 2;

    private static string DescribeNachiRequestByte(int requestCode) => requestCode switch
    {
        0 => "COORDINATE_BATCH_0x00",
        1 => "COORDINATE_BATCH_0x01",
        2 => "PENDING_TOTAL_0x02",
        _ => "UNSUPPORTED"
    };

    /// <summary>Dispatch to one of three primary protocol branches (no hidden merge of 0x00 into 0x01).</summary>
    private Task DispatchNachiRequestAsync(
        NetworkStream stream,
        string remoteIp,
        int requestCode,
        long robotReadMs,
        Stopwatch requestSw,
        CancellationToken token)
    {
        return requestCode switch
        {
            0 => SendCoordinateBatchForRequestAsync(stream, remoteIp, 0, robotReadMs, requestSw, token),
            1 => SendCoordinateBatchForRequestAsync(stream, remoteIp, 1, robotReadMs, requestSw, token),
            2 => SendPendingTotalForRequestAsync(stream, remoteIp, 2, token),
            _ => Task.CompletedTask
        };
    }

    /// <summary>0x00 / 0x01 — coordinate batch from Flask/DB (same binary layout; robot may use either byte).</summary>
    private async Task SendCoordinateBatchForRequestAsync(
        NetworkStream stream,
        string remoteIp,
        int requestCode,
        long robotReadMs,
        Stopwatch requestSw,
        CancellationToken token)
    {
        var resolveSw = Stopwatch.StartNew();
        RobotCoordinateBatch batch = await ResolveBatchAsync().ConfigureAwait(false);
        resolveSw.Stop();
        byte[] batchHeader = BuildHeader(batch.Results.Count, batch.TotalBeforeSend);
        var replySw = Stopwatch.StartNew();
        await stream.WriteAsync(batchHeader, token).ConfigureAwait(false);

        int bodyBytes = 0;
        if (batch.Results.Count > 0)
        {
            byte[] body = BuildCoordinatePayload(batch.Results);
            await stream.WriteAsync(body, token).ConfigureAwait(false);
            bodyBytes = body.Length;
        }
        replySw.Stop();

        string label = requestCode == 0 ? "0x00" : "0x01";
        PublishStatus(
            $"REQ {label} COORD BATCH FROM {remoteIp}: SENT {batch.Results.Count}/{batch.TotalBeforeSend} FROM {batch.Source}",
            requestCode);
        PublishStatus(
            _requestMonitor.RecordBinaryBatchSent(
                remoteIp, requestCode, DescribeNachiRequestByte(requestCode),
                batch.Results, batch.TotalBeforeSend, batchHeader.Length, bodyBytes,
                _settings.UseServerTestCoordinates),
            requestCode);
        await SaveBatchTimingAsync(batch, requestCode, robotReadMs, replySw.ElapsedMilliseconds, requestSw.ElapsedMilliseconds, resolveSw.ElapsedMilliseconds).ConfigureAwait(false);
    }

    /// <summary>0x02 — primary path: pending total only (MZ04 feeder check, no float body).</summary>
    private async Task SendPendingTotalForRequestAsync(
        NetworkStream stream,
        string remoteIp,
        int requestCode,
        CancellationToken token)
    {
        int totalBeforeSend = await ResolvePendingTotalAsync().ConfigureAwait(false);
        byte[] header = BuildHeader(totalBeforeSend, totalBeforeSend);
        await stream.WriteAsync(header, token).ConfigureAwait(false);
        PublishStatus($"REQ 0x02 TOTAL FROM {remoteIp}: pending={totalBeforeSend} (header only)", requestCode);
        PublishStatus(
            _requestMonitor.RecordBinaryBatchSent(
                remoteIp, requestCode, DescribeNachiRequestByte(requestCode),
                Array.Empty<VisionResult>(), totalBeforeSend, header.Length, 0,
                _settings.UseServerTestCoordinates),
            requestCode);
    }

    private async Task SaveBatchTimingAsync(
        RobotCoordinateBatch batch,
        int requestCode,
        long robotReadMs,
        long tcpReplyMs,
        long robotRequestTotalMs,
        long resolveMs)
    {
        if (batch.Timing is null || SaveCycleTimingAsync is null)
            return;

        batch.Timing.RequestCode = requestCode;
        batch.Timing.RobotReadMs = robotReadMs;
        batch.Timing.TcpReplyMs = tcpReplyMs;
        batch.Timing.RobotRequestTotalMs = robotRequestTotalMs;
        if (batch.Timing.RobotFetchMs <= 0)
            batch.Timing.RobotFetchMs = resolveMs;

        try
        {
            await SaveCycleTimingAsync(batch.Timing).ConfigureAwait(false);
        }
        catch
        {
            // Timing persistence is diagnostic only; it must not affect robot TCP handling.
        }
    }

    private async Task<int> ResolvePendingTotalAsync()
    {
        if (_settings.UseServerTestCoordinates)
            return 1;

        if (FetchPendingRobotCoordinateCountAsync is not null)
        {
            int? total = await FetchPendingRobotCoordinateCountAsync().ConfigureAwait(false);
            if (total.HasValue)
                return total.Value;
        }

        return PendingCount;
    }

    private async Task<RobotCoordinateBatch> ResolveBatchAsync()
    {
        if (_settings.UseServerTestCoordinates)
        {
            var test = new VisionResult
            {
                Name = "TEST",
                X = _settings.TestCoordinateX,
                Y = _settings.TestCoordinateY,
                Angle = _settings.TestCoordinateAngle,
                Score = 1.0,
                Source = "Server Test"
            };
            return new RobotCoordinateBatch
            {
                Results = new List<VisionResult> { test },
                TotalBeforeSend = 1,
                Source = "Server Test"
            };
        }

        if (FetchRobotCoordinateBatchAsync is not null)
        {
            RobotCoordinateBatch? remote = await FetchRobotCoordinateBatchAsync(_settings.MaxResultsPerRobotRequest).ConfigureAwait(false);
            if (remote is not null)
                return remote;
        }

        lock (_queueLock)
        {
            int total = _pendingResults.Count;
            var local = new List<VisionResult>();
            while (local.Count < _settings.MaxResultsPerRobotRequest && _pendingResults.Count > 0)
                local.Add(_pendingResults.Dequeue());

            return new RobotCoordinateBatch { Results = local, TotalBeforeSend = total, Source = "Local Queue" };
        }
    }

    private byte[] BuildHeader(int count, int total)
    {
        byte[] header = new byte[8];
        WriteInt32(header, 0, count);
        WriteInt32(header, 4, total);
        return header;
    }

    private byte[] BuildCoordinatePayload(IReadOnlyList<VisionResult> results)
    {
        byte[] payload = new byte[results.Count * 12];
        int offset = 0;
        foreach (VisionResult result in results)
        {
            WriteSingle(payload, offset, (float)result.X);
            WriteSingle(payload, offset + 4, (float)result.Y);
            WriteSingle(payload, offset + 8, (float)result.Angle);
            offset += 12;
        }

        return payload;
    }

    private VisionResult? DequeueLocalResult()
    {
        lock (_queueLock)
            return _pendingResults.Count > 0 ? _pendingResults.Dequeue() : null;
    }

    private async Task SendClientPacketAsync(VisionResult data)
    {
        try
        {
            using var client = new TcpClient();
            await client.ConnectAsync(_settings.RobotIP, _settings.RobotPort).ConfigureAwait(false);
            using NetworkStream stream = client.GetStream();
            string line = string.Create(CultureInfo.InvariantCulture, $"{data.X:F3},{data.Y:F3},{data.Angle:F3}\r\n");
            byte[] bytes = Encoding.ASCII.GetBytes(line);
            await stream.WriteAsync(bytes).ConfigureAwait(false);
            PublishStatus($"CLIENT SENT {line.Trim()}");
        }
        catch (Exception ex)
        {
            PublishStatus($"CLIENT SEND FAILED: {ex.Message}");
        }
    }

    private void WriteInt32(byte[] buffer, int offset, int value)
    {
        byte[] bytes = BitConverter.GetBytes(value);
        if (_settings.UseBigEndianRobotProtocol == BitConverter.IsLittleEndian)
            Array.Reverse(bytes);
        Array.Copy(bytes, 0, buffer, offset, 4);
    }

    private void WriteSingle(byte[] buffer, int offset, float value)
    {
        byte[] bytes = BitConverter.GetBytes(value);
        if (_settings.UseBigEndianRobotProtocol == BitConverter.IsLittleEndian)
            Array.Reverse(bytes);
        Array.Copy(bytes, 0, buffer, offset, 4);
    }

    private static string FormatBinaryPreview(byte[] message)
    {
        if (message.Length == 0)
            return "<empty>";

        int preview = Math.Min(message.Length, 16);
        var hex = new StringBuilder();
        for (int i = 0; i < preview; i++)
        {
            if (i > 0) hex.Append(' ');
            hex.Append(message[i].ToString("X2", CultureInfo.InvariantCulture));
        }

        if (message.Length > preview)
            hex.Append(" ...");

        return hex.ToString();
    }

    private static bool IsLikelyText(byte[] data)
    {
        int checkedBytes = Math.Min(data.Length, 32);
        if (checkedBytes <= 1)
            return false;

        int printable = 0;
        for (int i = 0; i < checkedBytes; i++)
        {
            byte value = data[i];
            if (value is 9 or 10 or 13 or (>= 32 and <= 126))
                printable++;
        }

        return printable >= checkedBytes - 1;
    }

    private static IPAddress ParseListenAddress(string address)
    {
        if (string.IsNullOrWhiteSpace(address) || address is "0.0.0.0" or "*")
            return IPAddress.Any;

        return IPAddress.Parse(address);
    }

    private static string GetRemoteIp(TcpClient client)
    {
        return client.Client.RemoteEndPoint is IPEndPoint endpoint
            ? endpoint.Address.ToString()
            : "unknown";
    }

    private void PublishStatus(string status, int? robotRequestByte = null)
    {
        if (robotRequestByte.HasValue)
            _lastRobotRequestByte = robotRequestByte.Value;

        OnRobotDataReceived?.Invoke(this, new RobotDataEventArgs
        {
            Status = status,
            PendingCount = PendingCount,
            LastRequestCode = _lastRobotRequestByte
        });
    }

    public void Dispose()
    {
        Disconnect();
    }
}
