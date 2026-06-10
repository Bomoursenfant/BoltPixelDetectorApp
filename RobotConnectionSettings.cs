using System.Text.Json;

namespace BoltPixelDetectorApp;

public sealed class RobotConnectionSettings
{
    public const string ModeServer = "Server";
    public const string ModeClient = "Client";
    public const string ProtocolAsciiLine = "AsciiLine";
    public const string ProtocolNachiBinaryBatch = "NachiBinaryBatch";

    public string ConnectionMode { get; set; } = ModeServer;
    public string PcListenIP { get; set; } = "0.0.0.0";
    public int PcListenPort { get; set; } = 48951;
    public string RobotIP { get; set; } = "192.168.100.133";
    public int RobotPort { get; set; } = 48951;
    public bool RestrictToRobotIP { get; set; } = true;
    public int ConnectionTimeoutMs { get; set; } = 5000;
    public bool AutoReconnect { get; set; } = true;
    public string CoordinateProtocol { get; set; } = ProtocolNachiBinaryBatch;
    public int MaxResultsPerRobotRequest { get; set; } = 12;
    public bool UseBigEndianRobotProtocol { get; set; }
    public double TestCoordinateX { get; set; } = 123.45;
    public double TestCoordinateY { get; set; } = 67.89;
    public double TestCoordinateAngle { get; set; } = 12.34;
    public bool UseServerTestCoordinates { get; set; }
    public string FlaskApiUrl { get; set; } = "http://192.168.100.103:5000";
    public bool EnableFlaskApi { get; set; } = true;
    public int FlaskTimeoutMs { get; set; } = 10000;
    public string DatabasePath { get; set; } = "vision_results.db";
    public bool EnableLocalDatabase { get; set; } = true;

    private static string ConfigPath => Path.Combine(AppContext.BaseDirectory, "RobotConnectionSettings.json");

    public bool IsServerMode => !string.Equals(ConnectionMode, ModeClient, StringComparison.OrdinalIgnoreCase);

    public bool UseBinaryBatchProtocol =>
        string.Equals(CoordinateProtocol, ProtocolNachiBinaryBatch, StringComparison.OrdinalIgnoreCase);

    public static RobotConnectionSettings Load()
    {
        try
        {
            if (File.Exists(ConfigPath))
            {
                var settings = JsonSerializer.Deserialize<RobotConnectionSettings>(
                    File.ReadAllText(ConfigPath),
                    new JsonSerializerOptions { PropertyNameCaseInsensitive = true });
                if (settings is not null)
                {
                    settings.Normalize();
                    return settings;
                }
            }
        }
        catch
        {
            // Defaults keep the app usable if the config is corrupt.
        }

        return new RobotConnectionSettings();
    }

    public void Save()
    {
        Normalize();
        File.WriteAllText(ConfigPath, JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true }));
    }

    public string GetResolvedDatabasePath()
    {
        if (Path.IsPathRooted(DatabasePath))
            return DatabasePath;

        return Path.Combine(AppContext.BaseDirectory, DatabasePath);
    }

    public bool IsValidRobotConfig()
    {
        return IsServerMode
            ? IsValidPort(PcListenPort) && !string.IsNullOrWhiteSpace(PcListenIP)
            : IsValidPort(RobotPort) && !string.IsNullOrWhiteSpace(RobotIP);
    }

    public bool IsValidFlaskConfig()
    {
        return !EnableFlaskApi || Uri.IsWellFormedUriString(FlaskApiUrl, UriKind.Absolute);
    }

    private void Normalize()
    {
        ConnectionMode = string.Equals(ConnectionMode, ModeClient, StringComparison.OrdinalIgnoreCase)
            ? ModeClient
            : ModeServer;
        CoordinateProtocol = string.Equals(CoordinateProtocol, ProtocolNachiBinaryBatch, StringComparison.OrdinalIgnoreCase)
            ? ProtocolNachiBinaryBatch
            : ProtocolAsciiLine;
        PcListenIP = string.IsNullOrWhiteSpace(PcListenIP) ? "0.0.0.0" : PcListenIP.Trim();
        RobotIP = string.IsNullOrWhiteSpace(RobotIP) ? "192.168.100.133" : RobotIP.Trim();
        FlaskApiUrl = string.IsNullOrWhiteSpace(FlaskApiUrl) ? "http://192.168.100.103:5000" : FlaskApiUrl.TrimEnd('/');
        PcListenPort = ClampPort(PcListenPort, 48951);
        RobotPort = ClampPort(RobotPort, 48951);
        ConnectionTimeoutMs = Math.Clamp(ConnectionTimeoutMs, 500, 60000);
        FlaskTimeoutMs = Math.Clamp(FlaskTimeoutMs, 500, 60000);
        MaxResultsPerRobotRequest = Math.Clamp(MaxResultsPerRobotRequest, 1, 12);
        DatabasePath = string.IsNullOrWhiteSpace(DatabasePath) ? "vision_results.db" : DatabasePath.Trim();
    }

    private static bool IsValidPort(int port) => port is > 0 and <= 65535;

    private static int ClampPort(int port, int fallback) => IsValidPort(port) ? port : fallback;
}
