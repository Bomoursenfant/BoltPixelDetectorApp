using System.Data.SQLite;

namespace BoltPixelDetectorApp;

public sealed class VisionDatabase : IDisposable
{
    private readonly SQLiteConnection _connection;

    public string DatabasePath { get; }
    public string? LastError { get; private set; }

    public VisionDatabase(string databasePath)
    {
        DatabasePath = Path.GetFullPath(databasePath);
        string? directory = Path.GetDirectoryName(DatabasePath);
        if (!string.IsNullOrWhiteSpace(directory))
            Directory.CreateDirectory(directory);

        _connection = new SQLiteConnection($"Data Source={databasePath};Version=3;Busy Timeout=5000;");
        _connection.Open();
        ExecuteNonQuery("PRAGMA journal_mode=WAL;");
        ExecuteNonQuery("PRAGMA synchronous=NORMAL;");
        CreateTables();
        MigrateSchema();
    }

    private void CreateTables()
    {
        ExecuteNonQuery("""
            CREATE TABLE IF NOT EXISTS vision_results (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                object_name TEXT NOT NULL,
                x_pixel REAL NOT NULL,
                y_pixel REAL NOT NULL,
                x_robot REAL NOT NULL,
                y_robot REAL NOT NULL,
                angle_degrees REAL NOT NULL,
                confidence REAL NOT NULL,
                source TEXT DEFAULT 'VISION',
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                status TEXT DEFAULT 'SUCCESS'
            )
            """);

        ExecuteNonQuery("""
            CREATE TABLE IF NOT EXISTS robot_coordinates (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x_coordinate REAL NOT NULL,
                y_coordinate REAL NOT NULL,
                angle_degrees REAL NOT NULL,
                object_name TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                sent_to_robot INTEGER DEFAULT 0
            )
            """);

        ExecuteNonQuery("""
            CREATE TABLE IF NOT EXISTS robot_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_type TEXT NOT NULL,
                event_data TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                status TEXT DEFAULT 'LOGGED'
            )
            """);

        ExecuteNonQuery("""
            CREATE TABLE IF NOT EXISTS vision_cycle_timings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                cycle_id TEXT NOT NULL,
                snapshot_index INTEGER,
                trigger_source TEXT NOT NULL,
                request_code INTEGER,
                detection_count INTEGER NOT NULL,
                robot_ready_count INTEGER NOT NULL,
                detect_ms INTEGER NOT NULL,
                db_flask_ms INTEGER NOT NULL,
                robot_fetch_ms INTEGER NOT NULL,
                robot_read_ms INTEGER NOT NULL,
                tcp_reply_ms INTEGER NOT NULL,
                total_cycle_ms INTEGER NOT NULL,
                robot_request_total_ms INTEGER NOT NULL,
                queue_source TEXT,
                notes TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
            """);

        ExecuteNonQuery("CREATE INDEX IF NOT EXISTS idx_vision_timestamp ON vision_results(timestamp)");
        ExecuteNonQuery("CREATE INDEX IF NOT EXISTS idx_robot_sent ON robot_coordinates(sent_to_robot, timestamp)");
        ExecuteNonQuery("CREATE INDEX IF NOT EXISTS idx_cycle_timing_timestamp ON vision_cycle_timings(timestamp)");
    }

    /// <summary>Rename legacy sent_to_plc column to sent_to_robot on existing databases.</summary>
    private void MigrateSchema()
    {
        if (ColumnExists("robot_coordinates", "sent_to_plc") && !ColumnExists("robot_coordinates", "sent_to_robot"))
            ExecuteNonQuery("ALTER TABLE robot_coordinates RENAME COLUMN sent_to_plc TO sent_to_robot");

        ExecuteNonQuery("DROP INDEX IF EXISTS idx_robot_sent");
        ExecuteNonQuery("CREATE INDEX IF NOT EXISTS idx_robot_sent ON robot_coordinates(sent_to_robot, timestamp)");
        ExecuteNonQuery("CREATE INDEX IF NOT EXISTS idx_cycle_timing_timestamp ON vision_cycle_timings(timestamp)");
    }

    private bool ColumnExists(string table, string column)
    {
        using var cmd = _connection.CreateCommand();
        cmd.CommandText = $"PRAGMA table_info({table})";
        using var reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            if (string.Equals(Convert.ToString(reader["name"]), column, StringComparison.OrdinalIgnoreCase))
                return true;
        }

        return false;
    }

    public bool SaveVisionResult(VisionResult result)
    {
        try
        {
            using var cmd = _connection.CreateCommand();
            cmd.CommandText = """
                INSERT INTO vision_results
                (object_name, x_pixel, y_pixel, x_robot, y_robot, angle_degrees, confidence, source, status)
                VALUES (@name, @xPixel, @yPixel, @xRobot, @yRobot, @angle, @confidence, @source, 'SUCCESS')
                """;
            cmd.Parameters.AddWithValue("@name", result.Name);
            cmd.Parameters.AddWithValue("@xPixel", result.PixelX);
            cmd.Parameters.AddWithValue("@yPixel", result.PixelY);
            cmd.Parameters.AddWithValue("@xRobot", result.X);
            cmd.Parameters.AddWithValue("@yRobot", result.Y);
            cmd.Parameters.AddWithValue("@angle", result.Angle);
            cmd.Parameters.AddWithValue("@confidence", result.Score);
            cmd.Parameters.AddWithValue("@source", result.Source);
            cmd.ExecuteNonQuery();
            LastError = null;
            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return false;
        }
    }

    public bool SaveRobotCoordinate(VisionResult result, bool sentToRobot = false)
    {
        try
        {
            using var cmd = _connection.CreateCommand();
            cmd.CommandText = """
                INSERT INTO robot_coordinates
                (x_coordinate, y_coordinate, angle_degrees, object_name, sent_to_robot)
                VALUES (@x, @y, @angle, @name, @sent)
                """;
            cmd.Parameters.AddWithValue("@x", result.X);
            cmd.Parameters.AddWithValue("@y", result.Y);
            cmd.Parameters.AddWithValue("@angle", result.Angle);
            cmd.Parameters.AddWithValue("@name", result.Name);
            cmd.Parameters.AddWithValue("@sent", sentToRobot ? 1 : 0);
            cmd.ExecuteNonQuery();
            LastError = null;
            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return false;
        }
    }

    public List<VisionResult> GetPendingRobotCoordinates(int maxItems, bool markSent)
    {
        var results = new List<(int id, VisionResult result)>();
        using (var cmd = _connection.CreateCommand())
        {
            cmd.CommandText = """
                SELECT id, x_coordinate, y_coordinate, angle_degrees, object_name
                FROM robot_coordinates
                WHERE sent_to_robot = 0
                ORDER BY timestamp ASC
                LIMIT @limit
                """;
            cmd.Parameters.AddWithValue("@limit", maxItems);
            using var reader = cmd.ExecuteReader();
            while (reader.Read())
            {
                results.Add((Convert.ToInt32(reader["id"]), new VisionResult
                {
                    Name = Convert.ToString(reader["object_name"]) ?? "DETECTED_OBJECT",
                    X = Convert.ToDouble(reader["x_coordinate"]),
                    Y = Convert.ToDouble(reader["y_coordinate"]),
                    Angle = Convert.ToDouble(reader["angle_degrees"]),
                    Score = 1.0,
                    Source = "Local DB"
                }));
            }
        }

        if (markSent && results.Count > 0)
        {
            using var cmd = _connection.CreateCommand();
            cmd.CommandText = $"UPDATE robot_coordinates SET sent_to_robot = 1 WHERE id IN ({string.Join(",", results.Select(r => r.id))})";
            cmd.ExecuteNonQuery();
        }

        return results.Select(r => r.result).ToList();
    }

    public int CountPendingRobotCoordinates()
    {
        using var cmd = _connection.CreateCommand();
        cmd.CommandText = "SELECT COUNT(*) FROM robot_coordinates WHERE sent_to_robot = 0";
        return Convert.ToInt32(cmd.ExecuteScalar());
    }

    /// <summary>
    /// Marks the oldest N pending rows as sent (FIFO), matching Flask <c>/pending?mark_sent=1</c>.
    /// Use when the robot dequeue happens via Flask so local SQLite stays in sync.
    /// </summary>
    public int MarkOldestPendingRobotCoordinatesSent(int count)
    {
        if (count <= 0)
            return 0;

        int marked = 0;
        try
        {
            for (int i = 0; i < count; i++)
            {
                using var cmd = _connection.CreateCommand();
                cmd.CommandText = """
                    UPDATE robot_coordinates SET sent_to_robot = 1 WHERE id = (
                        SELECT id FROM robot_coordinates WHERE sent_to_robot = 0 ORDER BY timestamp ASC LIMIT 1
                    )
                    """;
                if (cmd.ExecuteNonQuery() == 0)
                    break;
                marked++;
            }

            LastError = null;
            return marked;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return marked;
        }
    }

    /// <summary>Remove all robot queue rows before enqueueing a new snapshot (latest detect only).</summary>
    public int ClearPendingRobotCoordinates()
    {
        try
        {
            using var cmd = _connection.CreateCommand();
            cmd.CommandText = "DELETE FROM robot_coordinates";
            int cleared = cmd.ExecuteNonQuery();
            LastError = null;
            return cleared;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return 0;
        }
    }

    public bool LogRobotEvent(string eventType, string? eventData = null)
    {
        try
        {
            using var cmd = _connection.CreateCommand();
            cmd.CommandText = "INSERT INTO robot_events (event_type, event_data, status) VALUES (@type, @data, 'LOGGED')";
            cmd.Parameters.AddWithValue("@type", eventType);
            cmd.Parameters.AddWithValue("@data", eventData ?? "");
            cmd.ExecuteNonQuery();
            return true;
        }
        catch
        {
            return false;
        }
    }

    public bool SaveCycleTiming(VisionCycleTiming timing)
    {
        try
        {
            using var cmd = _connection.CreateCommand();
            cmd.CommandText = """
                INSERT INTO vision_cycle_timings
                (cycle_id, snapshot_index, trigger_source, request_code, detection_count, robot_ready_count,
                 detect_ms, db_flask_ms, robot_fetch_ms, robot_read_ms, tcp_reply_ms,
                 total_cycle_ms, robot_request_total_ms, queue_source, notes)
                VALUES
                (@cycleId, @snapshotIndex, @triggerSource, @requestCode, @detectionCount, @robotReadyCount,
                 @detectMs, @dbFlaskMs, @robotFetchMs, @robotReadMs, @tcpReplyMs,
                 @totalCycleMs, @robotRequestTotalMs, @queueSource, @notes)
                """;
            cmd.Parameters.AddWithValue("@cycleId", timing.CycleId);
            cmd.Parameters.AddWithValue("@snapshotIndex", timing.SnapshotIndex);
            cmd.Parameters.AddWithValue("@triggerSource", timing.TriggerSource);
            cmd.Parameters.AddWithValue("@requestCode", timing.RequestCode.HasValue ? timing.RequestCode.Value : DBNull.Value);
            cmd.Parameters.AddWithValue("@detectionCount", timing.DetectionCount);
            cmd.Parameters.AddWithValue("@robotReadyCount", timing.RobotReadyCount);
            cmd.Parameters.AddWithValue("@detectMs", timing.DetectMs);
            cmd.Parameters.AddWithValue("@dbFlaskMs", timing.DbFlaskMs);
            cmd.Parameters.AddWithValue("@robotFetchMs", timing.RobotFetchMs);
            cmd.Parameters.AddWithValue("@robotReadMs", timing.RobotReadMs);
            cmd.Parameters.AddWithValue("@tcpReplyMs", timing.TcpReplyMs);
            cmd.Parameters.AddWithValue("@totalCycleMs", timing.TotalCycleMs);
            cmd.Parameters.AddWithValue("@robotRequestTotalMs", timing.RobotRequestTotalMs);
            cmd.Parameters.AddWithValue("@queueSource", timing.QueueSource);
            cmd.Parameters.AddWithValue("@notes", timing.Notes);
            cmd.ExecuteNonQuery();
            LastError = null;
            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return false;
        }
    }

    private void ExecuteNonQuery(string sql)
    {
        using var cmd = _connection.CreateCommand();
        cmd.CommandText = sql;
        cmd.ExecuteNonQuery();
    }

    public void Dispose()
    {
        _connection.Dispose();
    }
}
