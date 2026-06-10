import os
import sqlite3
from datetime import datetime

from flask import Flask, jsonify, request
from flask_cors import CORS


app = Flask(__name__)
CORS(app)

DB_PATH = os.environ.get(
    "VISION_DB_PATH",
    os.path.join(os.path.dirname(__file__), "vision_results.db"),
)


def db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    os.makedirs(os.path.dirname(DB_PATH) or ".", exist_ok=True)
    with db() as conn:
        conn.executescript(
            """
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
            );
            CREATE TABLE IF NOT EXISTS robot_coordinates (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x_coordinate REAL NOT NULL,
                y_coordinate REAL NOT NULL,
                angle_degrees REAL NOT NULL,
                object_name TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                sent_to_robot INTEGER DEFAULT 0
            );
            CREATE TABLE IF NOT EXISTS robot_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_type TEXT NOT NULL,
                event_data TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                status TEXT DEFAULT 'LOGGED'
            );
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
            );
            """
        )
        migrate_schema(conn)


def migrate_schema(conn):
    cols = [row[1] for row in conn.execute("PRAGMA table_info(robot_coordinates)")]
    if "sent_to_plc" in cols and "sent_to_robot" not in cols:
        conn.execute("ALTER TABLE robot_coordinates RENAME COLUMN sent_to_plc TO sent_to_robot")
    conn.execute("DROP INDEX IF EXISTS idx_robot_sent")
    conn.execute(
        "CREATE INDEX IF NOT EXISTS idx_robot_sent ON robot_coordinates(sent_to_robot, timestamp)"
    )
    conn.execute(
        "CREATE INDEX IF NOT EXISTS idx_cycle_timing_timestamp ON vision_cycle_timings(timestamp)"
    )


@app.get("/health")
def health():
    return jsonify({"status": "ok", "database": DB_PATH})


@app.post("/api/vision/result")
def save_vision_result():
    data = request.get_json(force=True) or {}
    with db() as conn:
        conn.execute(
            """
            INSERT INTO vision_results
            (object_name, x_pixel, y_pixel, x_robot, y_robot, angle_degrees, confidence, source, status)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'SUCCESS')
            """,
            (
                data.get("object_name", "DETECTED_OBJECT"),
                data.get("x", 0),
                data.get("y", 0),
                data.get("x_robot", data.get("x", 0)),
                data.get("y_robot", data.get("y", 0)),
                data.get("angle", 0),
                data.get("confidence", 0),
                data.get("source", "BoltPixelDetectorApp"),
            ),
        )
    return jsonify({"ok": True})


@app.post("/api/robot/coordinates/clear-pending")
def clear_pending_coordinates():
    """Drop all queue rows so robot_coordinates shows only the latest detect snapshot."""
    with db() as conn:
        cur = conn.execute("DELETE FROM robot_coordinates")
        cleared = cur.rowcount
    return jsonify({"ok": True, "cleared": cleared})


@app.post("/api/robot/coordinates")
def save_robot_coordinates():
    data = request.get_json(force=True) or {}
    with db() as conn:
        cur = conn.execute(
            """
            INSERT INTO robot_coordinates
            (x_coordinate, y_coordinate, angle_degrees, object_name, timestamp, sent_to_robot)
            VALUES (?, ?, ?, ?, ?, 0)
            """,
            (
                data.get("x_coordinate", 0),
                data.get("y_coordinate", 0),
                data.get("angle_degrees", 0),
                data.get("object_name", "DETECTED_OBJECT"),
                data.get("timestamp", datetime.utcnow().isoformat()),
            ),
        )
    return jsonify({"ok": True, "id": cur.lastrowid})


@app.get("/api/robot/coordinates/pending")
def get_pending_coordinate():
    mark_sent = request.args.get("mark_sent", "0") == "1"
    with db() as conn:
        row = conn.execute(
            """
            SELECT id, x_coordinate, y_coordinate, angle_degrees, object_name, timestamp
            FROM robot_coordinates
            WHERE sent_to_robot = 0
            ORDER BY timestamp ASC
            LIMIT 1
            """
        ).fetchone()
        if row is None:
            return ("", 204)
        if mark_sent:
            conn.execute("UPDATE robot_coordinates SET sent_to_robot = 1 WHERE id = ?", (row["id"],))
    return jsonify(
        {
            "id": row["id"],
            "x": row["x_coordinate"],
            "y": row["y_coordinate"],
            "angle": row["angle_degrees"],
            "object_name": row["object_name"],
            "timestamp": row["timestamp"],
        }
    )


@app.post("/api/robot/event")
def save_robot_event():
    data = request.get_json(force=True) or {}
    with db() as conn:
        conn.execute(
            "INSERT INTO robot_events (event_type, event_data, status) VALUES (?, ?, 'LOGGED')",
            (data.get("event_type", "event"), data.get("event_data", "")),
        )
    return jsonify({"ok": True})


@app.post("/api/vision/timing")
def save_vision_timing():
    data = request.get_json(force=True) or {}
    with db() as conn:
        conn.execute(
            """
            INSERT INTO vision_cycle_timings
            (cycle_id, snapshot_index, trigger_source, request_code, detection_count, robot_ready_count,
             detect_ms, db_flask_ms, robot_fetch_ms, robot_read_ms, tcp_reply_ms,
             total_cycle_ms, robot_request_total_ms, queue_source, notes, timestamp)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                data.get("cycle_id", ""),
                data.get("snapshot_index", 0),
                data.get("trigger_source", "Manual"),
                data.get("request_code"),
                data.get("detection_count", 0),
                data.get("robot_ready_count", 0),
                data.get("detect_ms", 0),
                data.get("db_flask_ms", 0),
                data.get("robot_fetch_ms", 0),
                data.get("robot_read_ms", 0),
                data.get("tcp_reply_ms", 0),
                data.get("total_cycle_ms", 0),
                data.get("robot_request_total_ms", 0),
                data.get("queue_source", ""),
                data.get("notes", ""),
                data.get("timestamp", datetime.utcnow().isoformat()),
            ),
        )
    return jsonify({"ok": True})


@app.get("/api/statistics")
def statistics():
    with db() as conn:
        total = conn.execute("SELECT COUNT(*) FROM vision_results").fetchone()[0]
        pending = conn.execute("SELECT COUNT(*) FROM robot_coordinates WHERE sent_to_robot = 0").fetchone()[0]
        avg = conn.execute("SELECT COALESCE(AVG(confidence), 0) FROM vision_results").fetchone()[0]
    return jsonify(
        {
            "total_results": total,
            "today_results": total,
            "average_confidence": avg,
            "pending_robot_coordinates": pending,
        }
    )


if __name__ == "__main__":
    init_db()
    app.run(
        host=os.environ.get("VISION_API_HOST", "0.0.0.0"),
        port=int(os.environ.get("VISION_API_PORT", "5000")),
    )
