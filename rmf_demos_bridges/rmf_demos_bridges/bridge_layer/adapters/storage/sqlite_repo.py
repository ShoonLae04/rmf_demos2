import sqlite3
from threading import Lock


class SqliteBridgeRepository:
    def __init__(self, db_path: str) -> None:
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._lock = Lock()
        self._init_schema()

    def _init_schema(self) -> None:
        with self._lock:
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS seen_events (
                    event_id TEXT PRIMARY KEY,
                    created_at INTEGER NOT NULL DEFAULT (strftime('%s','now'))
                )
                """
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS mappings (
                    work_order_id TEXT PRIMARY KEY,
                    rmf_task_id TEXT NOT NULL,
                    tenant_id TEXT NOT NULL,
                    updated_at INTEGER NOT NULL DEFAULT (strftime('%s','now'))
                )
                """
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS status_cache (
                    work_order_id TEXT PRIMARY KEY,
                    status TEXT NOT NULL,
                    updated_at INTEGER NOT NULL DEFAULT (strftime('%s','now'))
                )
                """
            )
            self._conn.commit()

    def seen_event(self, event_id: str) -> bool:
        with self._lock:
            cur = self._conn.execute(
                "SELECT 1 FROM seen_events WHERE event_id = ? LIMIT 1",
                (event_id,),
            )
            return cur.fetchone() is not None

    def mark_event_seen(self, event_id: str) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT OR IGNORE INTO seen_events(event_id) VALUES (?)",
                (event_id,),
            )
            self._conn.commit()

    def save_mapping(self, work_order_id: str, rmf_task_id: str, tenant_id: str) -> None:
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO mappings(work_order_id, rmf_task_id, tenant_id, updated_at)
                VALUES (?, ?, ?, strftime('%s','now'))
                ON CONFLICT(work_order_id) DO UPDATE SET
                    rmf_task_id=excluded.rmf_task_id,
                    tenant_id=excluded.tenant_id,
                    updated_at=excluded.updated_at
                """,
                (work_order_id, rmf_task_id, tenant_id),
            )
            self._conn.commit()

    def get_rmf_task_id(self, work_order_id: str) -> str | None:
        with self._lock:
            cur = self._conn.execute(
                "SELECT rmf_task_id FROM mappings WHERE work_order_id = ?",
                (work_order_id,),
            )
            row = cur.fetchone()
            if row is None:
                return None
            return row[0]

    def save_last_status(self, work_order_id: str, status: str) -> None:
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO status_cache(work_order_id, status, updated_at)
                VALUES (?, ?, strftime('%s','now'))
                ON CONFLICT(work_order_id) DO UPDATE SET
                    status=excluded.status,
                    updated_at=excluded.updated_at
                """,
                (work_order_id, status),
            )
            self._conn.commit()

    def get_last_status(self, work_order_id: str) -> str | None:
        with self._lock:
            cur = self._conn.execute(
                "SELECT status FROM status_cache WHERE work_order_id = ?",
                (work_order_id,),
            )
            row = cur.fetchone()
            if row is None:
                return None
            return row[0]

    def list_mappings(self) -> list[tuple[str, str, str]]:
        with self._lock:
            cur = self._conn.execute(
                "SELECT work_order_id, rmf_task_id, tenant_id FROM mappings"
            )
            rows = cur.fetchall()
            return [(str(a), str(b), str(c)) for a, b, c in rows]
