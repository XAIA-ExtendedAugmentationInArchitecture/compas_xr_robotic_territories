import os
import time
from typing import Dict, Any
from compas.data import json_dumps, json_loads  # <- COMPAS serializer/deserializer
import threading


class SimpleLogger:
    def __init__(self, dir_path: str, sub_folder_name: str, participant_name: str, *,
                 print_each: bool = False, flush_every: int = 1):
        self.participant_name = participant_name
        self.dir_path = dir_path
        self.sub_folder_name = sub_folder_name
        self.print_each = print_each
        self.flush_every = max(1, flush_every)
        self._count = 0
        self._lock = threading.Lock()   # <-- add this

        # Paths
        session_start = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        folder = os.path.join(self.dir_path, self.sub_folder_name)
        os.makedirs(folder, exist_ok=True)
        self.file_path = os.path.join(
            folder, f"{session_start}_{self.sub_folder_name}_{self.participant_name}_log.ndjson"
        )

        self._fh = open(self.file_path, "a", buffering=1, encoding="utf-8")

        header = {
            "type": "session_start",
            "ts": time.time(),
            "ts_str": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "participant": self.participant_name,
            "subfolder": self.sub_folder_name,
        }
        self._fh.write(json_dumps(header) + "\n")  # <- COMPAS dumps

    def log(self, message: Dict[str, Any]):
        now = time.time()
        entry = dict(message)
        entry["ts"] = now
        entry["ts_str"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        self._fh.write(json_dumps(entry) + "\n")  # <- COMPAS dumps
        self._count += 1

        if self.print_each:
            print(f"[log {self._count}] {entry.get('event', entry)}")

        if (self._count % self.flush_every) == 0:
            self._fh.flush()

    def log_message(self, message):
        now = time.time()
        entry = dict(getattr(message, "data", {}))  # safe copy from Message.data
        entry["__type__"] = message.__class__.__name__
        entry["ts"] = now
        # entry["topic"] = topic
        entry["ts_str"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        line = json_dumps(entry) + "\n"
        with self._lock:
            self._fh.write(line)
            self._count += 1
            if self.print_each:
                print(f"[log {self._count}] {entry.get('event', entry['__type__'])}")
            if (self._count % self.flush_every) == 0:
                self._fh.flush()

    def log_with_mode(self, mode_name:str, message: Dict[str, Any]):
        now = time.time()
        entry = dict(message)
        entry["ts"] = now
        entry["ts_str"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        entry["mode"] = mode_name

        self._fh.write(json_dumps(entry) + "\n")  # <- COMPAS dumps
        self._count += 1

        if self.print_each:
            print(f"[log {self._count}] {entry.get('event', entry)}")

        if (self._count % self.flush_every) == 0:
            self._fh.flush()

    def save_log(self):
        if not self._fh.closed:
            self._fh.flush()
            self._fh.close()
        print(f"[Log Saved] {self.file_path}")