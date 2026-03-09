import os
import json
from datetime import datetime, timezone

def now_iso():
    return datetime.now(timezone.utc).isoformat()

class Logger:
    def __init__(self, base_dir="logs"):
        self.base_dir = base_dir
        os.makedirs(self.base_dir, exist_ok=True)


    def _path(self, name):
        return os.path.join(self.base_dir, f"{name}_log.jsonl")


    def _write(self, name, entry: dict):
        entry["ts"] = now_iso()
        with open(self._path(name), "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")


    # helpers:
    def motion(
            self,
            cmd_id,
            method,
            status,
            level = "INFO",
            target_q = None,
            actual_q = None,
            duration = None,
            extra = None):
        entry = {
            "cmd_id": cmd_id,
            "method": method,
            "status": status,
            "level": level,
            "target_q": target_q,
            "actual_q": actual_q,
            "duration": duration
        }
        if extra:
            entry["extra"] = extra
        self._write("motion", entry)

    def gripper(
            self,
            cmd_id,
            method,
            status,
            level = "INFO",
            message = None,
            extra = None):
        entry = {
            "cmd_id": cmd_id,
            "method": method,
            "status": status,
            "level": level,
            "message": message
        }
        if extra:
            entry["extra"] = extra
        self._write("gripper", entry)

    def camera(
            self,
            cmd_id,
            method,
            status,
            level = "INFO",
            data = None,
            extra = None):
        
        entry = {
            "cmd_id": cmd_id,
            "method": method,
            "status": status,
            "level": level,
            "data": data
        }
        if extra:
            entry["extra"] = extra
        self._write("camera", entry)


    def comm(
            self,
            cmd_id,
            direction,
            level = "INFO",
            data = None,
            extra = None
    ):
        entry = {
            "cmd_id": cmd_id,
            "direction": direction,
            "level": level,
            "data": data
        }
        if extra:
            entry["extra"] = extra
        self._write("comm", entry)


    def event(
            self,
            type,
            level,
            message = None,
            cmd_id = None,
            extra = None):
        entry = {
            "type": type,
            "level": level,
            "message": message,
            "cmd_id": cmd_id
        }
        if extra:
            entry["extra"] = extra
        self._write("event", entry)



if __name__ == "__main__":
    Logger().motion("moveJ_0001", "moveJ", "In Progress")
    Logger().event("file_check", "DEBUG", f"Checking if file exists")