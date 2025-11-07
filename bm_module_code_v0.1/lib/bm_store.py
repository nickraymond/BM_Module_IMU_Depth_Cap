# /lib/bm_store.py — tiny FS helpers for CircuitPython
import os, json

def ensure_dir(path: str):
    if not path or path == "/":
        return
    parts = [p for p in path.split("/") if p]
    cur = ""
    for p in parts:
        cur += "/" + p
        try:
            os.listdir(cur)
        except Exception:
            try:
                os.mkdir(cur)
            except Exception:
                pass

def read_json(path: str, defaults: dict):
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return dict(defaults)

def write_json_atomic(path: str, obj: dict) -> bool:
    d = path.rsplit("/", 1)[0] or "/"
    ensure_dir(d)
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(obj, f)
            f.flush()
        os.rename(tmp, path)
        return True
    except Exception:
        try:
            os.remove(tmp)
        except Exception:
            pass
        return False
