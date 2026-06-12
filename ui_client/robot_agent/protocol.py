import json
import hashlib
from typing import Any, Dict, List

def json_dumps(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False)

def json_loads(s: str) -> Any:
    return json.loads(s)

def commands_hash(commands: List[Dict[str, Any]]) -> str:
    payload = json_dumps(commands).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:16]