from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, List, Optional

Handler = Callable[[Dict[str, Any]], Awaitable[Dict[str, Any]]]

@dataclass(frozen=True)
class CommandSpec:
    name: str
    desc: str = ""
    args: Dict[str, Any] = None
    returns: Dict[str, Any] = None
    safety: Dict[str, Any] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "desc": self.desc,
            "args": self.args or {},
            "returns": self.returns or {"type": "object"},
            "safety": self.safety or {},
        }

class CommandRegistry:
    def __init__(self) -> None:
        self._specs: Dict[str, CommandSpec] = {}
        self._handlers: Dict[str, Handler] = {}

    def register(self, spec: CommandSpec, handler: Handler) -> None:
        if not spec.name:
            raise ValueError("command name is empty")
        if spec.name in self._specs:
            raise ValueError(f"duplicate command: {spec.name}")
        self._specs[spec.name] = spec
        self._handlers[spec.name] = handler

    def list_specs(self) -> List[Dict[str, Any]]:
        return [s.to_dict() for s in self._specs.values()]

    def get_handler(self, name: str) -> Optional[Handler]:
        return self._handlers.get(name)