import argparse
import asyncio
import contextlib
import json
from pathlib import Path
from typing import Any, Iterable

import yaml

from robot_agent.registry import CommandRegistry, CommandSpec


ROOT_DIR = Path(__file__).resolve().parents[1]
SMACH_CONFIG_PATH = ROOT_DIR / "brainco_ws/src/control_py/config/smach_config.yaml"


def flatten_active_bt_children(raw_children: Iterable[Any]) -> list[str]:
    children: list[str] = []

    for item in raw_children:
        if isinstance(item, (list, tuple)):
            children.extend(str(child) for child in item)
            continue

        if isinstance(item, dict):
            group_children = None
            for group_key in ("parallel", "group"):
                if group_key in item:
                    group_children = item[group_key]
                    break
            if group_children is not None:
                children.extend(str(child) for child in group_children)
                continue

        children.append(str(item))

    return children


def load_active_behavior_tree_nodes(
    cfg: dict[str, Any]
) -> tuple[set[str], set[str], dict[str, str]]:
    active_bt_cfg = cfg.get("behavior_trees", {}).get("active", {})
    root_nodes: set[str] = set()
    internal_nodes: set[str] = set()
    root_final_actions: dict[str, str] = {}

    for tree_def in active_bt_cfg.values():
        if str(tree_def.get("type", "")).strip().lower() != "sequence":
            continue

        children = flatten_active_bt_children(tree_def.get("children", []))
        if not children:
            continue

        root_nodes.add(children[0])
        internal_nodes.update(children[1:])
        root_final_actions[children[0]] = children[-1]

    return root_nodes, internal_nodes, root_final_actions


def normalize_active_expected_duration(value: Any) -> float | str | None:
    if value is None:
        return None

    if isinstance(value, str):
        normalized = value.strip()
        if not normalized:
            return None
        if normalized.lower() == "auto":
            return "auto"
        try:
            return float(normalized)
        except ValueError:
            return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class MockSmachBackend:
    def __init__(self) -> None:
        smach_cfg = self._load_yaml(SMACH_CONFIG_PATH)
        mode_cfg = smach_cfg["mode"]
        mode = mode_cfg.get("current_mode", "basic")
        states_config_path = (
            ROOT_DIR
            / "brainco_ws"
            / mode_cfg["states_config"]["SmFolder"]
            / mode_cfg["states_config"][mode]
        )
        cfg = self._load_yaml(states_config_path)

        self.current_state = "init_state"
        self.service_keys: dict[str, str] = {}
        self.action_name: dict[int, str] = {}
        self.hidden_action_reg = {
            str(action_idx) for action_idx in cfg.get("hidden_action_reg", [])
        }
        self.handside_required_actions = {
            str(action_idx) for action_idx in cfg.get("handside_required_actions", [])
        }
        raw_expected_duration = cfg.get("active_expected_duration", {})
        self.active_expected_duration = {
            str(action_idx): normalize_active_expected_duration(value)
            for action_idx, value in raw_expected_duration.items()
        }
        self.initial_active_key = "0"

        for event_name, trans_def in cfg.get("main_trans", {}).items():
            trans_id = trans_def.get("id")
            if trans_id:
                self.service_keys[str(trans_id)] = event_name

        active_children = {}
        for state in cfg.get("structure", []):
            if isinstance(state, dict) and "active" in state:
                active_cfg = state["active"]
                active_children = active_cfg.get("children", {})
                self.initial_active_key = str(active_cfg.get("initial", "0"))
                break

        self.action_name = {
            int(key): str(value) for key, value in active_children.items()
        }

        _, bt_internal_actions, self.bt_root_final_actions = load_active_behavior_tree_nodes(cfg)

        for key in active_children.keys():
            action_key = str(key)
            if action_key in bt_internal_actions:
                continue
            self.service_keys[action_key] = f"start_{action_key}"

    @staticmethod
    def _load_yaml(path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    @staticmethod
    def _format_active_state(action_key: str) -> str:
        normalized_key = str(action_key).strip()
        return f"active_{normalized_key}" if normalized_key else "active_unknown"

    async def run_smach_command(
        self,
        key: str,
        *,
        handside: str = "both",
        extra: str = "",
    ) -> dict[str, Any]:
        service = self.service_keys[key]

        if key == "f":
            self.current_state = "config_state"
            result_msg = "mock configure completed"
        elif key == "a":
            self.current_state = self._format_active_state(self.initial_active_key)
            result_msg = "mock activate completed"
        elif key == "d":
            self.current_state = "config_state"
            result_msg = "mock deactivate completed"
        elif key == "s":
            self.current_state = "finalized"
            result_msg = "mock shutdown completed"
        else:
            action_name = self.action_name.get(int(key), "UnknownAction")
            self.current_state = self._format_active_state(key)
            result_msg = f"mock action completed: {action_name}"

        print(
            "[mock-smach]",
            f"service={service}",
            f"key={key}",
            f"handside={handside}",
            f"extra={extra!r}",
            f"current_state={self.current_state}",
            flush=True,
        )

        await asyncio.sleep(0.05)

        return {
            "result": result_msg,
            "params": {
                "handside": handside,
                "extra": extra,
            },
            "current_state": self.current_state,
        }

    async def run_highcmd(self, cmd: str, value: str) -> dict[str, Any]:
        print(
            "[mock-highcmd]",
            f"cmd={cmd}",
            f"value={value}",
            f"current_state={self.current_state}",
            flush=True,
        )

        await asyncio.sleep(0.05)

        return {
            "result": f"mock {cmd} accepted",
            "success": True,
            "current_state": self.current_state,
        }

    def get_action_sleep(self, action_key: str) -> float | str | None:
        action_key = str(action_key)
        sleep_action_key = self.bt_root_final_actions.get(action_key, action_key)
        return self.active_expected_duration.get(sleep_action_key)

    def build_registries(self) -> tuple[CommandRegistry, CommandRegistry]:
        internal_reg = CommandRegistry()
        action_reg = CommandRegistry()

        internal_services = {
            key: value for key, value in self.service_keys.items() if not key.isdigit()
        }
        action_services = {
            key: value for key, value in self.service_keys.items() if key.isdigit()
        }

        for key, service in internal_services.items():

            async def handler(args: dict[str, Any], k: str = key) -> dict[str, Any]:
                return await self.run_smach_command(
                    k,
                    handside=args.get("handside", "both"),
                    extra=args.get("extra", ""),
                )

            internal_reg.register(
                CommandSpec(
                    name=f"smach_{key}",
                    desc=f"Trigger service {service}",
                    args={
                        "handside": {
                            "type": "str",
                            "desc": "both/left/right",
                            "required": False,
                        },
                        "extra": {
                            "type": "str",
                            "desc": "extra param",
                            "required": False,
                        },
                    },
                ),
                handler,
            )

        for key, _service in action_services.items():
            if key in self.hidden_action_reg:
                continue

            async def handler(args: dict[str, Any], k: str = key) -> dict[str, Any]:
                return await self.run_smach_command(
                    k,
                    handside=args.get("handside", "both"),
                    extra=args.get("extra", ""),
                )

            action_name = self.action_name.get(int(key), "UnknownAction")
            handside_required = key in self.handside_required_actions
            safety = {}
            sleep = self.get_action_sleep(key)
            if sleep is not None:
                safety["sleep"] = sleep
            action_reg.register(
                CommandSpec(
                    name=f"a{key}_{action_name}",
                    desc=f"Trigger {action_name}",
                    args={
                        "handside": {
                            "type": "str",
                            "desc": "both/left/right" if handside_required else "",
                            "required": handside_required,
                        },
                        "extra": {
                            "type": "str",
                            "desc": "extra param",
                            "required": False,
                        },
                    },
                    safety=safety,
                ),
                handler,
            )

        velocity_commands = {
            "h1_forward": "0.15 0 0 1",
            "h2_backward": "-0.15 0 0 1",
            "h3_left": "0 0.15 0 1",
            "h4_right": "0 -0.15 0 1",
            "h5_turn_left": "0 0 0.5 1",
            "h6_turn_right": "0 0 -0.5 1",
        }

        for name, value in velocity_commands.items():

            async def handler(
                _args: dict[str, Any],
                cmd_value: str = value,
            ) -> dict[str, Any]:
                return await self.run_highcmd("set_velocity", cmd_value)

            action_reg.register(
                CommandSpec(
                    name=name,
                    desc=f"set velocity {value}",
                ),
                handler,
            )

        return internal_reg, action_reg


class MockTcpAgent:
    def __init__(
        self,
        *,
        name: str,
        registry: CommandRegistry,
        tcp_port: int,
        host: str = "0.0.0.0",
    ) -> None:
        self.name = name
        self.registry = registry
        self.tcp_port = tcp_port
        self.host = host
        self._server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(
            self._handle_client,
            host=self.host,
            port=self.tcp_port,
        )

        addrs = ", ".join(
            str(sock.getsockname()) for sock in (self._server.sockets or [])
        )
        print(f"[mock-agent:{self.name}] listening on {addrs}", flush=True)

    async def serve_forever(self) -> None:
        if self._server is None:
            raise RuntimeError("mock agent not started")

        async with self._server:
            await self._server.serve_forever()

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peer = writer.get_extra_info("peername")
        print(f"[mock-agent:{self.name}] client connected: {peer}", flush=True)

        try:
            while True:
                line = await reader.readline()
                if not line:
                    break

                try:
                    req = json.loads(line.decode("utf-8"))
                except Exception:
                    await self._send(
                        writer,
                        {"type": "error", "ok": False, "err": "bad_json"},
                    )
                    continue

                req_type = req.get("type")
                req_id = req.get("req_id")

                if req_type == "ping":
                    await self._send(
                        writer,
                        {"type": "pong", "req_id": req_id, "ok": True},
                    )
                    continue

                if req_type == "get_commands":
                    await self._send(
                        writer,
                        {
                            "type": "commands",
                            "req_id": req_id,
                            "ok": True,
                            "schema_version": "1.0",
                            "commands": self.registry.list_specs(),
                        },
                    )
                    continue

                if req_type == "exec":
                    cmd = req.get("cmd")
                    args = req.get("args", {}) or {}
                    handler = self.registry.get_handler(cmd)

                    if handler is None:
                        await self._send(
                            writer,
                            {
                                "type": "result",
                                "req_id": req_id,
                                "ok": False,
                                "data": None,
                                "err": "unknown_cmd",
                            },
                        )
                        continue

                    try:
                        data = await handler(args)
                        await self._send(
                            writer,
                            {
                                "type": "result",
                                "req_id": req_id,
                                "ok": True,
                                "data": data,
                                "err": None,
                            },
                        )
                    except Exception as exc:
                        await self._send(
                            writer,
                            {
                                "type": "result",
                                "req_id": req_id,
                                "ok": False,
                                "data": None,
                                "err": f"exception:{exc}",
                            },
                        )
                    continue

                await self._send(
                    writer,
                    {
                        "type": "error",
                        "req_id": req_id,
                        "ok": False,
                        "err": "unknown_type",
                    },
                )
        finally:
            print(f"[mock-agent:{self.name}] client disconnected: {peer}", flush=True)
            writer.close()
            with contextlib.suppress(Exception):
                await writer.wait_closed()

    async def _send(self, writer: asyncio.StreamWriter, payload: dict[str, Any]) -> None:
        writer.write((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
        await writer.drain()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mock smach_agent for ui_client testing.",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port1", type=int, default=43210)
    parser.add_argument("--port2", type=int, default=46000)
    return parser.parse_args()


async def main() -> None:
    args = parse_args()

    backend = MockSmachBackend()
    internal_reg, action_reg = backend.build_registries()

    internal_agent = MockTcpAgent(
        name="SmachInternalAgentMock",
        registry=internal_reg,
        tcp_port=args.port1,
        host=args.host,
    )
    action_agent = MockTcpAgent(
        name="RobotAgentMock",
        registry=action_reg,
        tcp_port=args.port2,
        host=args.host,
    )

    await asyncio.gather(
        internal_agent.start(),
        action_agent.start(),
    )

    print(
        f"Mock agents ready. Connect ui_client to host and ports: {args.port1}, {args.port2}",
        flush=True,
    )

    await asyncio.gather(
        internal_agent.serve_forever(),
        action_agent.serve_forever(),
    )


if __name__ == "__main__":
    asyncio.run(main())
