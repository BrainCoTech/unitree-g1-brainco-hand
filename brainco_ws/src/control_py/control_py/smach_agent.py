import asyncio
import yaml
import rclpy

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sm_interfaces.srv import SmachCmd, HighCmd
from sm_interfaces.msg import SmachParam, HighCmdParam
from std_srvs.srv import Trigger

from loguru import logger

from control_py.udp_service_call.robot_agent import (
    RobotAgent,
    CommandRegistry,
    CommandSpec
)

# ---------- CONFIG ----------
CONFIG_PATH = "src/control_py/config/smach_config.yaml"

with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    smach_cfg = yaml.safe_load(f)

mode_cfg = smach_cfg["mode"]
MODE = mode_cfg.get("current_mode", "basic")

STATES_CONFIG_PATH = (
    mode_cfg["states_config"]["SmFolder"]
    + mode_cfg["states_config"][MODE]
)


def _flatten_active_bt_children(raw_children):
    children = []

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


def load_active_behavior_tree_nodes(cfg):
    active_bt_cfg = cfg.get("behavior_trees", {}).get("active", {})
    root_nodes = set()
    internal_nodes = set()
    root_final_actions = {}

    for _, tree_def in active_bt_cfg.items():
        if str(tree_def.get("type", "")).strip().lower() != "sequence":
            continue

        children = _flatten_active_bt_children(tree_def.get("children", []))
        if not children:
            continue

        root_nodes.add(children[0])
        internal_nodes.update(children[1:])
        root_final_actions[children[0]] = children[-1]

    return root_nodes, internal_nodes, root_final_actions


def normalize_active_expected_duration(value):
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


# -------------------------------------------------
# ROS2 CORE NODE
# -------------------------------------------------

class RobotCore(Node):

    def __init__(self):

        super().__init__("smach_whole_body_agent_node")

        # ---------- Smach ----------
        self.smach_client = self.create_client(
            SmachCmd,
            "/lifecycle_command"
        )

        self.available_trans_client = self.create_client(
            Trigger,
            "/get_available_transitions"
        )

        self.current_state_client = self.create_client(
            Trigger,
            "/get_current_state"
        )

        # ---------- HighCmd ----------
        self.highcmd_client = self.create_client(
            HighCmd,
            "g1_high_cmd"
        )

        # wait services
        while not self.smach_client.wait_for_service(timeout_sec=1.0):
            logger.info("waiting for /lifecycle_command...")

        # while not self.highcmd_client.wait_for_service(timeout_sec=1.0):
        #     logger.info("waiting for g1_high_cmd...")

        logger.info("All ROS2 services ready")

        # ---------- load smach yaml ----------
        with open(STATES_CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        service_keys = {}

        for event_name, trans_def in cfg.get("main_trans", {}).items():
            trans_id = trans_def.get("id")
            if trans_id:
                service_keys[trans_id] = event_name

        active_children = {}

        for state in cfg.get("structure", []):
            if isinstance(state, dict) and "active" in state:
                active_children = state["active"].get("children", {})
                break

        self.action_name = active_children
        (
            self.bt_root_actions,
            self.bt_internal_actions,
            self.bt_root_final_actions,
        ) = load_active_behavior_tree_nodes(cfg)
        raw_expected_duration = cfg.get("active_expected_duration", {})
        self.active_expected_duration = {
            str(action_idx): normalize_active_expected_duration(value)
            for action_idx, value in raw_expected_duration.items()
        }
        self.hidden_action_reg = {
            str(action_idx) for action_idx in cfg.get("hidden_action_reg", [])
        }
        self.handside_required_actions = {
            str(action_idx) for action_idx in cfg.get("handside_required_actions", [])
        }

        for k in active_children.keys():
            action_key = str(k)
            if action_key in self.bt_internal_actions:
                continue
            service_keys[action_key] = f"start_{action_key}"

        self.service_keys = service_keys
        self.state_final = False

    def get_action_sleep(self, action_key):
        action_key = str(action_key)
        sleep_action_key = self.bt_root_final_actions.get(action_key, action_key)
        return self.active_expected_duration.get(sleep_action_key)

    # -------------------------------------------------
    # SMACH SERVICE
    # -------------------------------------------------

    async def call_smach(self, data: str, handside="", extra=""):

        req = SmachCmd.Request()

        req.data = data

        req.param = SmachParam()
        req.param.handside = handside
        req.param.extra = extra

        loop = asyncio.get_event_loop()

        fut = loop.run_in_executor(
            None,
            self.smach_client.call,
            req
        )

        res = await fut

        if res:

            msg = res.message

            if "finalized" in msg.lower():
                self.state_final = True

            return msg

        return None

    async def get_current_state(self):

        req = Trigger.Request()

        loop = asyncio.get_event_loop()

        fut = loop.run_in_executor(
            None,
            self.current_state_client.call,
            req
        )

        res = await fut

        return res.message if res else None

    # -------------------------------------------------
    # HIGH CMD SERVICE
    # -------------------------------------------------

    async def call_highcmd(self, cmd: str, value: str):

        req = HighCmd.Request()

        req.data = "cmd"

        param = HighCmdParam()
        param.cmd = cmd
        param.value = value

        req.param = param

        future = self.highcmd_client.call_async(req)

        result = await future

        return result


# -------------------------------------------------
# HANDLERS
# -------------------------------------------------

class SmachHandler:

    def __init__(self, core: RobotCore):
        self.core = core

    async def run_action(self, key: str, handside="both", extra=""):

        service = self.core.service_keys[key]

        result_msg = await self.core.call_smach(
            service,
            handside,
            extra
        )

        new_state = await self.core.get_current_state()

        return {
            "result": result_msg,
            "params": {
                "handside": handside,
                "extra": extra
            },
            "current_state": new_state
        }


class HighCmdHandler:

    def __init__(self, core: RobotCore):
        self.core = core

    async def run_cmd(self, cmd: str, value: str):

        res = await self.core.call_highcmd(cmd, value)
    
        return {
            "result": res.message,
            "success": res.success,
            "current_state": res.current_state
        }


# -------------------------------------------------
# ROS2 SPIN
# -------------------------------------------------

async def ros2_spin(node: Node):

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_event_loop()

    await loop.run_in_executor(
        None,
        executor.spin
    )


# -------------------------------------------------
# MAIN
# -------------------------------------------------

async def main():

    rclpy.init()

    core = RobotCore()

    smach_handler = SmachHandler(core)
    highcmd_handler = HighCmdHandler(core)

    asyncio.create_task(ros2_spin(core))

    # -------------------------------------------------
    # registry
    # -------------------------------------------------

    internal_reg = CommandRegistry()
    action_reg = CommandRegistry()

    internal_services = {
        k: v for k, v in core.service_keys.items()
        if not k.isdigit()
    }

    action_services = {
        k: v for k, v in core.service_keys.items()
        if k.isdigit()
    }

    # -------------------------------------------------
    # internal agent commands
    # -------------------------------------------------

    for key, service in internal_services.items():

        async def _handler(args, k=key):

            handside = args.get("handside", "both")
            extra = args.get("extra", "")

            return await smach_handler.run_action(
                k,
                handside,
                extra
            )

        internal_reg.register(
            CommandSpec(
                name=f"smach_{key}",
                desc=f"Trigger service {service}",
                args={
                    "handside": {
                        "type": "str",
                        "desc": "both/left/right",
                        "required": False
                    },
                    "extra": {
                        "type": "str",
                        "desc": "extra param",
                        "required": False
                    }
                },
            ),
            _handler
        )

    # -------------------------------------------------
    # smach action commands
    # -------------------------------------------------

    for key, service in action_services.items():
        if key in core.hidden_action_reg:
            continue

        async def _handler(args, k=key):

            handside = args.get("handside", "both")
            extra = args.get("extra", "")

            return await smach_handler.run_action(
                k,
                handside,
                extra
            )

        action_name = core.action_name.get(int(key), "UnknownAction")
        handside_required = key in core.handside_required_actions
        safety = {}
        sleep = core.get_action_sleep(key)
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
                        "required": handside_required
                    },
                    "extra": {
                        "type": "str",
                        "desc": "extra param",
                        "required": False
                    }
                },
                safety=safety,
            ),
            _handler
        )

    # -------------------------------------------------
    # high cmd velocity commands
    # -------------------------------------------------

    velocity_commands = {
        "h1_forward": "0.15 0 0 1",
        "h2_backward": "-0.15 0 0 1",
        "h3_left": "0 0.15 0 1",
        "h4_right": "0 -0.15 0 1",
        "h5_turn_left": "0 0 0.5 1",
        "h6_turn_right": "0 0 -0.5 1",
    }

    def make_velocity_handler(value):

        async def _handler(args):

            return await highcmd_handler.run_cmd(
                "set_velocity",
                value
            )

        return _handler

    for name, value in velocity_commands.items():

        action_reg.register(
            CommandSpec(
                name=name,
                desc=f"set velocity {value}"
            ),
            make_velocity_handler(value)
        )

    # -------------------------------------------------
    # Agents
    # -------------------------------------------------

    internal_agent = RobotAgent(
        robot_id="unitree_internal",
        name="SmachInternalAgent",
        registry=internal_reg,
        tcp_port=43210,
        discoverable=False
    )

    action_agent = RobotAgent(
        robot_id="unitree_g1",
        name="RobotAgent",
        registry=action_reg,
        tcp_port=46000
    )

    await asyncio.gather(
        internal_agent.start(),
        action_agent.start()
    )

    logger.info("Agents started")

    await asyncio.gather(
        internal_agent.serve_forever(),
        action_agent.serve_forever()
    )


if __name__ == "__main__":
    asyncio.run(main())
