import asyncio
from robot_agent import RobotAgent, CommandRegistry, CommandSpec

class ArmCore:
    def __init__(self):
        self.enabled = False

    async def enable(self, args):
        self.enabled = True
        return {"enabled": True}

    async def disable(self, args):
        self.enabled = False
        return {"enabled": False}

    async def move_joint(self, args):
        if not self.enabled:
            raise RuntimeError("not_enabled")
        joint = int(args["joint"])
        angle = float(args["angle"])
        speed = float(args.get("speed", 0.5))
        # TODO: 调你的真实 SDK / 串口 / CAN
        await asyncio.sleep(0.05)
        return {"joint": joint, "angle": angle, "speed": speed}

async def main():
    core = ArmCore()
    reg = CommandRegistry()

    # 注册指令集（spec + handler）
    reg.register(
        CommandSpec(
            name="enable",
            desc="Enable actuators"
        ),
        core.enable,
    )

    reg.register(
        CommandSpec(
            name="disable",
            desc="Disable actuators"
        ),
        core.disable,
    )

    reg.register(
        CommandSpec(
            name="move_joint",
            desc="Move one joint",
            args={
                "joint": {
                    "type": "int",
                    "desc": "Joint index (0-based)",
                    "required": True,
                },
                "angle": {
                    "type": "float",
                    "desc": "Target angle (rad)",
                    "required": True,
                },
                "speed": {
                    "type": "float",
                    "desc": "Move speed",
                    "required": False,
                    "default": 0.5,
                },
            },
            returns={
                "joint": "int",
                "angle": "float",
                "speed": "float",
            },
            safety="enable_required",
        ),
        core.move_joint,
    )


    agent = RobotAgent(robot_id="arm-001", name="DemoArm", registry=reg, tcp_port=46000)
    await agent.start()
    await agent.serve_forever()

if __name__ == "__main__":
    asyncio.run(main())