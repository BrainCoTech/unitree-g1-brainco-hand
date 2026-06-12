import rclpy
from rclpy.node import Node
import subprocess
import os

from sm_interfaces.srv import HighCmd


class G1HighCmdService(Node):

    def __init__(self):
        super().__init__('high_cmd_node')

        self.srv = self.create_service(
            HighCmd,
            'g1_high_cmd',
            self.callback
        )

        self.bin_dir = os.path.expanduser("~/unitree_sdk2/build/bin")

        self.get_logger().info("G1 HighCmd service ready")

    def callback(self, request, response):

        cmd_name = request.param.cmd
        value = request.param.value

        # 生成命令
        if value != "":
            arg = f"--{cmd_name}={value}"
        else:
            arg = f"--{cmd_name}"

        cmd = [
            "./g1_loco_client",
            "--network_interface=eth0",
            arg
        ]

        try:
            subprocess.run(
                cmd,
                cwd=self.bin_dir,
                check=True
            )

            response.success = True
            response.message = f"executed {cmd_name} {value}"
            response.current_state = "ok"

        except subprocess.CalledProcessError as e:
            response.success = False
            response.message = str(e)
            response.current_state = "error"

        return response


def main(args=None):

    rclpy.init(args=args)

    node = G1HighCmdService()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()