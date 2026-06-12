(# Robot Controller

简要说明：这是一个演示用的机器人控制示例项目，包含一个机器人代理（agent）和一个客户端（client）。

主要文件：
- `robot/main.py`：示例机器人 Agent 实现，注册了 `enable`、`disable`、`move_joint` 等命令并在本地启动 TCP 服务（默认端口 46000）。
- `client.py`：示例客户端，使用 UDP 广播发现网络中的机器人实例，然后通过 TCP 与其交互，列出并执行命令。

运行要求：
- Python 3.8+
- 无需额外第三方包（只使用标准库），但请确保从项目根目录运行以便本地包 `robot_agent` 可被正确导入。

快速开始（最简单流程）：
1. 在一台终端启动机器人代理（先启动 agent）：

```bash
python robot/main.py
```

2. 在另一台终端运行客户端以发现并与机器人交互：

```bash
python client.py
```

运行说明：
- Discovery（广播）端口：UDP 37020；客户端回复端口：37021。
- 机器人默认 TCP 服务端口：46000（在 `robot/main.py` 中通过 `RobotAgent(..., tcp_port=46000)` 指定）。
- 客户端会广播 `discover` 消息并等待 `announce` 响应，随后连接第一个发现到的机器人并尝试获取命令列表并执行演示命令。

示例命令与参数：
- `enable`: 启用机械臂，返回 `{"enabled": true}`。
- `disable`: 禁用机械臂，返回 `{"enabled": false}`。
- `move_joint`: 示例移动命令，参数示例 `{"joint": 1, "angle": 0.5, "speed": 0.5}`，在未启用时会抛出 `not_enabled` 错误。

开发与调试提示：
- 如果在开发环境中无法使用广播发现（如不同子网或受限网络），可以修改 `client.py` 中直接使用 `RobotClient(host, port)` 连接已知 IP。
- 请在项目根目录运行命令，以确保相对导入（`robot_agent` 包）工作正常。
- 若需将机器人作为包运行，可在 `robot/` 下添加 `__init__.py`，之后使用 `python -m robot.main` 运行。

下一步（可选）：
- 运行快速 smoke 测试：在两台终端分别启动 `python robot/main.py` 和 `python client.py`，观察客户端输出是否列出命令并能执行演示命令。

