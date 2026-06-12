## 灵巧手安装
1. 使用合适的手腕转接件，将强脑灵巧手安装在宇树G1的手臂末端，螺丝固定。
2. 连接双485串口线，分别连接到左右手的 485 接口和宇树 G1 的 USB-485 转换板的 `485.0`, `485.1` 信号端口，将 USB-485 转换板供电端连接到 G1 顶部 `2` 号[电气接口](https://support.unitree.com/home/zh/G1_developer/about_G1)，type-c通信端连接到 `6` 号电气接口。头部相机插入任意 type-c 接口。
3. 确保连接牢固，避免在机器人运动过程中松动或脱落。
<p align="center">
  <img src="images/tutorial_connect.jpg" width="300">
</p>

## 固定头部 (相机)
G1 头部设计为被动关节，允许前后活动，以防止摔倒时头部结构件断裂。为固定头部相机位置，我们设计了[颈部支撑插件](cad/head_camera_bracket.STEP)，以防止行走时因振动产生头部位置偏移。

**注意:** 有摔倒风险且无需相机时，建议**移除**支撑插件以保护头部结构件。

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="images/tutorial_fix_head_1.jpg" width="200">
      </td>
      <td align="center">
        <img src="images/tutorial_fix_head_2.jpg" width="200">
      </td>
    </tr>
  </table>
</div>

## 机器人启动
1. 宇树G1开机，可参照[宇树文档中心|操作指南](https://support.unitree.com/home/zh/G1_developer/quick_start)。接电时，灵巧手手背指示灯亮起，手指自动复位。
2. 开机后等待宇树G1进入**零力矩模式(L2 + Y)**，使用遥控器，使机器人依次进入**阻尼模式(L2 + B)** -> **锁定站立模式(L2 + UP)** -> **(可选)常规运控模式(R1 + X)**，上肢低层运控与走跑高层运控不冲突。


## 远程连接
参考[宇树文档中心|快速开发](https://support.unitree.com/home/zh/G1_developer/quick_development)。
1. 使用网线连接G1和计算机，将计算机以太网IP设置为与宇树G1同网段 `192.168.123.XXX`。

2. 远程 SSH 访问 G1（默认密码`123`）
```sh
ssh unitree@192.168.123.164
```

3. 打开新的终端，输入`1`(即选择ROS环境为Foxy)，按下回车，进入ROS2环境