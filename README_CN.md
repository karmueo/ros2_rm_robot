<div align="right">

<!-- 本文档说明睿尔曼机械臂包及其在 FastUMI 工作区的共享环境使用方式。 -->

[中文简体](https://github.com/RealManRobot/ros2_rm_robot/blob/jazzy/README_CN.md)|
[English](https://github.com/RealManRobot/ros2_rm_robot/blob/jazzy/README.md)

</div>

# ros2_rm_robot

该功能包的主要作用为提供机械臂的ROS2支持，以下为使用环境。

* 当前支持的机械臂有RM65系列、RM75系列、ECO62系列、ECO63系列、ECO65系列、RML63系列、GEN72系列、RX75人形双臂系列，详细可参考网址 [RealMan robots](http://www.realman-robotics.com/)。
* 版本1.7.0.
* 基于机械臂控制器版本1.7.3。
* 基于的Ubuntu版本为24.04，ROS2版本为Jazzy。

下面为功能包安装使用教程。

## 1.搭建环境

---

在使用功能包之前我们首先需要进行如下操作。

* 1.[安装ROS2](#安装ROS2)
* 2.[安装Moveit2](#安装Moveit2)
* 3.[配置功能包环境](#配置功能包环境)
* 4.[编译](#编译)

### 安装ROS2

---

我们提供了ROS2的安装脚本ros2_install.sh，该脚本位于rm_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。

```
sudo bash ros2_install.sh
```

如果不想使用脚本安装也可以参考网址 [ROS2_INSTALL](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

### 安装Moveit2

---

我们提供了Moveit2的安装脚本moveit2_install.sh，该脚本位于rm_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。

```
sudo bash moveit2_install.sh
```

如果不想使用脚本安装也可以参考网址 [Moveit2_INSTALL](https://moveit.ros.org/install-moveit2/binary/)进行安装。

### 配置功能包环境

---

该脚本位于rm_driver功能包中的lib文件夹下，在实际使用时我们需要移动到该路径执行如下指令。

```
sudo bash lib_install.sh
```

### 编译

---

本项目已包含 `ros2_ws/src/ros2_rm_robot`，无需重新创建工作空间或复制功能包。所有机械臂包归属共享的 NumPy 1 环境；先按[工作区说明](../../README.md#两套共享-python-环境)创建环境，再从项目根目录执行：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv-numpy1/bin/activate
PATH="$VIRTUAL_ENV/bin:/opt/ros/jazzy/bin:/usr/bin:/bin" \
  python -m colcon build --build-base build \
  --symlink-install --packages-select rm_ros_interfaces
source install/setup.bash
PATH="$VIRTUAL_ENV/bin:/opt/ros/jazzy/bin:/usr/bin:/bin" \
  python -m colcon build --build-base build \
  --symlink-install --packages-select rm_moveit2 --cmake-clean-cache \
  --cmake-args -DBoost_DIR=/usr/lib/x86_64-linux-gnu/cmake/Boost-1.83.0
PATH="$VIRTUAL_ENV/bin:/opt/ros/jazzy/bin:/usr/bin:/bin" \
  python -m colcon build --build-base build \
  --symlink-install --packages-skip tracker_teleoperated
source install/setup.bash
```

本机同时安装了 `/usr/local` 下的 Boost 1.91 和系统 Boost 1.83。上述 `Boost_DIR` 指向与系统 `boost_system` 匹配的版本；`--cmake-clean-cache` 用于清除之前缓存的 Boost 路径。该路径适用于当前 Ubuntu x86-64 环境，其他机器请改为本机系统 Boost 的 `BoostConfig.cmake` 所在目录。命令中的 `PATH` 仅对相应的构建进程生效，避免 Anaconda 的库干扰 ROS 链接，同时保留 NumPy 1 环境的 Python。

编译完成后即可进行功能包的运行操作。

## 2.功能运行

---

功能包简介

1. 安装与环境配置([rm_install](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_install))

* 该功能包为机械臂使用辅助功能包，主要作用为介绍功能包使用环境安装与搭建方式，功能包的依赖库安装和功能包编译方法。

3. 硬件驱动([rm_driver](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_driver))

* 该功能包为机械臂的ROS2底层驱动功能包，其作用为订阅和发布机械臂底层相关话题信息。

5. 启动([rm_bringup](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_bringup))

* 该功能包为机械臂的节点启动功能包，其作用为快速启动多节点复合的机械臂功能。

6. 模型描述([rm_description](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_description))

* 该功能包为机械臂模型描述功能包，其作用为提供机械臂模型文件和模型加载节点，并为其他功能包提供机械臂关节间的坐标变换关系。

7. ROS消息接口([rm_ros_interfaces](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_ros_interfaces))

* 该功能包为机械臂的消息文件功能包，其作用为提供机械臂适配ROS2的所有控制消息和状态消息。

8. MoveIt2控制程序([rm_moveit2](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_moveit2))

* 该功能包提供基于MoveIt2的控制示例程序，支持真实机械臂与仿真机械臂的关节运动、位姿目标控制和笛卡尔轨迹控制。

9. Moveit2配置([rm_moveit2_config](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_moveit2_config))

* 该功能包为机械臂的moveit2适配功能包，其作用为适配和实现各系列机械臂的moveit2规划控制功能，主要包括虚拟机械臂控制和真实机械臂控制两部分控制功能。

10. Moveit2与硬件驱动通信连接([rm_config](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_control))

* 该功能包为底层驱动功能包（rm_driver）和moveit2功能包（rm_moveit2_config）之间的通信连接功能包，主要功能为将moveit2的规划点进行细分然后通过透传的形式传递给底层驱动功能包控制机械臂运动。

11. Gazebo仿真机械臂控制([rm_gazebo](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_gazebo))

* 该功能包为gazebo仿真机械臂功能包，主要功能为在gazebo仿真环境中显示机械臂模型，可通过moveit2对仿真的机械臂进行规划控制。

12. 使用案例([rm_examples](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_example))

* 该功能包为机械臂的一些使用案例，主要功能为实现机械臂的一些基本的控制功能和运动功能的使用案例。

13. 技术文档([rm_docs](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy/rm_doc))

* 该功能包为介绍文档的功能包，其主要包括为对整体的功能包内容和使用方式进行总体介绍的文档和对每个功能包中的内容和使用方式进行详细介绍的文档。

以上为当前的主要功能包，每个功能包都有其独特的作用，详情请参考 `rm_doc` 功能包中的相关文档。

### 2.1运行虚拟机械臂

---

在项目根目录执行如下指令，可以启动gazebo显示仿真机械臂，并同时启动moveit2进行仿真机械臂的规划操控。

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/.venv-numpy1/bin/activate
source ros2_ws/install/setup.bash
ros2 launch rm_bringup rm_<arm_type>_gazebo.launch.py
```

<arm_type>需要使用65、75、eco62、eco63、eco65、63、63_III、gen72、gen72_II字符进行代替。RX75 请直接使用 `rm_rx75_6fb_gazebo.launch.py` 或 `rm_rx75_6fb_v_gazebo.launch.py`。如使用RM65机械臂时，命令如下。

```
ros2 launch rm_bringup rm_65_gazebo.launch.py
```

启动成功后即可使用moveit2进行虚拟机械臂的控制。

### 2.2控制真实机械臂

---

在项目根目录执行如下指令，可以启动机械臂硬件驱动，并同时启动moveit2进行机械臂的规划操控。

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/.venv-numpy1/bin/activate
source ros2_ws/install/setup.bash
ros2 launch rm_bringup rm_<arm_type>_bringup.launch.py
```

<arm_type>需要使用65、75、eco62、eco63、eco65、63、63_III、gen72、gen72_II字符进行代替。RX75 请直接使用 `rm_rx75_6fb_bringup.launch.py` 或 `rm_rx75_6fb_v_bringup.launch.py`。如使用RM65机械臂时，命令如下。

```
ros2 launch rm_bringup rm_65_bringup.launch.py
```

启动成功后即可使用moveit2进行真实机械臂的控制。

#### 更换电脑或网络时配置 RM75 的 IP

`rm_75_bringup.launch.py` 会加载 `rm_driver` 的
[`rm_75_config.yaml`](rm_driver/config/rm_75_config.yaml)。在**运行驱动的电脑**上，
先确认机械臂控制器的实际 IP、TCP 端口，以及该电脑连接机械臂的网卡 IP，
再修改配置文件中的以下参数：

| 参数 | 填写内容 | 当前配置示例 |
| --- | --- | --- |
| `arm_ip` | 机械臂控制器的实际 IP，驱动向此地址建立 TCP 连接 | `192.168.1.18` |
| `tcp_port` | 控制器的 TCP 服务端口，需与控制器设置一致 | `8080` |
| `udp_ip` | **运行 `rm_driver` 的电脑**接收机械臂状态的网卡 IP；多网卡时选连接机械臂的网卡 | `192.168.1.10` |
| `udp_port` | 该电脑接收机械臂主动上报的 UDP 端口，需与控制器上报设置一致 | `8089` |

`arm_ip` 与 `udp_ip` 分别指向机械臂和驱动电脑。若将驱动迁到另一台电脑，
请把 `udp_ip` 改为**新驱动电脑**在机械臂网络中的地址；若机械臂的地址也变化，
同时修改 `arm_ip`。电脑网卡的 IP、子网掩码和路由应先在操作系统网络设置中
配置，使电脑能够访问机械臂。以下命令在驱动电脑上检查网卡和到机械臂的
路由；将地址替换为实际的 `arm_ip`：

```bash
ip -br -4 addr
ip route get 192.168.1.18
nc -vz -w 2 192.168.1.18 8080
```

`ip route get` 输出中的 `src` 应是驱动电脑连接机械臂所用的本机地址，并与
`udp_ip` 对应。若持续出现 `Waiting for connect`，先核对控制器是否上电、
网线及控制器实际 IP，再检查该 TCP 端口；这条日志发生在初始姿态运动之前。
TCP 已连通但收不到 `/joint_states` 或 `/rm_driver/udp_feedback_valid` 时，
再检查 `udp_ip`、`udp_port` 和该电脑的 UDP 防火墙规则。

本工作区使用 `--symlink-install`，已安装的 RM75 配置链接到上述源文件；
修改后停止并重新启动 launch 即可。若在另一台电脑使用普通复制安装，
修改源文件后需重新构建 `rm_driver` 并加载新的 `install/setup.bash`。
机械臂驱动与遥操节点分布在不同电脑时，`udp_ip` 仍填写**驱动电脑**的地址；
两台电脑的 ROS 2 节点还需能够互相发现，并使用相同的 `ROS_DOMAIN_ID`。
其他机械臂型号请修改 `rm_driver/config/` 下对应型号的 YAML 文件。

确认连接及关节反馈前，先在已加载工作区环境的终端禁用自动运动启动：

```bash
ros2 launch rm_bringup rm_75_bringup.launch.py move_to_initial_pose:=false
```

驱动连接成功后，在另一终端检查 `ros2 topic echo /joint_states --once` 和
`ros2 topic echo /rm_driver/udp_feedback_valid --once`；后者应收到 `data: true`。
确认反馈并停止本次 launch 后，再按下节启用初始姿态运动。
`move_to_initial_pose:=true` 会在反馈就绪后发送真实运动指令。

#### RM75 启动时移动到初始姿态

需要在启动 RM75 实机驱动后自动移动到预设关节姿态时，在已按上文加载
ROS 2 Jazzy、`.venv-numpy1` 和工作区 `install/setup.bash` 的终端执行：

```bash
ros2 launch rm_bringup rm_75_bringup.launch.py move_to_initial_pose:=true
```

`move_to_initial_pose` 默认为 `false`，省略该参数时不会执行初始姿态运动。
设为 `true` 后，启动文件会运行一次 `rm_75_initial_pose` 节点。目标关节角按
`joint1` 至 `joint7` 顺序为 `[0°, 20°, 0°, 70°, 0°, 90°, 90°]`，配置在
`rm_bringup/launch/rm_75_bringup.launch.py` 中。节点等待完整且新鲜的
`/joint_states`、有效的 `/rm_driver/udp_feedback_valid` 和驱动对
`/rm_driver/movej_cmd` 的订阅，然后以 20% 速度发送一次阻塞式 MoveJ。
已处于目标姿态时不会重复发送运动指令；等待就绪超过 30 秒或等待执行结果
超过 120 秒时，节点报错退出。启动文件的其他组件仍按正常流程运行。

此命令会控制真实机械臂运动。执行前确认 RM75 型号与关节配置正确、运动路径
和工作范围内无人及障碍物，并确保急停可用。需要同时打开 RViz2 时，可在命令
末尾追加 `use_rviz:=true`。

### 安全提示

---

在使用机械臂时，为保证使用者安全，请参考如下操作规范。

* 每次使用前检查机械臂的安装情况，包括固定螺丝是否松动，机械臂是否存在震动、晃动的情况。
* 机械臂在运行过程中，人不可处于机械臂落下或工作范围内，也不可将其他物体放到机械臂动作的安全范围内。
* 在不使用机械臂时，应将机械臂置于安全位置，防止震动时机械臂跌落而损坏或砸伤其他物体。
* 在不使用机械臂时应及时断开机械臂电源。

<!-- v1.7.0 -->
