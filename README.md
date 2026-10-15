# cr100机械臂规划接口说明

机器人模型文件已独立放置在同级 ROS 包 `robot_description` 中。该包负责维护 URDF/Xacro 和 Mesh 文件；本包继续负责 MoveIt 配置、IK 服务和规划节点。

## 工作空间

本文使用 `MOMA_WS` 表示 ROS 工作空间根目录。请根据实际环境设置该变量，
不要直接使用其他机器上的绝对路径：

```bash
export MOMA_WS=你的工作空间目录
cd "$MOMA_WS"
```

如果工作空间名称或位置不同，只需要修改 `MOMA_WS`，后续命令无需改动。

## 编译

首次使用或修改代码后，先编译工作空间：

```bash
cd "$MOMA_WS"
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake --pkg moveit_whole_body_ik -j4
source "$MOMA_WS/devel/setup.bash"
```

## 启动 MoveIt

```bash
cd "$MOMA_WS"
source /opt/ros/noetic/setup.bash
source "$MOMA_WS/devel/setup.bash"
roslaunch moveit_whole_body_ik real_robot_moveit.launch
```

> **注意：** 上述启动命令不会启动用于避障的 FAST-LIO 建图节点。

启动后确认服务存在：

```bash
rosservice list | grep /move_to_pose
```

## 发送目标位姿

另开一个终端，执行：

```bash
source "$MOMA_WS/devel/setup.bash"

rosservice call /move_to_pose "{x: 1.0, y: 0.0, z: 2.0,
  qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

目标字段说明：

| 字段 | 含义 |
| --- | --- |
| `x, y, z` | 目标位置，单位为米 |
| `qx, qy, qz, qw` | 目标姿态四元数 |

四元数应尽量归一化。例如：

```text
qx=0.0, qy=0.0, qz=0.0, qw=1.0
```

表示单位姿态。
