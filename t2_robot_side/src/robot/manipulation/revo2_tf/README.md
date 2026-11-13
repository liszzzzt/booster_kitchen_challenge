# revo2_tf —— 把 Revo2 手指坐标系放进 /tf、/tf_static

Booster 的 `robot_state_publisher` 用的是 `/opt/booster/Gait/configs/T2/robot.urdf`，只有 T2 的 31 个关节，
TF 树到 `left/right_wrist_roll_link` 就结束了，手指不在里面。这里补上手指：

```
/revo2/hand_state ──► revo2_joint_state.py ──► /revo2/left/joint_states  ──► robot_state_publisher (ns /revo2/left)  ──► /tf, /tf_static
   (插件, 20 Hz)        (closure → 关节角)     /revo2/right/joint_states ──► robot_state_publisher (ns /revo2/right) ──►
/joint_states ───────────────┘ (只取时间戳)     /revo2/joint_states (两只手合在一起)
```

## 运行（T2WS 终端，Studio 仿真已在跑、revo2_hand 插件已装）

```bash
ros2 launch /ws/t2_robot_side/src/robot/manipulation/revo2_tf/revo2_tf.launch.py
```

自动启动：`T2WS/windows/T2WS/docker/compose.yaml` 里的 `revo2_tf` 服务会在 `docker compose up -d` 时在后台跑这条 launch
（容器 `t2ws-revo2-tf`，看日志 `docker compose logs -f revo2_tf`，停止 `docker compose stop revo2_tf`）。手动运行时先停掉它，避免重复发布。

参数：`frame_prefix:=robot1/`（只有 Studio 给坐标系加了前缀时才需要；单机器人场景不用），`robot:=robot1`，`rate_hz:=50.0`。

## 真机（topics:=t2）

真机上 T2WS 只能看到机器人端中继出来的 `/t2/...` 话题（见 `robot/relay_bridges/revo2.py`），这时：

```bash
ros2 launch /ws/t2_robot_side/src/robot/manipulation/revo2_tf/revo2_tf.launch.py topics:=t2
```

输入改为 `/t2/revo2/hand_state` + `/t2/joint_states`，输出改为 `/t2/tf` + `/t2/tf_static`（MoveIt、RViz、Nav2 用的就是这两个）。
compose 里的 `revo2_tf` 服务用环境变量切换：启动前设 `REVO2_TF_TOPICS=t2`（默认 `sim`）。
Booster Studio 里保持默认 `sim`：手指坐标系先进仿真的 `/tf`，再由机器人端 TfRelay 复制到 `/t2/tf`；两种模式不要同时开，否则 `/t2/tf` 里每个手指坐标系会有两份。

## 新增的坐标系（每只手，以 right 为例）

| 坐标系 | 父坐标系 | 话题 |
|---|---|---|
| `right_base_link`（手掌） | `right_wrist_roll_link` | /tf_static |
| `right_thumb_metacarpal_link` → `right_thumb_proximal_link` → `right_thumb_distal_link` | 依次 | /tf |
| `right_{index,middle,ring,pinky}_proximal_link` → `..._distal_link` | `right_base_link` | /tf |
| `right_*_tip_link`（指尖点）、`right_*_touch_link`（指腹） | 各指末节 | /tf_static |

检查：

```bash
ros2 topic echo /revo2/right/joint_states --once
ros2 run tf2_ros tf2_echo right_wrist_roll_link right_index_tip_link
```

## 文件

| 文件 | 作用 |
|---|---|
| `revo2_joint_state.py` | 节点：closure 0..1 → 关节角。近端 = 下限 + closure×(上限−下限)（与插件 PD 目标一致），末端按 URDF `<mimic>`（拇指 ×1.0，四指 ×1.155）。纯换算在 `Revo2HandModel` 里，不依赖 ROS；以后接真机 Revo2 SDK，只要换输入。 |
| `revo2_tf.launch.py` | 启动上面的节点 + 两个 robot_state_publisher（左右手各一个，因为 URDF 只能有一个根）。 |
| `revo2_description/revo2_{left,right}.urdf` | 从 Booster 官方 `T2_31dof_Revo2Hand.urdf`（booster_assets 12a97516）截出的单手 URDF，根 = `<side>_wrist_roll_link`，运动学原样保留，去掉了网格。 |
| `revo2_description/make_hand_urdf.py` | 重新生成上面两个 URDF；`--meshes <目录>` 可保留网格给 RViz 显示。 |

## 已验证

- 22 个手指运动坐标系在开手和半握两种姿态下，与 Studio 实际 MuJoCo 模型（robot_t2_revo2.basset）中的手指 body 位姿一致，位置误差 1e-7 m。
- 仿真里手掌 body 与 `wrist_roll_link` 完全重合（位置、姿态误差 0），所以挂在 `wrist_roll_link` 下面是对的。
- 时间戳取自 `/joint_states`（与手臂 TF 一致）；`/joint_states` 0.5 s 没消息时改用本节点时钟。
