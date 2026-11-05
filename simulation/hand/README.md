# T2 Revo2 for Booster Studio

## 1. 机器人资产
把 robot_t2_revo2.basset 拷到：
  C:\Users\<用户名>\AppData\Roaming\Booster Studio\simulator-library\assets\
（重启 Studio 后资产库里出现 "T2 Revo2"）

## 2. 手指驱动插件（装进虚拟机器人容器）
先在 Studio 里启动虚拟机器人，查容器名：docker ps   （形如 <序列号>-t2）
PowerShell 在本目录执行：
  powershell -ExecutionPolicy Bypass -File .\install_plugin.ps1 -Container <容器名>
它会把 plugin\ 拷进容器、注册插件，然后 docker restart 容器。
（容器被 Studio 重建后要重装；重启不影响。卸载：加 -Uninstall）

## 3. 使用
Studio 打开场景 → 资产库里把 T2 Revo2 添加进场景（运行时添加；Studio 只允许内置机器人保存进场景，所以每次打开场景都要重新添加）。
ROS 2（容器重启后 T2WS 要 docker compose up -d --force-recreate）：
  ros2 topic pub --once /revo2/hand_cmd std_msgs/msg/String "{data: '{\"side\":\"right\",\"preset\":\"power\"}'}"
  ros2 topic echo /revo2/hand_state --once
指令：side = left/right/both；preset = open ready power pinch point fist（可加 scale）；
      或 closure / 顶层手指名：thumb_rotation thumb_flex index middle ring pinky（0 张开 .. 1 闭合）；可选 robot。
代码：revo2/hand.py 里 RosRevo2Hand("right")：preset() / set() / open() / state()；make_hand(robot, side) 真机走 SDK、Studio 走话题。

## 4. 重新生成资产（可选）
tools/make_revo2_basset.py --builtin <资产库>\assets\robot_t2.basset --revo2 <booster_assets 的 T2_31dof_Revo2Hand 目录> --out robot_t2_revo2.basset
（Revo2 模型来自 https://github.com/BoosterRobotics/booster_assets/tree/main/robots/T2/T2_31dof_Revo2Hand）

## 兼容性
在 Booster Studio 内置 T2 资产 rev 2026.08.2、仿真 MuJoCo 3.10.0 下验证。Studio 升级后若 install_in_container.sh 报 "layout changed"，需要按新版源码调整。
