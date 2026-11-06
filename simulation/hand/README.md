# T2 + Revo2 灵巧手 —— Booster Studio 仿真配置

让 Booster Studio 里的 T2 带上两只 BrainCo Revo2 手（每只 6 个电机），并通过 ROS 2 话题控制手指。
验证环境：Booster Studio 内置 T2 资产 rev `2026.08.2`、仿真 MuJoCo `3.10.0`、Windows + Docker Desktop。

```
simulation/hand/
  builtin_replacement/robot_t2.basset   替换内置 T2 用的资产（方式 A）
  robot_t2_revo2.basset                 独立的用户资产 "T2 Revo2"（方式 B）
  plugin/revo2_hand/                    手指驱动物理插件（装进虚拟机器人容器）
  plugin/install_in_container.sh        容器内安装 / 卸载插件
  install_plugin.ps1                    PowerShell 一键安装插件
  tools/patch_builtin_t2.py             替换 / 还原内置 T2（方式 A）
  tools/make_revo2_basset.py            重新生成两个 .basset
  revo2/hand.py                         Python 手部接口（RosRevo2Hand / SdkRevo2Hand / make_hand）
```

## 原理（为什么这么做）
- Studio 的场景（`.bscene`）**只能保存内置机器人资产**；用户资产只能在编辑器里临时添加，不能存进场景、也不能随场景导入。
  所以要让 `kitchen_v3_T2` 等场景一打开就是带手的机器人，只能替换内置 T2（方式 A）。
- 机器人资产必须恰好是 T2 的 31 个执行器，所以资产里**没有手指执行器**；手挂在躯干末尾，用 weld 焊到 `*_wrist_roll_link`，
  手指由容器里的物理插件 `revo2_hand` 用 PD 力矩驱动（远端指节按 URDF mimic 联动）。
- 插件对所有场景自动启用；场景里没有 Revo2 手时什么都不做。

---

## 配置步骤

### 第 1 步：机器人资产（二选一）

**方式 A（推荐）：替换内置 T2 —— 所有 T2 场景打开就带手，可保存**

1. 关闭 Booster Studio。
2. PowerShell，在 `simulation\hand` 目录执行（路径按自己的安装位置改）：
   ```powershell
   python tools\patch_builtin_t2.py apply `
       --studio "E:\Program Files\BoosterStudio" `
       --library "$env:APPDATA\Booster Studio\simulator-library" `
       --backup-dir "$env:USERPROFILE\booster_t2_builtin_backup"
   ```
   它会先备份、再替换三处：
   - `<Studio 安装目录>\resources\app\booster-native\statics\virtual-robot\library-seed\assets\robot_t2.basset`
   - 同目录 `seed-manifest.json` 里 `robot_t2` 那条的 `sha256` / `size`
   - `%APPDATA%\Booster Studio\simulator-library\assets\robot_t2.basset`

   资产的身份（`builtin-asset-robot_t2` / `2026.08.2`）不变，只换模型，所以现有场景不用改。
3. 打开 Studio。查看状态 / 还原：
   ```powershell
   python tools\patch_builtin_t2.py status  --studio "..." --library "..."
   python tools\patch_builtin_t2.py restore --studio "..." --library "..." --backup-dir "..."
   ```
   注意：影响**所有** T2 场景；**Studio 升级会恢复原版**，升级后再 `apply` 一次。备份目录不要放进仓库。

**方式 B：用户资产 —— 不改 Studio 安装目录，但每次都要手动添加**

1. 把 `robot_t2_revo2.basset` 拷到 `%APPDATA%\Booster Studio\simulator-library\assets\`，重启 Studio。
2. 打开场景后，在编辑器资产库里把 **T2 Revo2** 添加进场景（会叫 `robot2` 等）。不能保存，每次打开场景都要重新添加。

### 第 2 步：安装手指驱动插件（每个虚拟机器人容器一次）

1. 在 Studio 里启动虚拟机器人，查容器名（形如 `<序列号>-t2`，例如 `papt7-t2`）：
   ```powershell
   docker ps --format "table {{.Names}}\t{{.Status}}"
   ```
2. 在 `simulation\hand` 目录安装（会自动重启容器）：
   ```powershell
   powershell -ExecutionPolicy Bypass -File .\install_plugin.ps1 -Container papt7-t2
   ```
   或手动：
   ```powershell
   docker exec papt7-t2 rm -rf /tmp/revo2_plugin
   docker cp ".\plugin" papt7-t2:/tmp/revo2_plugin
   docker exec papt7-t2 sed -i 's/\r$//' /tmp/revo2_plugin/install_in_container.sh
   docker exec papt7-t2 bash /tmp/revo2_plugin/install_in_container.sh
   docker restart papt7-t2
   ```
   安装脚本把插件拷到容器 `/usr/local/booster_robot/booster_robocup_sim/physics/extensions/revo2_hand/`，
   并修改同目录的 `builtin.py`（注册）和 `host.py`（自动启用），原文件备份为 `*.revo2.orig`。
3. 确认：
   ```powershell
   docker logs papt7-t2 2>&1 | Select-String "revo2_hand" | Select-Object -Last 3
   ```
   应看到 `bound hands: [('robot1', 'left'), ('robot1', 'right')]` 和 `ROS 2: listening on /revo2/hand_cmd ...`。
4. 卸载：`powershell -ExecutionPolicy Bypass -File .\install_plugin.ps1 -Container papt7-t2 -Uninstall`

> **容器被 Studio 重建后插件会丢**（重启 Studio、切换机器人时常会重建；只是 `docker restart` 不会丢）。
> 症状：手指软垂、没有 `/revo2/...` 话题、日志里没有 `revo2_hand`。重做第 2 步即可。

### 第 3 步：连接 T2WS

T2WS 共用虚拟机器人容器的网络，**容器重启或重建后都要重建 T2WS**：
```powershell
cd ..\..\T2WS\windows\T2WS\docker
docker compose down
$env:SIM_CONTAINER="papt7-t2"
docker compose up -d
docker compose exec t2ws bash
```
（报 `No such container` / `cannot join network namespace` 就是忘了 `docker compose down` 或容器没在运行。）

### 第 4 步：测试
```bash
ros2 topic pub --once /revo2/hand_cmd std_msgs/msg/String "{data: '{\"side\":\"right\",\"preset\":\"power\"}'}"
ros2 topic echo /revo2/hand_state std_msgs/msg/String --once
```
`echo` 要写上消息类型，否则刚启动时可能报 `does not appear to be published yet`。

---

## 不用脚本的配置方法（全部手动）

下面每一步只用 PowerShell / docker 自带命令和文本编辑器，做的事情和上面的脚本完全一样。
示例里容器名是 `papt7-t2`、Studio 装在 `E:\Program Files\BoosterStudio`，按自己的环境改。

### M1. 替换内置 T2（对应方式 A，不用 patch_builtin_t2.py）

1. **关闭 Booster Studio。**
2. 定义路径（PowerShell，在 `simulation\hand` 目录）：
   ```powershell
   $seed = "E:\Program Files\BoosterStudio\resources\app\booster-native\statics\virtual-robot\library-seed"
   $lib  = "$env:APPDATA\Booster Studio\simulator-library"
   $bk   = "$env:USERPROFILE\booster_t2_builtin_backup"
   ```
3. **备份**三个原文件（只在第一次做；已经替换过就不要再备份，否则备份的是修改版）：
   ```powershell
   mkdir $bk
   Copy-Item "$seed\assets\robot_t2.basset" "$bk\seed_robot_t2.basset"
   Copy-Item "$seed\seed-manifest.json"     "$bk\seed-manifest.json"
   Copy-Item "$lib\assets\robot_t2.basset"  "$bk\library_robot_t2.basset"
   ```
   原版 `robot_t2.basset` 的 SHA-256 是 `efebebf83ef9947087cfc8d059f81fcc7275555ab12698c686516fd36787582a`，大小 `19632799`，
   可以用 `Get-FileHash "$seed\assets\robot_t2.basset"` 确认备份的是原版。
4. **复制**替换资产到两个位置（`Program Files` 下可能要用管理员身份运行 PowerShell）：
   ```powershell
   Copy-Item -Force ".\builtin_replacement\robot_t2.basset" "$seed\assets\robot_t2.basset"
   Copy-Item -Force ".\builtin_replacement\robot_t2.basset" "$lib\assets\robot_t2.basset"
   ```
5. **算出新文件的 SHA-256 和大小**（SHA 必须用小写）：
   ```powershell
   (Get-FileHash ".\builtin_replacement\robot_t2.basset" -Algorithm SHA256).Hash.ToLower()
   (Get-Item ".\builtin_replacement\robot_t2.basset").Length
   ```
   仓库里这版是 `aa91f2e16265fd6f24696e7148fdb77eb83f5c0118e2b049cb03638208faff08` / `26153816`。
6. **编辑 `$seed\seed-manifest.json`**（记事本或 VS Code，以管理员身份打开），在 `"assets"` 里找到这一条，只改 `sha256` 和 `size` 两个值：
   ```json
   {
     "builtin_id": "builtin-asset-robot_t2",
     "builtin_revision": "2026.08.2",
     "preferred_key": "robot_t2",
     "relative_path": "assets/robot_t2.basset",
     "robot_model": "T2",
     "sha256": "aa91f2e16265fd6f24696e7148fdb77eb83f5c0118e2b049cb03638208faff08",
     "size": 26153816
   }
   ```
   保存为 **UTF-8（不带 BOM）**；不要改别的条目，注意逗号和引号。
7. 打开 Studio。确认生效：日志 `%APPDATA%\Booster Studio\logs\<最新时间>\booster-native\native-addon.*.log` 里没有
   `ignored invalid simulator package robot_t2.basset`，打开 T2 场景机器人带黑色 Revo2 手。

**还原：** 关闭 Studio，把三个备份拷回原位置：
```powershell
Copy-Item -Force "$bk\seed_robot_t2.basset"    "$seed\assets\robot_t2.basset"
Copy-Item -Force "$bk\seed-manifest.json"      "$seed\seed-manifest.json"
Copy-Item -Force "$bk\library_robot_t2.basset" "$lib\assets\robot_t2.basset"
```

### M2. 安装手指驱动插件（不用 install_plugin.ps1 / install_in_container.sh）

容器内仿真目录：`/usr/local/booster_robot/booster_robocup_sim/physics/extensions`（下面记作 `EXT`）。

1. 在 Studio 里启动虚拟机器人，`docker ps` 查到容器名。
2. **拷插件目录进容器**（先删掉旧的，否则 `docker cp` 会套一层目录）：
   ```powershell
   docker exec papt7-t2 rm -rf /usr/local/booster_robot/booster_robocup_sim/physics/extensions/revo2_hand
   docker cp ".\plugin\revo2_hand" papt7-t2:/usr/local/booster_robot/booster_robocup_sim/physics/extensions/revo2_hand
   ```
3. **备份**容器里要改的两个文件：
   ```powershell
   docker exec papt7-t2 cp -n /usr/local/booster_robot/booster_robocup_sim/physics/extensions/builtin.py /usr/local/booster_robot/booster_robocup_sim/physics/extensions/builtin.py.revo2.orig
   docker exec papt7-t2 cp -n /usr/local/booster_robot/booster_robocup_sim/physics/extensions/host.py /usr/local/booster_robot/booster_robocup_sim/physics/extensions/host.py.revo2.orig
   ```
4. **把两个文件拷出来编辑**（用 VS Code 等编辑器，保持缩进为空格）：
   ```powershell
   docker cp papt7-t2:/usr/local/booster_robot/booster_robocup_sim/physics/extensions/builtin.py .\builtin.py
   docker cp papt7-t2:/usr/local/booster_robot/booster_robocup_sim/physics/extensions/host.py .\host.py
   ```
   **`builtin.py`**：加两行（标了 `# +` 的行，`# +` 本身不要写进去），改完是这样：
   ```python
   from __future__ import annotations

   from physics.extensions.registry import get_physics_extension, register_physics_extension
   from physics.extensions.qpos_recorder.extension import qpos_recorder_physics_extension
   from physics.extensions.revo2_hand.extension import revo2_hand_physics_extension      # +


   _registered = False


   def register_builtin_physics_extensions() -> None:
       global _registered
       if (
           _registered
           and get_physics_extension(qpos_recorder_physics_extension.name) is not None
       ):
           return
       _registered = True
       register_physics_extension(qpos_recorder_physics_extension)
       register_physics_extension(revo2_hand_physics_extension)                         # +
   ```
   **`host.py`**：在 `class PhysicsExtensionHost` 的 `activate()` 方法末尾、紧挨着
   `    def initialize(self, model: Any, data: Any) -> None:` 这一行**之前**，插入下面这段（每行开头 8 个空格，属于 `activate()` 的方法体）：
   ```python
           # --- robo@home revo2_hand: always on (idle without Revo2 actuators) ---
           if not any(c.name == 'revo2_hand' for c in self._contributions):
               _revo2 = get_physics_extension('revo2_hand')
               if _revo2 is not None:
                   _c = _PhysicsExtensionContribution(name='revo2_hand')
                   try:
                       _revo2.contribute(PhysicsExtensionContext(_c, model_path=self._model_path,
                                         simulation_config=self._simulation_config), _revo2.parse_config({}))
                       self._commit_contribution(_c)
                   except Exception as exc:
                       self._dispose_contribution(_c)
                       print(f'[PhysicsExtensionHost] revo2_hand auto-activation failed: {exc}')
           # --- end revo2_hand ---

   ```
   插入位置的上下文（插入前）：
   ```python
               except Exception as exc:
                   self._dispose_contribution(contribution)
                   self._fatal(f"Failed to activate physics extension '{declaration.name}': {exc}")
                                                                    <- 插在这里
       def initialize(self, model: Any, data: Any) -> None:
   ```
5. **拷回容器并检查语法**：
   ```powershell
   docker cp .\builtin.py papt7-t2:/usr/local/booster_robot/booster_robocup_sim/physics/extensions/builtin.py
   docker cp .\host.py    papt7-t2:/usr/local/booster_robot/booster_robocup_sim/physics/extensions/host.py
   docker exec papt7-t2 python3 -m py_compile /usr/local/booster_robot/booster_robocup_sim/physics/extensions/builtin.py /usr/local/booster_robot/booster_robocup_sim/physics/extensions/host.py /usr/local/booster_robot/booster_robocup_sim/physics/extensions/revo2_hand/extension.py
   ```
   没有输出就是没有语法错误。
6. **重启容器**：`docker restart papt7-t2`，然后按"第 3 步"重建 T2WS、按"第 2 步 / 3."看日志确认。

**卸载：**
```powershell
docker exec papt7-t2 bash -c "cd /usr/local/booster_robot/booster_robocup_sim/physics/extensions && mv -f builtin.py.revo2.orig builtin.py && mv -f host.py.revo2.orig host.py && rm -rf revo2_hand"
docker restart papt7-t2
```

> 说明：`host.py` 这段不加也能用，但那样插件只在场景 `extensions.json` 里声明了
> `{"point":"physics","name":"revo2_hand"}` 时才启用；Studio 编辑器保存的场景不会带这条声明，所以建议加上。

---

## 话题

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/revo2/hand_cmd` | 发给插件 | `std_msgs/msg/String`（JSON） | 手指指令 |
| `/revo2/hand_state` | 插件发布，20 Hz | `std_msgs/msg/String`（JSON） | 每只手一条：`robot`、`side`、`time`、`closure`（实际）、`target`（指令） |

Studio 自己的 `/robot1/joint_states` 只有 31 个身体关节，不含手指。

指令格式（闭合度 0 = 张开，1 = 闭合；全行程约 0.5 s；没写到的手指保持上一次的目标）：
```json
{"side": "right", "preset": "power"}
{"side": "right", "preset": "power", "scale": 0.5}
{"side": "left",  "closure": {"index": 0.8, "thumb_flex": 0.5}}
{"side": "both",  "preset": "open"}
{"robot": "robot1", "side": "right", "index": 0.0}
```
- `side`：`left` / `right` / `both`（默认 `right`）
- `preset`：`open` `ready`（拇指对掌、四指张开）`power`（握拳）`pinch`（捏）`point`（伸食指）`fist`
- 手指：`thumb_rotation` `thumb_flex` `index` `middle` `ring` `pinky`
- `robot`：可选，默认所有带 Revo2 手的机器人
- `closure` 明显小于 `target` → 手指被物体挡住（可用来判断是否抓到）

Python：
```python
from revo2.hand import RosRevo2Hand, make_hand
hand = RosRevo2Hand("right")        # Studio
# hand = make_hand(robot, "right")  # 自动：真机（GetHands 有手）走 SDK ControlDexterousHand，Studio 走话题
hand.preset("power"); hand.open(); hand.set(index=0.8); print(hand.state())
```

## 常见问题
| 现象 | 原因 / 处理 |
|---|---|
| `pub` 一直 `Waiting for at least 1 matching subscription` | 插件没装（容器被重建）或 T2WS 没重建 → 第 2、3 步 |
| 手一直握拳 | 之前发过 `power`，目标会一直保持 → 发 `open` |
| 场景里是银色手壳、没有 Revo2 手 | 方式 A 没生效（Studio 升级后被恢复？）→ `patch_builtin_t2.py status` / 重新 `apply` |
| 导入场景提示"文件内容无效" | 场景引用了用户资产（方式 B 的 T2 Revo2）——Studio 不允许，场景里只能用内置 T2 |
| `install_in_container.sh` 报 `layout changed` | Studio 版本变了，插件补丁位置需要按新版源码调整 |
| 日志位置 | `%APPDATA%\Booster Studio\logs\<时间>\booster-native\native-addon.*.log`（资产/场景校验）、`docker logs <容器>`（插件） |

## 重新生成资产（可选）
Revo2 模型来自 <https://github.com/BoosterRobotics/booster_assets/tree/main/robots/T2/T2_31dof_Revo2Hand>。
`--builtin` 必须是**原版** `robot_t2.basset`（替换过的会被拒绝，用备份）：
```powershell
python tools\make_revo2_basset.py --builtin <原版 robot_t2.basset> --revo2 <T2_31dof_Revo2Hand 目录> --out builtin_replacement\robot_t2.basset --replace-builtin
python tools\make_revo2_basset.py --builtin <原版 robot_t2.basset> --revo2 <T2_31dof_Revo2Hand 目录> --out robot_t2_revo2.basset
```
重新生成后再执行一次方式 A 的 `apply`。

## 已知限制
- 手指 PD 参数（kp 2–4 Nm/rad、kd 0.05、力矩上限取 Revo2 URDF effort）和行程速度是估计值，不是 Revo2 实测。
- 每只手比原内置 T2 多约 0.18 kg，运动控制器的 URDF 里没有手。
- 真机上 Revo2 的 `seq` 映射 / 角度方向（`ControlDexterousHand`）尚未确认。
