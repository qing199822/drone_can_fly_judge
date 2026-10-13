# 2025 代码跑通 + Docker 化 + 审计发现（结论汇总）

> 日期：2026-10-11 ｜ 机器：裁判端本机（Ubuntu 20.04 / ROS Noetic / Gazebo 11 / RTX 3060）
> 相关文档：[模拟裁判系统_跑法.md](模拟裁判系统_跑法.md)、[联调结论_对齐官方边界.md](联调结论_对齐官方边界.md)、
> [裁判bug反馈_NaN报点.md](裁判bug反馈_NaN报点.md)、[裁判bug反馈_人物活动边界.md](裁判bug反馈_人物活动边界.md)

---

## 一、2025 代码跑通（`2025_ZZU_FLY-main.zip` → `ws2025/`）

### 1.1 依赖清单（反推所得，Dockerfile 的直接输入）

| 类别 | 内容 |
|---|---|
| 基础 | Ubuntu 20.04 + ROS Noetic（`ros:noetic-robot`）+ Gazebo11 运行时不需要（仿真在裁判端） |
| 编译 | **`catkin build`（catkin_tools）**，非 `catkin_make` |
| 系统 ROS 包 | `cv-bridge`、`tf`、`tf2-ros`、**`tf2-geometry-msgs`**、`mavros-msgs`（只要消息）、`gazebo-msgs`、`image-transport`、`nav-msgs`、`geometry-msgs` |
| 系统库 | `python3-catkin-tools`、`pkg-config`、**`libjsoncpp-dev`**（`task_manager` 用 `pkg_check_modules` 找它） |
| **兼容层** | **`python-is-python3`** ← 坑#1 |
| pip | **`pyquaternion`** ← 坑#2、`torch`+`torchvision`（cu124）、`ultralytics`（含 opencv-python 等） |
| 权重 | `src/yolo/yolo11n_942.pt`（包内自带） |
| 运行布局 | **`$HOME/ZZU_FLY`**（工作空间）与 **`$HOME/XTDrone/communication`** ← `1.sh` 里写死的 |
| 算法包 | `gimbal look_up mix_nav pose_init tracking transform_tree yolo`（**仿真侧包已排除**） |

### 1.2 三个必须修的坑

| # | 现象 | 真因 | 修法 |
|---|---|---|---|
| 1 | `multi_vehicle_communication.sh: 行21: python：未找到命令` | Ubuntu 20.04 无 `python` | 装 **`python-is-python3`**；并把 `.sh` 里 `python ` → `python3 `（原文件已备份 `.orig_py2_*`） |
| 2 | `ModuleNotFoundError: pyquaternion` | XTDrone 通信桥的四元数库，**不在 2025 仓库的 import 统计里**，只有真跑才暴露 | `pip3 install pyquaternion` |
| 3 | **6 架 MAVROS 连上但不飞（悬停）** | `pose_init.cpp:87-92` **硬编码命名空间** `typhoon_h480_zzufly_*`，而裁判端是官方命名 `typhoon_h480_*` → `vision_pose` 无发布者 → 导航器无位姿 | 改那 6 行为 `typhoon_h480_*` 并重编译 `pose_init`（**已修复，验证 vision_pose 30 Hz → 导航输出 20 Hz**） |

### 1.3 另一处必须先改的配置
`~/XTDrone/communication/multi_vehicle_communication.sh` 上游默认 **`iris_num=3, typhoon_h480_num=0`** → 必须改成 **`iris_num=0, typhoon_h480_num=6`**，否则 MAVROS 桥去连不存在的 iris。

### 1.4 跑通判据（本机实测）
```
19 包 catkin build 通过 → 13 个窗格起 → 6 架 armed=True/OFFBOARD
d0 10 秒移动 ~36 m（按 mission_down.json 巡航）→ 节点总数 60+
```
> ⚠️ 作者的一键脚本 `1.sh` 是 **为容器内 tmux 设计**的；无 TTY 时 tmux 分屏会失败，**逐个窗格手起更可靠**。

---

## 二、审计发现（三件事）

### 2.1 深度问题：真因是"框中心落在天空"，不是带宽
**本机探针**（640×480 32FC1，与 Gazebo 同机、零延迟）：
```
有效率 89.9%（NaN 9.3%，零值 0%），有效范围 2.21~10.00 m（上限 = pointCloudCutoffMax）
图像中心 5×5 深度全部 ~3.9 m ✓
每 60 行有效率(%): [7.5, 100, 94.5, 100, 100, 100, 100, 100]  ← 只有最上 60 行（天空）是 NaN
```
而解算报错像素是 `(320,19)…(320,30)`，且实测检测框为 `xmin=292 ymin=0 xmax=349 ymax=134` → **框整个落在画面最顶部** → 框中心必然取不到深度。

**结论**：`bbox2coord_node.py` 单点 5×5 采样遇到"无人机俯仰/横滚时视野含天空"就会失败。
**A2 修复（已实施）**：
1. **天空门控**：框中心 `center_v < 0.15 * 图像高` → 直接跳过；
2. **框内下 2/3 区域 3×3 多点取有效值中位数**（无效则回退原逻辑）。

**效果**：上报从"**完全没有**"变为"**持续发布**"（日志出现 `發布 blue1 (blue) 位置到話題 '/actor_blue_info'`）。
**残留**：目标在地图边缘/远距（x≈150、>10 m 截断、地面/空洞）时仍会失败 → 需算法侧继续。

### 2.2 上报精度：2.11 m > 1 m 门限（系统性偏移）
```
上报 = (148.52, -8.53)   真值 = (150.00, -7.00)   误差 = 2.11 m  ❌
连续 3 次一致：x 偏 −1.5 m，y 偏 −1.5 m（两轴同向同量级 → 系统性）
```
**候选真因**：① 检测框中心 ≠ 模型原点（可见面 vs 脚底中心）；② 相机外参未标定；③ `pose_init` 表里 `z=0` 而裁判端出生点 `z=1`。
**最快验证**：持续采样 N 次求平均偏移向量；若稳定（现在看是稳定的 −1.5,−1.5），加常向量补偿即可进 1 m 门限。

### 2.3 红色目标映射缺陷（每场白丢最多 150 分）
`yolo11n.py:78`：
```python
label = f"{model.names[cls_id]}{cls_id}"     # 类别名 + 类别索引
```
模型类别表：`0=green 1=blue 2=brown 3=white 4=red 5=female 6=fire hydrant`
⇒ **`red` 的索引恒为 4 → 标签恒为 `"red4"`；`"red5"` 永远不会产生**（索引 5 是 female）。

而 `bbox2coord_node.py:240-244`：
```python
if class_name == "red4": publisher = self.actor_publishers[4]   # → /actor_red1_info
elif class_name == "red5": publisher = self.actor_publishers[5] # → /actor_red2_info（到不了）
```
**裁判端实测映射**（真值注入验证）：`/actor_red1_info ↔ actor_5` ✓（官方 `test.py` 亦如此：`red1→actor_5`、`red2→actor_4`）。
⇒ **所有红人报点都进 actor_5 的通道；另一个红人（actor_4）永远判不到 → 永远 0 分。**

**修法（算法侧，推荐 A）**：
- **A. 双发**：红人检测同时发 `/actor_red1_info` 与 `/actor_red2_info`；裁判按各自 1 m 门限自动只认对的那个。
- B. 按 `/map` 绝对坐标/先验位置区分 actor_4 vs actor_5 再选话题。

---

## 三、Docker 化与交付（已完成闭环）

### 3.1 范围原则（关键）
> **飞控（PX4 SITL）、MAVROS、仿真世界、人物与裁判系统，全部由裁判端提供并拉起。选手镜像只装算法。**

据此：镜像**不装** `mavros` 节点（只要 `mavros-msgs`）、**不装** `gazebo-ros/gazebo-plugins`（只要 `gazebo-msgs`）、**不拷** `src/gazebo_ros_pkgs`、`src/gazebo_ros_actor_plugin`、机型模型、`waypoint/`。
（这一条同时**解决了 `gazebo_ros` 编译失败**的问题。）

### 3.2 文件
| 文件 | 说明 |
|---|---|
| `docker/Dockerfile.player` | 21 层；清华源（Ubuntu/ROS `[trusted=yes]`/PyPI）→ 兼容层 → pip（pyquaternion/torch cu124/ultralytics）→ COPY → **镜像内 `catkin build`** → entrypoint |
| `docker/build.sh` | 上下文 = `robocup_fly/`；`.dockerignore` 只放行 `ws2025` 与 `XTDrone/communication` |
| `docker/run_player.sh` | `--network host` + `ROS_MASTER_URI/ROS_IP` 注入（含 `--gpus all` 说明） |
| `.dockerignore` | 排除 `build/devel/logs`、仿真侧包、模型/世界 |

### 3.3 构建过程中踩掉并固化的 6 个坑
| # | 现象 | 修法 |
|---|---|---|
| 1 | `FROM requires either one or three arguments` | **`FROM` 行尾不能写注释** |
| 2 | `Unable to locate package ros-noetic-*` | `packages.ros.org` 国内不可达 → 换**清华 ROS 源** |
| 3 | `NO_PUBKEY F42ED6FBAB17C654` | ROS 源加 **`[trusted=yes]`** |
| 4 | `/bin/sh: pip3: not found` | `pip3 config` 必须在**装完 `python3-pip` 之后** |
| 5 | `/opt/ros/noetic/setup.bash: Bad substitution` | Docker `RUN` 默认 **dash** → **`SHELL ["/bin/bash","-c"]`**（放在昂贵层之后以免作废缓存） |
| 6 | `catkin: command not found` / `jsoncpp` 缺 / `gazebo_ros` 编译失败 | 加 `python3-catkin-tools pkg-config libjsoncpp-dev`；**仿真侧包整体排除** |

### 3.4 构建与交付（实测数值）
```
镜像：zzu_fly_algo:v1   ID 30d4753653ec   8.8 GB   （catkin build: All 11 packages succeeded）
冒烟：torch 2.4.1+cu124 / ultralytics 8.4.175 / python→3.8.10 / rospack 113 包 / mission_*.json 齐全
容器⇄裁判端（--network host）：话题 1017、6 架 mavros/state 6/6、/gazebo 服务 30 个 ✓
容器内跑算法：54 节点、6 架 armed+OFFBOARD、d0 从 (-17,-3) 巡航到 (0,-11) ✓

交付包：/media/wangtao/53dde970-235d-4a69-a173-3beff432eb04/wangtao/zzu_fly_algo_v1.tar.gz
        3.6 GB    md5 2ef9e926370bb8e3ad80244c6aaace12
        gzip -t ✅    docker load → "Loaded image: zzu_fly_algo:v1"，ID 不变 ✅
```

### 3.5 对方侧使用
```bash
docker load -i zzu_fly_algo_v1.tar.gz
docker run -it --rm --network host -e ROS_MASTER_URI=http://<裁判机IP>:11311 \
       -e ROS_IP=<本机IP> zzu_fly_algo:v1 bash
# 容器内：
source /opt/ros/noetic/setup.bash && source /root/ZZU_FLY/devel/setup.bash && bash 1.sh
```

### 3.6 环境工程注意（本机经验，非镜像问题）
- `docker save` 若**卡住不动**：先看**磁盘剩余**（需要临时空间；本机曾降到 2.3 GB 导致 `docker save` 超时）；
- 压缩用 **`pigz -p 4`**（多线程，约 4~8 倍快于 gzip）；
- 用 `pgrep -f 'docker save'` 判断进程时会**匹配到自己的命令行**（假阳性）—— 用排除自身进程树的方式判断；
- 重启 docker 后 `/var/run/docker.sock` 权限被重置 → 需 `sudo chmod 666` 或让 `docker` 组生效（`usermod -aG docker $USER` + **注销重登**，`newgrp` 需密码常失败）；
- 外挂盘（ext4）挂载点属主是 root → 先 `sudo mkdir -p <盘>/<用户> && sudo chown -R $USER: <盘>/<用户>`；
- 把 Docker data-root 迁到 ext4 盘可同时解决"系统盘吃紧 + save/build 空间不足"。

---

## 三点五、PX4 版本对齐（1.11 → 1.13.2，与组委会一致）

> 背景：组委会明确 **裁判机使用 PX4 1.13.2**，而我们此前演练用 1.11。
> 直接结果：**MISMATCH 覆盖层（XTDrone 公开的 `sitl_config` 为 1.11 写）在 1.13.2 上完全跑不起来**。
> 下面 10 条是逐条定位并修好的（每条都有实测报错作依据），修完 **6/6 架 MAVROS 全连、整轮跑通**。

### 3.5.1 环境搭建（1.13.2 树）
```bash
git clone --depth 1 --branch v1.13.2 https://github.com/PX4/PX4-Autopilot.git PX4_Firmware_v1.13.2
cd PX4_Firmware_v1.13.2 && git submodule update --init --recursive --depth 1   # 走代理才通
# 拉 XTDrone 覆盖层来源
git clone --depth 1 https://gitee.com/robin_shaun/XTDrone.git XTDrone_full
# 装依赖（--no-nuttx）
bash Tools/setup/ubuntu.sh --no-nuttx
pip3 install --user -r Tools/setup/requirements.txt
```

### 3.5.2 十个必须修的坑

| # | 现象（实测报错） | 真因 | 修法 |
|---|---|---|---|
| 1 | `sh: 0: Can't open /etc/init.d/rc.mc_defaults` → `Startup script returned 15` | XTDrone 覆盖层用 **1.11 的绝对路径** | `. ${R}etc/init.d/rc.mc_defaults`（与 1.13 原生 iris/solo 一致） |
| 2 | 同机型两份 airframe，**被执行的是 1.11 那份**（参数名全错） | XTDrone 把 `6011_typhoon_h480(.post)` 放在 `init.d-posix/`，1.13 用 `airframes/` 并全目录扫描 | 用 `airframes/` 的 1.13 版**覆盖**被执行位置 |
| 3 | `ERROR [param] Parameter CAL_ACC_PRIME/CAL_GYRO_PRIME/CAL_MAG_PRIME not found` | 1.13 **已删除** `*_PRIME` 参数 | 删掉 rcS 里这三行（1.11 的"陀螺仪预校准"修法在 1.13 **已过时**） |
| 4 | `Parameter MPC_XY_VEL_P/I/D、MPC_Z_VEL_P/I、COM_ARM_EKF_AB not found` | 1.13 改名（加 `_ACC` 后缀等） | `sed` 批量改名 + 删 `COM_ARM_EKF_AB` → 参数错误**清零** |
| 5 | `MAVROS connected:False`（PX4 侧 `bind failed: Address already in use`） | `px4-rc.mavlink` 用 `14580/14540+i`，与组委会 MAVROS 的 `fcu_url udp://:2454i@localhost:3458i` 不符 | 改成 **`34580+i` / `24540+i`**（与 XTDrone 的 `rcS.backup` 一致） |
| 6 | `.post` 里 `mavlink: not found` / 端口为空 | `.post` 由 **`sh` 子壳**执行（`rcS:294`），未 export 的 `$udp_*` 是空的 | `.post` 改用**直接算术** `$((34580+px4_instance))`（`px4_instance` 是环境变量，子壳可见） |
| 7 | `rcS: 6: .: px4-alias.sh: not found` → `Startup script returned 512` | 该文件由构建生成在 **`build/px4_sitl_default/bin/`**，而 roslaunch 起 PX4 时 **cwd 是 `~/.ros`** | 重写 `$PX4_DIR/px4` 包装：**先 `cd` 到 `build/px4_sitl_default/bin`** 再 `exec ./px4 "$@"`（并把该目录加进 PATH） |
| 8 | `Error: Unknown model ''` | XTDrone 的 `single_vehicle_spawn_xtd.launch` 把 `<env PX4_SIM_MODEL/PX4_ESTIMATOR>` 写在 **`<node>` 外面** → roslaunch 忽略 | 把两个 `<env>` **移进 `<node>` 内**（与 1.11 的修法相同） |
| 9 | PX4 永远 `Waiting for simulator to accept connection on TCP port 456i` | **gazebo 插件从未编译**（`build_gazebo/` 无 `libgazebo_mavlink_interface.so`）→ 仿真↔PX4 桥接不存在 | `make px4_sitl gazebo` 补编（关键插件即使整体 build 报错也已产出；启动器 `GAZEBO_PLUGIN_PATH` 已含该目录） |
| 10 | `INFO [px4] PX4 daemon already running for instance N (Success)` → 整轮一架都连不上（且**每轮累积**） | 被中断的 PX4 在 `/tmp` 留下 **`px4_lock-<i>` 与 `px4-sock-<i>`** | 清场脚本新增 `clean_px4_locks()`；**必须每次清场都清** |

### 3.5.3 端口竞态（第 11 条）
`PX4 监听 3458i` 与 `MAVROS 监听 2454i` 由同一个 launch 并发拉起，偶尔 MAVROS 先占住 3458i → PX4 `bind failed` → **该架永远连不上**（实测 d5）。
已在 `start_official_judge.sh` 加 `PORT_RACE_RETRY`：
- 每架 spawn 前 `fuser -k -n udp 2454i 3458i` 释放端口；
- 整轮结束后，对未连上的架**自动重试一遍**（写 `drone_<i>_retry.log`）。

### 3.5.4 顺带记录的两条环境坑
- **浅克隆没有 tag** → `FAILED: src/lib/version/build_git_version.h`（`IndexError`）：在 `platforms/nuttx/NuttX/nuttx` 里造本地 tag `nuttx-11.0.0` 兜底（仅影响版本字符串）。
- **`px4-alias.sh` 被我误删**（清理 `px4-*` 时误伤）：`touch CMakeLists.txt && make px4_sitl_default` 触发 CMake 重新 configure 即可**重新生成**。
- `pip` 装 `pymavlink` 报 `SyntaxError: /tmp/dl.py` → `/tmp/dl.py` 是某工具的**报错文本**，被 setuptools 当成 `import dl` 撞上 → 删掉即可。

### 3.5.5 验证结果
```
机型定义: 6011_typhoon_h480 + 6011_typhoon_h480.post  ✓
世界就绪: 111 个模型（连续 3 次稳定）✓
[4/5] 结果: 6/6 架 MAVROS connected ✓（每架 3 秒内）
[5/5] 6 个人物控制（官方 control_actor.py）已启动 ✓
/score = -40（基线）   /left_actors = [0,1,2,3,4,5]   PX4 UDP 监听 6 个 ✓
```
启动方式（脚本已支持外部 PX4 树，无需改代码）：
```bash
PX4_DIR=/home/wangtao/PX4_Firmware_v1.13.2 bash tools/round_start.sh --seed=<种子> --referee-limit=3600
```

---

### 3.5.6 ⚠️ 1.13.2 遗留问题：EKF 无位置估计（尚未解决）

**现象**（6/6 架 MAVROS 都连上之后）：
```
/typhoon_h480_0/mavros/altitude            10 Hz ✓   （气压高度有）
/typhoon_h480_0/mavros/local_position/pose  —   ✗   （无水平位置）
/typhoon_h480_0/mavros/global_position/global —  ✗
sys_status: sensors_enabled 里有 PREARM_CHECK 但不健康
PX4: WARN [PreFlightCheck] Arming denied! manual control lost
     （解锁被拒 → XTDrone 的通信桥拿不到 current_position → 无法 hover/解锁）
```
**已排查并排除的**：
- 校准：`CAL_ACC0_ID/CAL_GYRO0_ID` 已改为 1.13.2 原生值 `1310988`，并把它从 XTDrone 的
  `param reset_nostart ... CAL_ACC* CAL_GYRO*` 重置清单里移除 → `uncalibrated` 警告**已消失** ✓
- 机型/估计器选择：`VEHICLE_TYPE=mc`（由 `rc.mc_defaults` 设置）→ `rc.vehicle_setup` 正常跑到 mc 分支 ✓
- 端口：`px4-rc.mavlink` 已改为 `34580+i`/`24540+i`（对齐组委会 `fcu_url`）✓
- mavlink stream：已把 stream 段挪到 offboard 实例启动之后（否则 `mavlink for network on port 34580 is not running`）✓

**仍存在的嫌疑（下一步）**：
1. **仿真 GPS 未被 EKF 融合**：`gpsstatus/gps1/raw` 有 3D fix / 10 星，但 `eph=epv=0`（无精度），
   且 `sys_status` 的 `sensors_present` 里**没有 GPS 位**；`GPS_1_CONFIG=0`（驱动未配置端口）。
   → 1.13.2 的 `sitl_gazebo` gps 插件与 XTDrone 的 1.11 SDF 组合可能不产生 HIL GPS；建议核对
   SDF 的 `gps_plugin` 块与官方 `Tools/sitl_gazebo/models/typhoon_h480` 的差异。
2. `EKF2_GPS_CHECK 0` / `EKF2_AID_MASK 24`+`EKF2_HGT_MODE 3` 已尝试写入 rcS（注意 rcS 里是 **tab 缩进 + `#param`**，
   用 `sed 's|^#param|param|'` 匹配不到，需按 `^[[:space:]]*#param` 处理）。
3. `Arming denied! manual control lost`：`COM_RC_IN_MODE` 需按 1.13.2 取值（0=RC/1=摇杆/…），
   或按 XTDrone 流程**先切 OFFBOARD 再解锁**（通信桥本来就这么做，但需要先有位姿）。

**结论**：1.13.2 的"对齐"目前到 **6/6 连通 + 世界/人物/计分可用**；**起飞还需解决 EKF 位置来源**
（本质是 XTDrone 1.11 覆盖层与 PX4 1.13.2 的 EKF/GPS 语义差异）。
**最快出路**：向组委会索取其裁判机所用的 `ROMFS/px4fmu_common/init.d-posix/`（rcS + airframes）与
`Tools/sitl_gazebo/models/typhoon_h480*`，直接替换 (他们的 1.13.2 环境是能跑的)。
**注意**：选手容器不含 PX4，**交付物不受此问题影响** ✓

### 3.5.7 ★ 1.13.2 无位置估计的**总根因**（已修复）

```
etc/init.d-posix/6011_typhoon_h480: 11: etc/init.d/rc.mc_defaults: param: not found
etc/init.d-posix/6011_typhoon_h480: 10: param: not found      ← 几十条
```

`param` / `mavlink` / `set` 等命令是 **`px4-alias.sh` 定义的 shell 函数**。XTDrone 的 1.11 版 rcS 用
**`sh <脚本>`** 启动子脚本 ✗ → 子 shell 拿不到这些函数 → **airframe / rc.mc_defaults / rc.interface /
rc.vehicle_setup 里所有 `param set` 静默失败** → 估计器与混控配置全没生效 → 无 `local_position`
→ 无 `linear_velocity` → XTDrone 桥 `current_position = None` → 不能 hover/arm。
1.13.2 原生 rcS 用 **`. ${R}etc/...`（点号 source，同壳）** ✓ 所以原生一直能跑。

**修复**（rcS 内 4 处）：
```diff
-sh "$autostart_file"                    +. "$autostart_file"
-sh etc/init.d/rc.vehicle_setup          +. ${R}etc/init.d/rc.vehicle_setup
-sh "$autostart_file".post               +. "$autostart_file".post
-sh etc/init.d/rc.logging                +. ${R}etc/init.d/rc.logging
```

**修复后实测**：
```
param: not found / mavlink: not found   → 0 次 ✓
/typhoon_h480_*/mavros/local_position/pose  50.0 Hz ✓   estimator_status 1.0 Hz ✓
rosservice call /typhoon_h480_0/mavros/cmd/arming true → success: True ✓
```

### 3.5.8 仍阻塞：OFFBOARD 切换（源码级定位）

PX4 1.13.2 `src/modules/commander/state_machine_helper.cpp:325`：
```cpp
// Require RC for all manual modes
case MAIN_STATE_ACRO/MANUAL/STAB/ALTCTL:
    if (status.rc_signal_lost && is_armed) enable_failsafe(..., no_rc);
```
而 `Commander.cpp:815` 构造函数里 `_status.rc_signal_lost = true;`（"haven't been found"），
**只有真的收到 RC 信号才会置 false**（`Commander.cpp:2694`）；`COM_RC_IN_MODE` 在 `Commander.cpp`
里**完全没被使用** ⇒ SITL 无遥控时 `rc_signal_lost` 恒为 true。

**后果链**：机停在 MANUAL/AUTO.LOITER（failsafe 动作）→ 解锁 → `no_rc` failsafe → 之后
`set_mode OFFBOARD` 返回 `mode_sent: True` 但模式不变（offboard 控制信号 `offboard_control_signal_lost`
仍为 true，PX4 拒绝进入 OFFBOARD）→ 30 s 后 `Disarmed by auto preflight disarming` → RTL。

**已排除的手段**：`COM_RC_IN_MODE` 1/3/4、`NAV_RCL_ACT 0`（**min=1，0 是非法值**）、
`NAV_DLL_ACT 0`、`COM_RC_LOSS_T 0`、把参数 `param save` 进 eeprom（已在 airframe 末尾 + save ✓）。

**下一轮的正解方向**：
1. **先 OFFBOARD 后 arm**（顺序）—— 但需先让 `offboard_control_signal_lost` 变 false；
2. 用 mavros 的 **`setpoint_raw/local` + `OFFBOARD_CONTROL_MODE`**（std 流程）确认 PX4 是否收到有效 offboard 流；
3. 或给 SITL 喂一条**模拟 RC/manual_control**（`mavros/manual_control/send`）让 `rc_signal_lost=false`；
4. 或把 `rc_signal_lost` 检查在 SITL 下改掉（补丁 `state_machine_helper.cpp`，仅在无 RC 场景）。

### 3.5.9 第二轮：OFFBOARD 阻塞的完整证据链（ulog 级）

**新增修复**：`rcS` 的 offboard mavlink 实例加 **`-f`**（forward external setpoints），并去掉 `.post` 里重复的
`mavlink start`（它一直报 `ERROR [mavlink] port 34580 already occupied` 而失败）。修后端口冲突 0 次 ✓。

**ulog 实证**（`build/px4_sitl_default/bin/sitl_typhoon_h480_N/log/…ulg`，pyulog 解码）：

```
offboard_control_mode: 285~403 条，velocity=1 占 100%          ← offboard 信号确实在发 ✓
vehicle_status_flags.offboard_control_signal_lost = 0 ✓        ← 不是信号丢失问题
vehicle_status_flags.local_position_valid / local_velocity_valid = 1 ✓
vehicle_status 时间线（d2）：
   t= -0.2s  nav=0(MANUAL)    failsafe=0  rc_lost=1  dl_lost=1   ← 开机时两路"丢失"就已是 1
   t=  6.5s  nav=4(AUTO_LOITER) failsafe=0
   t=212.9s  nav=4  failsafe=1 ✗                                 ← 我一开始发 setpoint / 调 set_mode 就触发
```

**结论（唯一阻塞点）**：SITL 没有遥控器也没有地面站心跳 ⇒ `rc_signal_lost=1`、`data_link_lost=1` **恒为真**；
PX4 一旦置 `failsafe=1`，**之后拒绝一切模式切换**（实测 `set_mode` 连 `AUTO.LOITER`/`OFFBOARD`/
`base_mode=1`/`PX4_OFFBOARD` 写法全部 `mode_sent: True` 但模式不变），于是永远进不了 OFFBOARD。

**已排除**：`-f` 缺失（已修 ✓）、`offboard_control_signal_lost`（ulog 证明为 0 ✓）、
`local_position/velocity_valid`（均为 1 ✓）、`COM_RC_IN_MODE` 1/3/4（Commander.cpp 里根本没用该参数）、
`NAV_RCL_ACT 0`（min=1 非法）、`NAV_DLL_ACT 0`、`COM_RC_LOSS_T 0`、`param save` 落盘、
注入 `mavros/manual_control/send` 10 Hz（运行时不生效，需重启读参数）。

**下一轮的三条出路（按代价排序）**：
1. **`COM_RC_IN_MODE=2`（MAVLink 手动输入当 RC）写进 eeprom + 重启**，再注入 `manual_control` →
   让 `rc_signal_lost=false`，failsafe 自然不触发；
2. **本地补丁**（仅演练环境）：在 `state_machine_helper.cpp` 的 armed+failsafe 判定里对
   `rc_signal_lost && data_link_lost` 且**无 RC 硬件**的 SITL 场景跳过 failsafe；
3. **接受现状**：比赛时裁判端由组委会提供（他们的环境有其自己的 RC/GCS 配置），
   本机 1.13.2 的用途只是"演练保真"，不是交付物 ✓

### 3.5.10 ✅ 第二轮收官：1.13.2 完全打通（可 OFFBOARD 起飞）

**最后一招**：本地 PX4 树 `Commander.cpp` 加环境开关（仅演练，不影响组委会环境）：

```cpp
// zzu_fly: SITL 无遥控/无地面站 ⇒ rc_signal_lost 恒真，failsafe 永久封锁模式切换
static const bool zzu_fly_skip_rc_failsafe = (std::getenv("PX4_SKIP_RC_FAILSAFE") != nullptr);
...
_status.rc_signal_lost = !zzu_fly_skip_rc_failsafe;   // 2 处
_status.data_link_lost = !zzu_fly_skip_rc_failsafe;   // 2 处
```
启动器已默认 `export PX4_SKIP_RC_FAILSAFE=1`（外部可设 0 关闭）。

**实测（全新一轮）**：
```
初始 armed=False mode=MANUAL
set_mode OFFBOARD → mode="OFFBOARD" ✓      （此前永远停在 AUTO.LOITER）
arming → success: True ✓
状态 armed=True mode=OFFBOARD ✓
高度 z: 11.99 → 17.67 → 22.26 m ✓✓          （offboard 速度控制真实生效）
```

⇒ **裁判端 1.13.2 与组委会版本对齐完成**：6/6 连接、EKF2 位置 50 Hz、OFFBOARD 起飞、
速度/位置 setpoint 均可用。选手容器（不含 PX4）不受影响 ✓

### 3.5.11 ✅ 第三轮：1.13.2 上跑通"起飞→巡航→计分"，跟踪链出目标

```
6/6 connected ✓ → 6 架 armed=True mode=OFFBOARD 起飞 z≈3.6~5.2 m ✓
d0 巡航到 (125.9,-36.5) ✓   /score=-40（含 13200 传感器费）✓   /left_actors=[0..5] ✓
/typhoon_h480_1/yolo11n/bounding_boxes 29.9 Hz ✓
/typhoon_h480_1/tracking_node/... → "TRACKING:blue1:2837.624000" ✓✓（真的跟踪到人）
```

**链路修复（4 项）**：
1. 官方算法栈是 **13 个窗格**（含 `multi_yolo_detecting.sh` / `multi_solving.sh` / `multi_gimbal_control.sh`），
   按容器内 `1.sh/2.sh/3.sh` 启动；漏了这 3 个 → 没有上报、没有云台指令 ✗
2. `score_cal` 曾退出 → 按 launcher 方式（source `devel/setup.bash`）重启 → `/score` 恢复 ✓
3. 镜像缺 **`ros-noetic-tf2-geometry-msgs`** → 解算窗格 `ModuleNotFoundError` ✗ ⇒ **Dockerfile 要补这条**
4. 云台 `mount_control/configure` 被拒 ✗ ← airframe 的 **`MNT_MODE_IN 4`**（XTDrone 1.11 是 `0`）⇒ 已改 0 ✓

**待解**：`/actor_*_info` 无消息（发布者 `coordinate_estimator_node_*` 在，跟踪已出目标 ⇒ 卡在
**深度→坐标估计**这一段）；`referee.log` 报 `UAV 4 is higher than 6 meters` ⇒ 巡航高度压到 ≤5 m。

### 3.5.12 ★ 组委会口径（2026-10-12 群内确认）

> 老师："你们能订阅的只有**传感器和飞机**，也就是 **mavros 下的话题**哈。"
> 老师（关于人物上报）："**不需要哈，只要颜色和坐标对就行，两个红色的也比较难区分**"

⇒ ① 订阅红线 = 仅 mavros/传感器/飞机话题（与我们红线判断一致 ✓）；
   ② 上报只需**颜色+坐标**，**两个红人无需区分** ⇒ 红色映射缺陷的判罚风险下调，
      **红人双发**仍建议保留（成本≈0，且明确被允许）。
详见 `docs/方案审查报告_五面.md` §13。

## 四、待办清单

### 算法侧（4 项）
1. **红人双发**（§2.3）—— 否则 actor_4 每场白丢最多 150 分；
2. **上报精度 2.11 m → <1 m**（§2.2）；
3. **巡航高度压到 ≤ 6 m**（裁判日志已警告 `UAV 5 is higher than 6 meters`，官方 `MAX_UAV_ALTITUDE=6.0`）；
4. `pose_init.cpp` 偏移表 `z: 0 → 1`（与裁判端出生点 `z=1` 一致）。

### 组委会（4 项）
1. **人物活动边界（核心阻塞）**：官方插件 x 钳制 **150** vs 官方 `control_actor.py` 上限 **130** ⇒ 人物跑进 **x=150**"无地图覆盖带"且**越界后永久卡死**（实测 T0=T1 静止、6 人全在 x=150）→ 无人机看不到/测不准 → 算不出分。**建议把插件 5 处 `min(150.0,…)` 与 `Rand::DblUniform(-50,150)` 改为 130**，或让人物活动范围收进城区。
2. 传感器费用口径：组委会 `robocup.launch` **无那 7 个参数**（默认 1700 → 扣 5.1；我队申报 13200 → 扣 39.6）；
3. `typhoon_h480_2d_lidar_nudt` 模型是否提供（组委会裁判机实际机型，含 2D 激光）；
4. 深度中继/深度格式许可（6 路 32FC1@20 Hz ≈ **1.18 Gb/s** > 千兆；官方独立深度话题**无压缩版本**）。

### 其他
- 官方缺陷三件套已反馈：`score_cal.py:119` NaN 不拦、插件 x 钳制不一致、`control_actor.py:668` 括号错（Py3 必崩）。
  → 后者可用 `tools/actor_watchdog.py` 兜住（不修改官方代码，只把退出的人物进程重新拉起），**是否在正式比赛启用需组委会认可**。

---

## 五、关键数字速查

| 量 | 值 |
|---|---|
| 官方出生点 | `(-17,-3)(-14,-3)(-17,0)(-14,0)(-17,3)(-14,3)`，`z=1` |
| 官方端口 | `fcu_url=udp://:2454i@localhost:3458i`、`tcp 4560+i`、`udp 14560+i`、gimbal `13030+i` |
| 官方计分 | `发现×50 + 消除×100 − 传感器费×0.003 − 丢失×罚分`；门限 `误差<1 m`、`间隔≤1 s`、`连续 15 s` |
| 相机内参 | `fx=fy=554.2547`（= FOV 1.047198 @640）；旧值 205.47 对应 FOV 2.0 |
| 深度 | `32FC1 640×480`，1.23 MB/帧，6 路@20 Hz ≈ 1.18 Gb/s；量程上限 10 m |
| 裁判端文件 | `score_cal.py` sha `557166fa9c12`、`control_actor.py` sha `03d13c7c331ef8b`（**均未改**） |
