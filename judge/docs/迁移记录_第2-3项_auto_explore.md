# 迁移记录 · 第 2/3 项：`auto_explore` 整包（视觉道路跟随 + 区域任务 + MPPI + 安全层）

> 依据：[10.12分支借鉴迁移方案.md](10.12分支借鉴迁移方案.md) §2.2 与 §7 第 2/3 项
> 来源：`refs_zzu_fly_1012/src/auto_explore`（GitHub `lishuskx/zzu_fly@10.12`，`a4aab62`）
> 目标：`ws2025/src/auto_explore` + `ws2025/best.pt`
> 日期：2026-10-13

---

## 1. 迁入内容

| 项 | 内容 |
|---|---|
| 包 | `ws2025/src/auto_explore` —— **39 个文件** |
| C++ 节点 | `area_mission_node`（区域任务/ROAD→PATROL）· `explore_node`（前沿探索，旧链）· `waypoint_mission_node` · `odom_broadcaster` · `scan_self_filter` · `scan_map_fuser` |
| 头文件 | `astar.hpp` · `lidar_route.hpp` · `map_motion_guard.hpp` · **`mppi_planner.hpp`** |
| 脚本 | **`road_follow.py`（视觉道路跟随核心）** · `road_follow_gpu.sh` · `save_map.sh` · `launch_rviz.py` |
| 配置 | `mppi_params.yaml`（MPPI 全套参数）· `area_mission_road.yaml` · `mapper_params_online_async.yaml` |
| Launch | **`auto_explore_swarm.launch`（六机比赛链路）** · `auto_explore.launch` · `explore.launch` · `slam.launch` |
| 模型 | `ws2025/best.pt` = **5,998,180 字节**，md5 `f02efa9a…`（与源一致 ✓）|
| 测试 | **15 个**（`astar_test.cpp`、`lidar_route_test.cpp`、`mppi_route_selection_test.cpp`、`patrol_route_test.cpp`、`map_motion_guard_test.cpp`、`area_mission_static_test.py`、`mission_handshake_test.py`、`alt_recover_test.py`、`road_follow_test.py` …）|

`best.pt` 的落位与 launch 默认值**完全吻合**：`<arg name="road_model" default="$(find auto_explore)/../../best.pt"/>` ⇒ `<workspace>/best.pt` ✓

---

## 2. 合规预检（结论：**无需打合规补丁** ✓）

包里有 `gazebo_msgs` 依赖 ✗（`area_mission_node.cpp:27`、`scan_map_fuser.cpp:4`、`waypoint_mission_node.cpp:25` 各有一个 `modelStatesCb` ✓，注释写"用 `/gazebo/model_states` 真值做高度检查，不可用则退回 pose_z（fail-open）"✗）。

**实测判定**：
1. **三个文件里都没有** `model_states_sub_ = nh_.subscribe(...)` 的建立语句 ✓ ⇒ 回调是**死代码** ✓
2. 构造函数里 `diag_truth_z_ = false;` 与 `altitude_truth_guard_ = false;` ✓ ⇒ 即便有订阅也**不参与控制** ✓
   （第 475 行注释"默认开"是**过期注释** ✗，与代码不符 ✓）
3. **全部订阅话题均为许可类** ✓：`pose` / `scan` / `depth` / `map` / `road` / `road_candidates` / `/drone_poses` / `/mission/control` —— **违规订阅数 = 0** ✓

⇒ 与 10.12 的 `tracking` 包同样情况：团队自己已把真值那路清理掉了 ✓ ⇒ **原样迁入** ✓

---

## 3. 静态一致性检查（skill 重点提醒的反模式）

| 检查 | 结果 |
|---|---|
| `CMakeLists` 的 `find_package` 组件 ⊆ `package.xml` 依赖 | ✅ 通过（无"CMake 找包但 package.xml 没声明"的情况 ✓）|
| `package.xml` 多出的 `map_server` / `slam_toolbox` | ✅ 仅 `slam.launch` 使用 ✓；**比赛链 `auto_explore_swarm.launch` 的 slam/map 引用 = 0** ✓ ⇒ 不需要这两个包 ✓ |
| 比赛 launch 关键参数 | `target_altitude=4.5` ✓、`road_class=0`（'road'）✓、`infer_imgsz=640` ✓、`infer_conf=0.25` ✓、**`infer_device=cuda:0`** ✓（契合我们容器 GPU 链路 ✓）、`cut_radius=0.7` ✓ |

---

## 4. 三处适配（其中一处是计划中的"减法"）

1. **`scripts/road_follow_gpu.sh` 去掉硬编码 conda** ✓
   原版要求 `~/miniconda3/envs/yolo_gpu`（为 Blackwell/RTX50 系准备的 py3.10+cu128）✗；
   改为**自适应**：存在 conda 就用 conda，否则用系统 `python3` ✓
   （我们容器内 `torch 2.4.1+cu124` + RTX 3060(sm_86) 已验证可用 ⇒ **不需要引入 conda** ✓ 减法 ✓）
   `bash -n` 通过 ✓
2. **硬编码路径**：`/home/lishu2/2025_ZZU_FLY` → `/root/ZZU_FLY` ✓（5 个文件：`explore_node.cpp`、3 个 launch、`save_map.sh`；复检残留 = 0 ✓）
3. **`best.pt` 落位** ✓（见 §1）

---

## 5. 待办：编译 + 15 个测试（**被 Docker 阻塞** ✗）

**阻塞原因**：数据盘被自动挂载器挂到了 `/media/wangtao/53dde970-…-3beff432eb04**1**`（多一个 `1` ✗），
Docker 的 `data-root` 仍指向旧名 ✗ ⇒ 守护进程不可用 ✗。
**修法（需 sudo，用户执行）**：
```bash
sudo ln -sf /media/wangtao/53dde970-235d-4a69-a173-3beff432eb041 /media/wangtao/53dde970-235d-4a69-a173-3beff432eb04
sudo systemctl restart docker && docker images | head     # 应看到 zzu_fly_algo:v1
```
（工程侧已把 `/home/wangtao/robocup_fly` 软链接重指到带 `1` 的正确挂载点 ✓，工作区已恢复 ✓）

**Docker 恢复后要跑的**：
```bash
# 1) 容器内（带 GPU 透传）拷入 auto_explore + best.pt
# 2) 依序编译：actor_msgs/serve_msg/look_up（已通）→ auto_explore
docker exec -it player_algo bash -lc 'cd /root/ZZU_FLY && source /opt/ros/noetic/setup.bash && catkin build auto_explore'
# 3) 跑自带测试（CMakeLists 里已接 rostest / 纯脚本两类）
#    astar_test / lidar_route_test / mppi_route_selection_test / patrol_route_test / map_motion_guard_test
#    area_mission_static_test.py / mission_handshake_test.py / alt_recover_test.py / road_follow_test.py …
```

---

## 6. 迁完之后怎么用（与现状的关系）

- **比赛链路**：`roslaunch auto_explore auto_explore_swarm.launch num_drones:=6` ✓
  ⇒ 六机各跑 `scan_self_filter` + **`road_follow`（best.pt 视觉道路跟随）** + `odom_broadcaster` + `area_mission_node` ✓
  ⇒ 状态机 `TAKEOFF → ROAD → PATROL` ✓，六区域固定分配 ✓，PATROL 蛇形覆盖 ✓
- **替代关系**：本包到位后，**旧 `task_manager` + `mission_*.json` 固定航点链应弃用** ✓
  （2026 规则随机地图 ⇒ 固定航点失效 ✗；我此前改的 `mission_down_plus.json` 同属弃用范围 ✓）
- **仍待验证**：与 `tracking` 的 `/<vehicle>/mission/state` 握手 ✓（`PAUSED/RESUME/RESUMED` ✓，
  迁移记录第 1 项已含 tracking 侧 ✓）、以及实机（演练）飞行时 MPPI 与安全层表现 ✓

---
*可回滚：`ws2025/src/auto_explore.old_migrate_<时间戳>`（若原已存在）+ `ws2025/_backup_before_migrate_*` ✓*

---

## 7. 编译与测试结果（2026-10-13，宿主原生编译 ✓）

**绕过 Docker 的办法**：宿主机自带 ROS Noetic，用 **独立 catkin 工作区**（`/tmp/ws_ae`，软链源码）编译，
完全不动你的 `ws2025/devel` ✓（原因是 `ws2025/.catkin_tools` 里缓存了**旧挂载点绝对路径** ✗ —— 挂载点改名
`…eb04` → `…eb041` 的后遗症，与 Docker 的 `data-root` 问题同源 ✓）。

### 7.1 编译
```
catkin build auto_explore  →  Finished <<< auto_explore [6.1 s]   All 2 packages succeeded ✓
（唯一警告：gazebo_msgs 已弃用 —— 无害 ✓）        6 个 C++ 节点全部产出 ✓
```

### 7.2 测试：**13/15 通过**

| 测试 | 结果 |
|---|---|
| `mppi_route_selection_test`（gtest，2 例）| ✅ PASS |
| `map_motion_guard_test`（gtest，3 例）| ✅ PASS |
| `patrol_route_test`（gtest，1 例）| ✅ PASS |
| `astar_test`（独立 C++）| ✅ `=== ALL TESTS PASSED ===` |
| `lidar_route_test`（独立 C++）| ✅ `PASS: observed lidar obstacle changes UAV4's complete transit route` |
| `area_mission_static_test.py` | ✅ 固定顺序 8 槽巡逻队列契约检查通过 |
| `mission_handshake_test.py` | ✅ mission handshake protocol checks passed |
| `road_follow_test.py` | ✅ road mask geometry and stage parsing tests passed |
| `lidar_map_fuser_test.py` | ✅ `PASS: /map starts unknown and contains only observed lidar hits`（需已编译节点 ✓）|
| `alt_recover_test.py` · `patrol_hover_line_advance_test.py` · `patrol_recovery_test.py` | ✅ 通过 |
| `mission_safety_fallback_test.py` | ✗ `FAIL: mission entered patrol; latest=None` |
| `mission_transit_control_test.py` | ✗ `AssertionError: TRANSIT did not publish forward commands: no diagnostics` |

### 7.3 ★ 那 2 个失败**与迁移无关**（对照实验证明）

用**未经任何改动的原始克隆**（`refs_zzu_fly_1012`）建第二个独立工作区编译，跑同样两个测试：
```
原始源码 : FAIL: mission entered patrol; latest=None
           AssertionError: TRANSIT did not publish forward commands: no diagnostics
我迁的版 : **逐字相同** ✗✗
```
⇒ 两者结果**完全一致** ⇒ 是团队代码/环境自身状态 ✓
（交接文档亦提到"Gazebo 的 `libprotobuf.so.17` 原生段错误是独立环境阻塞，完整六机长时间回归需在 Gazebo 稳定后重做" ✓
—— 这两个测试需要真实仿真/话题运行环境 ✓，与本机环境限制同类 ✓）

### 7.4 迁移侧的环境清理与遗留

- `ws2025/.catkin_tools` 里仍是**旧挂载点路径** ✗ ⇒ 该工作区**目前无法直接 `catkin build`** ✓
  修法二选一：① 在 **Docker 容器内**编译（容器用的是自己的 `/root/ZZU_FLY` ✓，不受影响 ✓）；
  ② 在宿主机执行 `cd ws2025 && catkin clean -b -d -y && catkin build`（会重建 devel ✓）
- 我为跑 `lidar_map_fuser_test` 临时在 `ws2025/devel/lib/` 下建过一个指向 `/tmp` 的软链接 ⇒ **已删除** ✓
- `/tmp/ws_ae`（本次编译产物）保留 ✓ 供复跑测试；`/tmp/ws_orig`（对照组）已清理 ✓

