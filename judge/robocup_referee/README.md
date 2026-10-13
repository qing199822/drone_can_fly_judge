# robocup_referee —— XTDrone/robocup 全链路集成裁判程序

把开源仓库 [XTDrone](https://gitee.com/robin_shaun/XTDrone) 在 `robocup/` 下的四个部件
**集成成一个程序**：地图生成 → 人物（actor）生成/运动 → 裁判评分。

> **本目录是独立新增的，不改动你现有的任何东西**
> （`src/`、根目录 `score_cal.py`、`start_referee.sh`、`tools/map_gen/`、`XTDrone/` 全部原样）。
> 它只**读取** `XTDrone/robocup/` 里的底图与 `rover_static.world` 作为输入。

## 文件

| 文件 | 说明 |
|---|---|
| `referee.py` | 集成裁判程序本体（单文件，2613 行）：`gen-map` / `run` / `selftest` / `info` 四个子命令 |
| `selftest_fidelity.py` | **字节级保真测试**：同种子跑真实的上游 `map_generator.py`，与 `gen-map` 产物逐字节比对 |
| `selftest_scorer_equiv.py` | **评分等价测试**：import 上游 `score_cal.py` 本体，同输入逐步比对分数与状态 |
| `selftest_ros.py` | ROS 集成冒烟测试：在**私有 master（端口 11400）**上起临时 roscore + 假 Gazebo，跑一遍完整流程 |
| `README.md` | 本文档 |

## 代码构成（哪些是移植、哪些是我新写的）

| 区段 | 行数 | 来源 |
|---|---|---|
| `MapGenerator` | 636 | 移植 `map_generator.py` |
| `ObstacleAvoid` | 140 | 移植 `ObstacleAvoid.py` |
| `HumanSpawner` | 47 | 移植 `control_actor.create_human_point()` |
| `ActorBrain` | 437 | 移植 `control_actor.py` |
| `RefereeScorer` | 481 | 移植 `score_cal.py` |
| `WorldState` | 98 | **新写**：共享一份 `/gazebo/model_states` 缓存 + UAV 速度差分（上游是每进程各订阅 6 路 odom）|
| `RefereeNode` | 212 | **新写**：单进程 ROS 装配、tracker 让位、dry-run、环境兜底 |
| CLI + 自检 + 头部 | 562 | **新写** |
| 合计 | 2613 | 移植 ≈67%，新写 ≈33% |

## 对应关系（每一块都能追到上游原件）

| 本程序 | 上游原件（提交 `5171c58`） |
|---|---|
| `MapGenerator`（`gen-map`）| `robocup/map_generator.py`：base 底图选择、13 栋建筑随机、20 个 rover 随机、`black_box.txt`、`obstacle.txt`、`house.world`/`rover.world`/`robocup.world` 组装 |
| `MapGenerator.create_human_point()` | `map_generator.py` 里 6 个人物 **开局** `init_pose` 随机（含 5171c58 新增的"人物相互 ≥8 m、离 6 个起飞点 ≥8 m"）|
| `HumanSpawner` | `control_actor.py` 里运行期**传送/重生**落点随机 |
| `ObstacleAvoid` | `ObstacleAvoid.py`（含 5171c58 的 `2024.txt` → `obstacle.txt` 修正）|
| `ActorBrain` ×6（`run`）| `control_actor.py` + `control_actors.sh`（原版是 6 个进程，现在 6 个大脑在 1 个进程里）|
| `RefereeScorer`（`run`）| `score_cal.py`：发现/消除/跟踪/高度/超时/失联 + `/score` `/time_usage` `/left_actors` `/find_actor_i` |

## 快速开始

```bash
source /opt/ros/noetic/setup.bash          # 消息包路径程序会自己挂，不必 source devel/setup.bash

# ① 生成这一局的世界（在起 gzserver 之前跑一次）
python3 robocup_referee/referee.py gen-map --out /tmp/round01 --seed 1234
#   --base 1|2|3|random  默认跟上游一致：random.randint(1,3)
#   --rovers 20          默认 20
#   --fix-known-bugs     修上游三个缺陷，见下

# ② 起完 gzserver + PX4 + MAVROS 之后，跑裁判（人物运动 + 评分）
python3 robocup_referee/referee.py run \
    --black-box /tmp/round01/black_box.txt \
    --obstacle  /tmp/round01/obstacle.txt

# 只读干跑：订阅/计算全做，但不发 /score、不发 cmd_motion、不删模型、不传送
python3 robocup_referee/referee.py run --dry-run --duration 30 \
    --black-box /tmp/round01/black_box.txt --obstacle /tmp/round01/obstacle.txt

# 无 ROS 的纯逻辑自检（含真地图生成到临时目录）
python3 robocup_referee/referee.py selftest

# ROS 集成冒烟测试（私有 master，不碰你的仿真）
python3 robocup_referee/selftest_ros.py
```

`gen-map` 的产物：

| 文件 | 用途 |
|---|---|
| `robocup.world` | 直接喂给 gzserver 的 `world:=` 参数（含 6 个人物 `init_pose`）|
| `house.world` / `rover.world` | 中间产物（建筑随机 / rover 随机）|
| `black_box.txt` | 建筑包围盒 → `run --black-box`（人物巡逻避障 + 落点判定）|
| `obstacle.txt` | 栅格障碍点（含 4 条边界）→ `run --obstacle` |
| `manifest.json` | 本次生成的指纹：seed / 底图 / sha256 / 6 个人物落点 / 是否走了回退 |

## 与你现有链路的关系

- **地图生成**：`gen-map` 和你的 `tools/map_gen/gen_round_world.py` 是两条独立的路，产物格式一致
  （`robocup.world` + `black_box.txt` + `obstacle.txt`），可以互换，也可以只用一个。
- **人物运动**：当前 `start_referee.sh` 为 6 个人物分别启动 `XTDrone/robocup/control_actor.py`，
  它们读取本轮的 `black_box.txt` 和 `obstacle.txt`，向 `/actor_<i>/cmd_motion` 发布行走/避让指令。
  `src/tracking/src/state_machine.cpp` 不再发布人物的 `/actor_<i>/cmd_vel`。本程序的 `run`
  子命令包含另一套人物控制实现；在当前链路中不要同时启动它的人物控制功能。
  `--no-actor-control` 可让本程序只负责评分。
- **评分**：接口与 `score_cal.py` 完全一致（订阅 `/actor_{green,blue,brown,white,red1,red2}_info`，
  发布 `/score` `/time_usage` `/left_actors` `/find_actor_i`），所以可以直接换着跑，
  但**不要和 `score_cal.py` 同时跑**（两边都会发 `/score`、都会删模型）。
  要让你的 `start_referee.sh` 用上它，建议复制一份脚本改那一行，而不是改原文件：

  ```bash
  sed 's#setsid python3 -u "$SCRIPT_DIR/score_cal.py" "$UAV_TYPE"#setsid python3 -u "$SCRIPT_DIR/robocup_referee/referee.py" run --tracker-yield 0 --black-box "$ROUND_DIR/black_box.txt" --obstacle "$ROUND_DIR/obstacle.txt" --no-actor-control#' \
      start_referee.sh > start_referee_new.sh
  ```

- **传感器费用**：`--score-mode extended` 的口径与你根目录 `score_cal.py` 一致（默认的
  `repo` 是上游原式）。

## 两个接口坑（程序里已经处理）

1. **`/xtdrone/<type>_<i>/ground_truth/odom` 可能不存在**（你当前部署就没有）。
   上游 `control_actor.py` 靠它判断"哪架 UAV 在追我"。本程序 `--uav-odom auto`（默认）
   先订阅它，某架机 3 秒收不到就单独降级为用 `/gazebo/model_states` 位姿差分估速。
2. **`~/.ros/log` / `~/.ros` 只读**的容器/沙箱里 `rospy.init_node` 与 `roscore`
   都会直接崩（后者是 `roscore-<port>.pid` 写不进去）。本程序在 `init_node` 之前自动把
   `ROS_LOG_DIR`→`<工作区>/logs/ros`、`ROS_HOME`→`<工作区>/logs/ros_home`。
   同理，`devel/lib/python3/dist-packages` 会自动挂进 `sys.path`，不需要先
   `source devel/setup.bash`。

## 顺手发现并绕开的一个上游缺陷：`/find_actor_i` 的时间戳被 float32 截断

上游 `score_cal.py` 用 `Float32` 发布 `/find_actor_i`，载荷是 `rospy.get_time()`。
而 `rospy.get_time()` 是 1.79e9 量级的 epoch 秒，float32 只有 24 位尾数 →
**时间戳被量化到 128 秒一档**（实测：两次相隔数分钟的运行的 `/find_actor_0` 载荷完全相同）。
上游 `control_actor.py` 恰好用这个值来算"被发现 25 秒后传送人物"，于是那个 25 秒会随机
偏移 ±128 秒；一旦偏到未来，那个 actor 进程就会在回调里一直忙等（整个进程卡死）。

本程序的处理：

- `ActorBrain.on_found()` **不使用消息里的值**，改用"收到消息的本地时刻"作基准 →
  传送时刻确定；
- 额外发布 `/find_actor_<i>_f64`（`Float64`）作为高精度补偿，上游原话题保持不变，
  需要精确时间的消费者可以订阅它。

> 这个缺陷是 ROS 集成冒烟测试逼出来的：一开始 35 秒的用例时好时坏，
> 12 秒的用例却总过——差的就是量化落在"过去"还是"未来"。

## 测试

```bash
python3 robocup_referee/referee.py selftest          # 23 项纯逻辑（含真地图生成）
python3 robocup_referee/selftest_fidelity.py         # 6 种子 × 5 产物 与上游逐字节比对
python3 robocup_referee/selftest_scorer_equiv.py     # 评分/状态机 与上游 score_cal 本体比对
python3 robocup_referee/selftest_ros.py              # 17 项 ROS 集成（私有 master，两阶段）
```

`selftest_ros.py` 在**端口 11400 起一个临时 roscore** + 假 Gazebo（发 `/gazebo/model_states`、
`/actor_green_info`，并提供 `get/set_model_state`、`delete_model` 服务），分两阶段验证：

| 阶段 | 参数 | 验证点 |
|---|---|---|
| A | `--duration 9 --teleport-interval 3` | 发现 → 3 秒后人物被 `set_model_state` 传送 |
| B | `--duration 26 --teleport-interval 60` | 连续保持 15 秒 → `delete_model` 消除、`/left_actors` 少一个、连续跟踪 20 秒拿 +80、被追时逃跑 |

> 为什么要分两段：消除要求"连续保持 15 秒"，而传送会让坐标跳变、判定按规则清零。
> 把传送间隔设得比 15 秒短，人物就永远消除不掉（这是规则行为，不是 bug）。

实测结果（2026-09-28）：

```
selftest              : 23 通过 / 0 失败
selftest_fidelity     : 6 种子 × 5 产物 = 30/30 逐字节相同（含 1 次 0 字节回退用例）
selftest_scorer_equiv : 5 项全一致（33 步状态机逐步 / 486 组公式矩阵 / inline 2580 / 越界裁剪）
selftest_ros          : 17 通过 / 0 失败（连跑 2 次一致）
gen-map 修前          : 13 栋建筑 → 1 个不同落点（复现上游缺陷）
gen-map 修后          : 13 栋建筑 → 13 个不同落点
```

`selftest_scorer_equiv.py` 覆盖不到的部分（都在上游 main 循环里、不是函数，只能靠读代码对齐）：
高度 >6 m 判 0、600 s 超时、UAV 失联扣分。


## 上游已知缺陷与 `--fix-known-bugs`

`gen-map` 默认**忠实复现上游 5171c58 的行为**，包括它的三个缺陷：

1. `create_point()` 连试 10 轮失败 → 走 `Base world is used!` 回退，而回退里
   `content.readlines()` 复用了已读完的句柄 → 产出 **0 字节 world**（实测约 4%）；
2. 13 栋建筑走 `center_list[i]` 的既有索引逻辑 → 实测**全部叠在同一个点**
   （本程序 `selftest` 第 [3] 项会打印 `建筑不同点=1` 作为证据）；
3. 只改模型级 `<pose>`、不同步底图 `<state>` 里的旧位姿 → 旧快照可能覆盖随机化结果。

加 `--fix-known-bugs` 后：(1) 底图只读一次；(2) 每点最多 2000 候选、取余量最大者，
不再回退；(3) 按模型名精确定位模型级 `<pose>` 并同步 `<state>`。
这三条**等价于你 2026-09-19 给 `XTDrone/robocup/map_generator.py` 打的那份补丁**
（同一套修复思路，代码是新写的）。实测：修前 13 栋 = 1 个不同落点，修后 = 13 个不同落点。

**默认忠实模式是逐字节对齐的**（`selftest_fidelity.py`，6 个种子 × 5 个产物 = 30/30 相同，
含一次真的走进回退、产出 0 字节 world 的用例）。写这一版时踩过两个"看起来等价、
其实不等价"的坑，都已按上游真实行为改回：

1. `create_point()` 里的 `count` 在上游是**跨 9 个落点累积**的（从不按点清零）——
   一旦累计到 1000，后面几个点的 `while` 会立刻退出、直接复用上一轮的 x/y。
   我第一版按点清零，会改变回退概率和落点。
2. `rover.world` 里上游那段"按 link 名加偏移量"的代码（`if count == 2 and num > 0:`）
   **永远不会执行**（外层已被 `count == 1` 限死），等效于死代码。我第一版照"意图"
   把它补全了→ `rover.world` 与上游不同。现在按真实行为只替换每个 rover 的模型级 pose。

### 评分层的两处修正（`selftest_scorer_equiv.py` 抓出来的）

1. 上游 `_progress_score()` 里**没有**"完成 → 2580 − 用时"分支：那个 2580 是上游在
   "6 个人物全被消除"那一处 **inline** 算的。我第一版把这个分支塞进了 `progress_score()`，
   于是 repo 模式下每 tick 发布的分都会随时间漂移 —— 486 组计分公式矩阵直接把它暴露了。
   现在 repo 模式 = 上游逐字一致；extended 模式才走 2580 分支。
2. 上游**没有** `_dump_score_file` / `atexit`，分数落盘是你根目录 `score_cal.py` 加的。
   本程序默认保留（你的 `start_referee.sh` 依赖 `SCORE_OUTPUT_PATH`），但它是**附加功能**，
   可用 `--no-score-dump` 关掉。

## `run` 的其它参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--score-mode` | `repo` | `repo` = 上游原式；`extended` = 加"起飞×10 + 碰撞×30"（你的口径）|
| `--tracker-yield` | `2.0` | `/actor_*/cmd_vel` 出现后让位秒数；`0` = 独占 |
| `--uav-odom` | `auto` | `auto` / `on` / `off` |
| `--no-actor-control` | 关 | 只评分不驱动人物 |
| `--dry-run` | 关 | 什么都不发、不删、不传送 |
| `--duration` | `0` | 跑满 N 秒主动退出（测试用）|
| `--teleport-interval` | `25` | 人物被"发现"后多少秒换落点（上游 25 s）|
| `--debug-teleport` | 关 | 打印每个待传送人物的倒计时 |
| `--enable-shot-detection` | 关 | 上游那段"卡住=被击中"的死代码（原版从未置 True）|
| `--score-output` | `$SCORE_OUTPUT_PATH` 或 `./referee_score.txt` | 最终分落盘路径（注意：**不写**你根目录的 `score.txt`）|

## 与上游 `control_actor.py` 的行为差异（集成必须做的三处）

1. 上游每个 actor 一个进程、各自订阅一遍 `/gazebo/model_states` 与 6 路 odom；
   这里 6 个 `ActorBrain` 共享一份 `WorldState`。
2. 上游在 `/find_actor_i` 回调里**忙等 25 秒**（`while not success: if now-t<25: continue`），
   会把整个进程卡死；这里改成在 tick 里判到期后传送一次，语义相同但不阻塞。
3. 上游用 `loop()` 里的局部变量 `middd_pos` 存子目标（恰好因为 Python 作用域而"能用"）；
   这里显式用 `subtarget_pos` + 下标，已逐行核对行为等价。

其余（巡逻候选规则、1000/2000 次判据、101 点路径采样 × 1.5 m 阈值、`spon_dis=5`、
被追判定 2 秒 × 速度>1 m/s × 距离<20 m、只有触发追踪的那架机才能解除、逃跑目标几何、
`normal_speed=1.0`/`tracked_speed=2.0`/`separation_speed=2.0`、避让半径 4 m / UAV 安全半径 7 m、
重叠时按 id 给确定性脱离角）都与上游一致。
