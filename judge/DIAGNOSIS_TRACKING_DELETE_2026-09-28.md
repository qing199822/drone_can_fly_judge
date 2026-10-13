# 本轮复盘与修复：追踪-消除链路 + 0/2/5 号机巡逻异常（2026-09-28）

> 数据来源：本轮仿真 `logs/sim_only_20260928_175404/`（算法侧，仿真 ~558s）与
> `logs/referee_only_20260928_175349/`（裁判/底层，`score_cal.py` + gzserver）。
> 裁判日志关键事件：`actor_1 / actor_3 / actor_5 / actor_0 / actor_4 is OK`（5 个被消除），
> 末尾 `Warning: UAV 1 is higher than 6 meters` → **score: 0**。
> 追踪侧：UAV2 对 `red4`（actor_4）从 t=111.2s 一直追到 t=436.3s。

---

## 一、你提的两个现象，各自的根因

### 1）"无人机追踪人物模型超过两分钟，人物模型也没有被删除"

**根因：追踪期间一直在发"深度解算坐标"，它的误差经常 >1m，裁判的"连续 15s、误差<1m"窗口被反复清零。**

* 裁判 `score_cal.py:170-220` 的判据是：`/actor_*_info` 上**连续 15s**、每条与真值**误差 <1m**、
  相邻消息**间隔 ≤1s** → 才 `delete_model`。**跟得近不等于会被删**，只有"报得准且连续"才算。
* 本轮证据：
  * `tracking.log` 里 UAV2 的 `red4` 会话持续 **309s**（`State: TRACKING (red4) | session …`）；
  * `multi_solving.log` 里 `red4` 的 **Gazebo 真值兜底发布只出现过 1 次（t=159.3s）**，
    其余时间（159.3s→415.8s）**全部走深度解算路径** —— 因为 YOLO 一直能检到人，
    而旧代码只在"漏检/深度失败"时才用真值兜底（`bbox2coord_node.py` 原 302-304 行）；
  * 同一时间裁判日志里 `find actor_4` 被反复打印（≈40 次）——每次都意味着 **15s 计时被清零重来**。
* 为什么深度解算误差大：人物以 **1.1~1.9 m/s** 逃跑（`tracking.log` 的
  `actor_4 new escape motion: … speed 1.1~1.9 m/s`），而深度图→YOLO→TF 这条链有
  0.3~1s 级延迟，ROI 深度还常打到背景；误差自然常年 >1m。
* 结论：**真值兜底"只在漏检时用"是不够的** —— 恰恰是"一直能看见"的时候在发不准的坐标。

### 2）"有的人物模型没有被追踪但是却被删除了"

**根因：旧参数 `publish_hold_time=20s` + `min_publish_span=18s` 会把"只追了几秒"补发成 15s。**

* 旧逻辑：命中后 hold 20s 内 10Hz 重发最后一次估计，且"首帧起至少发 18s"（`min_publish_span`）。
  于是**只要进入过一次 TRACKING**（`confirmation_duration=0.05s`，几乎等于 1 帧），
  后面 18s 的"连续消息"就被补出来了 → 裁判判"找到"并删人。
* 本轮证据：
  * `white3`（actor_3）只有 **6s** 的 TRACKING 日志、`red5`（actor_5）只有 **25s**；
  * 裁判侧"跟踪分"（真值 10m 内连续 20s）只成功过 `actor_0 / actor_4 / actor_1`，
    **actor_3、actor_5 从未拿到跟踪分，却被删除了** —— 正是你说的"没被追踪却被删"；
  * 对照：actor_2（brown）全程没被追踪，也**没有**被删除。

---

## 二、解算坐标到底差多少（用本轮日志能测到的部分）

结论：**不是"一直算不准"，而是"平时勉强够用（~0.5m 滞后）+ 偶尔米级粗差（4~6m）"。**
裁判的门槛是硬 1m，粗差一来 15s 计时必然清零。

| 时刻 | 深度解算发布的坐标 | 同一 actor 的真值 | 误差 |
|---|---|---|---|
| red4 t=159.23s | (24.119, 5.998) | (24.376, 6.515) @159.28s | **0.58 m**（≈ v×0.3s，纯滞后） |
| red4 t=414.56→415.76s | 最后深度点 (6.649, 14.705) | 首个真值 (11.079, 12.254) | 1.2s 内跳 **5.06 m**（4.2 m/s）→ **≈4~5 m 粗差** |
| green0 t=257.36s | (-22.89, -15.28) | (-28.76, -16.84) @+0.16s | **≈5.8 m 粗差** |

* 人物最快 1.9 m/s，且 actor 插件对 dt 做了 0.1s 限幅（`ActorPluginRos.cpp:263-265`，防瞬移），
  所以"1.2s 走 5m"只能是**我们的估计错了**，不是人跑得快。
* 聚合指标（发布序列 vs 指令速度积分的逐秒残差，含速度模型误差，属上界）：
  中位 **0.40m**、p90 **1.18m**、**19% 的样本 >1m**。
* 与裁判日志吻合：15s 窗口能凑满（5 次删除）但平均每 ~7s 被清零一次（≈40 次 find）。

**误差预算**：人物 1.1~1.9 m/s × 链路延迟 ~0.3s = 0.35~0.6m（系统性滞后）；
裁判自己用 10Hz 采样的 `actors_pos` 对照，再加 0.15~0.6m；
粗差来源是 5×5 ROI 平均深度打到人背后的墙/地面，以及 TF 用"最新变换"
而不是图像时间戳（机体 1~2.5 m/s、时间不同步 → 最多 2.5m）。
⇒ 1m 门槛只剩 ~0.4m 余量，粗差必然打穿。

**已加统计**（`bbox2coord_node.py`，1s 节流一条）：
`解算精度统计(<class>): 本次 x.xxm | 均值 | 最大 | >1m n/N 次 | 检测延迟 x.xxs | 深度帧延迟 x.xxs | 采纳解算 n / 真值兜底 n / 丢弃 n`
—— 下一轮日志可以直接读出误差分布与"有多少帧真的用了视觉解算值"，不用再靠倒推。

**发布规则（2026-09-28 按你的要求实现）**：发之前先跟真值比 ——
`≤1m` 发解算值、`>1m` 走真值兜底、兜底关闭又超差就不发；消息率限到 ~10Hz。

---

## 三、修复内容（已编译 / 已跑测试）

### A. 坐标发布端 `src/yolo/bbox2coord_node.py`

| 改动 | 说明 |
|---|---|
| **发布前与真值比对（2026-09-28 用户定的规则）** | 解算值与真值差 **≤ `~visual_gate_radius`(默认 1.0m) → 发解算值**（保留真实视觉链路）；**超差 → 发真值兜底**；真值兜底关闭又超差 → **本帧不发**（发一个 >1m 错点会把裁判 15s 计时清零，比漏一帧代价大）。规则抽成纯函数 `choose_publish_source()`，已单测 |
| **深度采样合理性检查**（`~range_gate_*`，默认开） | ROI 平均深度与"人框高度反推的距离"差 > max(3m, 80%) → 按解算失败处理（交真值兜底）。专治 5×5 ROI 打到人背后墙导致的 4~6m 粗差 |
| **发布限频** `~publish_min_interval`（默认 0.1s = 10Hz） | 检测本身 ~10Hz，hold 重发再插进来会到 20Hz+；统一限到 ~10Hz（裁判只要求间隔 ≤1s，余量两个数量级） |
| **hold 20s → 3s**，**min_publish_span 18s → 0（关闭）** | 补发只用于填"漏检毛刺"（裁判只要求间隔 ≤1s），不再替裁判伪造 15s |
| **盲发上限**（新 `~max_blind_publish`，默认 1.5s） | 本机超过 1.5s 没真正检出该人物就停止补发（人不在画面里就别再报点） |
| **认错人闸门**（`~truth_geometry_check_radius`，默认 6m） | 解算值与真值差 >6m 时本帧不发：红色两个模型外观一样，YOLO 认错人时真值会指向另一个 actor，这道闸门防止"删掉没人跟过的人" |
| **订阅 `/left_actors`** | 裁判把人物移出名单（= 已判定消除）后立刻停发；比等 Gazebo 真正删模型（本轮实测延迟 ~20s）更早、更权威 |
| 诊断日志 | 真值发布改为 1s 节流；统计行含"采纳解算 / 真值兜底 / 丢弃"计数（见第二节） |

### B. 追踪端 `src/tracking/src/state_machine.cpp` / `.h`

| 改动 | 说明 |
|---|---|
| **订阅 `/left_actors`，人物被消除即归还控制权** | 不再"追一个已经拿不到分的人" |
| **单次追踪时长上限** `state_machine/max_tracking_seconds`（默认 90s） | 本轮 UAV2 被一个人拖住 **325s**（占全局 59%，期间任务级 PAUSE，全程没巡逻）——这是 2 号"没有导航输出"的直接原因 |
| **重捕冷却** `state_machine/reacquire_cooldown_seconds`（默认 60s） | 超时放弃后不会原地立刻又锁同一个人 |

### C. 巡逻/安全 `src/auto_explore/src/area_mission_node.cpp`

| 改动 | 说明 |
|---|---|
| **front_inhibit 锁存时间戳修复**（`ros::Time()` → `ros::Time::now()`） | 这是**死代码级 bug**：进入锁存时把起始时间写成 0，于是 `latch_expired` 恒 false，"锁存超 3s 强制放行"的兜底从未生效。实测 5 号在深度 0.85~2.5m、激光同时 <2.5m 时被连续锁停 8.5s 无释放（65 条 `force release after X` 里的 X 全等于当时仿真秒，正是 X = now − 0 的痕迹） |
| **释放计数加安全前置** | 只要任一传感器仍低于"进入阈值"，本周期不算净空，避免"激光 3m 但深度 1m"被 `\|\|` 判成 clear 而放行 |
| **DIAG 补 `pose_x/pose_y/pose_yaw` 与 `dist_wp`** | 本轮最大盲点：PATROL 段 `dist` 恒为 0（只在 ROAD 赋值）、日志里没有位置，导致"0 号是绕圈还是被墙困住"只能猜 |

### D. 启动参数 `src/yolo/multi_solving.sh`、`src/tracking/config/params.yaml`

新参数全部有默认值并可用环境变量覆盖：
`PUBLISH_HOLD_TIME=3.0`、`MIN_PUBLISH_SPAN=0.0`、`MAX_BLIND_PUBLISH=1.5`、
`VISUAL_GATE_RADIUS=1.0`、`PUBLISH_MIN_INTERVAL=0.1`、`PREFER_TRUTH_WHILE_TRACKING=0`、
`TRUTH_GEOMETRY_CHECK_RADIUS=6.0`、`RANGE_GATE_ENABLED=1`、`RANGE_GATE_ABS=3.0`、
`RANGE_GATE_REL=0.8`、`PERSON_HEIGHT=1.7`。环境变量同名即可覆盖，例如：

```bash
VISUAL_GATE_RADIUS=0.8 bash src/yolo/multi_solving.sh 6      # 门槛收紧到 0.8m
TRUTH_FALLBACK=0      bash src/yolo/multi_solving.sh 6       # 正式比赛口径（超差帧不发）
PREFER_TRUTH_WHILE_TRACKING=1 bash src/yolo/multi_solving.sh 6  # 追踪期间一律发真值
```

---

## 四、验证

1. **发布端端到端回归**（私有 roscore，不碰你的环境）：
   ```bash
   bash src/yolo/test/run_bbox_tests.sh          # 端口可用 ROS_TEST_PORT 覆盖
   ```
   6/6 通过（本轮实测数字）：
   * 真值兜底 + 18s 追踪：消息 **153 条**、跨度 **16.0s**、最大间隔 **0.24s** → 裁判 15s 窗口能凑满；
   * **发布限频**：平均 **9.5 Hz**（限制 10Hz；改造前同一场景是 ~18.6Hz）；
   * **精度闸门纯逻辑**：8 个用例（≤1m 发解算值 / >1m 发真值 / 无兜底则不发 / prefer_truth 等）全部符合；
   * 只追 3s：消息 **6 条**、跨度 **1.0s** → 不再伪造"找到"；
   * 多机一致性闸门：注入 20m 偏差期间本机发布 **0 条**；
   * `/left_actors` 闸门：裁判移除该人物后 **0 条**；
   * `find_start_delay`：首帧出现在 **+3.42s**（设置 3.0s）。
2. **编译**：`catkin build tracking auto_explore --no-deps` → 2/2 成功；
   `strings` 确认新字段（`dist_wp=`、`left_actors`）已进二进制。
3. **静态契约测试**：`python3 src/auto_explore/test/area_mission_static_test.py` 全部通过。

---

## 五、本轮还有一个更致命的问题：**越 6m → 直接判 0**（1044.9 → 0）

裁判日志最后三行：

```
actor_4 is OK            ← t≈409.5s（delete_model 返回）
Time usage: 409.492
score: 1044.9
Warning: UAV 1 is higher than 6 meters
score: 0                 ← score_cal.py:421-427：任何一架 z>6m → score=0 且立刻结束
```

也就是说：**5 个人物被消除、1044.9 分，被"某架机超过 6m"一次抹平。**
而 `sim.log` 里 UAV1 的 `pose_z` 在 t=505~558s 是这样的（每行间隔 1s）：

```
t=556.1  z=4.56   vz_pub=-0.04     t=557.1  z=6.29   vz_pub=0.00
t=553.0  z=4.58   vz_pub=-0.05     t=555.1  z=6.23   vz_pub=0.00
t=551.0  z=4.55   vz_pub=0.00      t=552.0  z=6.10   vz_pub=0.00
```

**估计高度在 1s 内于 4.55 ↔ 6.3 之间来回跳**（EKF 双分支/高度源切换的典型症状），
而 `vz_pub` 几乎恒为 0 —— 任务节点那道"`pose_z_>5.5` 就强制下压"的高守卫
（`area_mission_node.cpp` MISSION 分支）只在估计跳到高分支时才生效，所以压不住真实高度。
本轮各机估计高度峰值：UAV1 **6.32**、UAV3 **6.17**、UAV0 5.95、UAV2 5.92、UAV5 5.89、UAV4 5.79 ——
**六架都在往 6m 线上飘**，这是下一轮最该先解决的评分风险。

本轮为此加了两个诊断字段（默认开，`~diag_truth_z:=false` 可关）：

* `pose_x / pose_y / pose_yaw`：位置与朝向（PATROL 段"绕圈 vs 被困"的判读依据）；
* `pose_z_truth`：`/gazebo/model_states` 里本机的**真值高度**（1e9 = 未收到）。
  下一轮拿它和 `pose_z` 直接对照，就能判定"越 6m"是真超高还是 EKF 估计偏差，
  再决定是修高度环（守门阈值/双向控制）还是修里程计源头（vision z 漂移 / baro 重置）。
  在此之前**不要在任务节点里凭 `pose_z` 硬压高度**——估计本身在跳，越压越乱。

---

## 六、下一轮复跑请重点核对这几条日志

```bash
D=logs/sim_only_<新时间戳>
# ① 真值发布（追踪期间应持续出现，1s 一条节流）
grep "使用 Gazebo model_states 真值發布" $D/multi_solving.log | tail
# ② 短追踪不许凑 15s：应看到"停止补发坐标"或"连续发布结束：实际跨度 3~5s"
grep -E "停止补发坐标|连续发布结束" $D/multi_solving.log | tail
# ③ 裁判侧：find 与 is OK 的时间应明显靠拢（一个人 20~25s），且不再出现同一 actor 几十次 find
grep -vE "findDecoder" logs/referee_only_<新时间戳>/referee.log | grep -E "find actor|is OK|tracking success"
# ④ 追踪端：被消除后应立刻归还控制权
grep -E "no longer in referee /left_actors|max tracking duration|absent from fresh" $D/tracking.log | tail
# ⑤ 5 号：front_inhibit 的 force release 耗时应变成 ~3s（而不是等于仿真秒）
grep "force release after" $D/sim.log | tail
# ⑥ 0 号：新 DIAG 字段应能直接判读绕圈 vs 被困
grep "typhoon_h480_0 .*DIAG" $D/sim.log | tail -3   # 看 pose_x/pose_y/dist_wp
# ⑦ 高度（本轮判 0 的元凶）：pose_z 与 pose_z_truth 的差
grep -oE "pose_z=[-0-9.]+ vz_pub=[-0-9.]+|pose_z_truth=[-0-9.e+]+" $D/sim.log | tail -20
# ⑧ 解算精度/延迟分布 + 有多少帧真的用了视觉解算值（新增统计）
grep "解算精度统计" $D/multi_solving.log | tail -20
# ⑨ 精度闸门：解算值被采纳 vs 超差走真值兜底
grep -cE "改用 Gazebo 真值兜底" $D/multi_solving.log
grep -cE "深度采样可疑" $D/multi_solving.log
```

---

## 七、仍待确认 / 建议下一轮继续

0. **高度（优先级最高，见第五节）**：先看 `pose_z_truth` vs `pose_z`。
   若真值确实 >6m 而估计只有 4.5m（估计偏低 ~1.5m），问题在里程计/EKF 高度源，
   任务节点里加任何"下压"都只是在错误的反馈上打补丁；若两者一致、只是缓慢上飘，
   则应把高守卫从"`pose_z_>5.5` 才压"改成"接近目标高度就做双向闭环"并加 5.5m 硬上限。
1. **Gazebo 删除 actor 的延迟（已做独立复现实验，结论如下）**：
   本轮裁判 t≈409.5s 打印 `actor_4 is OK`（`delete_model` 返回成功），但 actor_4 直到
   **t≈431.2s** 才从 `/gazebo/model_states` 消失。独立实验（私有 roscore/gzserver +
   最小 world）结果：
   * **`delete_model` 的忙等会阻塞该插件共享的 ROS 回调队列** —— 删除期间
     `/gazebo/get_model_state` 单次耗时 = 忙等时长（实测 400.7ms / 204.6ms，基线 1.74ms），
     机制与 `gazebo_ros_api_plugin.cpp:826-895` + roscpp 单一 `CallbackQueue` 完全吻合。
     这解释了裁判主循环为什么冻结、`Time usage` 为什么停在 409.492。
   * **最小世界里 actor 删除只要 0.247s，没有 actor 特有的 20s 延迟**；
     且 `success=True` 只可能在 `ModelByName()` 变空之后返回，而 `model_states` 遍历的是同一张
     模型表 —— 所以本轮那个 ~21.7s 只能是**忙等本身持续了 21.7s**（`time_usage` 是被冻结的
     旧值），而不是"删完还继续出现在 model_states 里"。
     最可能的原因是全场馆（91 个模型 + 6 个带动画 actor + 6 架机 + 相机渲染）下
     Gazebo 世界更新被严重拖慢；最小 world 复现不出来，需要用完整 world 再验一次。
   * 对我们的意义：本轮修复已让链路**不再依赖"模型立刻消失"**
     （`/left_actors` 权威信号 + `max_tracking_seconds` 上限），这个延迟最多让我们多追 ~20s，
     不再影响得分。
   * 对裁判侧的建议（**未改任何代码**）：把 `delete_model` 换成异步调用/独立进程，
     评分主循环只轮询 `model_states`，即可彻底消除主循环冻结（冻结期间裁判对
     "越 6m / 失联"完全失明，本轮就是这样在恢复瞬间判了 0 分）。
2. **0 号的"绕圈 vs 被困"**：本轮已加 `pose_x/pose_y/dist_wp`，下一轮可用数据判定；
   若确认是 MPPI 近目标极限环（净位移/指令里程 ≈12%），再决定是改 hover watchdog 的
   触发条件（加"确实被阻挡"前置）还是修 MPPI 的近目标代价。
3. **恢复退出净空 4.5m**：`patrol_recovery_exit_clearance=4.5` 来自
   `laser_stop_max_(3.0)+robot_radius_(1.0)` 的安全推导，窄缝里永远达不到 → 每次 12s 空转
   （ACTIVE→TIMEOUT_HOLD→换边→GAVE_UP）。**本轮没有动它**（降它会有撞墙风险），
   等 0 号/5 号的新 DIAG 位置数据出来再评估。
4. **真实比赛模式（`TRUTH_FALLBACK=0`）**：现在已经实现"超差点**不发**"（而不是发错点）
   与深度采样合理性检查，剩下的两项还没做：① TF 改用图像时间戳而非"最新变换"；
   ② 按实测延迟做速度外推（解算值被丢弃的那几帧用短时预测顶住，避免 >1s 空洞）。
