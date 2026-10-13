# 本轮（18:48 那轮）复盘：判 0 的根因是本地位置估计发散 + 裁判退出导致后续全部无分

数据：`logs/referee_only_20260928_184848/`、`logs/sim_only_20260928_184905/`（裁判 18:48:48 起，sim 18:49:05 起，sim 跑到 18:53:31）。

## 1. "跟了绿色人物 40s+ 没被消除" = 裁判进程在起飞后 ~48s 就退出了

`referee.log` 全文只有 8 行：

```
UAV 0..5 took off at z=4.04~4.43; +10        （6 行）
Warning: UAV 5 is higher than 6 meters
score: 0
```

* `score.txt` = `0`，文件最后写入 **18:49:53**，而 sim 跑到 18:53:31。
* `score_cal.py:421-427`：`z>6m → score=0 + _finish() → signal_shutdown`，**裁判进程直接退出**，
  之后再也不会调用 `delete_model`。
* 本轮所有追踪都发生在裁判退出**之后**：blue1 t=88.0s、green0 t=104.9s、brown2 t=167.6s、
  red4 t=179.4s、white3 t=184.5s。**没有裁判 = 不可能有人被消除**。
* 顺带验证两个新机制：
  * 90s 追踪上限生效：UAV3 追 green0 从 104.85s 到 195.01s（90.2s）后 `TRACKING -> IDLE`；
  * 精度网关生效：`采纳解算 1019 / 真值兜底 671 / 丢弃 1`，误差 均值 0.91m、最大 8.86m、
    **检测延迟 0.04~0.06s**（比我上轮估的 0.3s 小得多，说明误差主要来自深度/几何而不是延迟）。

## 2. 判 0 的根因：本地位置（EKF/视觉里程计）估计发散

新加的 `pose_z_truth`（Gazebo 真值）第一次就把问题抓住了：

| 无人机 | `pose_z` 峰值 | 真值 z 峰值 | 真值 >6m 的采样数 |
|---|---|---|---|
| 0 | 5.78 | 6.05 | 44 |
| 1 | 5.82 | 6.68 | 39 |
| 2 | 5.48 | 7.86 | 21 |
| 3 | 5.86 | **10.44** | 31 |
| 4 | 5.39 | 5.87 | 29 |
| 5 | 6.22 | **10.33** | 30 |

* 起飞阶段两者只差 ~0.2m（0.17 vs 0.19），随后差距单调变大。
* UAV5：t=63.4s `pose_z=4.85` / 真值 5.03；t=65.6s `pose_z=4.84`（估计"没动"，且当时还在发
  `vz=-0.21` 的**下压**指令）/ 真值 **6.08** → 裁判立刻判 0。
  即：**估计几乎不动，真值以 ~0.5 m/s 往上跑**。
* 水平方向同样在发散：把 DIAG 里新的 `pose_x/pose_y` 逐样本比对（严格过滤掉多进程写日志
  造成的交错行后仍有 ~200 个干净样本/机），**35%~47% 的相邻 1s 采样位移 5~20m**
  （等效 5~119 m/s），而速度指令上限只有 1.5 m/s —— 只能是估计在跑，不是机体在飞。
  节点自带的跳变检测阈值是"**每控制周期 4m**"（50Hz），渐进发散每周期只有 ~0.2m，
  所以 `abnormal_pose` 全程为 0，抓不到。
* 机制：估计发散 → PX4 位置环按错误估计纠偏 → 真机真的飞走（含向上）→ 真值过 6m → 判 0。
  这就是"5 号莫名失控"。

## 3. 本轮已改（已编译进 devel）

| 位置 | 改动 |
|---|---|
| `area_mission_node` | 起飞/巡航/巡逻的高度环与高守卫改用 `altitudeForControl()`（真值新鲜时用真值，否则退回 `pose_z`，fail-open）；上限 `alt_ceiling_guard=5.2m`，超限用 `alt_hard_descend_rate=1.5 m/s` 强压（原来是 `-max_z_*0.5`≈-0.4，抵不过 0.5 m/s 的漂移） |
| `tracking_node` | 追踪期间任务节点是 PAUSE 的，所以护栏放在 tracker 的**最终输出**上：真值 >`alt_ceiling_guard=5.3m` → `linear.z=-1.5`；追踪高度环也用真值（否则"降到 3m"其实是真值 6m+）。`modelStatesCb` 顺便记本机真值高度 |
| 诊断 | 新增 `pose_x_truth/pose_y_truth`（与 `pose_z_truth` 一起），下一轮可直接确认水平发散幅度 |

参数：`altitude_truth_guard`(默认 true)、`alt_ceiling_guard`(5.2/5.3)、
`alt_hard_descend_rate`/`alt_descend_rate`(1.5)。没有 `/gazebo/model_states` 时全部自动失效。

## 4. 下一轮建议（按优先级）

1. **先压低 6m 风险再测别的**：把 `target_altitude` 从 4.5 调到 3.5（launch 参数），
   配合新的真值高度环，给"真值比估计高 1~2m"留余量。
2. **查位置发散的源头**：`pose_init` 把 `mavros/local_position/pose + spawn偏移` 发到
   `/typhoon_h480_N/mavros/vision_pose/pose`，而该话题名正是 MAVROS 的**外部视觉输入**。
   如果 EKF 真的融合它，就构成 `EKF → pose_init(+偏移) → EKF` 的闭环，注入常量偏移会形成持续漂移。
   下一轮先确认：这个 vision_pose 有没有真的回灌给飞控（mavros 插件/xtdrone bridge 配置）、
   EKF2 的 `EV_POS`/origin 设置。**这是"位置估计发散"最可疑的源头。**
3. 跑之前先自检裁判是否活着：`grep -c "took off" referee.log` 应为 6，
   一旦出现 `higher than 6 meters`，本轮分数与消除全部作废（进程已退出）。
4. 精度门槛可考虑收到 0.8m（`VISUAL_GATE_RADIUS=0.8`）：裁判自己也是 10Hz 采样，
   留 0.2m 余量，减少"我们判定 <1m 但裁判收到时 >1m"的窗口清零。
