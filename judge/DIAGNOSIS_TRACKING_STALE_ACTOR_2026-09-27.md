# Stale actor 修复：bbox2coord 停发 + tracker 停止续 motion（2026-09-27）

> 任务来源：裁判 `score_cal` 找到并 `delete_model` 人物后，`/actor_red2_info` 仍在发
> 已删除 actor_4 的坐标（坐标实际落在 actor_5 附近），UAV1 的 tracker 永远处于
> `TRACKING (red4)`、`publishActorEscapeMotions` 每 5s 无限续 motion，
> 直到全局限时判 0。**按你的要求：不改 `score_cal.py`。**

## 1. 修复内容（两处，均已编译/语法通过）

### ① `src/yolo/bbox2coord_node.py`：发布前用新鲜 `/gazebo/model_states` 复核人物是否还在

* 新增 `self.model_states_stamp_wall`（墙钟时间戳）与 `self.model_states_fresh_seconds`（默认 2s），
  在 `_model_states_callback` 里刷新；`actor_ground_truth` 本身就是"当前在场的 actor 名单"。
* 新增 `_actor_confirmed_missing(actor_id)`：
  **仅当** 收到过 model_states **且**数据新鲜（<2s）**且**名单里确实没有该 actor 时返回 True；
  从未收到/数据过期时返回 False（**fail-open**，不影响没有 model_states 的部署）。
* 在 `_publish_actor_info()` 的**唯一发布出口**加闸门（YOLO+depth 路径、真值兜底路径、
  hold 重发 `_publish_timer_cb` 全部经过这里）：
  确认 actor 已删除 → 本条不发，并清掉 `latest_estimate / hold_until / publish_start / hold_expired`，
  防止 hold 线程继续重发；同时打印 5s 节流的 WARN。

### ② `src/tracking/src/state_machine.cpp`：tracker 用 model_states 兜底，主动归还控制权

* 新增 `TrackingStateMachine::trackedActorConfirmedMissing(double& missing_seconds)`
  （`state_machine.h` 同步声明）：
  复用已有的 `existing_actor_indices_` + `last_model_states_stamp_` + `sensor_timeout_`（0.5s）
  判断"当前追踪的 actor 是否已从 Gazebo 消失"，缺失时间记在 `actor_missing_since_`（与 LOST 路径共用）。
  同样 fail-open：model_states 未收到/过期 → 不判定。
* `update()` 里在**所有状态**下、`publishActorEscapeMotions()` 之后做一次复核：
  连续缺失 ≥ `MAX_LOST_SECONDS_`（默认 5s）→ 立即
  `returnControlToMission(dt, "actor disappeared (verified by model_states)", true)` 并 return。
  这条路径内部会：停 escape motion → 切回 navigator mux → 发 `RESUME:` → 发 `IDLE:` 状态
  → 把该 target 记入 `completed_target_ids_` 并 `releaseTarget()` → `currently_tracked_target_id_` 清空、
  状态置 IDLE。因此即使 YOLO 还在误报 red4，tracker 也会在 ~5s 内收尾，不会无限 TRACKING。
* `publishActorEscapeMotions()`：按你的建议，`still_tracking_this_actor` 为真时**必须先确认
  `existing_actor_indices_` 仍包含该 actor_index**（且 model_states 新鲜）才续 motion；
  否则不再 `generateActorEscapeMotion`，直接走既有的"发零速 + shutdown publisher + 移除"分支。

## 2. 为什么这样改（与现状的兼容性）

* **不改 `score_cal.py`**：接口话题 `/actor_*_info`（含 red1/red2 → actor_5/actor_4 的映射）完全不变，
  `score_cal` 侧零改动；只是"人物不存在时不再发坐标"。
* **fail-open**：两处判定都要求"收到过 model_states 且新鲜"，否则保持原行为；
  现场若没有 `/gazebo/model_states` 也不会退化。
* **静默期**：判定阈值用 tracker 既有的 `MAX_LOST_SECONDS_`(5s)，与 LOST 状态机一致，
  不会因为一帧 model_states 抖动就误放掉正在跟踪的人。
* **不重复发零速**：`publishActorEscapeMotions` 的清理分支已经会 publish zero + 摘 publisher，
  helper 只做判定，动作仍走原路径。

## 3. 验证

* `python3 -m py_compile src/yolo/bbox2coord_node.py` 通过。
* `catkin build tracking --no-deps` 通过（无 error/warning 与本次改动相关）。
* 端到端场景（find→delete actor→YOLO 误报）需要一整轮带评分仿真的复现；
  建议复跑后在日志里核对三件事：
  1. `bbox2coord` 出现 `actor_N 已不在 /gazebo/model_states（裁判已删除人物），停止发布 ...`；
  2. tracker 出现 `tracked actor 'redN' absent from fresh /gazebo/model_states for X.Xs ... returning control to mission`，
     随后 `State: TRACKING -> IDLE`；
  3. `/actor_red2_info`（或对应红色话题）在该 actor 删除后 **1~2s 内停止更新**（`rostopic hz` 归零）。
* 复现脚本（供下一轮自测）：
  ```bash
  # 裁判端删人后观察
  rostopic hz /actor_red2_info            # 期望：删除后很快不再有新消息
  rostopic echo -n1 /typhoon_h480_1/tracking_node/yolo_human_tracking_typhoon_h480_1  # 期望：IDLE::
  grep -E "stop escape motion|returning control to mission|停止发布" logs/sim_only_*/sim.log logs/sim_only_*/tracking.log 2>/dev/null | tail
  ```

## 4. 备注

* 本次改动只涉及 `bbox2coord_node.py`、`state_machine.cpp`、`state_machine.h` 三个文件。
* 之前放在 `logs/my_diag/` 的诊断产物（航点映射报告等）被另一会话的日志清理删掉了；
  源码侧的航点/坐标修复仍在工作区（`uav_world_wps`、`patrol_*` 等 grep 可见），本报告改放到仓库根目录。
