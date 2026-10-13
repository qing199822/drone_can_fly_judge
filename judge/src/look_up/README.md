# look_up

该节点实现了一种查询服务：当追踪节点要锁定目标时，会先查询该目标是否被其他无人机锁定，当返回值为False时，无人机就开始跟踪该目标，并将这个目标置为True，此时其他无人机即使再发现这个目标也不会触发追踪；当目标消失时请求重置为False；这样就实现了一个目标同一时间只能被一架无人机锁定

spawn_mux_swarm.launch 是一个总控节点，用于在导航控制和跟踪控制之间切换

---

## 2026-09-28 任务1：目标锁"租约 + 可抢占"

### 为什么改

上一轮实测：某人物一旦被某机锁定，其它机的请求**全被拒**（`target_lookup_service.cpp` 的
"请求被拒绝" 分支，本轮共 2114 次拒绝）；叠加用户策略 `return_only_on_elimination=true`
（只在人物被裁判消除时才归还控制权），一旦锁错/追不上就**永不释放** —— 3 号抱着 white3
自旋 188s（其解算误差均值 55.6m），0 号连续 32.5s 出框 656 次却始终进不了 DETECTING。

### 现在的语义

| 场景 | 行为 |
| --- | --- |
| 目标空闲 | 批准并建立租约（记录 `holder / granted_at / last_renew / lease_expiry`） |
| 持有者**再次** request 同一 target | **续租**：刷新 `last_renew / lease_expiry`（复用现有 request 服务，**没有改 .srv**） |
| 租约**未过期**、别的机来请求 | 仍然拒绝（老行为，日志原文保留，便于下一轮 grep 对比次数） |
| 租约**已过期**、别的机来请求、抢占开启 | **抢占**：新请求者接管；打印 `[lock] lock preempted: target=... old_holder=... new_holder=... expired_for=...s` |
| 租约已过期、抢占关闭 | 仍然拒绝，打印 `lease expired but preempt DISABLED` |
| 原持有者被抢占后的下一次心跳 | 收到 `success=false`（日志 `...的续租请求被拒绝：...已被 X 抢占`）→ tracking 侧归还控制权 + 释放目标 |
| 释放 | **只有持有者能释放**；被抢占的原持有者释放会被拒绝（`release REFUSED`），避免把新持有者的锁抢掉 |

"同一时刻只有一个持有者"由结构保证：`std::map<std::string, LockLease> leases_`
是**每 target 单槽**的 map，授予/抢占都是 `leases_[target_id] = lease;`（覆盖，不 append）。

持有者身份来自 ROS 服务连接头的 `callerid`（`ros::ServiceEvent::getCallerName()`），
例如 `/typhoon_h480_0/tracking_node` —— 所以**不需要**在 `.srv` 里加 requester 字段。

### 参数与一键回退

| 参数（私有 `~`） | 默认 | 说明 |
| --- | --- | --- |
| `~lock_lease_seconds` | `20.0` | 锁租约时长（s）。**`<=0` = 永不过期**（等价旧行为，一键关租约） |
| `~lock_preempt_enabled` | `true` | `false` = 一键回退"永不抢占"（租约过期也拒绝新请求 = 旧行为） |

命令行/launch 覆盖示例（不用改代码）：

```bash
# 回到旧行为（锁永不过期、永不抢占）
rosrun look_up target_lookup_service _lock_lease_seconds:=0 _lock_preempt_enabled:=false
# 现场调参：租约 30s、仍允许抢占
rosrun look_up target_lookup_service _lock_lease_seconds:=30
```

tracking 侧对应的参数是 `state_machine/lock_heartbeat_seconds`（默认 5.0s，必须 < 租约时长），
见 `src/tracking/config/params.yaml`；心跳被拒的处理在 `tracking/src/state_machine.cpp`
的 `heartbeatTargetLock()`。

### 静态契约测试

```bash
python3 src/look_up/test/lock_lease_static_test.py
python3 src/tracking/test/return_policy_static_test.py   # 末尾追加了任务1/任务2 的契约
```
