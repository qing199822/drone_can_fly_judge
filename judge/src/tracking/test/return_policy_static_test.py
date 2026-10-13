#!/usr/bin/env python3
# =============================================================================
# tracking 归还策略静态契约测试（2026-09-28 用户决定）
#
#   正常持续可见的目标由裁判消除后归还；真实 YOLO 检测中断 5 秒后归还。
#   max_tracking_seconds 的普通超时路径仍受开关限制。
#
# 纯文本断言，不需要 roscore / Gazebo：
#   python3 src/tracking/test/return_policy_static_test.py
#
# 断言覆盖：
#   ① 开关 state_machine/return_only_on_elimination 存在且默认 true（yaml + C++ 装载 + 成员初值）；
#   ④ 每个 returnControlToMission( 调用点都被下面的分类表覆盖（数量与"被 gate 的数量"匹配）；
#   ② max_tracking_seconds 超时归还被该开关 gate（gate 时不写冷却 / 不清 target / 不标记完成）；
#   ③ /left_actors 与 model_states 连续缺失两条"消除类"路径**没有**被 gate。
# =============================================================================
import re
from pathlib import Path

import yaml

root = Path(__file__).parents[1]                       # src/tracking
source = (root / "src" / "state_machine.cpp").read_text()
header = (root / "include" / "tracking" / "state_machine.h").read_text()
params_path = root / "config" / "params.yaml"
params_text = params_path.read_text()
params = yaml.safe_load(params_text)

# -----------------------------------------------------------------------------
# 调用点分类表（报告里的表与此处逐条一致）
#   reason（returnControlToMission 第三个实参前面的字符串）
#     -> (类型, 是否被 return_only_on_elimination gate)
#   类型：elimination     = 消除类（裁判已把该人物消除，照旧归还）
#         non-elimination = 非消除类提前归还（用户决定停用）
#   safety = mux 切换失败等安全/异常保护：不走本表（它不调用 returnControlToMission）
# -----------------------------------------------------------------------------
CALL_SITES = {
    "actor disappeared (verified by model_states)": ("elimination", False),      # update() 全局复核
    "actor eliminated by referee (/left_actors)": ("elimination", False),        # 裁判权威信号
    "actor disappeared": ("elimination", False),                                 # handleLostState 内
    "max tracking duration reached": ("non-elimination", True),                  # 被 gate
}
GATED_COUNT = sum(1 for _cls, gated in CALL_SITES.values() if gated)
assert GATED_COUNT == 1, "按用户决定：只有 max_tracking_seconds 一条非消除类路径被 gate"


def _slice(start_anchor, end_anchor, why=""):
    assert start_anchor in source, f"缺少代码锚点（{why}）：{start_anchor}"
    start = source.index(start_anchor)
    assert end_anchor in source[start:], f"缺少代码锚点（{why}）：{end_anchor}"
    return source[start:source.index(end_anchor, start)]


# =============================== ① 开关 =====================================
sm = params["state_machine"]
assert "return_only_on_elimination" in sm, "params.yaml 必须有 state_machine/return_only_on_elimination"
assert sm["return_only_on_elimination"] is True, "开关默认必须是 true（只有裁判消除才归还）"
assert 'nh_.param("state_machine/return_only_on_elimination", return_only_on_elimination_, true);' in source, \
    "C++ 必须从参数服务器读取该开关且默认 true"
assert "bool return_only_on_elimination_ = true;" in header, "成员默认值必须是 true"
# 一键恢复：节点用私有 NodeHandle("~")，yaml 键带 state_machine/ 前缀，
# 因此必须显式接受扁平私有键 _return_only_on_elimination:=false 作为最高优先级覆盖。
assert 'nh_.getParam("return_only_on_elimination", override_val)' in source and \
       "override_val.getType() == XmlRpc::XmlRpcValue::TypeBoolean" in source, \
    "必须支持 _return_only_on_elimination:=false 一键恢复（扁平私有参数覆盖）"

# ===================== ④ 调用点全部被分类 + 唯一 gate =======================
sites = re.findall(r'returnControlToMission\(dt,\s*"([^"]+)"', source)
assert len(sites) == len(CALL_SITES), \
    f"调用点数量 {len(sites)} != 分类表条数 {len(CALL_SITES)}（新增/删除调用点必须同步分类表）"
assert set(sites) == set(CALL_SITES), f"存在未分类的调用点：{set(sites) ^ set(CALL_SITES)}"
assert source.count("returnControlToMission(") == len(CALL_SITES) + 1, \
    "returnControlToMission( 的出现次数必须 = 调用点 + 1（定义）—— 不允许有绕过分类表的调用"
# 分类表里的类型必须与实参一致：消除类传 true，非消除类传 false。
for reason, (cls, _gated) in CALL_SITES.items():
    expected = "true" if cls == "elimination" else "false"
    assert f'returnControlToMission(dt, "{reason}", {expected});' in source, \
        f"{reason} 的 elimination 实参应为 {expected}"
# 全文件只允许一处 gate 调用点（= max_tracking_seconds 分支）+ 一处函数定义：
# 任何"把消除类路径也 gate 掉"的改动都会让这里变成 3 处而被抓住。
_gate_def = source.index("bool TrackingStateMachine::suppressNonEliminationReturn(const std::string& what,")
assert source.count("suppressNonEliminationReturn(") == 2, \
    "suppressNonEliminationReturn( 只允许 1 处调用（max_tracking_seconds 分支）+ 1 处定义"
_gate_calls = [m.start() for m in re.finditer(r"suppressNonEliminationReturn\(", source) if m.start() < _gate_def]
assert len(_gate_calls) == 1, "全文件只允许一处 gate 调用点"
_cap_start = source.index("if (max_tracking_seconds_ > 0.0 && !tracking_session_start_time_.isZero()")
_cap_call = source.index('returnControlToMission(dt, "max tracking duration reached", false);')
assert _cap_start < _gate_calls[0] < _cap_call, "唯一的 gate 调用点必须位于 max_tracking_seconds 分支内"

# ========================= ② 超时归还被 gate ================================
_cap = _slice("if (max_tracking_seconds_ > 0.0 && !tracking_session_start_time_.isZero() &&",
              'returnControlToMission(dt, "max tracking duration reached", false);',
              "max_tracking_seconds 超时分支")
assert "if (suppressNonEliminationReturn(" in _cap, \
    "max_tracking_seconds 超时归还必须被 suppressNonEliminationReturn() gate"
_gate = _cap.index("if (suppressNonEliminationReturn(")
_else = _cap.index("} else {", _gate)
_gated_region = _cap[_gate:_else]
for forbidden in ("returnControlToMission", "target_reacquire_cooldown_until_[",
                  "completed_target_ids_", "currently_tracked_target_id_.clear()"):
    assert forbidden not in _gated_region, f"被 gate 的分支不得出现 {forbidden}（只打 1Hz WARN）"
assert _else < _cap.index("target_reacquire_cooldown_until_["), \
    "重捕冷却只能写在 gate 的 else（return_only_on_elimination=false）里"
assert "elimination ? 1 : 0" in source, "elimination 参数必须反映到日志"
assert "if (elimination && !currently_tracked_target_id_.empty()) {" in source, \
    "只有消除类归还才标记 completed_target_ids_（旧行为保持一致）"
assert "if (!return_only_on_elimination_) return false;" in source, \
    "开关=false 时必须放行（一键回退旧行为）"
assert sm.get("lost_timeout") == 5.0, "真实检测丢失后应在 5s 归还控制权"
assert sm.get("lost_buffer_frames") == 30, "短时漏检缓冲应保持约 1s"
_loss_start = source.index("const bool real_box_now =")
_loss_end = source.index("if (max_tracking_seconds_ > 0.0", _loss_start)
_loss_region = source[_loss_start:_loss_end]
assert "realBoxAgeSeconds()" in _loss_region
assert "!real_box_now" in _loss_region
assert "real_box_age >= lost_timeout_.toSec()" in _loss_region
assert 'forced_release_reason_ = "real detection lost timeout";' in _loss_region
assert "force_non_elimination_return = true;" in _loss_region
assert "suppressNonEliminationReturn" not in _loss_region, \
    "真实检测中断 5s 的归还不能被消除策略闸门阻断"
_visual_start = source.index("const darknet_ros_msgs::BoundingBox* locked_target_bbox = nullptr;")
_visual_end = source.index("switch (current_state_) {", _visual_start)
_visual_region = source[_visual_start:_visual_end]
assert "noteBoxAuthenticity(now, it->second);" in _visual_region
assert "if (!isSyntheticTruthBox(it->second)) {" in _visual_region and \
       "locked_target_bbox = &it->second;" in _visual_region, \
    "真值补框和漏检保持框不能驱动追踪，也不能触发 LOST -> TRACKING"
_tracking_start = source.index("void TrackingStateMachine::handleTrackingState(")
_tracking_end = source.index("void TrackingStateMachine::handleLostState(", _tracking_start)
assert "realBoxAgeSeconds() >= 1.0" in source[_tracking_start:_tracking_end], \
    "真实检测丢失约 1s 后必须开始 LOST 旋转搜索"
assert "const bool initial_lock_grace =" in source and \
       "if (initial_lock_grace)" in source, \
    "TRACKING 首次接管必须有短暂偏航保护"
assert "tracking_edge_yaw_limit_" in source and \
       "tracking_yaw_slew_limit_" in source, \
    "TRACKING 需要具备目标贴边和偏航变化率保护"
assert sm.get("target_bbox_area") == 1200.0
assert sm.get("initial_tracking_yaw_limit") == 0.3
assert sm.get("initial_tracking_yaw_window") == 1.0

# ========================= ③ 消除类路径不被 gate ============================
for anchor, call in (
    ("if (trackedActorConfirmedMissing(actor_missing_seconds) &&",
     'returnControlToMission(dt, "actor disappeared (verified by model_states)", true);'),
    ("if (trackedActorEliminatedByReferee()) {",
     'returnControlToMission(dt, "actor eliminated by referee (/left_actors)", true);'),
    ("if ((now - missing_it->second).toSec() >= MAX_LOST_SECONDS_) {",
     'returnControlToMission(dt, "actor disappeared", true);'),
):
    branch = _slice(anchor, call, f"消除类路径 {anchor}")
    assert "suppressNonEliminationReturn" not in branch, f"消除类路径不得被 gate：{anchor}"
    assert "return_only_on_elimination_" not in branch, f"消除类路径不得受开关影响：{anchor}"

# 不存在独立的 lost_timeout 归还路径（该参数已不再触发归还）。
for match in re.finditer(r"returnControlToMission\(dt,", source):
    window = source[max(0, match.start() - 600):match.start()]
    assert "lost_timeout_" not in window, "不得存在 lost_timeout_ 触发的归还路径"

# mux 切换失败属安全/异常保护：不 gate、保持追踪会话存活。
_mux_start = source.index("void TrackingStateMachine::returnControlToMission(")
_mux = source.index("if (!mux_ok) {", _mux_start)
_mux_block = source[_mux:_mux + 300]
assert "keeping tracking session alive" in _mux_block, "mux 切换失败必须保留追踪会话（安全保护）"
assert "suppressNonEliminationReturn" not in _mux_block, "安全保护不得被 gate"

# ========================= 日志与诊断 =======================================
assert '[%s_%s Tracker] return control: reason=%s (elimination=%d)' in source, \
    "归还必须打印明确原因行（[.. Tracker] return control: reason=%s (elimination=%d)）"
_helper_start = source.index("bool TrackingStateMachine::suppressNonEliminationReturn(const std::string& what,")
_helper = source[_helper_start:source.index("\n}\n", _helper_start)]
assert "ROS_WARN_THROTTLE(1.0" in _helper, "被 gate 的分支必须按 1Hz 打 WARN"
assert "session=%.1fs v_pub=%.2f w_pub=%.2f" in _helper, "WARN 必须含 session 时长与 w_pub/v_pub"
assert "last_published_v_" in _helper and "last_published_w_" in _helper
assert "last_published_v_ = twist_cmd.linear.x;" in source, "v_pub 必须取实际发布的 v"
assert "last_published_w_ = twist_cmd.angular.z;" in source, "w_pub 必须取实际发布的 w"
assert "double last_published_v_ = 0.0;" in header and "double last_published_w_ = 0.0;" in header
assert '"[%s_%s Tracker] return_only_on_elimination=true: KEEP TRACKING' in source, "gate WARN 串必须可 grep"

# ========================= params.yaml 文档 =================================
_cap_line = [ln for ln in params_text.splitlines() if ln.strip().startswith("max_tracking_seconds:")][0]
assert "不生效" in _cap_line and "return_only_on_elimination=true" in _cap_line, \
    "max_tracking_seconds 必须注明：return_only_on_elimination=true 时不生效"
_cd_line = [ln for ln in params_text.splitlines()
            if ln.strip().startswith("reacquire_cooldown_seconds:")][0]
assert "return_only_on_elimination=false" in _cd_line, \
    "reacquire_cooldown_seconds 必须注明仅在开关=false 时生效"
assert "_return_only_on_elimination:=false" in params_text, "必须写明一键回退方法"
assert "真实检测连续丢失 5s" in params_text, "必须写明真实检测丢失后的巡逻恢复条件"

print("tracking 归还策略静态契约检查通过：开关默认 true；max_tracking_seconds 超时归还被 gate"
      f"（被 gate 调用点 {GATED_COUNT}/{len(CALL_SITES)}，全文件唯一 gate 调用点）；"
      "消除类 3 条路径（/left_actors、model_states 全局复核、LOST 内 model_states）均未被 gate；"
      "真实检测丢失 5s 可归还；mux 失败安全保护未被 gate")

# =============================================================================
# ↓↓↓ 2026-09-28 任务1/任务2 追加断言（本轮新增；上面老断言逐字未改）↓↓↓
#
#   任务1：目标锁"租约 + 可抢占"（look_up：src/target_lookup_service.cpp + 头文件 + .srv）
#     ① ~lock_lease_seconds 存在且默认 20，可一键关（<=0 = 永不过期）；
#        ~lock_preempt_enabled 默认 true，false = 一键回退"永不抢占"；
#     ② 抢占路径存在，且同一 target 至多一个持有者（leases_ 是单槽 map 赋值，
#        没有"持有者列表"），未过期仍然拒绝；续租用现有 request 服务（.srv 未改）。
#   任务2："只有真值合成框续命"这一种非消除类例外会强制归还（state_machine.cpp）
#     ③ 触发被 90s（max_tracking_seconds）与 5s（synthetic_only_release_seconds）两个阈值约束；
#     ④ 该例外不写 completed_target_ids_（走 elimination=false 的同一条非消除类出口）。
# =============================================================================

# ------------------------- 任务1：look_up 租约与抢占 -------------------------
lookup_root = root.parent / "look_up"                    # src/look_up
lookup_src = (lookup_root / "src" / "target_lookup_service.cpp").read_text()
lookup_hdr = (lookup_root / "include" / "look_up" / "target_lookup_service.h").read_text()
srv_request = (lookup_root / "srv" / "RequestTarget.srv").read_text()
srv_release = (lookup_root / "srv" / "ReleaseTarget.srv").read_text()

# ① 租约参数：默认 20s，可一键关（<=0 等于永不过期）；抢占默认开、可一键回退。
assert 'pnh_.param("lock_lease_seconds", lock_lease_seconds_, 20.0);' in lookup_src, \
    "look_up 必须从 ~lock_lease_seconds 读租约时长且默认 20.0"
assert "double lock_lease_seconds_ = 20.0;" in lookup_hdr, "租约时长成员默认值必须是 20.0"
assert 'pnh_.param("lock_preempt_enabled", lock_preempt_enabled_, true);' in lookup_src, \
    "look_up 必须从 ~lock_preempt_enabled 读抢占开关且默认 true"
assert "bool lock_preempt_enabled_ = true;" in lookup_hdr, "抢占开关成员默认值必须是 true"
assert "if (lock_lease_seconds_ <= 0.0) return false;" in lookup_src, \
    "lock_lease_seconds<=0 必须表示'永不过期'（一键关租约 = 旧行为）"
assert "lease.lease_expiry = (lock_lease_seconds_ > 0.0)" in lookup_src, \
    "成功锁定时必须记录 lease_expiry（<=0 时写 ros::Time(0) = 永不过期）"
assert "it->second.last_renew = now;" in lookup_src, "续租必须刷新 last_renew"
assert "lease.last_renew = now;" in lookup_src, "首次锁定必须记录 last_renew"

# ② 抢占路径 + "同一时刻只有一个持有者"
assert "lock preempted:" in lookup_src, "抢占必须打印含 'lock preempted' 的 WARN"
for token in ("old_holder", "new_holder", "expired_for"):
    assert token in lookup_src, f"抢占 WARN 必须包含 {token}（target/原持有者/新持有者/过期时长）"
assert 'kLockLogPrefix << " lock preempted' in lookup_src, \
    "抢占 WARN 必须由 [lock] 前缀 + lock preempted 串开头（便于 grep）"
assert "std::map<std::string, LockLease> leases_;" in lookup_hdr, \
    "持有者必须存在'每 target 单槽'的 map 里（结构上不可能有两个持有者）"
for forbidden in ("std::vector<LockLease>", "std::vector<std::string> holders",
                  "std::multimap", "std::vector<LockLease> leases_"):
    assert forbidden not in lookup_hdr, f"不允许出现可容纳多个持有者的结构：{forbidden}"
assert "leases_[target_id] = lease;" in lookup_src, \
    "授予/抢占必须是单槽赋值（覆盖旧持有者），不能是 append"
assert "grantLeaseLocked(target_id, requester, now);" in lookup_src
# 抢占只在"租约已过期"分支里；抢占关闭时仍然拒绝（一键回退旧行为）
assert "if (leaseExpiredLocked(target_id, now)) {" in lookup_src, \
    "必须存在【租约已过期】分支（抢占入口；否则请求会永远被拒）"
_preempt_anchor = lookup_src.index("if (leaseExpiredLocked(target_id, now)) {")
_preempt_region = lookup_src[_preempt_anchor:lookup_src.index("// ④ 租约有效且请求者不是持有者", _preempt_anchor)]
assert "if (!lock_preempt_enabled_) {" in _preempt_region, \
    "租约过期分支里必须先判 ~lock_preempt_enabled（false = 永不抢占）"
assert "res.success = false;" in _preempt_region, "抢占关闭时必须拒绝新请求"
assert "res.success = true;" in _preempt_region, "抢占开启时必须让新持有者接管"
# 未过期（租约有效）→ 仍然拒绝，并保留原文日志便于下一轮 grep 对比
assert "请求被拒绝: 目标 '" in lookup_src and "已被其他无人机追踪" in lookup_src, \
    "租约未过期时的拒绝日志必须保留原文（下一轮 grep 拒绝次数用）"
assert "原持有者" in lookup_src and "的续租请求被拒绝" in lookup_src, \
    "被抢占的原持有者下次心跳必须收到 success=false（并有明确日志）"
# 续租用现有接口：requester 身份来自 callerid，.srv 一个字段都没改
assert "event.getCallerName()" in lookup_src, "requester 身份必须取自 callerid（不改 .srv）"
assert "ros::ServiceEvent<look_up::RequestTarget::Request," in lookup_src, \
    "request 回调必须用 ServiceEvent 才能拿到 callerid"
assert "ros::ServiceEvent<look_up::ReleaseTarget::Request," in lookup_src, \
    "release 回调必须用 ServiceEvent（只有持有者能释放）"
assert "string target_id" in srv_request and srv_request.count("---") == 1, \
    "RequestTarget.srv 不许改（仍然只有 target_id / success）"
assert "string target_id" in srv_release and srv_release.count("---") == 1, \
    "ReleaseTarget.srv 不许改（仍然只有 target_id / success）"
# 释放必须按持有者收窄（否则被抢占的原持有者会把新持有者的锁抢掉）
_rel_anchor = lookup_src.index("bool TargetLookupService::handleReleaseTarget(")
_rel_region = lookup_src[_rel_anchor:lookup_src.index("std::string TargetLookupService::requesterFromEvent(", _rel_anchor)]
assert "release REFUSED" in _rel_region and "*holder != requester" in _rel_region, \
    "非持有者的释放必须被拒绝（保护新持有者的锁）"

# ------------------ 任务1：tracking 侧心跳续租 / 抢占处理 ------------------
assert "void TrackingStateMachine::heartbeatTargetLock() {" in source, \
    "task1 必须实现 heartbeatTargetLock()（同一 requester 再 request 同一 target = 续租）"
assert "heartbeatTargetLock();" in source, "update() 每次循环必须调用心跳"
assert "services_.renewTargetLock(target_id)" in source, "心跳必须走 ServiceManager::renewTargetLock"
assert "LockRenewResult::LOST_PREEMPTED" in source, \
    "心跳被拒（success=false）必须被识别为'锁被抢占'"
assert "bool lock_preempted_pending_ = false;" in header, "抢占标志必须是成员并有明确初值"
# 抢占优先于"继续追"策略：锁已经被抢走时不得再压制归还（否则两机同时追同一个人）
assert "if (lock_preempted_pending_) return false;" in source, \
    "锁被抢占时 suppressNonEliminationReturn 必须放行（抢占优先于 return_only_on_elimination）"
assert "if (!force_non_elimination_return && syntheticOnlyHoldExpired(real_box_age)) {" in source, \
    "已有强制归还原因时不得再覆盖成'合成框例外'（锁抢占和检测丢失原因优先）"
assert "bool force_non_elimination_return = lock_preempted_pending_;" in source, \
    "被抢占后必须强制走非消除类归还（不能用 elimination=true 假称裁判消除）"
assert "lock preempted: target '%s' 的锁已被其它机接管" in source, \
    "被抢占必须打印可 grep 的 WARN（含 lock preempted）"
assert 'nh_.param("state_machine/lock_heartbeat_seconds", lock_heartbeat_seconds_, 5.0);' in source, \
    "心跳周期必须参数化且默认 5.0s（< look_up 租约 20s）"
assert "double lock_heartbeat_seconds_ = 5.0;" in header, "心跳周期成员默认值必须是 5.0"
assert sm.get("lock_heartbeat_seconds") == 5.0, "params.yaml 必须写死心跳周期 5.0"
assert "LockRenewResult::UNAVAILABLE" in source and "lock heartbeat service unreachable" in source, \
    "服务不可达 ≠ 锁丢了：必须区分 UNAVAILABLE，避免 lookup 重启时误放控制权"
assert "renewTargetLock" in (root / "include" / "tracking" / "service_manager.h").read_text(), \
    "ServiceManager 必须提供 renewTargetLock 接口"
assert "enum class LockRenewResult { RENEWED, LOST_PREEMPTED, UNAVAILABLE };" in \
    (root / "include" / "tracking" / "service_manager.h").read_text(), \
    "心跳结果必须是三态（续租成功 / 被抢占 / 服务不可达）"

# ---------------- 任务2：真框 vs 合成框（不改 yolo 代码的判据） ----------------
assert "bool TrackingStateMachine::isSyntheticTruthBox(" in source, \
    "task2 必须能区分真框与 yolo11n.py 的真值合成框"
assert "kSyntheticProbabilityMarker = 0.999" in source and \
       "box.probability >= kSyntheticProbabilityMarker" in source, \
    "合成框判据必须是现有约定 probability≈1.0（真框 probability=float(conf)<1.0）"
assert "noteBoxAuthenticity(now, it->second);" in source, \
    "每帧必须为当前追踪目标的框记录真/合成（合成框不得刷新真框年龄）"
_auth = source[source.index("void TrackingStateMachine::noteBoxAuthenticity("):
               source.index("double TrackingStateMachine::realBoxAgeSeconds() const {")]
assert "last_synthetic_box_time_ = now;" in _auth and "return;" in _auth, \
    "合成框只能记录 synthetic 时间并 return（绝不刷新 last_real_box_time_）"
assert "last_real_box_time_ = now;" in _auth, "真框必须刷新 last_real_box_time_"
# 与 yolo11n.py 的耦合：合成框 probability 约定必须仍是 1.0（改了这里必须同步改 tracking）
_yolo_src = (root.parent / "yolo" / "yolo11n.py").read_text()
_yolo_syn = _yolo_src[_yolo_src.index("def add_synthetic_tracking_box"):
                      _yolo_src.index("def image_callback")]
assert "bbox.probability = 1.0" in _yolo_syn, \
    "yolo11n.py 合成框的 probability 约定变了（不再是 1.0）→ tracking 侧判据必须同步修改"
assert "bbox.probability = float(conf)" in _yolo_src, \
    "真实推理框必须仍然写 float(conf)（保证 <1.0，与合成框可区分）"

# ------------- 任务2 ③④：合成框-only 超时强制归还（90s + 5s 约束） -------------
assert "bool syntheticOnlyHoldExpired(double real_box_age_seconds) const;" in header
assert "double synthetic_only_release_seconds_ = 5.0;" in header, "5s 阈值成员默认值必须是 5.0"
assert "bool synthetic_only_release_enabled_ = true;" in header, "合成框例外默认必须开启"
assert 'nh_.param("state_machine/synthetic_only_release_seconds", synthetic_only_release_seconds_, 5.0);' in source
assert 'nh_.param("state_machine/synthetic_only_release_enabled", synthetic_only_release_enabled_, true);' in source
assert sm.get("synthetic_only_release_seconds") == 5.0, "params.yaml 必须写死 5.0"
assert sm.get("synthetic_only_release_enabled") is True, "params.yaml 必须写死 true（默认开）"
assert 'nh_.getParam("synthetic_only_release_enabled", override_val)' in source, \
    "必须支持 _synthetic_only_release_enabled:=false 一键关（扁平私有参数）"
# 90s 约束：例外判定必须写在 max_tracking_seconds 分支内（老断言已锁死该分支的 gate 结构）
_syn_call = source.index("syntheticOnlyHoldExpired(real_box_age)")
_syn_region = source[_syn_call:
                     source.index('returnControlToMission(dt, "max tracking duration reached", false);',
                                  _syn_call)]
assert _cap_start < _syn_call < source.index('returnControlToMission(dt, "max tracking duration reached", false);'), \
    "合成框例外必须位于 max_tracking_seconds（90s）分支内，且走同一条非消除类归还出口"
assert "real_box_age = realBoxAgeSeconds()" in source, "必须用真框年龄参与判定"
_syn_fn = source[source.index("bool TrackingStateMachine::syntheticOnlyHoldExpired(double real_box_age_seconds) const {"):
                 source.index("// ============================================================================\n// 归还控制权（唯一出口）")]
assert "if (real_box_age_seconds <= synthetic_only_release_seconds_) return false;" in _syn_fn, \
    "5s 约束：真框年龄未超阈值时不得强制归还"
assert "if (!synthetic_only_release_enabled_) return false;" in _syn_fn, \
    "一键关：开关=false 时不得强制归还"
assert "if (!truthActorPresentForTrackedTarget()) return false;" in _syn_fn, \
    "必须有 /gazebo/model_states 佐证（actor 仍在真值列表里 → 不是消除类）"
# ④ 例外不写 completed_target_ids_，也不冒充"消除"
assert "completed_target_ids_" not in _syn_region, \
    "合成框例外不得写 completed_target_ids_（人物并未被消除）"
# 例外唯一的出口就是这条 elimination=false 的调用（区间内不允许出现别的归还调用）。
_syn_next_call = source.index("returnControlToMission(", _syn_call)
assert source[_syn_next_call:_syn_next_call + len('returnControlToMission(dt, "max tracking duration reached", false);')] == \
    'returnControlToMission(dt, "max tracking duration reached", false);', \
    "例外必须走 elimination=false 的同一条非消除类出口（不得另开 elimination=true 的调用）"
assert "returnControlToMission(" not in _syn_region, "例外区间内不得出现其它归还调用"
assert "forced_release_reason_ = \"synthetic-only hold released\";" in _syn_region, \
    "例外必须把官方 return control 行的 reason 覆盖为 synthetic-only hold released"
assert "reason=synthetic-only hold released" in source, "日志必须可直接 grep 'synthetic-only hold released'"
assert "reason=%s (elimination=%d)" in source and "effective_reason" in source, \
    "官方 return control 行必须打印真实原因（forced_release_reason_ 覆盖）"
assert "const char* forced_release_reason_ = nullptr;" in header, "reason 覆盖必须是成员且初值为空"
# 正常"真框持续可见"的追踪不许放手：例外只由 syntheticOnlyHoldExpired 触发
assert source.count("syntheticOnlyHoldExpired(") == 2, \
    "syntheticOnlyHoldExpired 只允许 1 处定义 + 1 处调用（多出来的调用点会绕过阈值约束）"
assert header.count("syntheticOnlyHoldExpired(") == 1, "只允许 1 处声明"

print("tracking 任务1/任务2 追加契约检查通过：look_up 租约默认 20s（<=0 永不过期）+ 抢占默认开、"
      "单槽 leases_ 保证同一 target 只有一个持有者、续租复用 request 服务（.srv 未改）、"
      "释放按持有者收窄；tracker 心跳 5s 续租、被抢占走 elimination=false 归还；"
      "合成框例外被 90s+5s 约束、有 model_states 佐证、不写 completed_target_ids_")
