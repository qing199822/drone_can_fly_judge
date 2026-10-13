#!/usr/bin/env python3
# =============================================================================
# look_up 目标锁"租约 + 可抢占"静态契约测试（2026-09-28 任务1）
#
# 背景：上一轮实测某人物一旦被某机锁定，其它机的请求**全被拒**（2114 次），叠加
#   return_only_on_elimination=true（只在人物被裁判消除时才归还）→ 一旦锁错/追不上
#   就永不释放（3 号抱着 white3 自旋 188s）。修法：锁带租约（默认 20s）+ 过期可抢占。
#
# 纯文本断言，不需要 roscore / Gazebo：
#   python3 src/look_up/test/lock_lease_static_test.py
#
# 断言覆盖：
#   ① ~lock_lease_seconds 存在且默认 20.0（<=0 = 永不过期，一键关）；
#      ~lock_preempt_enabled 默认 true（false = 一键回退"永不抢占"，拒绝日志与旧版一致）；
#   ② 每次成功锁定记录 lease_expiry + last_renew；持有者再次 request 同一 target = 续租；
#   ③ 租约过期后其它机的请求能接管；抢占 WARN 含 target / 原持有者 / 新持有者 / 过期时长；
#   ④ 同一时刻只有一个持有者：leases_ 是"每 target 单槽"map 单槽赋值，没有持有者列表；
#   ⑤ requester 身份取自 callerid（ServiceEvent），.srv 未改（仍然只有 target_id / success）；
#   ⑥ 释放按持有者收窄：被抢占的原持有者无法释放新持有者的锁；
#   ⑦ 原持有者下次心跳收到 success=false（tracking 侧据此归还控制权 + 释放目标）。
# =============================================================================
from pathlib import Path

root = Path(__file__).parents[1]                       # src/look_up
source = (root / "src" / "target_lookup_service.cpp").read_text()
header = (root / "include" / "look_up" / "target_lookup_service.h").read_text()
srv_request = (root / "srv" / "RequestTarget.srv").read_text()
srv_release = (root / "srv" / "ReleaseTarget.srv").read_text()
launch = (root / "launch" / "target_lookup_service.launch").read_text()

tracking_root = root.parent / "tracking"               # src/tracking（只读，用于交接契约）
tracking_sm = (tracking_root / "src" / "state_machine.cpp").read_text()
tracking_sm_hdr = (tracking_root / "include" / "tracking" / "state_machine.h").read_text()
tracking_svc_hdr = (tracking_root / "include" / "tracking" / "service_manager.h").read_text()

# ========================= ① 租约/抢占参数 ==================================
assert 'pnh_.param("lock_lease_seconds", lock_lease_seconds_, 20.0);' in source, \
    "必须从 ~lock_lease_seconds 读租约时长，默认 20.0s"
assert "double lock_lease_seconds_ = 20.0;" in header, "租约时长成员默认值必须是 20.0"
assert 'pnh_.param("lock_preempt_enabled", lock_preempt_enabled_, true);' in source, \
    "必须从 ~lock_preempt_enabled 读抢占开关，默认 true"
assert "bool lock_preempt_enabled_ = true;" in header, "抢占开关成员默认值必须是 true"
assert "ros::NodeHandle pnh_;" in header, "参数必须来自私有句柄 ~（launch/命令行 _lock_xxx:= 可一键覆盖）"
assert "lock lease enabled: lease=%.1fs preempt_enabled=%s" in source, \
    "启动日志必须打印租约时长与抢占开关（下一轮可直接确认参数生效）"
# 一键关租约（<=0 = 永不过期 = 旧行为）与一键关抢占（false = 永不抢占）
assert "if (lock_lease_seconds_ <= 0.0) return false;" in source, \
    "lock_lease_seconds<=0 必须表示'永不过期'（一键关租约）"
assert "lock lease DISABLED (lock_lease_seconds=%.1f <= 0)" in source, \
    "租约关闭时必须有明确 WARN（说明回到旧行为）"
assert "if (!lock_preempt_enabled_) {" in source, "必须存在'抢占关闭'分支"
assert "lease expired but preempt DISABLED" in source, "抢占关闭时必须打印明确 WARN"

# ==================== ② 租约记录 + 续租（复用现有接口）====================
for field in ("std::string holder;", "ros::Time granted_at;", "ros::Time last_renew;",
              "ros::Time lease_expiry;"):
    assert field in header, f"租约记录必须包含字段：{field}"
assert "lease.lease_expiry = (lock_lease_seconds_ > 0.0)" in source, \
    "成功锁定（grantLeaseLocked）必须写 lease_expiry；<=0 时写 ros::Time(0)（永不过期）"
assert "lease.last_renew = now;" in source, "首次锁定必须记录 last_renew"
assert "it->second.last_renew = now;" in source, "续租必须刷新 last_renew"
assert "it->second.lease_expiry = (lock_lease_seconds_ > 0.0)" in source, "续租必须刷新 lease_expiry"
assert "const bool same_holder = (holder != nullptr) && (*holder == requester);" in source, \
    "续租判据必须是'同一 requester + 同一 target'"
assert "if (same_holder) {" in source and "renewLeaseLocked(target_id, now);" in source, \
    "同一持有者再 request 同一 target 必须走续租（不新建锁、不拒绝）"
assert 'ROS_INFO_THROTTLE(5.0, "%s lock renewed: target=' in source, \
    "续租必须按节流打印可 grep 的 lock renewed（1Hz 级别，避免刷屏）"
# 续租用**现有** request 服务：没有新增服务类型 / 没有改 .srv
assert "string target_id" in srv_request and srv_request.count("---") == 1, \
    "RequestTarget.srv 不许改（仍然只有 target_id / success）"
assert "string target_id" in srv_release and srv_release.count("---") == 1, \
    "ReleaseTarget.srv 不许改（仍然只有 target_id / success）"
assert "add_service_files(" in (root / "CMakeLists.txt").read_text(), "服务定义方式不变"

# ======================= ③ 过期可抢占 + 抢占日志 ============================
assert "bool TargetLookupService::leaseExpiredLocked(" in source, "必须有租约过期判定"
assert "return now >= it->second.lease_expiry;" in source, "过期判据必须是 now >= lease_expiry"
assert "if (leaseExpiredLocked(target_id, now)) {" in source, \
    "请求处理必须先在'租约已过期'分支里决定是否抢占"
_preempt_anchor = source.index("if (leaseExpiredLocked(target_id, now)) {")
_preempt_end = source.index("// ④ 租约有效且请求者不是持有者", _preempt_anchor)
_preempt = source[_preempt_anchor:_preempt_end]
assert "grantLeaseLocked(target_id, requester, now);" in _preempt, \
    "抢占必须把租约授予新请求者（新持有者接管）"
assert "res.success = true;" in _preempt, "抢占后必须让新持有者的请求成功（这正是'不再全被拒'）"
assert "res.success = false;" in _preempt, "抢占关闭时仍必须拒绝"
assert "kLockLogPrefix" in source and "lock preempted:" in _preempt, \
    "抢占必须打印 [lock] lock preempted（便于下一轮 grep \"lock preempted\"）"
for token in ("old_holder", "new_holder", "expired_for", "lock_lease_seconds_"):
    assert token in _preempt, f"抢占 WARN 必须包含 {token}"
assert "preempt_count" in _preempt, "抢占必须计数（可确认本轮真的发生了接管）"
# 原持有者会收到明确的"锁被抢占"信号：下一次心跳 success=false（有专门日志）
assert "preempted_holder_[target_id] = old_holder;" in _preempt, \
    "必须记录被抢占的原持有者"
assert "原持有者" in source and "的续租请求被拒绝" in source, \
    "原持有者下次心跳必须被明确告知'锁已被抢占'（success=false）"

# ==================== ④ 同一时刻只有一个持有者 ==============================
assert "std::map<std::string, LockLease> leases_;" in header, \
    "持有者必须存在'每 target 单槽'的 map 里"
for forbidden in ("std::vector<LockLease>", "std::vector<std::string> holders", "std::multimap",
                  "std::map<std::string, std::vector"):
    assert forbidden not in header, f"不允许出现可同时容纳多个持有者的结构：{forbidden}"
assert "leases_[target_id] = lease;" in source, "授予/抢占必须是单槽赋值（覆盖旧持有者）"
assert source.count("leases_.insert(") == 0 and source.count("leases_.emplace(") == 0, \
    "不允许 append 式写入（append 会造出两个持有者）"
assert "target_status_[target_id] = STATE_TRACKED;" in source and \
       "target_status_[target_id] = STATE_AVAILABLE;" in source, \
    "状态机仍然是 AVAILABLE/TRACKED 两态（未引入'多持有者'状态）"
# 未过期时仍然拒绝（不削弱语义），并保留原文拒绝日志
assert "请求被拒绝: 目标 '" in source and "已被其他无人机追踪" in source, \
    "租约未过期时必须仍然拒绝，并保留原文日志（下一轮 grep 对比拒绝次数）"

# ================= ⑤ requester 身份来自 callerid（.srv 未改）===============
assert "#include <ros/service_callback_helper.h>" in header, "必须 include ServiceEvent 头"
assert "event.getCallerName()" in source, "requester 身份必须取自 callerid"
assert "ros::ServiceEvent<look_up::RequestTarget::Request," in source and \
       "ros::ServiceEvent<look_up::ReleaseTarget::Request," in source, \
    "两个回调都必须用 ServiceEvent 版本"
assert "boost::function<bool(RequestTargetEvent&)>" in source and \
       "boost::function<bool(ReleaseTargetEvent&)>" in source, \
    "advertiseService 必须显式绑成 ServiceEvent 回调（否则重载定型失败）"
assert "anonymous#" in source, "callerid 缺失时必须给匿名调用者一个绝不重合的身份"

# ==================== ⑥ 释放按持有者收窄（保护新持有者）====================
_rel_anchor = source.index("bool TargetLookupService::handleReleaseTarget(")
_rel_end = source.index("std::string TargetLookupService::requesterFromEvent(", _rel_anchor)
_release = source[_rel_anchor:_rel_end]
assert "*holder != requester" in _release, "释放必须判断调用者是否为持有者"
assert "release REFUSED" in _release, "非持有者释放必须被拒绝并有明确日志"
assert "res.success = false;" in _release, "拒绝时必须返回 success=false（调用方不会误以为已释放）"
assert "leases_.erase(target_id);" in _release, "持有者释放时租约必须立即失效"
assert "preempted_holder_.erase(target_id);" in _release, "释放后清掉被抢占记录"
assert "res.success = true;" in _release, "持有者释放与'本来就空闲'都必须返回成功（旧兼容）"

# ============ ⑦ tracking 侧交接契约：心跳续租 + 被抢占后归还 ===============
assert "enum class LockRenewResult { RENEWED, LOST_PREEMPTED, UNAVAILABLE };" in tracking_svc_hdr, \
    "tracking 必须把心跳结果分成三态（续租/被抢占/服务不可达）"
assert "LockRenewResult ServiceManager::renewTargetLock(const std::string& target_id)" in \
    (tracking_root / "src" / "service_manager.cpp").read_text(), \
    "tracking 必须实现 renewTargetLock（复用 request 服务 = 续租）"
assert "void TrackingStateMachine::heartbeatTargetLock() {" in tracking_sm, \
    "tracking 必须周期性发心跳"
assert "heartbeatTargetLock();" in tracking_sm, "update() 必须真的调用心跳"
assert "LockRenewResult::LOST_PREEMPTED" in tracking_sm, "心跳被拒必须被识别为被抢占"
assert "bool lock_preempted_pending_ = false;" in tracking_sm_hdr and \
       "bool force_non_elimination_return = lock_preempted_pending_;" in tracking_sm, \
    "被抢占后必须强制归还控制权（走 elimination=false，不冒充裁判消除）"
assert "lock_heartbeat_seconds_" in tracking_sm, "心跳周期必须参数化（< 租约 20s）"

print("look_up 目标锁租约/抢占静态契约检查通过：租约默认 20s（<=0 永不过期）+ 抢占默认开、"
      "同 requester 重 request = 续租（.srv 未改，身份取 callerid）、过期后新机可接管并 WARN"
      "（target/old_holder/new_holder/expired_for）、单槽 leases_ 保证同一 target 只有一个持有者、"
      "释放按持有者收窄、原持有者心跳收到 success=false 后由 tracking 归还控制权并释放目标")
