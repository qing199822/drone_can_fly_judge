#include "look_up/target_lookup_service.h"
#include <ros/ros.h>
#include <algorithm>
#include <boost/bind.hpp>

using namespace boost::placeholders;  // 用于 _1, _2 等占位符

namespace {
// 目标锁租约的日志前缀，方便下一轮直接 grep（lock preempted / lock lease / lock renewed）。
const char* kLockLogPrefix = "[lock]";

// 把"当前可用目标"打成一行 INFO（与旧行为一致，仅抽出便于复用）。
std::string joinAvailableTargets(const std::map<std::string, std::string>& status)
{
    std::string available_list;
    for (const auto& pair : status)
    {
        if (pair.second == STATE_AVAILABLE)
        {
            if (!available_list.empty()) available_list += ", ";
            available_list += pair.first;
        }
    }
    return available_list;
}
}

TargetLookupService::TargetLookupService()
    : pnh_("~")
{
    // 初始化目标状态
    for (const auto& target_id : TARGET_IDS)
    {
        target_status_[target_id] = STATE_AVAILABLE;
    }

    // 2026-09-28 任务1：租约参数（私有参数，命令行可一键覆盖）。
    //   ~lock_lease_seconds  ：锁租约时长（s）。默认 20.0；<=0 表示"永不过期"（旧行为）。
    //   ~lock_preempt_enabled：true（默认）租约过期后允许其它机抢占；false = 一键回退到"永不抢占"。
    pnh_.param("lock_lease_seconds", lock_lease_seconds_, 20.0);
    pnh_.param("lock_preempt_enabled", lock_preempt_enabled_, true);

    // 为每个目标ID创建服务
    for (const auto& target_id : TARGET_IDS)
    {
        std::string req_service_name = "/lookup/request_" + target_id;
        std::string rel_service_name = "/lookup/release_" + target_id;

        // 创建请求服务 - 使用boost::bind
        // 2026-09-28 任务1：回调改用 ServiceEvent 版本，用于取 callerid（持有者身份）。
        //   服务定义（.srv）保持不变：RequestTarget 仍然只有 target_id / success。
        //   注意必须显式包成 boost::function<...ServiceEvent...>，否则 boost::bind 的
        //   占位符无法在 (MReq&,MRes&) 与 ServiceEvent 两个重载之间定型。
        typedef ros::ServiceEvent<look_up::RequestTarget::Request,
                                  look_up::RequestTarget::Response> RequestTargetEvent;
        ros::ServiceServer req_server = nh_.advertiseService(
            req_service_name,
            boost::function<bool(RequestTargetEvent&)>(
                boost::bind(&TargetLookupService::handleRequestTarget, this, _1)));
        request_services_.push_back(req_server);

        // 创建释放服务（同样用 ServiceEvent：只有持有者能释放）
        typedef ros::ServiceEvent<look_up::ReleaseTarget::Request,
                                  look_up::ReleaseTarget::Response> ReleaseTargetEvent;
        ros::ServiceServer rel_server = nh_.advertiseService(
            rel_service_name,
            boost::function<bool(ReleaseTargetEvent&)>(
                boost::bind(&TargetLookupService::handleReleaseTarget, this, _1)));
        release_services_.push_back(rel_server);

        ROS_INFO_STREAM("服务已就绪: '" << req_service_name << "'");
        ROS_INFO_STREAM("服务已就绪: '" << rel_service_name << "'");
    }

    ROS_INFO("=========================================");
    ROS_INFO("目标锁定服务中心已成功初始化。");

    std::string target_list;
    for (size_t i = 0; i < TARGET_IDS.size(); ++i)
    {
        target_list += TARGET_IDS[i];
        if (i != TARGET_IDS.size() - 1) target_list += ", ";
    }
    ROS_INFO_STREAM("当前管理的目标列表: " << target_list);
    if (lock_lease_seconds_ > 0.0) {
        ROS_INFO("%s lock lease enabled: lease=%.1fs preempt_enabled=%s "
                 "(持有者心跳续租；租约过期后其它机可接管)",
                 kLockLogPrefix, lock_lease_seconds_, lock_preempt_enabled_ ? "true" : "false");
    } else {
        ROS_WARN("%s lock lease DISABLED (lock_lease_seconds=%.1f <= 0): 锁永不过期，"
                 "行为与旧版本一致（永不因超时被抢占）",
                 kLockLogPrefix, lock_lease_seconds_);
    }
    ROS_INFO("=========================================");
}

// ============================================================================
// 租约原语（调用方必须已持有 mutex_）
// ============================================================================
bool TargetLookupService::leaseExpiredLocked(const std::string& target_id,
                                             const ros::Time& now) const
{
    // lock_lease_seconds_ <= 0 → 没有租约概念，永不过期（一键回退旧行为）。
    if (lock_lease_seconds_ <= 0.0) return false;
    auto it = leases_.find(target_id);
    if (it == leases_.end()) return false;              // 没有租约记录 → 不按过期处理
    if (it->second.lease_expiry.isZero()) return false; // 零时刻 = 永不过期
    return now >= it->second.lease_expiry;
}

void TargetLookupService::grantLeaseLocked(const std::string& target_id,
                                           const std::string& requester,
                                           const ros::Time& now)
{
    LockLease lease;
    lease.holder = requester;
    lease.granted_at = now;
    lease.last_renew = now;
    // 单槽赋值：同一 target 永远只有一个 holder（抢占时直接覆盖旧记录，
    // 不存在"两个持有者"的中间态）。
    lease.lease_expiry = (lock_lease_seconds_ > 0.0)
                             ? (now + ros::Duration(lock_lease_seconds_))
                             : ros::Time(0);
    leases_[target_id] = lease;
    target_status_[target_id] = STATE_TRACKED;
}

void TargetLookupService::renewLeaseLocked(const std::string& target_id,
                                           const ros::Time& now)
{
    auto it = leases_.find(target_id);
    if (it == leases_.end()) return;
    it->second.last_renew = now;
    it->second.lease_expiry = (lock_lease_seconds_ > 0.0)
                                  ? (now + ros::Duration(lock_lease_seconds_))
                                  : ros::Time(0);
}

const std::string* TargetLookupService::holderLocked(const std::string& target_id) const
{
    auto it = leases_.find(target_id);
    if (it == leases_.end()) return nullptr;
    return &it->second.holder;
}

// ============================================================================
// 2026-09-28 任务1：请求目标 = 首次锁定 / 续租 / 抢占
//   requester 身份来自 callerid（不改 .srv）：
//     ① 目标空闲                          → 批准并建立租约；
//     ② 目标被占 + requester == 持有者    → 续租（"同一 requester 再 request 同一 target"）；
//     ③ 目标被占 + 租约已过期 + 抢占开启  → 抢占：WARN（target/原持有者/新持有者/过期时长）
//                                            + 记录"被抢占"供原持有者下次心跳收到 success=false；
//     ④ 目标被占 + 租约未过期（或抢占关闭）→ 拒绝（老行为）。
// ============================================================================
bool TargetLookupService::handleRequestTarget(
    ros::ServiceEvent<look_up::RequestTarget::Request,
                      look_up::RequestTarget::Response>& event)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string target_id = event.getRequest().target_id;
    look_up::RequestTarget::Response& res = event.getResponse();
    const std::string requester = requesterFromEvent(event);
    const ros::Time now = ros::Time::now();

    ROS_DEBUG_STREAM("处理请求目标: " << target_id << " requester=" << requester);

    // 检查目标ID是否有效
    bool target_exists = false;
    for (const auto& valid_id : TARGET_IDS) {
        if (target_id == valid_id) {
            target_exists = true;
            break;
        }
    }

    if (!target_exists) {
        ROS_ERROR_STREAM("收到未知目标的请求: '" << target_id << "'。");
        res.success = false;
        return true;
    }

    const bool tracked = (target_status_[target_id] == STATE_TRACKED);
    const std::string* holder = holderLocked(target_id);
    const bool same_holder = (holder != nullptr) && (*holder == requester);

    if (!tracked) {
        // ① 首次锁定
        grantLeaseLocked(target_id, requester, now);
        ROS_INFO_STREAM("请求已批准: 目标 '" << target_id << "' 已被锁定。holder="
                        << requester << " lease=" << lock_lease_seconds_ << "s");
        const std::string available_list = joinAvailableTargets(target_status_);
        if (!available_list.empty()) {
            ROS_INFO_STREAM("--- 当前剩余可用目标: " << available_list);
        } else {
            ROS_INFO("--- 所有目标均已被锁定。");
        }
        res.success = true;
        return true;
    }

    if (same_holder) {
        // ② 续租（持有者心跳）：刷新 last_renew / lease_expiry，锁不会被别人抢走。
        renewLeaseLocked(target_id, now);
        const auto it = leases_.find(target_id);
        const double left = (it != leases_.end() && !it->second.lease_expiry.isZero())
                                ? (it->second.lease_expiry - now).toSec()
                                : -1.0;
        ROS_INFO_THROTTLE(5.0, "%s lock renewed: target='%s' holder=%s lease_left=%.1fs last_renew=%.3f",
                          kLockLogPrefix, target_id.c_str(), requester.c_str(), left, now.toSec());
        res.success = true;
        return true;
    }

    if (leaseExpiredLocked(target_id, now)) {
        const std::string old_holder = holder ? *holder : std::string("(unknown)");
        double expired_for = 0.0;
        {
            auto it = leases_.find(target_id);
            if (it != leases_.end() && !it->second.lease_expiry.isZero()) {
                expired_for = (now - it->second.lease_expiry).toSec();
            }
        }

        if (!lock_preempt_enabled_) {
            // ⑤ 一键回退：不抢占（等价旧行为：锁一直归原持有者）。
            ROS_WARN_STREAM(kLockLogPrefix << " lease expired but preempt DISABLED "
                            "(~lock_preempt_enabled=false): target='" << target_id
                            << "' still held by " << old_holder << " (expired_for="
                            << expired_for << "s); rejecting " << requester);
            res.success = false;
            return true;
        }

        // ③ 抢占：记录被抢占者 → 单槽覆盖租约（原持有者立刻失去 holder 身份）。
        ++lock_preempt_count_;
        preempted_holder_[target_id] = old_holder;
        grantLeaseLocked(target_id, requester, now);
        ROS_WARN_STREAM(kLockLogPrefix << " lock preempted: target='" << target_id
                        << "' old_holder=" << old_holder << " new_holder=" << requester
                        << " expired_for=" << expired_for << "s lease="
                        << lock_lease_seconds_ << "s preempt_count=" << lock_preempt_count_);
        ROS_WARN_STREAM(kLockLogPrefix << " 原持有者 " << old_holder << " 已失去目标 '"
                        << target_id << "' 的锁：它下一次心跳(request)会收到 success=false，"
                        "由 tracking 侧归还控制权并释放目标。");
        res.success = true;
        return true;
    }

    // ④ 租约有效且请求者不是持有者：拒绝（日志保留原文，便于下一轮 grep 对比拒绝次数）。
    double left = -1.0;
    {
        auto it = leases_.find(target_id);
        if (it != leases_.end() && !it->second.lease_expiry.isZero()) {
            left = (it->second.lease_expiry - now).toSec();
        }
    }
    if (holder != nullptr && preempted_holder_.count(target_id) != 0 &&
        preempted_holder_[target_id] == requester) {
        // 原持有者被抢占后的心跳：给出明确的"锁被抢占"信号（success=false）。
        ROS_WARN_STREAM(kLockLogPrefix << " 原持有者 " << requester << " 的续租请求被拒绝：目标 '"
                        << target_id << "' 的锁已被 " << *holder
                        << " 抢占（租约过期后接管）");
    } else {
        ROS_WARN_STREAM("请求被拒绝: 目标 '" << target_id << "' 已被其他无人机追踪。holder="
                        << (holder ? *holder : std::string("(unknown)"))
                        << " lease_left=" << left << "s requester=" << requester);
    }
    res.success = false;
    return true;
}

// ============================================================================
// 2026-09-28 任务1：释放目标（只有持有者能释放）
//   为什么收窄：抢占发生后，原持有者下一次心跳会失败并被要求"释放目标"；
//   如果释放不区分身份，原持有者会把**新持有者**的锁抢掉，等于两机轮流抱同一个
//   目标（正是上一轮 2114 次拒绝 + 188s 自旋的根因之一）。
//   兼容性：没有租约记录（旧客户端 / 服务重启后）时照旧无条件释放，保持旧语义。
// ============================================================================
bool TargetLookupService::handleReleaseTarget(
    ros::ServiceEvent<look_up::ReleaseTarget::Request,
                      look_up::ReleaseTarget::Response>& event)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string target_id = event.getRequest().target_id;
    look_up::ReleaseTarget::Response& res = event.getResponse();
    const std::string requester = event.getCallerName();
    const ros::Time now = ros::Time::now();

    ROS_DEBUG_STREAM("处理释放目标: " << target_id << " requester=" << requester);

    // 检查目标ID是否有效
    bool target_exists = false;
    for (const auto& valid_id : TARGET_IDS) {
        if (target_id == valid_id) {
            target_exists = true;
            break;
        }
    }

    if (!target_exists) {
        ROS_ERROR_STREAM("收到未知目标的释放请求: '" << target_id << "'。");
        res.success = false;
        return true;
    }

    if (target_status_[target_id] == STATE_TRACKED)
    {
        const std::string* holder = holderLocked(target_id);
        if (holder != nullptr && !requester.empty() && *holder != requester) {
            // 非持有者释放 → 拒绝，保护新持有者的锁（不削弱"同一时刻只有一个持有者"）。
            res.success = false;
            ROS_WARN_STREAM(kLockLogPrefix << " release REFUSED: target='" << target_id
                            << "' is leased to " << *holder << " but released by "
                            << requester << " (被抢占的原持有者不允许释放他人的锁)");
            return true;
        }

        target_status_[target_id] = STATE_AVAILABLE;
        leases_.erase(target_id);           // 释放 = 租约立即失效
        preempted_holder_.erase(target_id);
        ROS_INFO_STREAM("目标 '" << target_id << "' 已被释放，现在可用。released_by="
                        << (requester.empty() ? std::string("(unknown)") : requester)
                        << " at " << now.toSec());

        const std::string available_list = joinAvailableTargets(target_status_);
        if (!available_list.empty()) {
            ROS_INFO_STREAM("--- 当前可用目标: " << available_list);
        }
    }
    else
    {
        ROS_DEBUG_STREAM("目标 '" << target_id << "' 原本就是可用状态，无需释放。");
    }

    // 无论目标之前的状态是什么，都返回成功
    res.success = true;
    return true;
}

std::string TargetLookupService::requesterFromEvent(
    const ros::ServiceEvent<look_up::RequestTarget::Request,
                            look_up::RequestTarget::Response>& event)
{
    // callerid 由 ROS 连接头自动带上（roscpp/rospy 客户端都有），例如
    // /typhoon_h480_0/tracking_node。缺省时用一个"绝不重合"的身份，
    // 保证匿名调用者不会被误认为持有者、也不会互相续租。
    const std::string& caller = event.getCallerName();
    if (!caller.empty()) return caller;
    return std::string("anonymous#") + std::to_string(++anonymous_caller_seq_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_lookup_service");

    try
    {
        TargetLookupService service;
        ROS_INFO("目标锁定服务节点开始运行...");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("目标锁定服务节点异常: " << e.what());
        return 1;
    }

    ROS_INFO("目标锁定服务节点已关闭。");
    return 0;
}
