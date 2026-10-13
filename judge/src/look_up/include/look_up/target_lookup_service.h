#ifndef TARGET_LOOKUP_SERVICE_H
#define TARGET_LOOKUP_SERVICE_H

#include <ros/ros.h>
// ros::ServiceEvent：不改 .srv 也能拿到调用方节点名（connection header 的 callerid），
// 这是"续租 / 抢占"判定"谁是持有者"的唯一依据（见 2026-09-28 任务1）。
#include <ros/service_callback_helper.h>
#include <look_up/RequestTarget.h>
#include <look_up/ReleaseTarget.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>

// 定义目标状态的常量
const std::string STATE_AVAILABLE = "AVAILABLE";
const std::string STATE_TRACKED = "TRACKED";

// 目标ID列表
const std::vector<std::string> TARGET_IDS = {"green0", "blue1", "brown2", "white3", "red4", "red5", "person"};

// 2026-09-28 任务1：目标锁"租约 + 可抢占"。
//   背景：本轮实测某人物一旦被某机锁定，其它机的请求**全被拒**（2114 次），叠加用户
//   新策略 return_only_on_elimination=true，一旦锁错/追不上就永不释放（3 号抱着 white3
//   自旋 188s）。修法：锁不再是"永久占用"，而是**有租约**（默认 20s），持有者持续追踪时
//   用同一个 request 服务心跳续租（同一 requester + 同一 target = 续租，不改 .srv）；
//   租约过期且别人来请求时，新请求者可以**接管**（抢占），并给原持有者明确的
//   "锁已被抢占"信号（它下一次心跳会收到 success=false + 专门的 WARN）。
struct LockLease {
    std::string holder;         // 持有者 = 调用方节点全名（callerid），例如 /typhoon_h480_0/tracking_node
    ros::Time granted_at;       // 本次成功锁定时刻
    ros::Time last_renew;       // 最近一次续租时刻（每次"持有者重新 request 同一 target"刷新）
    ros::Time lease_expiry;     // 租约到期时刻；ros::Time(0) = 永不过期（lock_lease_seconds<=0）
};

class TargetLookupService
{
private:
   ros::NodeHandle nh_;
   ros::NodeHandle pnh_;       // 私有句柄：~lock_lease_seconds / ~lock_preempt_enabled
   std::map<std::string, std::string> target_status_;
   std::mutex mutex_;
   std::vector<ros::ServiceServer> request_services_;
   std::vector<ros::ServiceServer> release_services_;

   // 每个 target 至多一条租约记录（map 单槽赋值覆盖）——"同一时刻只有一个持有者"的结构保证。
   std::map<std::string, LockLease> leases_;
   // target -> 被抢占的原持有者（供原持有者下次心跳时打印"锁被抢占"诊断）
   std::map<std::string, std::string> preempted_holder_;
   double lock_lease_seconds_ = 20.0;   // ~lock_lease_seconds（s）；<=0 = 永不过期（旧行为）
   bool lock_preempt_enabled_ = true;   // ~lock_preempt_enabled；false = 一键回退到"永不抢占"
   unsigned long long lock_preempt_count_ = 0;
   unsigned long long anonymous_caller_seq_ = 0;

   // --- 租约原语（调用方必须已持有 mutex_）---
   bool leaseExpiredLocked(const std::string& target_id, const ros::Time& now) const;
   void grantLeaseLocked(const std::string& target_id, const std::string& requester, const ros::Time& now);
   void renewLeaseLocked(const std::string& target_id, const ros::Time& now);
   const std::string* holderLocked(const std::string& target_id) const;
   // callerid 缺失时的兜底：生成一个"绝不与任何人相同"的身份，保证它不会误判为持有者/续租。
   std::string requesterFromEvent(const ros::ServiceEvent<look_up::RequestTarget::Request,
                                                         look_up::RequestTarget::Response>& event);

   /**
    * 处理"请求追踪目标"的服务回调函数（带 ServiceEvent，可拿到 callerid）
    *   同一 requester 再次 request 同一 target = 续租；租约过期 = 可被抢占。
    */
   bool handleRequestTarget(ros::ServiceEvent<look_up::RequestTarget::Request,
                                              look_up::RequestTarget::Response>& event);

   /**
    * 处理"释放追踪目标"的服务回调函数（带 ServiceEvent，只有持有者能释放）
    *   非持有者释放会被拒绝（保护新持有者的锁，避免两机同时追同一人）。
    */
   bool handleReleaseTarget(ros::ServiceEvent<look_up::ReleaseTarget::Request,
                                              look_up::ReleaseTarget::Response>& event);

public:
   /**
    * 构造函数 - 初始化目标锁定服务
    */
   TargetLookupService();

   /**
    * 析构函数
    */
   ~TargetLookupService() = default;

   // 删除拷贝构造函数和赋值操作符，确保单例行为
   TargetLookupService(const TargetLookupService&) = delete;
   TargetLookupService& operator=(const TargetLookupService&) = delete;
};

#endif // TARGET_LOOKUP_SERVICE_H
