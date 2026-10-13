#ifndef SERVICE_MANAGER_H
#define SERVICE_MANAGER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

// Include the required service definition headers
#include "topic_tools/MuxSelect.h"
#include "look_up/RequestTarget.h"
#include "look_up/ReleaseTarget.h"

/**
 * @class ServiceManager
 * @brief Manages all ROS service client interactions for the tracking node.
 *
 * This class handles the initialization and calling of services for:
 * - MUX control switching.
 * - Requesting and releasing target locks from a central lookup node.
 *
 * 2026-09-28 任务1：目标锁变成"租约 + 可抢占"。
 *   - 首次 request 同一 target = 申请锁；
 *   - 持有者**再次** request 同一 target = 续租（lookup 侧按 callerid 判定，服务定义没变）；
 *   - 续租被拒（response.success=false）= 锁已被其它机抢占 / 租约过期 → 必须归还控制权。
 */
class ServiceManager {
public:
    /**
     * @brief 目标锁心跳/续租的结果。
     *   RENEWED         = 续租成功（还是本机的锁）；
     *   LOST_PREEMPTED  = 服务正常应答但 success=false：锁已被别的机接管（或租约过期被拒）；
     *   UNAVAILABLE     = 服务调用本身失败（lookup 节点不在/重启中）：租约未续，
     *                     下次心跳继续尝试，**不**据此判定被抢占（避免误放控制权）。
     */
    enum class LockRenewResult { RENEWED, LOST_PREEMPTED, UNAVAILABLE };

    /**
     * @brief Constructor for ServiceManager.
     * @param nh ROS NodeHandle for creating service clients.
     * @param vehicle_type The type of the vehicle (e.g., "iris").
     * @param vehicle_id The ID of the vehicle (e.g., "0").
     */
    ServiceManager(ros::NodeHandle& nh, const std::string& vehicle_type, const std::string& vehicle_id);

    /**
     * @brief Switches the control MUX to a specified topic.
     * @param topic_name The destination topic for the MUX.
     * @return True if the service call was successful, false otherwise.
     */
    bool switchControl(const std::string& topic_name);

    /**
     * @brief Requests to lock a target via the lookup service.
     * @param target_id The ID of the target to request (e.g., "person").
     * @return True if the target was successfully locked, false otherwise.
     */
    bool requestTarget(const std::string& target_id);

    /**
     * @brief 心跳续租：持有者再次 request 同一 target（同一个服务接口，同一 target）。
     * @param target_id 当前持有的目标 ID。
     * @return LockRenewResult（见上）。
     */
    LockRenewResult renewTargetLock(const std::string& target_id);

    /**
     * @brief Releases a locked target via the lookup service.
     * @param target_id The ID of the target to release.
     * @return True 仅当 lookup 真的把锁释放了（response.success=true）。
     *         被抢占的原持有者释放别人的锁时会被拒绝（返回 false）——
     *         这是"同一时刻只有一个持有者"的保护，不是错误。
     */
    bool releaseTarget(const std::string& target_id);

private:
    /**
     * @brief Initializes the MUX selection service client.
     */
    void initMuxService();

    /**
     * @brief Initializes all target management service clients.
     */
    void initTargetServices();

    ros::NodeHandle nh_;
    std::string vehicle_type_;
    std::string vehicle_id_;

    // Client for switching the controller MUX
    ros::ServiceClient mux_select_client_;

    // Maps to hold clients for requesting and releasing multiple targets
    std::map<std::string, ros::ServiceClient> request_clients_;
    std::map<std::string, ros::ServiceClient> release_clients_;

    // List of supported target IDs
    const std::vector<std::string> TARGET_IDS_ = {
        "green0", "blue1", "brown2", "white3", "red4", "red5", "person"
    };
};

#endif // SERVICE_MANAGER_H
