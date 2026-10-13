#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

root = Path(__file__).parents[1]
source = (root / "src" / "area_mission_node.cpp").read_text()
mppi_source = (root / "include" / "auto_explore" / "mppi_planner.hpp").read_text()
launch_path = root / "launch" / "auto_explore_swarm.launch"
launch = launch_path.read_text()
ET.parse(launch_path)

# A* output is map/world frame, while pose_x_/pose_y_ are local odom.
# Guard against mixing the frames (the error is especially visible for UAV5).
_astar_look = source[source.index("bool patrolAstarLookahead"):source.index("void scanCb", source.index("bool patrolAstarLookahead"))]
assert "pose_world_x = worldPoseX()" in _astar_look
assert "pose_world_y = worldPoseY()" in _astar_look
assert "astar_path_world_[i].first - pose_world_x" in _astar_look
assert "astar_path_world_[i].second - pose_world_y" in _astar_look
assert "astar_path_world_[idx].first - pose_world_x" in _astar_look
assert "astar_path_world_[idx].second - pose_world_y" in _astar_look
assert "astar_path_world_[i].first - pose_x_" not in _astar_look
assert "astar_path_world_[i].second - pose_y_" not in _astar_look

# 2026-09-27：MPPI/depth/front_inhibit 权重集中在 mppi_params.yaml，launch 行内
# 不再写同名 param。每架无人机都会通过 rosparam load 加载该 yaml 一次。
mppi_yaml = root / "config" / "mppi_params.yaml"
assert mppi_yaml.exists(), "MPPI 参数 yaml 必须存在"
assert launch.count('command="load" file="$(find auto_explore)/config/mppi_params.yaml') >= 5, \
    "每架无人机的 area_mission_node 块都必须加载 mppi_params.yaml"
assert "reload_mppi_params" in source, "需提供 reload_mppi_params 服务用于热重载"
assert "configureMppiFromParams" in source, "MPPI 配置需抽出为可复用方法"
# 2026-09-27：DWB 风格 PathAlign / PathDist 评分器
assert "path_align_weight" in mppi_source, "需在 Planner 中实现 PathAlign 评分器"
assert "path_dist_weight" in mppi_source, "需在 Planner 中实现 PathDist 评分器"
assert "setPath" in mppi_source, "Planner 需提供 setPath() 接口以注入期望路径"
assert "projectOntoPath" in mppi_source, "Planner 需将 rollout 末态投影到期望路径"
yaml_data = yaml.safe_load(mppi_yaml.read_text())
# 2026-09-27：自身物理半径（m）从障碍距离中扣除；MPPI 不再当作质点。
assert "robot_radius" in mppi_source, "mppi::Planner 需暴露 robot_radius 参数"
assert "mppi_robot_radius" in yaml_data, "yaml 必须包含 mppi_robot_radius 字段"
assert yaml_data["mppi_robot_radius"] >= 0.0
assert yaml_data["mppi_robot_radius"] <= 5.0
for key in ("mppi_path_align_weight", "mppi_path_dist_weight"):
    assert key in yaml_data, f"yaml 必须包含 {key} 字段"
assert yaml_data["mppi_path_align_weight"] >= 0
assert yaml_data["mppi_path_dist_weight"] >= 0
# 行内参数已迁出（不再写六遍 mppi_*），但 camera intrinsics 仍允许在 launch 中覆盖
for key in ("mppi_num_samples", "mppi_update_rate", "mppi_obstacle_weight",
            "depth_obstacle_weight", "front_inhibit_depth_enter"):
    inline = launch.count(f'name="{key}" value=')
    assert inline <= 1, f"{key} 仍被 launch 行内 {inline} 次重复覆盖，应该都来自 yaml"

road_branch = source[source.index("if (mode_ == ROAD && !return_to_pause_)"):source.index("else\n        {\n          // PATROL", source.index("if (mode_ == ROAD && !return_to_pause_)"))]
assert "road_mppi_.setGoal" in road_branch
# setScan moved into the 10Hz planner-rate helper so ROAD/PATROL/TRANSIT all
# share the cache; verify the helper is used from the ROAD branch instead.
assert "planAtRate(road_mppi_, road_mppi_cache_" in road_branch
assert "road_mppi_cache_" in source
assert "mppi_cache_" in source
assert "planAtRate(mppi_, mppi_cache_" in source
assert "far_road_cruise" in road_branch
assert "bounded target approach" in road_branch
assert "road_target_reorient_timeout_" in road_branch
assert "target_approach" in road_branch
assert "road_approach_target_weight_" in road_branch
assert "road_approach_prior_weight_" in road_branch
assert "target bearing" in road_branch
# 2026-09-27：6 架机巡逻航点改为硬编码 4 点（BL → BR → TR → TL 顺时针循环）。
# 不能再依赖 patrol_rings_ / patrol_ring_ratio_ / patrol_outer_margin_ 来生成。
assert "uav_world_wps[6][4][2]" in source, "loadPresetWaypoints 必须保存 6 架机的世界系 4 点航点"
assert "uav_world_wps[drone_id_][i][0]," in source
assert "uav_world_wps[drone_id_][i][1]);" in source
assert "patrol_world.push_back" not in source, "旧的环生成逻辑必须移除"
# A* path is world/map coordinates; pose comparisons must add this UAV's spawn offsets.
_astar_look = source[source.index("bool patrolAstarLookahead"):source.index("void scanCb", source.index("bool patrolAstarLookahead"))]
assert "pose_world_x = worldPoseX()" in _astar_look
assert "pose_world_y = worldPoseY()" in _astar_look
assert "astar_path_world_[idx].first - pose_world_x" in _astar_look
assert "astar_path_world_[idx].second - pose_world_y" in _astar_look
# wp_reach_tolerance is 3.0m — every UAV must declare it.
assert launch.count('name="wp_reach_tolerance" value="3.0"') == 6
# 2026-09-27：road_approach_*_weight 现在同时存在于 road_follow 与 area_mission 块（各自语义不同）；
# road_follow 节点有 6 份，area_mission 节点也已加载，但 area_mission 通过 mppi_params.yaml 设置同名键，
# 因此 road_approach_* 不应再在 area_mission 行内出现。只在 road_follow 块保留 6 份即可。
assert launch.count('name="road_approach_target_weight" value="0.8"') == 6
assert launch.count('name="road_approach_prior_weight" value="0.2"') == 6
# 2026-09-24：远距沿路分支（drone 4/5 全段、通用 12m+）必须让位 region_return_active_。
# 否则区域回归目标会被道路先验覆盖，0 号卡边界的根因之一就是这里。
assert "drone_id_ == 4 || drone_id_ == 5) && !region_return_active_ && road_visual_useful" in source
assert "road_visual_useful && !region_return_active_ && !target_align && dist > road_align_distance_" in source
# 2026-09-25：ROAD 超时强制进 PATROL 必须对所有机生效（原来挂在 road_full_align_uav_set_
# 的 bit4/bit5 上，0~3 号够不到目标就永远进不了 PATROL；而巡逻航点只在 mode_==PATROL 里被读，
# 于是航点/hover-skip/段级 watchdog 全部失效 → 贴区域边界被硬保护夹住）。
# 六架都必须显式配置 road_max_seconds，且 2026-09-28 起全部置 0.0（取消硬时限，≤0 = 不限时）。
assert launch.count('name="road_max_seconds"') == 6
assert launch.count('name="road_max_seconds" value="0.0"') == 6
_tmo_start = source.index("const bool road_timeout_force_global")
_tmo_block = source[_tmo_start:_tmo_start + 400]
assert "road_full_align_uav_set_" not in _tmo_block
assert "(ros::Time::now() - road_stage_start_).toSec() >= road_max_seconds_" in _tmo_block
# 航点只在 PATROL 里被读取：这条是"有航点也救不了 ROAD 里的无人机"的代码依据。
assert "if (mode_ == PATROL)" in source
assert source.index("if (mode_ == PATROL)") < source.index("tx = (*wps)[wp_idx_].first - spawn_x_;")
safety = source[source.index("const double fwd_lidar"):source.index("enforceRegionBounds", source.index("const double fwd_lidar"))]
for required in ("depth_stop_threshold_", "brake_dist", "avoidPeers(cmd)"):
    assert required in safety, required
# road_mppi_v_des / max_v / min_v 已迁到 mppi_params.yaml（yaml 同时覆盖六机）。
assert launch.count('name="road_mppi_v_des"') == 0
assert launch.count('name="road_mppi_max_v"') == 0
assert launch.count('name="road_mppi_min_v"') == 0
assert launch.count('arg name="road_mppi_v_des"') == 0, "road_mppi_v_des arg 已删除（yaml 兜底）"
# ===================== 2026-09-14：ROAD 原地旋转/不前进修复 =====================
# 参数与诊断
for name in ("road_min_speed_floor", "road_min_speed_floor_clearance",
             "road_guidance_filter_alpha", "road_guidance_max_step",
             "scan_stale_grace_time", "scan_stale_grace_speed",
             "road_mppi_obstacle_weight", "road_mppi_safe_radius",
             "road_mppi_speed_weight", "road_mppi_goal_weight",
             "road_mppi_yaw_weight", "diag_rate"):
    assert 'pnh_.param<double>("%s"' % name in source or 'pnh_.param<double>("%s"' % name in source, name
assert 'pnh_.param<std::string>("diag_topic"' in source
assert 'road_candidate_switch_margin", road_candidate_switch_margin_, 0.35' in source
assert 'road_candidate_confirm_cycles", road_candidate_confirm_cycles_, 25' in source

# ① 速度下限：必须在安全层之后、区域边界之前，并且受"软降速 + 净空充足"约束
floor_at = source.index("ROAD speed floor")
safety_at = source.index("const double fwd_lidar")
region_at = source.index("enforceRegionBounds(cmd);", floor_at)
assert safety_at < floor_at < region_at
floor_block = source[floor_at - 2000:floor_at + 800]
for required in ("road_min_speed_floor_clearance_", "progress_recovery_until_",
                 "road_held_active_", "road_recovery_active_", "far_enough",
                 "clearance_ok", "!depth_fresh || fwd_depth >",
                 "!recovery && clearance_ok && far_enough"):
    assert required in floor_block, required
# 硬停路径必须在速度下限之前就 break（下限不会覆盖硬停）
for stop_code in ("DIAG_STOP_CODE] = 1.0f", "DIAG_STOP_CODE] = 4.0f",
                  "DIAG_STOP_CODE] = 5.0f", "DIAG_STOP_CODE] = 6.0f"):
    assert source.index(stop_code) < floor_at, stop_code
# 区域边界仍然拥有最终否决权（下限之后还会被边界裁剪，并记录 stop_code=7）
assert "DIAG_STOP_CODE] = 7.0f" in source

# ② 方向低通 + 单周期限幅 + 视觉丢失时重置
source_norm = " ".join(source.split())
assert "road_guidance_max_step_" in source
assert "road_guidance_filtered_ = clamp(road_guidance_filtered_ + step, -1.0, 1.0);" in source_norm
assert "road_guidance_filter_init_ = false;" in source
assert "road_guidance_filter_init_ = true;" in source

# ③ ROAD-MPPI 专用代价覆盖（不再沿用 PATROL 的 80/4.0）
for name, default in (("road_mppi_obstacle_weight", "35.0"),
                      ("road_mppi_safe_radius", "2.5"),
                      ("road_mppi_speed_weight", "0.8"),
                      ("road_mppi_goal_weight", "2.0"),
                      ("road_mppi_yaw_weight", "0.15")):
    assert 'pnh_.param<double>("%s", road_mppi_prm.' % name in source, name

# ④ 诊断：20 个字段 + 发布 + 实际输出记录
for field in ("DIAG_FLOOR_STATE", "DIAG_RECOVERY",
              "DIAG_MPPI_VALID", "DIAG_MPPI_V", "DIAG_MPPI_W", "DIAG_V_MPPI",
              "DIAG_V_DEPTH", "DIAG_V_TTC", "DIAG_V_PEER", "DIAG_V_REGION",
              "DIAG_V_FLOOR", "DIAG_V_PUB", "DIAG_W_PUB", "DIAG_ROAD_VALID",
              "DIAG_ROAD_HELD", "DIAG_ROAD_CONF", "DIAG_GUIDANCE",
              "DIAG_FWD_LIDAR", "DIAG_FWD_DEPTH", "DIAG_DIST_GOAL",
              "DIAG_STOP_CODE", "DIAG_MODE", "DIAG_COUNT"):
    assert field in source, field
assert "void publishDiag(bool force = false)" in source
# 数组版走 diag_array_topic_，人读 String 版走 diag_topic_
assert 'pnh_.param<std::string>("diag_array_topic"' in source
# 下限为何生效/被跳过的四个状态码 + recovery 标记
assert "DIAG_FLOOR_STATE" in source and "DIAG_RECOVERY" in source
assert "diag_[DIAG_FLOOR_STATE] = 1.0f;" in source   # 下限生效
assert "diag_[DIAG_FLOOR_STATE] = 2.0f;" in source   # recovery 跳过
assert "diag_[DIAG_FLOOR_STATE] = 3.0f;" in source   # 净空不足跳过
assert "diag_[DIAG_FLOOR_STATE] = 4.0f;" in source   # 太靠近目标跳过
assert '<< " floor_state="' in source and '<< " recovery="' in source
# vector 初始化陷阱：必须用 (DIAG_COUNT, 0.0f) 显式构造，不能写 {DIAG_COUNT, 0.0f}
assert "std::vector<float> diag_ = std::vector<float>(DIAG_COUNT, 0.0f);" in source
assert "std::vector<float> diag_{DIAG_COUNT, 0.0f};" not in source
# controlCb 末尾统一发布（覆盖所有 break / 早退路径中的 publishDiag 调用）
assert source.count("publishDiag();") >= 2
assert "diag_[DIAG_V_PUB] = static_cast<float>(smooth_x_);" in source
assert "diag_[DIAG_V_PUB] = 0.0f;" in source

# S1-S3 front inhibit / depth envelope contract.
for name, default in (("front_inhibit_laser_enter", "2.5"),
                      ("front_inhibit_laser_release", "4.0"),
                      ("front_inhibit_depth_enter", "4.0"),
                      ("front_inhibit_depth_release", "6.0"),
                      ("depth_stop_speed_gain", "0.8"),
                      ("near_target_guard_margin", "4.0")):
    assert 'pnh_.param<double>("%s",' % name in source
    assert default in source
assert 'pnh_.param<int>("front_inhibit_release_cycles", front_inhibit_release_cycles_, 5)' in source
assert "front_inhibit_" in source and "front_inhibit_release_count_" in source
assert "const double env_clearance" in source
assert "const double speed_cap = clamp(patrol_speed_limit_ * (env_clearance - 1.5) /" in source
assert "(6.0 - 1.5)" in source
assert "const bool near_target_guard = mode_ == PATROL || mode_ == TRANSIT_TO_REGION" in source
assert "const bool depth_latch_allowed = mode_ == ROAD;" in source
assert "if (patrol_astar_look_ok) setLocalMppiPath();" in source
assert "lidar_route::plan(lidar_grid, awx, awy, gwx, gwy)" in source
assert "map_msg_.data, static_cast<int>(map_msg_.info.width)" in source
assert "astar_path_world_ = planPathWorld(awx, awy, gwx, gwy, true);" in source
_transit_control = source[source.index("if (patrol_astar_look_ok) setLocalMppiPath();"):
                          source.index("// ROAD mode keeps YOLO", source.index("if (patrol_astar_look_ok) setLocalMppiPath();"))]
assert "mppi_.setGoal(transit_goal_x, transit_goal_y);" in _transit_control
assert "else if (mode_ == ROAD && !return_to_pause_)" in source, \
    "TRANSIT 的 MPPI 指令不能再被后续 PATROL 分支覆盖"
assert "if (front_inhibit_)\n          cmd.linear.x = 0.0;" in source
for field in ("DIAG_FRONT_INHIBIT", "DIAG_FRONT_INHIBIT_RELEASE_COUNT",
              "DIAG_ENV_CLEARANCE", "DIAG_SPEED_CAP"):
    assert field in source, field
assert '<< " front_inhibit="' in source
assert '<< " front_inhibit_reason="' in source
assert '<< " env_clearance="' in source and '<< " speed_cap="' in source

# 激光短时过期限速蠕行（不再一次抖动就钉死）
assert "scan_stale_grace_time_" in source and "scan_stale_grace_speed_" in source
assert "smooth_x_ > 0.2" in source

# 到达日志与实际容差同步（原来写死 "within 2m"）
assert "visual road target reached within %.1fm" in source
assert "visual road target reached within 2m" not in source

# launch 六个机块统一加载 ROAD 调参 YAML，且 YAML 内容正确
assert launch.count('<rosparam file="$(find auto_explore)/config/area_mission_road.yaml" '
                    'command="load"/>') == 6
# 必须放在 <node> 内部且不能再加 ns=（否则参数落到 .../area_mission_node/area_mission_node/ 而被忽略）
assert 'ns="area_mission_node"' not in launch
yaml_text = (root / "config" / "area_mission_road.yaml").read_text()
expected = {
    "road_min_speed_floor": "1.0",
    "road_min_speed_floor_clearance": "3.0",
    "road_guidance_filter_alpha": "0.25",
    "road_guidance_max_step": "0.05",
    "road_candidate_switch_margin": "0.35",
    "road_candidate_confirm_cycles": "25",
    "road_mppi_obstacle_weight": "28.0",
    "road_mppi_safe_radius": "1.5",
    "road_mppi_speed_weight": "0.8",
    "road_mppi_goal_weight": "2.5",   # §15.20（2026-09-24）把 yaml 从 2.0 调到 2.5，测试同步钉住
    "road_mppi_yaw_weight": "0.30",
    "road_align_distance": "12.0",
    "road_align_perp_weight": "0.7",
    "road_geom_only_after_lost": "1.0",
    "scan_stale_grace_time": "1.0",
    "scan_stale_grace_speed": "0.5",
    "diag_rate": "5.0",
    "open_space_yaw_gain": "1.2",
    "open_space_max_w": "1.0",
    "region_return_threshold": "2.0",
    "region_conflict_cycles": "10",
    "region_return_speed": "1.5",
}
assert "diag_array" in yaml_text and "floor_state" in yaml_text
for key, value in expected.items():
    assert any(line.split("#")[0].strip() == "%s: %s" % (key, value)
               for line in yaml_text.splitlines()), (key, value)

# ====== 2026-09-14 追加：激光语义（开阔 vs 坏数据）+ 区域回归（4 号 stop=7 死锁）===
# 激光语义：全 inf = 开阔（可用、直飞）；全 NaN/0 = 坏数据（不可用、停车）
assert "bool scanClear() const" in source
assert "return scan_inf_hits_ * 2 >= total;" in source
assert "if (scan_nan_hits_ * 2 >= total) return 2;" in source
assert "if (!road_mppi_.canPlan() && !scanClear())" in source
assert "if (!mppi_.canPlan() && !open_space_patrol)" in source
assert "const bool open_space_patrol = patrolOpenSpaceSafe();" in source
assert "const bool open_space_patrol = patrolOpenSpaceSafe() && !scan_open_latched_;" not in source
assert "open_space_direct_ = true;" in source
assert 'pnh_.param<double>("open_space_yaw_gain"' in source
assert 'pnh_.param<double>("open_space_max_w"' in source
# 日志必须区分"时间戳过期"和"消息新鲜但不可用"
assert "scan timestamp stale (sim %.2fs wall %.2fs); holding" in source
assert "scan message fresh but unusable" in source
assert "stale_by_time" in source
# 区域回归：区域外 / 边界连续清零 -> 禁用道路先验、直奔区域内
assert 'pnh_.param<double>("region_return_threshold"' in source
assert 'pnh_.param<int>("region_conflict_cycles"' in source
assert "outside region by %.1fm; returning to " in source
assert "const double recovery_inset = std::min(region_margin_, 3.0);" in source
assert "const double inside_x = clamp(wx_now, region_xmin_ + recovery_inset" in source
# 2026-09-23 修复：inset = 距最近边距离（区域内为正），
# 退出深度必须小于 recovery_inset；否则目标只在 3m 内，6m 退出条件永远达不到。
assert "const double inset = std::min(std::min(wx_now - region_xmin_, region_xmax_ - wx_now)," in source
assert "std::min(wy_now - region_ymin_, region_ymax_ - wy_now));" in source
assert "else if (inset > std::min(region_margin_, 3.0) - 0.5)" in source
# 2026-09-23 修复：ROAD 远距沿路分支去掉 std::max(0.1, …)，
# 否则右转（sin 负）被抬成 +0.1 = 永远不能右转，0 号因此一路朝东撞边界。
assert "goal_bx = std::cos(forced_road_angle) * 8.0;" in source
assert "goal_by = std::sin(forced_road_angle) * 8.0;" in source
# 旧式 std::max(0.1, std::cos/sin(forced_road_angle) * 8.0) 必须在 ROAD 远距分支被移除
# （其他 std::max(0.1, std::hypot(...)) 仍用于防除零，保留即可）
assert "(road_recovery || region_return_active_) ? 0.0" in source
assert "region_clamp_killed_ = (std::fabs(forward_before) >= 0.3 &&" in source
assert "region_return_active_ ? region_return_speed_ :" in source
for field in ("DIAG_SCAN_VALID_HITS", "DIAG_SCAN_INF_HITS", "DIAG_SCAN_NAN_HITS",
              "DIAG_SCAN_CLEAR", "DIAG_SCAN_UNUSABLE_REASON",
              "DIAG_REGION_OUTSIDE", "DIAG_REGION_RETURN"):
    assert field in source, field
assert '<< " scan_clear="' in source and '<< " region_return="' in source

# ============ P0 PATROL 卡死检测/航点黑名单契约 ============
for name, default in (("patrol_stall_window", "20.0"),
                      ("patrol_stall_min_progress", "3.0"),
                      ("patrol_stall_blocked_ratio", "0.5"),
                      ("patrol_wp_blacklist_time", "90.0")):
    assert 'pnh_.param<double>("%s"' % name in source
    assert default in source
for field in ("DIAG_STALL_COUNT", "DIAG_BLOCKED_RATIO", "DIAG_WINDOW_PROGRESS",
              "DIAG_WP_BLACKLISTED", "DIAG_WP_IDX"):
    assert field in source, field
assert "patrol_stall_window_start_" in source
assert "patrol_stall_min_progress_" in source
assert "patrol_blocked_ratio_ > patrol_stall_blocked_ratio_" in source
assert "front_inhibit_ || laser_hard_stop || depth_hard_stop" in source
assert "patrol_recovery_active_" not in source  # 死变量已替换为 patrol_recovery_state_（见 §15.16）
assert "PATROL stall skip wp old=" in source
assert "patrol_wp_blacklist_until_" in source
assert "if (!patrol_queue_.empty()) wp_idx_ = patrol_queue_.front();" in source
assert "patrol_queue_.front()" in source
assert "const bool by_dist" in source
# 2026-09-25：推进必须使用实际位移（段起点）+ 真 by_time，不能用 min(0.2, dist_to_wp) 假进度。
assert "patrol_advance_origin_x_" in source and "patrol_advance_origin_y_" in source
assert "const double actual_dist = std::hypot(" in source
assert "const bool by_time = dwell >= patrol_advance_interval_" in source
assert "actual_dist < 0.7" in source
assert "patrol_advance_interval_, 5.0" in source
assert "const bool by_time" in source
# 未到达分支的 no-progress 跳点与 no_escape 放弃都必须真正黑名单+跳点+retreat。
assert "no-progress skip wp old=" in source
assert "return GAVE_UP;" in source
assert "patrol_recovery_gave_up" in source
assert "recovery gave up; blacklist wp=" in source
assert "patrol_recovery_exit_cycles_" in source

# ============ 2026-09-14 追加：诊断可达性 + scan stale 双判据 ============
# 诊断必须"抓得到"：latch + 墙钟定时器 + 汇总话题 + 日志 + 参数快照
assert 'pnh_.param<std::string>("params_topic"' in source
assert 'pnh_.param<std::string>("swarm_diag_topic", swarm_diag_topic_, "/swarm_diag");' in source
assert "diag_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(diag_array_topic_, 1, true);" in source
assert "diag_string_pub_ = nh_.advertise<std_msgs::String>(diag_topic_, 1, true);" in source
assert "swarm_diag_pub_ = nh_.advertise<std_msgs::String>(swarm_diag_topic_, 10, true);" in source
assert "params_pub_ = nh_.advertise<std_msgs::String>(params_topic_, 1, true);" in source
# 墙钟定时器：仿真时钟冻结时诊断仍要发布
assert "createWallTimer" in source and "diagTimerCb(const ros::WallTimerEvent&)" in source
assert "void diagTimerCb(const ros::WallTimerEvent&) { publishDiag(); }" in source
# 墙钟限速（不能用 ROS 时间限速）
assert "(now_wall - last_diag_wall_stamp_).toSec() < 0.5 / diag_rate_" in source
# 汇总 + 日志 + 参数快照
assert 'swarm.data = vehicle_ + " " + line;' in source
assert 'ROS_INFO_THROTTLE(1.0, "[area_mission] %s DIAG %s"' in source
assert 'snapshot << "vehicle=" << vehicle_ << " build=" << __DATE__' in source
assert 'direct_goal_mode=" << (direct_goal_mode_ ? "true" : "false")' in source
assert 'ROS_INFO("[area_mission] %s node up (build %s %s)"' in source
# scan stale 双判据：仿真时间 or 墙钟时间
assert "bool freshEnough(const ros::Time& sim_stamp" in source
assert "return sim_age <= timeout || wall_age <= timeout;" in source
assert "bool depthFresh() const { return depth_received_ && freshEnough(" in source
assert "const bool scan_fresh = freshEnough(scan_stamp_, scan_wall_stamp_, sensor_timeout_)" in source
assert "pnh_.param<bool>(\"direct_goal_mode\", direct_goal_mode_, false);" in source
assert 'name="direct_goal_mode" value="true"' in launch
assert 'name="disable_for_vehicle" value="true"' in launch
assert "const bool pose_fresh = pose_received_ && freshEnough(pose_stamp_, pose_wall_stamp_, pose_timeout_);" in source
assert 'pnh_.param<double>("pose_stale_recovery_timeout", pose_stale_recovery_timeout_, 2.0);' in source
assert 'pnh_.param<double>("pose_stale_recovery_speed", pose_stale_recovery_speed_, 0.6);' in source
assert 'safety_reason_ = "pose_stale_recovery";' in source
assert "const bool road_fresh = !direct_goal_mode_" in source
# 时钟跳变检测 + 计数入诊断
assert "if (step > 0.3)" in source and "++clock_jumps_;" in source
for field in ("DIAG_SCAN_AGE_SIM", "DIAG_SCAN_AGE_WALL", "DIAG_POSE_AGE_SIM", "DIAG_CLOCK_JUMPS"):
    assert field in source, field
assert '<< " scan_age="' in source and '<< " clock_jumps="' in source
# 数组字段数 = 26（与 diag_array 的文档一致）
assert source.count("DIAG_COUNT") >= 2
# 巡逻航线内缩：区域边界不变，六机统一从边界内缩12m生成蛇形航点。
assert launch.count('name="region_margin" value="6.0"') == 6
assert 'name="region_margin" value="8.0"' not in launch
assert 'name="region_margin" value="5.0"' not in launch
# 4/5 号 ROAD 全阶段沿路引导（位标志）：UAV4=bit4、UAV5=bit5，其它机不设。
assert 'name="road_full_align_uav_set" value="16"' in launch
assert 'name="road_full_align_uav_set" value="32"' in launch
assert "road_full_align_prior_weight_" in source
assert "road_full_align_target_weight_" in source
assert "road_full_align_uav_set_" in source
assert "((road_full_align_uav_set_ & (1 << drone_id_)) != 0)" in source
# 4/5 号接近段进入 PATROL：只看距离，深度跳变不再清零。
assert 'name="near_goal_strict_depth" value="true"' in launch
assert "near_goal_dwell_seconds_" in source
assert "near_goal_inside_" in source
assert "near_goal_enter_stamp_" in source
assert "near_goal_dwell_ok" in source
# 默认情况（strict_depth=false）下仍要求连续满足，避免误触
assert "near_goal_strict_depth_" in source
assert "if (dist_near_goal && (depth_near_goal || !use_strict_dwell))" in source
# ROAD 超时兜底：2026-09-28 用户取消硬时限 → 6 处 launch 全部置 0.0（≤0 = 不限时）。
# 分支代码保留但被 `road_max_seconds_ > 0` 守卫（本轮契约见文件末尾）。
assert "road_max_seconds_" in source
assert "road_stage_start_" in source
assert 'name="road_max_seconds" value="180.0"' not in launch, "180s 硬时限必须已取消"
assert 'name="road_max_seconds" value="60.0"' not in launch, "60s 硬时限必须已取消"
assert launch.count('name="road_max_seconds" value="0.0"') == 6
# 4/5 号外圈接近兜底：dist<=15m 累计 2s 强制进入 PATROL。
assert "near_goal_outer_distance_" in source
assert "near_goal_outer_dwell_seconds_" in source
assert "near_goal_outer_inside_" in source
assert "near_goal_outer_enter_stamp_" in source
assert "near_goal_outer_ok" in source
assert 'name="near_goal_outer_distance" value="15.0"' in launch
assert 'name="near_goal_outer_dwell_seconds" value="2.0"' in launch
assert "double region_margin_ = 6.0" in source
assert "patrol_outer_margin_" in source  # 2026-09-27 用 patrol_outer_margin 替代 patrol_spacing
assert "case 4" in source and "xmin = 40.0; xmax = 130.0; ymin = -50.0; ymax = 0.0;" in source
assert "default:" in source and "xmin = 45.0; xmax = 130.0; ymin = 0.0; ymax = 50.0;" in source
assert "void enforceRegionBounds(geometry_msgs::Twist& cmd)" in source
# TRANSIT must expose the actual road state and use bounded road/goal fusion.
for name in ("transit_road_weight_min", "transit_road_weight_max"):
    assert 'pnh_.param<double>("%s"' % name in source
assert "diag_[DIAG_ROAD_VALID] = road_valid ? 1.0f : 0.0f;" in source
assert "diag_[DIAG_ROAD_HELD] = road_held ? 1.0f : 0.0f;" in source
assert "const double alignment = std::cos(road_angle - target_angle);" in source
assert "road_weight = clamp(" in source
assert launch.count('name="transit_road_weight_min" value="0.45"') == 6
assert launch.count('name="transit_road_weight_max" value="0.75"') == 6
# S10（2026-09-21）：旧实现只在高于目标 0.3m 时下降、低于目标时恒输出 0，
# 于是巡逻中持续掉高后永远爬不回来，一旦跌破 min_safe_altitude_ 就永久 stop=3
# （S7' 300s 长测 6/6 全部锁死）。现在必须是双向高度控制 + 可退出的 ALT_RECOVER。
assert "if (pose_z_ > target_altitude_ + 0.3 && !patrol_recovery_active_)" not in source
assert "if (zerr > alt_climb_deadband_)" in source
assert "cmd.linear.z = std::min(alt_climb_rate_, clamp(k_z_ * zerr, 0.0, max_z_));" in source
for name in ("alt_climb_deadband", "alt_descend_deadband", "climb_rate",
             "alt_recover_enter_z", "alt_recover_exit_z"):
    assert 'pnh_.param<double>("%s"' % name in source, name
    assert launch.count('name="%s"' % name) == 6, name
assert "alt_recover_active_ = true;" in source
assert "if (pose_z_ >= altRecoverExitZ())" in source
assert "diag_[DIAG_STOP_CODE] = 10.0f;" in source        # 10 = ALT_RECOVER
# 低高度不再走"整机硬停 + 永不重新基线化"的旧路径（只有"过高"才整机硬停）
assert "altitude_low || pose_z_ < altRecoverEnterZ()" in source
assert "const bool altitude_high = pose_received_ && pose_z_ > max_safe_altitude_;" in source
# 交接守卫：tracker RESUME 时高度偏低则先爬升再恢复前向
assert "RESUME with low altitude" in source
# 进/出高度必须按 target_altitude 钳制，否则 target 低于 exit 时会变成只爬不降
assert "double altRecoverEnterZ() const" in source
assert "double altRecoverExitZ() const" in source
# diag 必须带 pose_z / vz_pub / alt_climb / alt_recover（掉高排查的唯一直接证据）
for field in ("DIAG_POSE_Z", "DIAG_VZ_PUB", "DIAG_ALT_CLIMB", "DIAG_ALT_RECOVER"):
    assert field in source, field
assert '" pose_z=" << diag_[DIAG_POSE_Z]' in source
assert '" vz_pub=" << diag_[DIAG_VZ_PUB]' in source
assert '" alt_climb=" << static_cast<int>(diag_[DIAG_ALT_CLIMB])' in source
assert '" alt_recover=" << static_cast<int>(diag_[DIAG_ALT_RECOVER])' in source
assert "diag_[DIAG_VZ_PUB] = static_cast<float>(cmd.linear.z);" in source
# 区域覆盖：真实位移 + by_time 超时判定，禁止 min(0.2, dist_to_wp) 假进度
assert "const double step_adv = std::min(0.2, dist_to_wp);" not in source
assert "patrol_advance_origin_x_" in source and "patrol_advance_origin_y_" in source
assert "const bool by_dist = actual_dist >= patrol_advance_distance_;" in source
assert "const bool by_time = dwell >= patrol_advance_interval_" in source
assert "actual_dist < 0.7" in source
assert "patrol_advance_interval_, 5.0" in source
assert "patrol_wp_skip_count_" in source
# 未到达分支段级 no-progress：停留 >= interval 且实际净位移 < epsilon 才跳点。
assert "const bool fast_stuck = skip_cooldown_ok &&" in source
assert "std::fabs(cmd.linear.x) < 0.05" in source
assert "patrol_fast_stuck_dist_" in source
assert "const bool slow_stuck = skip_cooldown_ok &&" in source
assert "dwell_seg > patrol_segment_timeout_" in source
# 2026-09-25 修复（6 机全悬停）：快路径计时必须是时间戳按秒比较，
# 不能用 int 逐周期自增（50Hz 下 3 个周期就当 2.5s 用）。
assert "patrol_fast_stuck_seconds_" not in source
assert "ros::Time patrol_fast_stuck_since_;" in source
assert "(now_seg - patrol_fast_stuck_since_).toSec()" in source
# 跳点后必须有冷却，否则 segment_origin 被重置到当前位置后会每周期重复跳点。
assert 'pnh_.param<double>("patrol_segment_skip_cooldown"' in source
assert "patrol_segment_skip_stamp_ = now_seg;" in source
assert "const bool skip_cooldown_ok = patrol_segment_skip_stamp_.isZero()" in source
# 跳点路径不得再清零 front_inhibit 释放计数（它由 latch 自身滞回 + max_latch 兜底管），
# 否则任何高频跳点路径都会把 latch 永久压死在未释放状态 → v 恒为 0。
# 注意范围只到"段级 no-progress 跳点"块为止：到点推进是一次性事件，清计数是允许的。
_seg_start = source.index("if (by_progress)")
_seg_end = source.index('显式"到达"判据', _seg_start)
assert "front_inhibit_release_count_ = 0;" not in source[_seg_start:_seg_end]
assert "actual_dist < 0.7" in source
assert 'ROS_WARN("[area_mission] %s segment no-progress skip wp old=' in source
# 2026-09-25（到点推进）：显式到点判据 + 段起点按 wp_idx 惰性重置 + 近距刹停。
# 这组是"无人机坐在航点上原地绕圈"的修复，缺一不可：
#   ① 没有任何机制依赖"停下来"就能推进 wp_idx；
#   ② patrol_advance_* 的惰性重置必须记住 wp_idx，否则 dwell 恒为 0、by_time 永不触发；
#   ③ 局部目标进停靠半径时两个分支都要刹停（v 和 yaw 一起），否则仍会原地打转。
assert 'pnh_.param<double>("patrol_arrive_radius"' in source
assert 'pnh_.param<int>("patrol_arrive_cycles"' in source
assert "if (mode_ == PATROL && dist < patrol_arrive_radius_)" in source
assert "patrol_arrive_count_ >= patrol_arrive_cycles_" in source
assert "arrived wp (dist=%.2f < %.2f, cycles=%d), immediate queue advance to wp=%zu" in source
assert "patrol_advance_wp_idx_ != wp_idx_" in source
_adv_start = source.index("patrol_advance_wp_idx_ != wp_idx_")
_adv_reset = source.index("patrol_advance_wp_idx_ = wp_idx_;", _adv_start)
assert _adv_reset < source.index("const double actual_dist", _adv_start)
assert 'pnh_.param<double>("patrol_close_range_x_zero_radius"' in source
assert "if (region_return_active_ && local_goal_dist < patrol_close_range_x_zero_radius_)" in source
_return_start = source.index("if (region_return_active_ && local_goal_dist < patrol_close_range_x_zero_radius_)")
_close_start = source.index("else if (local_goal_dist < patrol_close_range_x_zero_radius_)", _return_start)
assert "cmd.linear.x = std::min(region_return_speed_, 0.8 * local_goal_dist)" in source[_return_start:_close_start]
assert "cmd.angular.z = 0.0;" in source[_close_start:_close_start + 180]
assert "patrol_spin_escape_enabled_ && !region_return_active_" in source
# no_escape 必须向调用点返回 GAVE_UP，再由调用点黑名单/跳点/retreat。
assert "return GAVE_UP;" in source
assert "const bool patrol_recovery_gave_up = recovery_result == GAVE_UP;" in source
assert "recovery gave up; blacklist wp=" in source
assert 'pnh_.param<double>("front_inhibit_max_latch_seconds"' in source
assert "latch_expired" in source
assert "front_inhibit_latch_start_ = ros::Time();" in source
assert "front_inhibit force release after" in source
# PATROL 区域覆盖目标：区域中心 + 蛇形参考
assert "patrol_goal_bx = " in source
assert "patrol_goal_by = " in source
# 2026-09-25 修复（原地绕圈）：region_* 是世界系、pose_* 是本地系，区域中心必须先扣 spawn，
# 否则合成目标落到无人机身边 ~3m，MPPI 会在近距离目标上绕圈、航点永远推进不了。
assert "0.5 * (region_xmin_ + region_xmax_) - worldPoseX()" in source
assert "0.5 * (region_ymin_ + region_ymax_) - worldPoseY()" in source
assert "0.5*(region_xmin_ + region_xmax_) - pose_x_;" not in source
# 合成目标到手后必须交还给真航点（否则近距离目标继续诱导绕圈）
assert 'pnh_.param<double>("patrol_goal_handoff_radius"' in source
assert "patrol_goal_handoff_wp_ == wp_idx_" in source
assert "blended_goal_dist < patrol_goal_handoff_radius_" in source
assert "patrol goal handoff wp=" in source
# PATROL 障碍恢复：有侧向净空就绕行，无安全方向仍硬停。
for name in ("patrol_recovery_speed", "patrol_recovery_side_speed",
             "patrol_recovery_yaw_rate", "patrol_recovery_side_safe",
             "patrol_recovery_min_progress", "patrol_recovery_timeout",
             "patrol_recovery_obstacle_clearance"):
    assert 'pnh_.param<double>("%s"' % name in source
assert "RecoveryResult updatePatrolRecovery(double fwd_lidar" in source
assert "void applyPatrolRecovery(geometry_msgs::Twist& cmd)" in source
assert "applyPatrolRecovery(cmd);" in source
# 深度/TTC 硬停只在 PATROL 未进入恢复分支时触发。
assert "!patrol_recovery_on && depth_fresh && depth_clearance < depth_stop_threshold_" in source
assert "!patrol_recovery_on &&" in source
# 区域边界保护仍是最终裁决；恢复不取消边界。
assert "region_clamp_killed_ = (std::fabs(forward_before)" in source
assert "patrol_recovery_clamp_count_" in source
# 巡逻循环：wp_idx_ 超过 size 时回绕到 0，确保反复巡逻。
assert "if (wp_idx_ >= wps->size()) wp_idx_ = 0;" in source
# P0 v3: recovery/safety contract.
for required in ("stopForwardKeepRecovery", "sectorClearance", "escapeClearance",
                 "patrol_recovery_state_", "recovery_timeout", "recovery_timeout",
                 "laser_hard_stop", "depth_hard_stop", "ttc_hard_stop",
                 "safety_reason", "patrol_recovery_use_yaw", "patrol_recovery_forward_gain"):
    assert required in source, required
assert "cmd.linear.x = 0.0;" in source
assert launch.count('name="robot_radius" value="1.0"') == 6
assert launch.count('name="robot_radius" value="2.0"') == 0
# 2026-09-24：六机并行起飞，不能按 drone_id_ 错峰等待 8s/机。
assert "const double stagger = drone_id_ * 8.0" not in source
assert "if ((now - node_start_).toSec() < stagger" not in source
assert "所有无人机收到新鲜位姿后同时进入 OFFBOARD/ARM/爬升" in source

print("area mission ROAD MPPI and shared safety static checks passed")
print("ROAD rotation/no-forward regression checks passed (floor/filter/diag/yaml)")
print("scan semantics (open vs broken) + region-return checks passed")

# ============ 2026-09-20：P0 v3.1 恢复契约（round_20260920_181208 回归后修订）============
# 该轮 300s 六机回归暴露三个机制问题，本段把"修好了"固化成源码契约：
#   (a) safety_reason_ 未每周期重置 -> 出现 "stop=0 但 safety_reason=depth_hard_stop"
#       这种上一周期残留值，误导读日志的人；
#   (b) 位姿异常保护无条件 publishImmediateStop()+return，抢占恢复侧移；且
#       last_valid_pose_ 只在"无跳变"时更新 -> 一次碰撞位移后 pose_jump 恒真，
#       stop=3 永久锁死（1~5 号全部保持到测试结束）；
#   (c) 恢复状态机在 IDLE/ACTIVE/TIMEOUT_HOLD 之间空转（进入 2.8m、判 clear 1.5m，
#       无滞回；方向每次重进都重选）-> 30s 净侧移 < 0.1m、从不完成。

# (a) 每周期归零：字符串 + 三个 hard_stop 标志位都必须在控制周期最前面清掉
cycle_head = source[source.index("void controlCb"):
                    source.index("const ros::Time now_sim = ros::Time::now();")]
assert 'safety_reason_ = "none";' in cycle_head
assert "diag_[DIAG_LASER_HARD_STOP] = 0.0f;" in cycle_head
assert "diag_[DIAG_DEPTH_HARD_STOP] = 0.0f;" in cycle_head
assert "diag_[DIAG_TTC_HARD_STOP] = 0.0f;" in cycle_head

# (b) 位姿异常：必须区分"高度发散(过高)"与"碰撞类跳变"；跳变期间允许恢复侧移；反锁死重新基线化
#     S10 起低高度不再算 altitude_bad（不再整机硬停），而是走 ALT_RECOVER。
assert "float modeDiagCode() const" in source
assert "diag_[DIAG_MODE] = modeDiagCode();" in source
assert "const bool altitude_high = pose_received_ && pose_z_ > max_safe_altitude_;" in source
assert "const bool altitude_low = pose_received_ && pose_z_ < min_safe_altitude_;" in source
assert "const bool altitude_bad = altitude_high;" in source
assert "abnormal_pose_kind_ = altitude_bad ? 1 : 2;" in source
assert "max_safe_altitude_" in source
assert "!altitude_bad && patrol_recovery_state_ == 1 && patrol_recovery_dir_ != 0" in source
assert "pose_abnormal_escape_time_" in source
assert "keeping recovery " in source
assert "pose_rebaseline_time_" in source
assert "re-baselining to current pose" in source

# (c) 恢复状态机：进入/退出滞回、方向锁定、承诺位移、TIMEOUT_HOLD 保留侧移、冷却期
rec = source[source.index("RecoveryResult updatePatrolRecovery"):source.index("void stopForwardKeepRecovery")]
for required in ("patrol_recovery_exit_clearance_", "patrol_recovery_min_commit_time_",
                 "patrol_recovery_max_attempts_", "patrol_recovery_cooldown_",
                 "patrol_recovery_cooldown_until_", "patrol_recovery_dir_changes_",
                 "patrol_recovery_attempt_"):
    assert required in rec or required in source, required
assert "const bool committed = elapsed >= patrol_recovery_min_commit_time_ &&" in source
assert "front_clear && side_clear" in rec
# 方向只在 IDLE->ACTIVE 选一次；之后只有"超时且还有机会"才换边
assert rec.count("patrol_recovery_dir_ = (left >= right)") == 1
assert "patrol_recovery_dir_ = -patrol_recovery_dir_;" in rec
apply_body = source[source.index("void applyPatrolRecovery"):
                    source.index("void enforceRegionBounds")]
assert "patrol_recovery_state_ != 1 && patrol_recovery_state_ != 2" in apply_body
assert "cmd.linear.x = cmd.linear.y = cmd.angular.z = 0.0;" not in apply_body

# 侧向输出必须可观测：v_pub 只记 smooth_x_，仅靠它无法判断侧移到底发没发出去
assert "diag_[DIAG_V_LAT_PUB] = static_cast<float>(cmd.linear.y);" in source
for field in ("DIAG_V_LAT_PUB", "DIAG_ABNORMAL_POSE", "DIAG_ABNORMAL_CYCLES",
              "DIAG_RECOVERY_ATTEMPT", "DIAG_RECOVERY_DIR_CHANGES", "DIAG_SPEED_ENVELOPE"):
    assert field in source, field
assert '<< " v_lat_pub="' in source and '<< " abnormal_pose="' in source
# 新增字段必须追加在 DIAG_COUNT 之前尾部，不能插在中间（否则老曲线下标错位）
enum_start = source.index("enum DiagField {")
enum_block = source[enum_start:source.index("DIAG_COUNT", enum_start)]
assert enum_block.index("DIAG_TTC_HARD_STOP") < enum_block.index("DIAG_V_LAT_PUB")
assert enum_block.index("DIAG_V_LAT_PUB") < enum_block.index("DIAG_POSE_Z")
assert enum_block.index("DIAG_SPEED_ENVELOPE") < enum_block.index("DIAG_POSE_Z")
assert (enum_block.index("DIAG_POSE_Z") < enum_block.index("DIAG_VZ_PUB") <
        enum_block.index("DIAG_ALT_CLIMB") < enum_block.index("DIAG_ALT_RECOVER"))
assert enum_block.index("DIAG_ALT_RECOVER") < enum_block.index("DIAG_STALL_COUNT")

# PATROL 前向速度包络：降低"顶到墙"的碰撞动能（位姿跳变是被撞出来的）
assert "patrol_speed_envelope_range_" in source and "patrol_speed_envelope_min_" in source
assert "diag_[DIAG_SPEED_ENVELOPE] = 1.0f;" in source
# stop_code 9 = PATROL 恢复超时保持，不再和 TTC(6) 混在一起
assert "diag_[DIAG_STOP_CODE] = 9.0f;" in source
# launch 六机一致：高度上界
assert launch.count('name="max_safe_altitude" value="30.0"') == 6
assert launch.count('name="robot_radius" value="1.0"') == 6

print("P0 v3.1 patrol-recovery contract checks passed "
      "(per-cycle safety reset / pose-abnormal escape / no deadlock / hysteresis)")

# =============================================================================
# 2026-09-22：PATROL hover-skip 快路径（MPPI × 蛇形航点冲突）
# =============================================================================

# 参数声明 + 默认值
for name, default in (("patrol_hover_deadline", "12.0"),
                      ("patrol_hover_skip_blacklist", "30.0"),
                      ("patrol_hover_v_threshold", "0.2")):
    assert ('pnh_.param<double>("%s", %s_, %s)' % (name, name, default)) in source, name

# diag 枚举（DIAG_HOVER_* 必须存在；DIAG_COUNT 之前）
assert "DIAG_HOVER_SKIP_COUNT" in source
assert "DIAG_HOVER_DEADLINE_S" in source

# 跳线逻辑（PATROL hover-skip / segment skip）
assert "PATROL hover deadline" in source
assert "hover_skip" in source
assert "patrol_hover_skip_blacklist" in source

# effective 死线 = max(用户期望, 2×recovery_timeout)
assert "2.0 * patrol_recovery_timeout_" in source

# 取值来源：必须用 smooth_x_（上一周期真正发布的速度）。
# hover 判定表达式不得出现 std::fabs(cmd.linear.x)——
# 切出 hover 块再断言（块必须出现在 PATROL 块内部、stall 块之后）。
hover_marker_start = source.find("//---stall end---")
assert hover_marker_start != -1, "源码 stall 块必须以 //---stall end--- 结束"
# hover 块严格范围：从 "--- (新) hover 死线检测" 注释到 PATROL 块结束 `}`。
hover_block_start = source.find("// --- (新) hover 死线检测", hover_marker_start)
assert hover_block_start != -1, "源码必须包含 hover 块起点注释"
# PATROL 块结束：从 hover 块起点向后找第一个 "else if (mode_ != PATROL)"
hover_block_end = source.find("else if (mode_ != PATROL)", hover_block_start)
assert hover_block_end != -1, "PATROL 块结束标志缺失"
hover_block = source[hover_block_start:hover_block_end]
# 允许出现 cmd.linear.x = 0.0（跳点分支里清零），不允许用作悬停判据。
assert "std::fabs(smooth_x_)" in hover_block
assert "std::fabs(cmd.linear.x)" not in hover_block

# M1 顺序：stall 块的"判定 → 滚动重置"必须先判定后重置。
# 用 //---stall begin--- / //---stall end--- marker 注释切完整一段。
stall_start = source.find("//---stall begin---")
stall_end   = source.find("//---stall end---", stall_start)
assert stall_start != -1 and stall_end != -1, "stall 块需有显式 marker 注释"
stall_block = source[stall_start:stall_end]
ratio_pos   = stall_block.find("patrol_blocked_ratio_ = patrol_stall_window_cycles_")
reset_pos   = stall_block.find("else if (window_elapsed >= patrol_stall_window_)")
assert ratio_pos != -1 and reset_pos != -1, "ratio / rolling reset 缺一"
assert ratio_pos < reset_pos, "ratio 计算必须在滚动重置 else-if 之前"

# M3：两处死变量必须各自独立替换（不能只改一处）。
assert source.count("patrol_recovery_active_") == 0, "死变量必须全部清掉"
assert "patrol_recovery_state_ == 0" in source, "line 1556 高度下降守卫未替换"
assert "patrol_recovery_state_ != 0" in source, "line 2321 blocked 公式未替换"

# 周期计数（与 20Hz 解耦）
assert "patrol_stall_window_cycles_" in source

# 现算 dist，避免 stall 跳点后还用旧 tx/ty
assert "dist_now = std::hypot((*wps)[wp_idx_].first - worldPoseX()" in hover_block

# ALT_RECOVER 进入/退出复位 hover 窗口
alt_enter_idx = source.index("alt_recover_active_ = true;")
alt_enter_block = source[alt_enter_idx:alt_enter_idx + 600]
assert "hover_low_count_ = 0;" in alt_enter_block
assert "hover_window_start_ = ros::Time();" in alt_enter_block
# 退出 ALT_RECOVER 时也要复位。用 "ALT_RECOVER done" 定位，而不是
# `alt_recover_active_ = false;` 的首次出现 —— 2026-09-24 新增的"位姿不可信 →
# 退出 ALT_RECOVER"分支也含这句，会先命中它。
alt_exit_idx = source.index("ALT_RECOVER done: pose_z=")
alt_exit_block = source[alt_exit_idx - 400:alt_exit_idx + 600]
assert "alt_recover_active_ = false;" in alt_exit_block
assert "hover_low_count_ = 0;" in alt_exit_block
assert "hover_window_start_ = ros::Time();" in alt_exit_block

# diag 文本输出追加 hover 字段
assert "hover_hold_s=" in source
assert "hover_skip=" in source

print("PATROL hover-skip 快路径契约检查通过（deadline 5s + 线推进 + 死变量替换 + 滚动 stall）")

# ===================== 2026-09-24：位姿合理性 + ALT_RECOVER 超时 =====================
# 背景：实测 uav2 多轮出现 z=-128 ~ -1503m 的 EKF 高度发散（单步只漂 ~0.2m，pose_jump
# 抓不到）。必须"停 + 等估计恢复"，而不是把它当"低空"去爬（会一直爬到 >6m 被裁判判 0）。
for name in ("pose_z_min_valid", "alt_recover_timeout"):
    assert 'pnh_.param<double>("%s"' % name in source, name
assert "const bool pose_z_invalid = pose_received_ && pose_z_ < pose_z_min_valid_;" in source
# ① 不可信高度不得进 ALT_RECOVER
assert "!pose_z_invalid &&" in source
assert "altitude_low || pose_z_ < altRecoverEnterZ()" in source
# ② 不可信高度 → 停 + 标 stop=3 / abnormal_pose=3 / safety_reason=pose_z_invalid
assert 'safety_reason_ = "pose_z_invalid";' in source
assert "diag_[DIAG_ABNORMAL_POSE] = 3.0f;" in source
assert "pose_invalid_cycles_" in source
# ③ 该分支必须在 PAUSE/RESUME 之后（tracker 交接不能被坏位姿挡住）
_pause_idx = source.index("if (pause_requested_ && !paused_)")
_invalid_idx = source.index('safety_reason_ = "pose_z_invalid";')
assert _invalid_idx > _pause_idx, "pose_z_invalid 分支必须在 PAUSE/RESUME 之后"
# ④ ALT_RECOVER 必须能超时退出（否则 z 卡在低位就无限爬）
assert "alt_recover_timeout_ > 0.0" in source
assert "ALT_RECOVER timeout after" in source
# ⑤ 参数快照要带上这两个可调项（事后能确认到底加载了没）
assert "pose_z_min_valid=" in source and "alt_recover_timeout=" in source
# ⑥ 不允许再出现不存在的标识符
assert "alt_recovery_enter_z_" not in source, "多余的下划线：应为 alt_recover_enter_z_"
assert "mppi_valid_" not in source, "area_mission 没有 mppi_valid_ 成员（只有 mppi_.valid()）"
print("位姿合理性 + ALT_RECOVER 超时契约检查通过（负高度停+等恢复 / 超时放弃爬升）")

# ===================== 2026-09-24：放弃型退出(no_escape)用更长冷却 =====================
# 障碍还在前方时 1s 就恢复前向会撞上去；该冷却期同时受"前向限速 40%"约束。
assert 'pnh_.param<double>("patrol_recovery_giveup_cooldown"' in source
assert "double patrol_recovery_giveup_cooldown_ = 2.5;" in source
assert source.count("patrol_recovery_giveup_cooldown_") >= 4     # 参数+成员+2 处使用
assert source.count('safety_reason_ = "no_escape";') == 3, "no_escape 路径条数变了"
assert source.count("now + ros::Duration(patrol_recovery_giveup_cooldown_)") == 3
assert source.count("now + ros::Duration(patrol_recovery_cooldown_)") == 2   # 成功/净空恢复仍 1.0s
print("放弃型 cooldown 延长契约检查通过（no_escape 3 处 2.5s / 正常退出 2 处 1.0s）")
# ===== 2026-09-25：深度点过滤（自机 + 地面）+ 内参修正 =====
# 背景：depthCb 的点同时喂 depth_min_（安全层）和 mppi_.setDepthPoints（MPPI 深度代价）。
# 不过滤时自机部件（实测最小有效深度 0.05~0.11m）和地面/屋顶点会被当成前方障碍，
# 把前向速度锁死（front_inhibit_reason=depth 占 99%）、窄通道进不去。
assert 'pnh_.param<bool>("depth_ground_filter"' in source
assert 'pnh_.param<bool>("depth_self_filter"' in source
assert 'pnh_.param<bool>("depth_pitch_follows_body"' in source
assert 'pnh_.param<double>("depth_ground_clearance"' in source
assert "accept_point(x, y, d, xb, yb)" in source
assert source.count("accept_point(x, y, d, xb, yb)") == 2, "两种编码分支都要过滤"
assert "if (!accept_point" in source
assert "std::hypot(xb, yb) < depth_self_radius_" in source
assert "(cam_h + zb) < depth_ground_clearance_" in source
# 位姿异常时跳过地面判定（uav3 出现过 pose_z=-2.88，若照滤会把真障碍全滤掉）
assert "cam_h > 0.15 && cam_h < 20.0" in source
assert "pose_pitch_" in source
# 旋转符号修正：旧实现是 +sp*yc（把下视方向投反），必须只剩修正后的 -sp*yc
assert "cp * d - sp * yc" in source
assert "cp * z + sp * yc" not in source
# 内参：该机型的 depth 相机实测 camera_info fx=fy=205.4696（554.0 对应 60° FOV，是别的机型）
assert 'name="depth_fx" value="554.0"' not in launch
assert 'name="depth_fy" value="554.0"' not in launch
assert launch.count('name="depth_fx" value="205.4696"') == 6
assert launch.count('name="depth_fy" value="205.4696"') == 6
# MPPI 深度点仍在喂，且过滤发生在喂之前
assert source.index("accept_point") < source.index("mppi_.setDepthPoints(depth_points)")
assert "road_mppi_.setDepthPoints(depth_points)" in source
print("深度点过滤（自机/地面）+ 内参/旋转符号修正契约检查通过")
# ===== 2026-09-25：深度外参走 TF + MPPI 深度代价总开关 =====
# 相机外参在仓库里有三份互相不一致的描述（static_tf.launch / SDF 传感器 pose / 此处参数），
# 且 static_tf 的复合旋转还带 ~41° roll。因此：外参优先用 TF，失败才退回参数化外参；
# 在投影经 /depth/points 对拍确认前，MPPI 深度代价默认关闭（错误点云比没有更糟）。
assert 'pnh_.param<std::string>("depth_extrinsic_mode"' in source
assert 'pnh_.param<bool>("mppi_depth_enabled"' in source
assert "tf_buffer_.lookupTransform(" in source
assert '"base_link", msg->header.frame_id' in source
assert "tf_ok = true;" in source and "catch (const tf2::TransformException&)" in source
assert "tf2_ros::Buffer tf_buffer_;" in source
assert "tf2_ros::TransformListener tf_listener_{tf_buffer_};" in source
assert "if (mppi_depth_enabled_)" in source
# 深度点/过期语义不变：depth_received_ 仍按点集是否为空判定（不受开关影响）
assert "depth_received_ = !depth_points.empty();" in source
assert launch.count('name="mppi_depth_enabled" value="false"') == 0, "mppi_depth_enabled 已迁入 yaml"
assert launch.count('name="depth_extrinsic_mode" value="tf"') == 6
print("深度外参 TF 优先 + MPPI 深度代价总开关契约检查通过")

# ===================== 2026-09-28：hover-after-tracking 回归修复契约 =====================
# 背景（logs/hover_diag/REPORT_hover_after_tracking.md，已定案）：本次未提交重构把 MISSION
# 里整段速度生成（目标选择 + 区域/道路逻辑 + MPPI + 恢复/避障 + publishSmooth）包进了
# `if (return_to_pause_) { ... } else { ... }` 的 else，于是"追踪结束、正飞回 PAUSE 点"
# 期间一个速度指令都不发（DIAG 表现：stop=0 且 v_pub/mppi_samples/fwd_eval 全 0、dist 不变）；
# 而 return_to_pause_ 只由 dist < wp_reach_tolerance_ 清除 → 无人机不动 → 永远清不掉
# → 永久悬停。下面四条把修复钉成静态契约，防止整块再次被塞回 else。
import re as _re

def _strip_cpp_comments(text):
    text = _re.sub(r"/\*.*?\*/", "", text, flags=_re.S)
    return _re.sub(r"//[^\n]*", "", text)

_code = _strip_cpp_comments(source)

# ① 主修复：`if (return_to_pause_)` 的到达判定之后不允许再紧跟 else（速度生成块必须无条件执行）。
_anchor = _code.index("if (return_pause_since_.isZero())")
_ret_open = _code.rindex("if (return_to_pause_)", 0, _anchor)
_naked_open = _code.index("const bool road_timeout_force", _anchor)   # 速度生成裸块的第一行
# return_to_pause_ 的 if 体结束（最后一个 }）到裸块 { 之间只允许 `}` + `{`，不许出现 else。
_junction = _code[_code.rindex("}", 0, _naked_open):_naked_open]
assert "else" not in _junction, \
    "return_to_pause_ 块后仍紧跟 else：速度生成块又会被跳过 → 静默悬停死锁回归"
assert _re.search(r"\}\s*\{", _junction), "到达判定之后应保留大括号为无条件裸块"
assert not _re.search(r"\}\s*else\s*\{\s*const bool road_timeout_force", _code), \
    "回归：整段速度生成又被 else 包住（hover-after-tracking 根因原样复活）"
# 速度生成块确实无条件执行：仍在 return_to_pause_ 判定之后，且仍会 publishSmooth。
assert _naked_open > _ret_open
assert _code.index("publishSmooth(cmd)", _ret_open) > _ret_open

# ② 兜底 A：RESUME 处记录 return_pause_since_，回飞超过 30s 强制 RESUMED（防再次死锁）。
assert 'pnh_.param<double>("return_pause_timeout", return_pause_timeout_, 30.0);' in source
assert "double return_pause_timeout_ = 30.0;" in source
assert "return_pause_since_ = ros::Time::now();" in source
assert "return_pause_elapsed >= return_pause_timeout_" in source
# 超时判定必须真的在 return_to_pause_ 块内（放在块外就永远不会在回飞时触发）
assert _ret_open < _code.index("return_pause_elapsed >= return_pause_timeout_") < _naked_open
assert "return_pause_since_.isZero()" in source, "计时器需自愈（未置位也要能算出 elapsed）"
assert "RETURN_TO_PAUSE timeout after" in source, "超时必须打 WARN（含 dist/elapsed）"
assert source.count("return_to_pause_ = false; mode_ = resume_mode_; wp_idx_ = resume_wp_idx_;") == 2, \
    "只允许两处复位：正常到达 + 30s 超时"
assert source.count('publishMissionState("RESUMED:"') == 2, "超时必须和正常恢复一样发 RESUMED:"

# ③ 兜底 B：指令看门狗 —— publishSmooth 维护 last_cmd_pub_stamp_，
#    controlCb 末尾 MISSION && !paused_ 且 >0.5s 未发指令 → 补发零速 + ROS_ERROR_THROTTLE。
assert 'pnh_.param<double>("cmd_pub_silence_timeout", cmd_pub_silence_timeout_, 0.5);' in source
assert "double cmd_pub_silence_timeout_ = 0.5;" in source
assert "last_cmd_pub_stamp_ = ros::Time::now();" in source
assert "if (state_ == MISSION && !paused_)" in source
assert "(now_cmd - last_cmd_pub_stamp_).toSec() > cmd_pub_silence_timeout_" in source
assert "no cmd_vel published for >" in source
assert "ROS_ERROR_THROTTLE(1.0," in source
_watchdog = source[source.index("兜底 B"):]
assert "cmd_vel_pub_.publish(zero);" in _watchdog, "看门狗必须真的补发零速（不能只报警）"
assert "smooth_x_ = 0.0;" in _watchdog, "补零速时平滑器也要清零（否则恢复控制会速度阶跃）"
assert "diag_[DIAG_CMD_SILENT_CYCLES] = static_cast<float>(cmd_silent_cycles_);" in _watchdog
assert source.index("cmd_vel_pub_.publish(zero);") > source.index("case DONE:"), \
    "看门狗必须在 switch(state_) 之后（任何 break/return 都要能被兜住）"

# ④ DIAG 追加 return_to_pause / cmd_silent_cycles / return_pause_elapsed 三个字段，
#    且只能追加在枚举末尾（DIAG_COUNT 之前），老下标一个都不许动。
_enum_start = source.index("enum DiagField {")
# 去掉行尾注释后再判"最后一项"，否则注释会挡在末尾
_enum = _strip_cpp_comments(source[_enum_start:source.index("DIAG_COUNT", _enum_start)])
# 2026-09-28（本轮）：在 DIAG_SPIN_ESCAPE_COUNT 之后又追加了 DIAG_TURN_GUARD_ACTIVE，
# "末尾"断言同步延长一项（只延长尾部，老字段顺序/下标语义不变）。
assert _re.search(r"DIAG_RETURN_PAUSE_ELAPSED,\s*DIAG_SKIP_ENABLED,\s*DIAG_ARRIVE_RADIUS,\s*DIAG_SPIN_ESCAPE_COUNT,\s*DIAG_TURN_GUARD_ACTIVE,\s*$", _enum), \
    "三个新字段必须仍是 DiagField 的最后一段（2026-09-28 只允许在其后追加 SKIP_ENABLED/ARRIVE_RADIUS/SPIN_ESCAPE_COUNT/TURN_GUARD_ACTIVE）"
for _name in ("DIAG_RETURN_PAUSE,", "DIAG_CMD_SILENT_CYCLES,", "DIAG_RETURN_PAUSE_ELAPSED,"):
    assert _name in _enum, f"DiagField 缺少追加字段 {_name}"
assert _enum.index("DIAG_TRUTH_Y") < _enum.index("DIAG_RETURN_PAUSE") \
    < _enum.index("DIAG_CMD_SILENT_CYCLES") < _enum.index("DIAG_RETURN_PAUSE_ELAPSED"), \
    "新字段顺序/位置不对（必须整体追加在旧字段之后）"
for _name in ("DIAG_RETURN_PAUSE", "DIAG_CMD_SILENT_CYCLES", "DIAG_RETURN_PAUSE_ELAPSED"):
    assert source.count(_name) >= 2, _name
assert '<< " return_to_pause=" << static_cast<int>(diag_[DIAG_RETURN_PAUSE])' in source
assert '<< " cmd_silent_cycles=" << static_cast<int>(diag_[DIAG_CMD_SILENT_CYCLES])' in source
assert '<< " return_pause_elapsed=" << diag_[DIAG_RETURN_PAUSE_ELAPSED]' in source
assert source.index('" mppi_replanned="') < source.index('" return_to_pause="'), \
    "新字段必须追加在人读 DIAG 串末尾"
# 回飞期间必须有 1Hz 日志（"149-199s 任务节点一句话没有"是本轮最大盲点）
assert 'ROS_WARN_THROTTLE(1.0, "[area_mission] %s RETURN_TO_PAUSE dist=%.1f elapsed=%.1fs"' in source
print("hover-after-tracking 回归契约检查通过（去 else 裸块 / 30s 回飞超时 / 0.5s 指令看门狗 / DIAG 追加三字段）")

# ============ 2026-09-28：巡逻到点半径 5m + 停滞恢复开关契约 ============
# 需求：① 巡逻航点的"到达"判据 2.0 → 5.0 m（patrol_arrive_radius 与 wp_reach_tolerance 分工见下）；
#      ② patrol_skip_enabled 默认 false：只允许到点推进，保持用户指定的航点顺序。
# 统计口径：文本级——每条跳点路径（索引推进 / 黑名单写入 / retreat 赋值）向前 1800 字符内
#          必须出现 `if (!patrol_skip_enabled_)`；到点推进那一处必须**不**被它包住。

# ① 到点半径默认 5.0，运行时巡逻判据用的就是它；wp_reach_tolerance_ 为 3.0。
assert 'pnh_.param<double>("patrol_arrive_radius", patrol_arrive_radius_, 5.0);' in source
assert "double patrol_arrive_radius_ = 5.0;" in source
assert "if (mode_ == PATROL && dist < patrol_arrive_radius_)" in source
assert "arrived wp (dist=%.2f < %.2f, cycles=%d), immediate queue advance to wp=%zu" in source
# wp_reach_tolerance_ 只服务 ROAD 到点 + return_to_pause_ 清除，保持为 3m。
assert 'pnh_.param<double>("wp_reach_tolerance", wp_reach_tolerance_, 3.0);' in source
assert "double control_rate_ = 20.0, wp_reach_tolerance_ = 3.0;" in source
assert "if (dist < wp_reach_tolerance_" in source
# 连续周期数不动。
assert 'pnh_.param<int>("patrol_arrive_cycles", patrol_arrive_cycles_, 3);' in source

# ② 停滞跳点默认关闭，运行时仍可显式启用。
assert 'pnh_.param<bool>("patrol_skip_enabled", patrol_skip_enabled_, false);' in source
assert "bool patrol_skip_enabled_ = false;" in source
assert launch.count('name="patrol_skip_enabled" value="false"') == 6
assert "patrol skip ENABLED (patrol_skip_enabled:=true)" in source
assert "if (escape_active)" in source
assert source.index("if (escape_active)") < source.index("if (spin_held >= patrol_spin_escape_hold_s_")
assert "cmd.linear.x = 0.0;" in source[source.index("if (escape_active)"):source.index("if (escape_active)") + 600]
assert "tx = clamp(tx, region_xmin_ + path_margin, region_xmax_ - path_margin);" in source
assert "cell_wx < region_xmin_ + path_margin" in source

# ③ 所有 watchdog 推进路径仍可由 patrol_skip_enabled 显式关闭。
_GATE = "if (!patrol_skip_enabled_)"
_GATE_WINDOW = 1800  # 每个跳点分支到它自己的 gate 的距离上限（字符）

def _gated_sites(pattern, label):
    sites = [m.start() for m in _re.finditer(pattern, _code)]
    assert sites, "没找到跳点路径：%s" % label
    for off in sites:
        assert _GATE in _code[max(0, off - _GATE_WINDOW):off], \
            "%s（offset %d）没有被 patrol_skip_enabled 包住：%r" % (
                label, off, _code[max(0, off - 120):off + 80])
    return sites

# 队列按固定顺序推进：到点正常推进，停滞恢复受参数 gate 控制。
_arrive_marker = _code.index("arrived wp (dist=%.2f")
_arrive_site = _code.index('advancePatrolQueue("arrived")')
assert "patrol_arrive_count_ >= patrol_arrive_cycles_" in _code[_arrive_site - 700:_arrive_site]
assert _GATE not in _code[max(0, _arrive_site - _GATE_WINDOW):_arrive_site]
for reason in ("segment watchdog", "fast/slow watchdog", "stall watchdog",
               "hover watchdog", "recovery give-up", "segment advance"):
    marker = 'advancePatrolQueue("%s")' % reason
    assert marker in source, marker
    if reason != "arrived":
        off = _code.index(marker)
        assert _GATE in _code[max(0, off - _GATE_WINDOW):off], \
            "停滞推进必须受 patrol_skip_enabled gate: %s" % reason

# 目标选择器里"进到达半径就换下一个候选点"也是未确认到点的索引推进，禁用时必须关掉。
print("不巡逻/原地打转三处修复契约检查通过（setPath 机体系帧修复 / 自旋逃逸兜底+DIAG 计数 / 到点判据排除返航段）")


# ============ 2026-09-28（本轮）：取消 ROAD 硬时限 + 转向栅门限速解耦（末尾纯追加） ============
# 用户拍板：① 不许因为 ROAD 计时到点就强制离开道路段 —— road_max_seconds 语义改为 <=0 = 不限时，
#             默认 0 = 取消；超时分支保留但必须被 `> 0` 守卫（随时可恢复）。
#          ② 转向栅门的"净空来源替换"是有意为之的拐角修复，保持不动；但限速必须只在"真的在转弯"
#             时生效（turning_now），且限速值参数化 patrol_turn_speed_cap（1.2，替换硬编码 0.9）。
# 依据：实测 0/1/5 号巡逻 v_pub 中位 = 0.90，而 MPPI 要 2.00 → 全程半速。

# ---- ① 取消 ROAD 硬时限：C++ 默认 0 + 两处超时分支都被 `> 0` 守卫 + launch 6 处全 <= 0 ----
assert 'pnh_.param<double>("road_max_seconds", road_max_seconds_, 0.0);' in source, \
    "road_max_seconds 默认值必须为 0.0（<=0 = 不限时 = 默认取消硬时限）"
assert "double road_max_seconds_ = 0.0;" in source, "成员默认值也必须为 0.0"
# 两处「ROAD 超时强制进 PATROL」分支（全局版 + use_strict_dwell 版）都必须先判 > 0。
for _decl in ("const bool road_timeout_force_global", "const bool road_timeout_force "):
    _i = _code.index(_decl)
    _seg = _code[_i:_i + 320]
    assert "(road_max_seconds_ > 0.0)" in _seg, \
        "%s 缺少 `road_max_seconds_ > 0` 守卫（<=0 时硬时限必须完全不生效）" % _decl
    assert _seg.index("(road_max_seconds_ > 0.0)") < _seg.index("road_max_seconds_;"), \
        "%s 的守卫必须在超时比较之前（否则超时判定照样执行）" % _decl
assert _code.count("(road_max_seconds_ > 0.0)") == 2, \
    "守卫必须恰好出现在两处超时分支（别漏掉 use_strict_dwell 版）"
# launch：6 处显式配置必须全部 <= 0。否则 launch 会把正数灌进节点，C++ 的默认 0 形同虚设
# （param 名不是 arg，命令行 road_max_seconds:=0 覆盖不了）。
_root_xml = ET.parse(launch_path).getroot()
_rm_launch = [p.get("value") for p in _root_xml.iter("param")
              if p.get("name") == "road_max_seconds"]
assert len(_rm_launch) == 6, "六架都必须显式配置 road_max_seconds，实得 %r" % (_rm_launch,)
for _v in _rm_launch:
    assert float(_v) <= 0.0, \
        "launch 里仍有正数硬时限（%s）：它会强制 ROAD 超时切 PATROL，等于硬时限没取消" % _v

# ---- ② 限速只在真的转弯时生效 + 值参数化（净空来源替换那一半逐字保留） ----
assert "if (guard_goal_valid_ && (mode_ == PATROL || mode_ == TRANSIT_TO_REGION) && fwd_lidar > 1.2)" in source, \
    "净空替换的进入条件被改动（拐角修复回归）"
assert "guard_lidar = plan_clear;" in source, "净空来源替换本身必须保留"
assert "const double need = std::max(2.5, patrol_recovery_obstacle_clearance_ + 0.5);" in source
assert "sectorClearance(plan_angle, 20.0 * M_PI / 180.0)" in source
# turning_now 判据：A* 前视方向 vs 机头夹角 **或** |cmd.angular.z| 角速度阈值。
assert "const bool turning_now =" in source, "缺少 turning_now（真的在转弯）判据"
assert "std::atan2(guard_goal_by_, guard_goal_bx_)" in source, "夹角项必须取自 A* 前视方向"
assert "guard_plan_angle_deg > turn_guard_angle_deg_" in source, "缺少夹角阈值判据"
assert "std::fabs(cmd.angular.z) > turn_guard_yaw_rate_" in source, "缺少偏航角速度阈值判据"
_tn_i = source.index("const double guard_plan_angle_deg")
_tn_seg = source[_tn_i:_tn_i + 500]
assert "const bool turning_now =" in _tn_seg and "||" in _tn_seg, \
    "两个转弯条件必须是『或』关系（满足其一即算在转弯）"
assert "* 180.0 / M_PI" in _tn_seg, "夹角阈值必须换算成度（与 turn_guard_angle_deg 对齐）"
# 限速：turning_guard && turning_now，值取 patrol_turn_speed_cap_，硬编码 0.9 必须消失。
assert "if (turning_guard && turning_now)" in source, \
    "限速条件必须同时要求 turning_guard（净空已替换）与 turning_now（真的在转弯）"
_cap_i = source.index("if (turning_guard && turning_now)")
_cap_seg = source[_cap_i:_cap_i + 160]
assert "cmd.linear.x = std::min(cmd.linear.x, patrol_turn_speed_cap_);" in _cap_seg, \
    "限速必须改用参数 patrol_turn_speed_cap_"
assert "std::min(cmd.linear.x, 0.9)" not in source, \
    "硬编码 0.9 限速必须已删除（否则直路仍被压到 0.9 = 巡逻全程半速）"
assert not _re.search(r"if \(turning_guard\)\s*\n\s*cmd\.linear\.x", source), \
    "限速又变成只看 turning_guard（没有 turning_now）→ 直路半速回归"
# 参数与成员默认值（三个参数：夹角阈值 / 角速度阈值 / 限速值）。
assert 'pnh_.param<double>("turn_guard_angle_deg", turn_guard_angle_deg_, 25.0);' in source
assert 'pnh_.param<double>("turn_guard_yaw_rate", turn_guard_yaw_rate_, 0.35);' in source
assert 'pnh_.param<double>("patrol_turn_speed_cap", patrol_turn_speed_cap_, 2.0);' in source
assert "double turn_guard_angle_deg_ = 25.0;" in source
assert "double turn_guard_yaw_rate_ = 0.35;" in source
assert "double patrol_turn_speed_cap_ = 2.0;" in source

# ---- ③ 可观测：DIAG_TURN_GUARD 进 formatDiag，DIAG_TURN_GUARD_ACTIVE 追加在 DIAG_COUNT 之前 ----
assert '<< " turn_guard=" << static_cast<int>(diag_[DIAG_TURN_GUARD])' in source, \
    "DIAG_TURN_GUARD 必须打进 formatDiag（本轮之前只在枚举里、从未打印）"
assert '<< " turn_guard_active=" << static_cast<int>(diag_[DIAG_TURN_GUARD_ACTIVE])' in source
assert '<< " turn_speed_cap=" << patrol_turn_speed_cap_' in source
assert "const bool turn_guard_active = turning_guard && turning_now;" in source
assert "diag_[DIAG_TURN_GUARD_ACTIVE] = turn_guard_active ? 1.0f : 0.0f;" in source
assert "diag_[DIAG_TURN_GUARD] = turning_guard ? 1.0f : 0.0f;" in source
# 人读串顺序：老字段在前，本轮三个字段追加在最后。
assert source.index('" spin_escape="') < source.index('" turn_guard="') \
    < source.index('" turn_guard_active="') < source.index('" turn_speed_cap="'), \
    "新字段必须追加在人读 DIAG 串末尾"
# 枚举位置：只能追加在 DIAG_SPIN_ESCAPE_COUNT 之后、DIAG_COUNT 之前（老下标一个不动）。
_enum_start4 = source.index("enum DiagField {")
_enum4 = _strip_cpp_comments(source[_enum_start4:source.index("DIAG_COUNT", _enum_start4)])
assert _re.search(r"DIAG_SPIN_ESCAPE_COUNT,\s*DIAG_TURN_GUARD_ACTIVE,\s*$", _enum4), \
    "DIAG_TURN_GUARD_ACTIVE 必须整段追加在 DiagField 末尾（DIAG_COUNT 之前）"
assert _enum4.index("DIAG_TURN_GUARD,") < _enum4.index("DIAG_SPIN_ESCAPE_COUNT") \
    < _enum4.index("DIAG_TURN_GUARD_ACTIVE"), \
    "老下标 DIAG_TURN_GUARD 不许移动，新字段只能在最末尾追加"
print("ROAD 硬时限取消 + 转向栅门限速解耦契约检查通过（road_max_seconds<=0 守卫 / turning_now 双判据 / patrol_turn_speed_cap=2.0 / DIAG_TURN_GUARD_ACTIVE 末尾追加）")


# Fixed-route queue contract: eight buffered route entries, ordered dequeue/refill,
# queue head owns wp_idx, arrival advances once, and cached MPPI plan is invalidated.
assert "static constexpr size_t patrol_queue_capacity_ = 8;" in source
assert "std::deque<size_t> patrol_queue_;" in source
assert "void refillPatrolQueue()" in source
assert "void advancePatrolQueue(const char* reason)" in source
assert "patrol_queue_.pop_front();" in source
assert "patrol_queue_.push_back(next);" in source
assert "wp_idx_ = patrol_queue_.front();" in source
assert "mppi_cache_.ready = false;" in source
assert 'advancePatrolQueue("arrived");' in source
assert 'if (!patrol_queue_.empty()) wp_idx_ = patrol_queue_.front();' in source
print("固定顺序 8 槽巡逻队列契约检查通过")
