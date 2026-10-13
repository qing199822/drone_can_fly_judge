#coding: utf-8
import rospy
import atexit
import sys
import math
from std_msgs.msg import Int16,String, Time, Float32
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel,GetModelState
import time
import os
try:
    import cv2
except ImportError:
    cv2 = None


SCORE_OUTPUT_PATH = os.environ.get('SCORE_OUTPUT_PATH',
                                    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'score.txt'))


uav_type = (sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-')
            else 'typhoon_h480')
actor_num = 6
uav_num = 6
err_threshold = 1
coordx_bias = 3
coordy_bias = 9
actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}

MAX_UAV_ALTITUDE = 6.0
DETECTION_INTERVAL = 1.0
DETECTION_DURATION = 15.0
MISSION_TIMEOUT = 600.0
DEFAULT_UAV_LOSS_PENALTY = 100.0
DEFAULT_COLLISION_PENALTY = 30.0
UAV_MISS_LIMIT = 3
# 2026-09-26：对齐官方 2026 规则。
#   起飞飞机数×10 / 发现目标数×50 / 消除目标数×100 / 跟踪目标数×80
#   传感器费用×0.003 / 无人机碰撞次数×30
# 官方没有"无人机失联扣分"，保留 user 的自定义项 uav_loss_count × penalty。
TRACKING_DURATION = 20.0
TRACKING_DISTANCE = 10.0
uav_positions = [None] * uav_num
tracking_start_time = [None] * actor_num
tracked_actors = set()



left_actors = []
find_actors = []
actors_pos = [None] * actor_num
count_flag = [False] * actor_num
topic_arrive_time = [0.0] * actor_num
find_time = [0.0] * actor_num
detection_report_count = [0] * actor_num
detection_max_gap = [0.0] * actor_num
detection_max_error = [0.0] * actor_num


find_actor_pub = []
mission_finished = False
mission_completed = False        # 2026-09-26：跟踪"是否进入官方 2580 - 用时 公式"
uav_loss_count = 0
uav_loss_penalty = DEFAULT_UAV_LOSS_PENALTY
collision_count = 0               # 2026-09-26：累计碰撞次数（按官方规则扣 30/次）
collision_penalty = DEFAULT_COLLISION_PENALTY
# 2026-09-26：碰撞启发式检测参数。速度<0.1 m/s 持续 1 s + 高度在最近 1 s 内掉 ≥ 0.5 m
# 才计为一次碰撞（避免起飞/贴地常态被误判）。
COLLISION_LOW_V_THRESHOLD = 0.1
COLLISION_HEIGHT_DROP_THRESHOLD = 0.5
COLLISION_LOW_V_DWELL_SEC = 1.0
collision_low_v_since_ = [None] * uav_num
collision_last_height_ = [None] * uav_num
uav_took_off = [False] * uav_num    # 2026-09-26：起飞飞机数×10，每架只记一次
sensor_cost = 0.0
target_finish = 0
find_finish = 0
score = 0.0
time_usage = 0.0
start_time = 0.0



def _now():
    return rospy.get_time()


def _publish_score():
    if 'score_pub' not in globals():
        return
    value = int(round(max(-32768, min(32767, score))))
    score_pub.publish(value)
    _dump_score_file(value)


def _dump_score_file(final_value=None):
    try:
        value = final_value if final_value is not None else int(round(max(-32768, min(32767, score))))
    except Exception:
        return
    try:
        with open(SCORE_OUTPUT_PATH, 'w', encoding='utf-8') as fh:
            fh.write(str(value) + '\n')
    except Exception as exc:
        print('score dump failed: %s' % exc)


atexit.register(lambda: _dump_score_file())


def _progress_score():
    track_finish = len(tracked_actors)
    # 2026-09-26：对齐官方 2026 规则。
    #   未完成：起飞飞机数×10 + 发现目标数×50 + 消除目标数×100 + 跟踪目标数×80
    #           − 传感器费用×0.003 − 无人机碰撞次数×30
    #   完成（进入 mission_completed）：用 2580 − 用时 公式
    #   user 保留自定义 uav_loss_count（官方无失联扣分项，但 user 决定保留）
    take_off_count = sum(1 for x in uav_took_off if x)
    if mission_completed:
        elapsed = max(0.0, _now() - start_time)
        return (2580.0 - elapsed
                - sensor_cost * 3e-3
                - collision_count * collision_penalty
                - uav_loss_count * uav_loss_penalty)
    return (take_off_count * 10
            + find_finish * 50
            + track_finish * 80
            + target_finish * 100
            - sensor_cost * 3e-3
            - collision_count * collision_penalty
            - uav_loss_count * uav_loss_penalty)



def _finish(reason, final_score=None):
    global score, mission_finished
    if mission_finished:
        return
    mission_finished = True
    if final_score is not None:
        score = final_score
    print(reason)
    print('score:', score)
    _publish_score()
    signal_shutdown = getattr(rospy, 'signal_shutdown', None)
    if signal_shutdown:
        signal_shutdown(reason)


def _reset_detection(actor_id):
    count_flag[actor_id] = False
    find_time[actor_id] = 0.0
    topic_arrive_time[actor_id] = 0.0
    detection_report_count[actor_id] = 0
    detection_max_gap[actor_id] = 0.0
    detection_max_error[actor_id] = 0.0


def _audit_report(msg, actor_id, now, position, error, gap, elapsed, decision, reason):
    header = getattr(msg, '_connection_header', None) or {}
    topic = header.get('topic', 'unknown')
    ref_x = position.x if position is not None else float('nan')
    ref_y = position.y if position is not None else float('nan')
    print('[detection_audit] event=report wall=%.6f sim=%.6f topic=%s actor=%s '
          'cls=%s reported=(%.3f,%.3f) reference=(%.3f,%.3f) error=%.3f '
          'gap=%.3f elapsed=%.3f decision=%s reason=%s' %
          (time.time(), now, topic, actor_id, getattr(msg, 'cls', ''),
           msg.x, msg.y, ref_x, ref_y, error, gap, elapsed, decision, reason),
          flush=True)


def _audit_deletion(actor_id, now, result):
    print('[detection_audit] event=deletion wall=%.6f sim=%.6f actor=%d '
          'result=%s start=%.6f end=%.6f duration=%.3f reports=%d '
          'max_gap=%.3f max_error=%.3f required_duration=%.1f '
          'allowed_gap=%.1f allowed_error=%.1f' %
          (time.time(), now, actor_id, result, find_time[actor_id], now,
           now - find_time[actor_id], detection_report_count[actor_id],
           detection_max_gap[actor_id], detection_max_error[actor_id],
           DETECTION_DURATION, DETECTION_INTERVAL, err_threshold), flush=True)


def _delete_actor(actor_id):
    global target_finish, score
    try:
        response = del_model('actor_' + str(actor_id))
        if hasattr(response, 'success') and not response.success:
            print('Unable to remove actor_%d: %s' % (actor_id, getattr(response, 'status_message', 'unknown error')))
            return False
    except Exception as exc:
        print('Unable to remove actor_%d: %s' % (actor_id, exc))
        return False
    if actor_id in left_actors:
        left_actors.remove(actor_id)
    target_finish = actor_num - len(left_actors)
    score = _progress_score()
    return True


def _process_actor_detection(msg, actor_ids):
    global find_finish, score, time_usage, mission_completed
    now = _now()
    if mission_finished or not getattr(msg, 'cls', None) or not actor_ids:
        reason = ('mission_finished' if mission_finished else
                  'missing_class' if not getattr(msg, 'cls', None) else 'unknown_class')
        _audit_report(msg, 'none', now, None, float('nan'), float('nan'),
                      0.0, 'ignored', reason)
        return
    for actor_id in actor_ids:
        if actor_id not in left_actors:
            _audit_report(msg, actor_id, now, None, float('nan'), float('nan'),
                          0.0, 'ignored', 'already_deleted')
            continue
        position = actors_pos[actor_id]
        previous = topic_arrive_time[actor_id]
        topic_arrive_time[actor_id] = now
        gap = now - previous if previous != 0.0 else float('nan')
        elapsed = now - find_time[actor_id] if count_flag[actor_id] else 0.0
        if position is None:
            _audit_report(msg, actor_id, now, None, float('nan'), gap,
                          elapsed, 'reset', 'position_unavailable')
            _reset_detection(actor_id)
            continue
        distance_sq = ((msg.x - position.x) ** 2 + (msg.y - position.y) ** 2)
        error = math.sqrt(distance_sq)
        continuous = previous == 0.0 or now - previous <= DETECTION_INTERVAL
        if distance_sq >= err_threshold ** 2 or not continuous:
            reason = ('coordinate_error+report_gap' if distance_sq >= err_threshold ** 2 and not continuous
                      else 'coordinate_error' if distance_sq >= err_threshold ** 2
                      else 'report_gap')
            _audit_report(msg, actor_id, now, position, error, gap,
                          elapsed, 'reset', reason)
            _reset_detection(actor_id)
            continue
        if not count_flag[actor_id]:
            count_flag[actor_id] = True
            find_time[actor_id] = now
            detection_report_count[actor_id] = 1
            detection_max_error[actor_id] = error
            _audit_report(msg, actor_id, now, position, error, gap,
                          0.0, 'accepted', 'timer_started')
            if actor_id in find_actors:
                find_actors.remove(actor_id)
            find_finish = actor_num - len(find_actors)
            if actor_id < len(find_actor_pub):
                find_actor_pub[actor_id].publish(now)
            score = _progress_score()
            print('find actor_' + str(actor_id))
            _publish_score()
            continue
        detection_report_count[actor_id] += 1
        detection_max_gap[actor_id] = max(detection_max_gap[actor_id], gap)
        detection_max_error[actor_id] = max(detection_max_error[actor_id], error)
        _audit_report(msg, actor_id, now, position, error, gap,
                      elapsed, 'accepted', 'timer_continued')
        if now - find_time[actor_id] < DETECTION_DURATION:
            continue
        _audit_deletion(actor_id, now, 'attempt')
        if not _delete_actor(actor_id):
            _audit_deletion(actor_id, now, 'service_failed_reset')
            _reset_detection(actor_id)
            continue
        _audit_deletion(actor_id, now, 'success')
        _reset_detection(actor_id)
        print('actor_%d is OK' % actor_id)
        print('Time usage:', time_usage)
        if target_finish == actor_num:
            # 2026-09-26：官方"完成任务"公式。Mission finished 触发后，
            #   mission_completed 仍按"未完成"路径算分；要切到 2580 - 用时 必须再调一次
            #   _progress_score()。但代码不主动切——给 _finish 一个 final_score=0 让
            #   mission_completed 状态由本轮 _progress_score 触发即可。
            # 简化做法：直接算 final_score 由 _finish 触发 _progress_score。
            mission_completed = True
            score = _progress_score()  # 走"完成"分支 = 2580 - 用时 - sensor*0.003 - 碰撞*30 - uav_loss*100
            _finish('Mission finished', score)
        else:
            print('score:', score)
            _publish_score()


def actor_info_callback(msg):
    ids = actor_id_dict.get(getattr(msg, 'cls', ''), [])
    _process_actor_detection(msg, ids)

def actor_info1_callback(msg):
    _process_actor_detection(msg, [5] if getattr(msg, 'cls', '') == 'red' else [])

def actor_info2_callback(msg):
    _process_actor_detection(msg, [4] if getattr(msg, 'cls', '') == 'red' else [])
    
def _update_tracking(now):
    global score

    for actor_id in list(left_actors):

        actor_pos = actors_pos[actor_id]

        if actor_pos is None:
            tracking_start_time[actor_id] = None
            continue

        in_tracking_range = False

        for uav_id in range(uav_num):

            if uav_lost[uav_id]:
                continue

            uav_pos = uav_positions[uav_id]

            if uav_pos is None:
                continue

            dx = uav_pos.x - actor_pos.x
            dy = uav_pos.y - actor_pos.y

            distance_sq = dx * dx + dy * dy

            if distance_sq <= TRACKING_DISTANCE ** 2:
                in_tracking_range = True
                break

        # 当前至少有一架无人机在跟踪范围内
        if in_tracking_range:

            # 本轮连续跟踪开始
            if tracking_start_time[actor_id] is None:
                tracking_start_time[actor_id] = now

            # 连续跟踪达到20秒
            elif (
                now - tracking_start_time[actor_id]
                >= TRACKING_DURATION
            ):

                # 同一目标跟踪分只记一次
                if actor_id not in tracked_actors:

                    tracked_actors.add(actor_id)

                    print(
                        'actor_%d tracking success' % actor_id
                    )

                    score = _progress_score()
                    _publish_score()

        else:
            # 本轮连续跟踪失败，重新计时
            # 但不会取消已经获得的跟踪分
            tracking_start_time[actor_id] = None




if __name__ == "__main__":
    left_actors = list(range(actor_num))
    find_actors = list(range(actor_num))
    actors_pos = [None] * actor_num
    count_flag = [False] * actor_num
    topic_arrive_time = [0.0] * actor_num
    detection_report_count = [0] * actor_num
    detection_max_gap = [0.0] * actor_num
    detection_max_error = [0.0] * actor_num
    tracking_start_time = [None] * actor_num
    tracked_actors = set()

    find_time = [0.0] * actor_num
    find_actor_pub = []
    mission_finished = False
    uav_loss_count = 0
    uav_loss_penalty = DEFAULT_UAV_LOSS_PENALTY
    collision_count = 0
    collision_penalty = DEFAULT_COLLISION_PENALTY
    uav_collision_quota = 5
    collision_count_per_uav_ = [0] * uav_num
    collision_low_v_since_ = [None] * uav_num
    collision_last_height_ = [None] * uav_num
    uav_took_off = [False] * uav_num
    uav_seen = [False] * uav_num
    uav_lost = [False] * uav_num
    uav_miss_count = [0] * uav_num
    rospy.init_node('score_cal')
    time.sleep(1)
    start_time = rospy.get_time()
    del_model = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState) 
    score_pub = rospy.Publisher("/score",Int16,queue_size=1) 
    time_usage_pub = rospy.Publisher("/time_usage",Int16,queue_size=1) 
    left_actors_pub = rospy.Publisher("/left_actors",String,queue_size=1)
    actor_blue_sub = rospy.Subscriber("/actor_blue_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_green_sub = rospy.Subscriber("/actor_green_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_white_sub = rospy.Subscriber("/actor_white_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_brown_sub = rospy.Subscriber("/actor_brown_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_red1_sub = rospy.Subscriber("/actor_red1_info",ActorInfo,actor_info1_callback,queue_size=1)
    actor_red2_sub = rospy.Subscriber("/actor_red2_info",ActorInfo,actor_info2_callback,queue_size=1)
    for i in range(actor_num):
        find_actor_pub.append(rospy.Publisher("/find_actor_%d"%i, Float32, queue_size=5))


    mono_cam = int(rospy.get_param('~mono_cam', 1))
    stereo_cam = int(rospy.get_param('~stereo_cam', 0))
    laser1d = int(rospy.get_param('~laser1d', 0))
    laser2d = int(rospy.get_param('~laser2d', 1))
    laser3d = int(rospy.get_param('~laser3d', 0))
    gimbal = int(rospy.get_param('~gimbal', 1))
    bino_cam = int(rospy.get_param('~bino_cam', 0))
    sensor_cost = (mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2
                   + laser2d * 1e3 + laser3d * 2e3 + gimbal * 2e2
                   + bino_cam * 1e3)


    uav_loss_penalty = float(rospy.get_param('~uav_loss_penalty', DEFAULT_UAV_LOSS_PENALTY))
    collision_penalty = float(rospy.get_param('~collision_penalty', DEFAULT_COLLISION_PENALTY))
    uav_collision_quota = int(rospy.get_param('~uav_collision_quota', 5))
    collision_count_per_uav_ = [0] * uav_num
    collision_low_v_since_ = [None] * uav_num
    collision_last_height_ = [None] * uav_num
    uav_took_off = [False] * uav_num
    target_finish = 0
    find_finish = 0
    score = _progress_score()
    time_usage = 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for i in range(uav_num):
            try:
                response = get_model_state(uav_type + '_' + str(i), 'ground_plane')
                success = getattr(response, 'success', False)
                if success:
                    uav_pos_tmp = response.pose.position

                    # 保存该无人机当前位置
                    uav_positions[i] = uav_pos_tmp

                    uav_seen[i] = True
                    uav_miss_count[i] = 0

                    z_val = uav_pos_tmp.z

                    # 官方规则：起飞飞机数 × 10；每架只记一次。
                    if (not uav_took_off[i] and math.isfinite(z_val) and z_val > 0.5):
                        uav_took_off[i] = True
                        score = _progress_score()
                        _publish_score()
                        print('UAV %d took off at z=%.2f; +10' % (i, z_val))

                    # 官方规则：碰撞次数 × 30。没有碰撞 topic 时使用保守启发式：
                    # 水平速度 <0.1 m/s 持续1s，且高度下降 >=0.5m；每架最多5次。
                    twist = getattr(response, 'twist', None)
                    if (twist is not None and math.isfinite(z_val)
                            and math.isfinite(twist.linear.x)
                            and math.isfinite(twist.linear.y)):
                        v_now = math.hypot(twist.linear.x, twist.linear.y)
                        now_wall = time.time()
                        if v_now < COLLISION_LOW_V_THRESHOLD:
                            if collision_low_v_since_[i] is None:
                                collision_low_v_since_[i] = now_wall
                                collision_last_height_[i] = z_val
                            elif (now_wall - collision_low_v_since_[i] >= COLLISION_LOW_V_DWELL_SEC
                                  and collision_last_height_[i] - z_val >= COLLISION_HEIGHT_DROP_THRESHOLD
                                  and collision_count_per_uav_[i] < uav_collision_quota):
                                height_drop = collision_last_height_[i] - z_val
                                collision_count += 1
                                collision_count_per_uav_[i] += 1
                                collision_low_v_since_[i] = None
                                collision_last_height_[i] = None
                                score = _progress_score()
                                _publish_score()
                                print('UAV %d collision: height_drop=%.2f; total=%d'
                                      % (i, height_drop, collision_count))
                        else:
                            collision_low_v_since_[i] = None
                            collision_last_height_[i] = None
                    else:
                        collision_low_v_since_[i] = None
                        collision_last_height_[i] = None

                    if math.isnan(z_val) or math.isinf(z_val):
                        continue
                    if z_val > MAX_UAV_ALTITUDE:
                        score = 0
                        _finish(
                            'Warning: UAV %d is higher than 6 meters' % i,
                            0
                        )
                        break
                elif uav_seen[i] and not uav_lost[i]:
                    uav_miss_count[i] += 1
                    if uav_miss_count[i] >= UAV_MISS_LIMIT:
                        uav_lost[i] = True
                        uav_loss_count += 1
                        uav_positions[i] = None
                        score = _progress_score()
                        print('UAV %d lost; penalty %.1f' % (i, uav_loss_penalty))
            except Exception as exc:
                if uav_seen[i] and not uav_lost[i]:
                    uav_miss_count[i] += 1
                    if uav_miss_count[i] >= UAV_MISS_LIMIT:
                        uav_lost[i] = True
                        uav_loss_count += 1
                        score = _progress_score()
                        print('UAV %d lost (%s); penalty %.1f' % (i, exc, uav_loss_penalty))
        if mission_finished:
            break
        for i in list(left_actors):
            try:
                response = get_model_state('actor_' + str(i), 'ground_plane')
                if not getattr(response, 'success', False):
                    continue
                actors_pos_tmp = response.pose.position
                if actors_pos_tmp.x ** 2 + actors_pos_tmp.y ** 2 != 0:
                    actors_pos[i] = actors_pos_tmp
            except Exception:
                continue
        # 更新本轮时间
        now = rospy.get_time()
        time_usage = now - start_time

        # 根据 UAV 和目标真实位置进行连续跟踪判定
        _update_tracking(now)

        if time_usage > MISSION_TIMEOUT:
            _finish('Time out, mission failed', score)
            break

        score = _progress_score()
        _publish_score()
        time_usage_pub.publish(int(time_usage))
        left_actors_pub.publish(str(left_actors))
        if cv2 is not None and (os.name == 'nt' or os.environ.get('DISPLAY')):
            try:
                background = cv2.imread(os.path.join(os.path.dirname(__file__), "white_background.png"))
                if background is not None:
                    cv2.putText(background,"Score: "+str(score) ,(60,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.putText(background,"Time usage: "+str(time_usage) ,(360,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.putText(background,"Left targets: "+str(left_actors) ,(760,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.imshow("Multi-UAV search simulation competition judgment system",background)
                    cv2.waitKey(1)
            except Exception:
                pass
        rate.sleep()
