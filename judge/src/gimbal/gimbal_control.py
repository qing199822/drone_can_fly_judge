import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from gazebo_msgs.srv import GetLinkState
import sys
import std_msgs.msg

if __name__ == "__main__":
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('gimbal_control'+'_'+vehicle_type+'_'+vehicle_id)
    mountCnt = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/mavros/mount_control/command', MountControl, queue_size=1)
    mountConfig = rospy.ServiceProxy(vehicle_type+'_'+vehicle_id+'/mavros/mount_control/configure', MountConfigure)
    rate=rospy.Rate(30)  # 云台控制指令发布频率（Hz）
    # 相机俯视角度已由 SDF 中深度相机的静态 pose 提供（pitch=+0.5236 rad ≈ +30°），
    # 云台 pitch 这里保持 0°，避免两者叠加成 60°（相机俯角过大、看不到前方人物）。
    gimbal_pitch_ = 0.0  # 云台俯仰角置 0：相机俯角已由 SDF 深度相机 pose 提供，避免叠加成 60°
    gimbal_yaw_ = 0.0  # 云台偏航角置 0：不左右转动
    gimbal_roll_ = 0.0  # 云台滚转角置 0：保持水平
    srvheader=std_msgs.msg.Header()
    srvheader.stamp=rospy.Time.now()
    srvheader.frame_id="map"
    cam_pose_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cam_pose', PoseStamped, queue_size=1)
    cam_pose = PoseStamped()
    gazeboLinkstate = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

    try:
        mountConfig(header=srvheader, mode=2, stabilize_roll=0,  # mode=2: 角度控制模式；stabilize_* 为增稳开关，0=关闭
                    stabilize_yaw=0, stabilize_pitch=0)
    except rospy.ServiceException as error:
        rospy.logwarn("Mount configure failed; continuing with command publishing: %s", error)
    print(vehicle_type+'_'+vehicle_id+': Gimbal control')
    while not rospy.is_shutdown():
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2  # 云台控制模式：2=角度控制模式
        msg.pitch = gimbal_pitch_  # 期望云台俯仰角（rad）
        msg.roll = gimbal_roll_  # 期望云台滚转角（rad）
        msg.yaw = gimbal_yaw_  # 期望云台偏航角（rad）
        mountCnt.publish(msg)
        try:
            response = gazeboLinkstate(vehicle_type+'_'+vehicle_id+'::cgo3_camera_link','ground_plane::link')
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5.0, "Gazebo link state service call failed: %s", error)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            continue
        cam_pose.header.stamp = rospy.Time.now()
        cam_pose.pose = response.link_state.pose
        cam_pose_pub.publish(cam_pose)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
