#!/usr/bin/env python3

import threading

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String


class XTDroneMavrosBridge:
    MODE_COMMANDS = {
        "OFFBOARD": "OFFBOARD",
        "LAND": "AUTO.LAND",
        "RTL": "AUTO.RTL",
        "HOVER": "OFFBOARD",
    }

    def __init__(self):
        self.vehicle = rospy.get_param("~vehicle", "typhoon_h480_0")
        self.setpoint_rate = float(rospy.get_param("~setpoint_rate", 30.0))
        self.velocity_timeout = rospy.Duration(
            float(rospy.get_param("~velocity_timeout", 0.5))
        )
        self.mavros_ns = "/{}/mavros".format(self.vehicle)
        self.xtdrone_ns = "/xtdrone/{}".format(self.vehicle)

        self.lock = threading.Lock()
        self.state = State()
        self.velocity = Twist()
        self.last_velocity = rospy.Time(0)
        self.desired_mode = None
        self.desired_armed = None
        self.last_mode_request = rospy.Time(0)
        self.last_arm_request = rospy.Time(0)

        self.setpoint_pub = rospy.Publisher(
            self.mavros_ns + "/setpoint_raw/local", PositionTarget, queue_size=10
        )
        rospy.Subscriber(
            self.mavros_ns + "/state", State, self.state_callback, queue_size=10
        )
        rospy.Subscriber(
            self.xtdrone_ns + "/cmd_vel_flu",
            Twist,
            self.velocity_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self.xtdrone_ns + "/cmd", String, self.command_callback, queue_size=10
        )

        self.arm_service = rospy.ServiceProxy(
            self.mavros_ns + "/cmd/arming", CommandBool
        )
        self.mode_service = rospy.ServiceProxy(self.mavros_ns + "/set_mode", SetMode)

        rospy.Timer(rospy.Duration(1.0 / self.setpoint_rate), self.publish_setpoint)
        rospy.Timer(rospy.Duration(0.2), self.update_flight_state)
        rospy.loginfo(
            "[%s] XTDrone compatibility bridge: %s -> %s",
            self.vehicle,
            self.xtdrone_ns,
            self.mavros_ns,
        )

    def state_callback(self, message):
        with self.lock:
            self.state = message

    def velocity_callback(self, message):
        with self.lock:
            self.velocity = message
            self.last_velocity = rospy.Time.now()

    def command_callback(self, message):
        command = message.data.strip().upper()
        with self.lock:
            if command in self.MODE_COMMANDS:
                self.desired_mode = self.MODE_COMMANDS[command]
                if command == "HOVER":
                    self.velocity = Twist()
                    self.last_velocity = rospy.Time.now()
            elif command == "ARM":
                self.desired_armed = True
            elif command == "DISARM":
                self.desired_armed = False
                self.velocity = Twist()
                self.last_velocity = rospy.Time.now()
            else:
                rospy.logwarn("[%s] Unsupported XTDrone command: %s", self.vehicle, command)
                return

        rospy.loginfo("[%s] XTDrone command received: %s", self.vehicle, command)

    def publish_setpoint(self, _event):
        now = rospy.Time.now()
        with self.lock:
            velocity = self.velocity
            is_fresh = (
                not self.last_velocity.is_zero()
                and now - self.last_velocity <= self.velocity_timeout
            )

        target = PositionTarget()
        target.header.stamp = now
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        if is_fresh:
            # setpoint_raw accepts ROS base_link/FLU values. MAVROS performs
            # the FLU-to-FRD conversion for FRAME_BODY_NED before MAVLink I/O.
            target.velocity.x = velocity.linear.x
            target.velocity.y = velocity.linear.y
            target.velocity.z = velocity.linear.z
            target.yaw_rate = velocity.angular.z
        self.setpoint_pub.publish(target)

    def update_flight_state(self, _event):
        now = rospy.Time.now()
        with self.lock:
            state = self.state
            desired_mode = self.desired_mode
            desired_armed = self.desired_armed
            last_mode_request = self.last_mode_request
            last_arm_request = self.last_arm_request

        if not state.connected:
            return

        if (
            desired_mode
            and state.mode != desired_mode
            and now - last_mode_request >= rospy.Duration(1.0)
        ):
            with self.lock:
                self.last_mode_request = now
            try:
                response = self.mode_service(base_mode=0, custom_mode=desired_mode)
                if response.mode_sent:
                    rospy.loginfo("[%s] Requested mode %s", self.vehicle, desired_mode)
                else:
                    rospy.logwarn("[%s] FCU rejected mode %s", self.vehicle, desired_mode)
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(
                    5.0, "[%s] Mode service unavailable: %s", self.vehicle, error
                )
            return

        if (
            desired_armed is not None
            and state.armed != desired_armed
            and (not desired_armed or desired_mode is None or state.mode == desired_mode)
            and now - last_arm_request >= rospy.Duration(1.0)
        ):
            with self.lock:
                self.last_arm_request = now
            try:
                response = self.arm_service(value=desired_armed)
                if response.success:
                    rospy.loginfo(
                        "[%s] Vehicle %s",
                        self.vehicle,
                        "armed" if desired_armed else "disarmed",
                    )
                else:
                    rospy.logwarn(
                        "[%s] FCU rejected %s request",
                        self.vehicle,
                        "arm" if desired_armed else "disarm",
                    )
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(
                    5.0, "[%s] Arming service unavailable: %s", self.vehicle, error
                )


if __name__ == "__main__":
    rospy.init_node("xtdrone_mavros_bridge")
    XTDroneMavrosBridge()
    rospy.spin()
