#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()
current_alt = 0.0

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_alt
    current_alt = msg.pose.position.z

def sinusoid_velocity(t, forward_speed, amplitude, frequency, current_alt, target_alt, Kp=0.5, max_vz=0.5):
    cmd = TwistStamped()
    cmd.header.stamp = rospy.Time.now()
    cmd.twist.linear.x = forward_speed
    cmd.twist.linear.y = amplitude * math.sin(2.0 * math.pi * frequency * t)
    vz = Kp * (target_alt - current_alt)
    vz = max(min(vz, max_vz), -max_vz)
    cmd.twist.linear.z = vz
    cmd.twist.angular.z = 0.0
    return cmd

TAKEOFF = 0
CONTROLLER = 1

if __name__ == "__main__":
    rospy.init_node("offboard_velocity_controller")

    forward_speed = rospy.get_param("~forward_speed", 0.5)
    amplitude = rospy.get_param("~amplitude", 0.6)
    frequency = rospy.get_param("~frequency", 0.25)
    target_alt = rospy.get_param("~target_alt", 2.0)
    altitude_kp = rospy.get_param("~altitude_kp", 0.5)
    max_vz = rospy.get_param("~max_vz", 0.5)
    rate_hz = rospy.get_param("~rate", 20)

    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", TwistStamped, pose_cb)
    vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    rospy.wait_for_service("mavros/cmd/arming")
    arm_srv = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    mode_srv = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    zero_vel = TwistStamped()
    for _ in range(50):
        vel_pub.publish(zero_vel)
        rate.sleep()

    last_req = rospy.Time.now()
    start_time = rospy.Time.now().to_sec()
    offboard_enabled = False
    armed = False
    state = TAKEOFF

    while not rospy.is_shutdown():

        if not offboard_enabled and (rospy.Time.now() - last_req > rospy.Duration(1.0)):
            try:
                res = mode_srv(0, "OFFBOARD")
                if res.mode_sent:
                    offboard_enabled = True
            except rospy.ServiceException:
                pass
            last_req = rospy.Time.now()

        if not armed and (rospy.Time.now() - last_req > rospy.Duration(1.0)):
            try:
                res = arm_srv(True)
                if res.success:
                    armed = True
            except rospy.ServiceException:
                pass
            last_req = rospy.Time.now()

        t = rospy.Time.now().to_sec() - start_time

        if state == TAKEOFF:
            vz = altitude_kp * (target_alt - current_alt)
            vz = max(min(vz, max_vz), -max_vz)
            takeoff_cmd = TwistStamped()
            takeoff_cmd.header.stamp = rospy.Time.now()
            takeoff_cmd.twist.linear.x = 0.0
            takeoff_cmd.twist.linear.y = 0.0
            takeoff_cmd.twist.linear.z = vz
            takeoff_cmd.twist.angular.z = 0.0
            vel_pub.publish(takeoff_cmd)
            if current_alt >= target_alt - 0.05:
                state = CONTROLLER
                start_time = rospy.Time.now().to_sec()

        elif state == CONTROLLER:
            vel_msg = sinusoid_velocity(t, forward_speed, amplitude, frequency,
                                        current_alt, target_alt, altitude_kp, max_vz)
            vel_pub.publish(vel_msg)

        rate.sleep()
