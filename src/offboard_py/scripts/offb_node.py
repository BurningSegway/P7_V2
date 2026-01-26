#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg
    print(msg)


def pose_cb(msg):
    global current_yaw
    # Extract yaw from quaternion
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)

latest_body_cmd = Twist()  # store latest teleop command

def vel_pub(msg):
    global latest_body_cmd
    latest_body_cmd = msg  # update the latest body-frame command

def publish_velocity():
    global latest_body_cmd, current_yaw
    vx_body = latest_body_cmd.linear.x
    vy_body = latest_body_cmd.linear.y
    vz_body = latest_body_cmd.linear.z
    yaw_rate = latest_body_cmd.angular.z

    # Rotate to world frame
    vx_world = vx_body * math.cos(current_yaw) - vy_body * math.sin(current_yaw)
    vy_world = vx_body * math.sin(current_yaw) + vy_body * math.cos(current_yaw)

    vel_msg = TwistStamped()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.header.frame_id = "robot_map"  # world frame

    vel_msg.twist.linear.x = vx_world
    vel_msg.twist.linear.y = vy_world
    vel_msg.twist.linear.z = vz_body
    vel_msg.twist.angular.z = yaw_rate

    drone_vel_pub.publish(vel_msg)



if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    vel_sub = rospy.Subscriber("incoming/cmd_vel", Twist, callback = vel_pub)   #reads from teleop node

    pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pose_cb)   #reads pose of drone from mavros

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)  #publishes position setpoints to mavros for the drone to follow

    drone_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)  #follows velocity commands in world frame

    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    j = 1

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        while j == 1:
            for i in range(100):
                if(rospy.is_shutdown()):
                    break

                local_pos_pub.publish(pose)
                rate.sleep()
            rospy.loginfo("Drone in the air")    
            j = 0

        #local_pos_pub.publish(pose)
        publish_velocity()
        rate.sleep()