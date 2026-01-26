#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# --------------------------
# Globals
# --------------------------
current_state = None
current_yaw = 0.0
current_altitude = 0.0
current_drone_x = 0.0
current_drone_y = 0.0

latest_safe_cmd = Twist()

# Summit data
summit_pose_x = 0.0
summit_pose_y = 0.0
last_time = None
last_x = 0.0
last_y = 0.0
summit_speed = 0.0

# Goal
goal_x = 0.0
goal_y = 0.0
goal_received = False
GOAL_THRESHOLD = 0.5

# Controller states
STATE_INIT = 0
STATE_TAKEOFF = 1
STATE_HOLD = 2
STATE_SINUSOID = 3
controller_state = STATE_INIT
state_start_time = time.time()

# Controller parameters
target_alt = 3.0
Kp_alt = 1.2
A = 0.5
omega = 0.6
forward_speed = 1.0
HOLD_OFFSET = 1.0
SUMMIT_SPEED_THRESHOLD = 0.05
rate_hz = 20
max_vz = 1.8

# --------------------------
# Callbacks
# --------------------------
def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_yaw, current_altitude, current_drone_x, current_drone_y
    current_drone_x = msg.pose.pose.position.x
    current_drone_y = msg.pose.pose.position.y
    current_altitude = msg.pose.pose.position.z

    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)

def summit_odom_cb(msg):
    global summit_pose_x, summit_pose_y
    summit_pose_x = msg.pose.pose.position.x
    summit_pose_y = msg.pose.pose.position.y

def nav_goal_cb(msg):
    global goal_x, goal_y, goal_received
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    goal_received = True

def vel_cb(msg):
    global latest_safe_cmd
    latest_safe_cmd = msg

# --------------------------
# Helper functions
# --------------------------
def change_state(new_state):
    global controller_state, state_start_time
    controller_state = new_state
    state_start_time = time.time()

def update_summit_speed():
    global last_time, last_x, last_y, summit_speed
    current_time = rospy.Time.now().to_sec()
    if last_time is None:
        last_time = current_time
        last_x = summit_pose_x
        last_y = summit_pose_y
        summit_speed = 0.0
        return

    dt = current_time - last_time
    if dt <= 1e-6:
        return

    dx = summit_pose_x - last_x
    dy = summit_pose_y - last_y
    summit_speed = math.sqrt(dx**2 + dy**2) / dt

    last_time = current_time
    last_x = summit_pose_x
    last_y = summit_pose_y

def distance(a_x, a_y, b_x, b_y):
    return math.sqrt((a_x - b_x)**2 + (a_y - b_y)**2)

def auto_state_switch():
    global controller_state

    dist_to_goal = None
    if goal_received:
        dist_to_goal = distance(summit_pose_x, summit_pose_y, goal_x, goal_y)

    if summit_speed < SUMMIT_SPEED_THRESHOLD or (dist_to_goal is not None and dist_to_goal < GOAL_THRESHOLD):
        if controller_state != STATE_HOLD and abs(target_alt - current_altitude) < 0.15:
            rospy.loginfo("Summit stopped or near goal -> HOLD")
            change_state(STATE_HOLD)
    else:
        if goal_received and controller_state != STATE_SINUSOID:
            rospy.loginfo("Summit moving towards goal -> SINUSOID")
            change_state(STATE_SINUSOID)


def publish_to_cbf(vx_body, vy_body, vz_body, yaw_rate):
    """
    Publish velocity command to CBF filter.
    This is the DESIRED velocity that CBF will constrain.
    """
    vel_msg = Twist()
    vel_msg.linear.x = vx_body
    vel_msg.linear.y = vy_body
    vel_msg.linear.z = vz_body
    vel_msg.angular.z = yaw_rate
    drone_vel_pub.publish(vel_msg)


def publish_to_mavros():
    """
    Publish CBF-filtered velocity directly to MAVROS.
    This uses velocity feedthrough - no position-based computation lag.
    """
    # Transform from body to world frame
    vx_body = latest_safe_cmd.linear.x
    vy_body = latest_safe_cmd.linear.y
    vz_body = latest_safe_cmd.linear.z
    yaw_rate = latest_safe_cmd.angular.z

    vx_world = vx_body * math.cos(current_yaw) - vy_body * math.sin(current_yaw)
    vy_world = vx_body * math.sin(current_yaw) + vy_body * math.cos(current_yaw)

    vel_msg = TwistStamped()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.twist.linear.x = vx_world
    vel_msg.twist.linear.y = vy_world
    vel_msg.twist.linear.z = vz_body
    vel_msg.twist.angular.z = yaw_rate

    final_vel.publish(vel_msg)


def compute_takeoff_velocity():
    """Compute velocity for takeoff phase - only vertical control"""
    if current_altitude < 2.0:
        vz = 1.85
    else:
        vz = max(min(Kp_alt * (target_alt - current_altitude), max_vz), -max_vz)
    
    return 0.0, 0.0, vz, 0.0


def compute_hold_velocity_desired():
    """
    Compute DESIRED velocity for HOLD state.
    This gets sent to CBF to be constrained, then comes back filtered.
    """
    # Vector from summit to goal
    vec_x = goal_x - summit_pose_x if goal_received else 1.0
    vec_y = goal_y - summit_pose_y if goal_received else 0.0
    norm = math.sqrt(vec_x**2 + vec_y**2)
    if norm < 1e-3:
        norm = 1e-3
    dir_x = vec_x / norm
    dir_y = vec_y / norm

    # Right-hand perpendicular (offset to the right)
    perp_x = dir_y
    perp_y = -dir_x

    # Target position = right of Summit
    target_x = summit_pose_x + HOLD_OFFSET * perp_x
    target_y = summit_pose_y + HOLD_OFFSET * perp_y

    # Compute position error
    dx_err = target_x - current_drone_x
    dy_err = target_y - current_drone_y

    # Convert position error to velocity command
    # Scale to reasonable velocity range
    Kp_xy = 1.0
    vx = Kp_xy * dx_err
    vy = Kp_xy * dy_err
    vz = Kp_alt * (target_alt - current_altitude)

    return vx, vy, vz, 0.0


def compute_sinusoid_velocity_desired(t):
    """
    Compute DESIRED velocity for SINUSOID state.
    This gets sent to CBF to be constrained, then comes back filtered.
    """
    # Vector from summit to goal
    vec_x = goal_x - summit_pose_x
    vec_y = goal_y - summit_pose_y
    norm = math.sqrt(vec_x**2 + vec_y**2)
    if norm < 1e-3:
        norm = 1e-3
    dir_x = vec_x / norm
    dir_y = vec_y / norm

    # Perpendicular for sinusoidal motion
    perp_x = -dir_y
    perp_y = dir_x
    sine = A * math.sin(omega * t)

    vx = forward_speed * dir_x + sine * perp_x
    vy = forward_speed * dir_y + sine * perp_y
    vz = Kp_alt * (target_alt - current_altitude)

    return vx, vy, vz, 0.0

# --------------------------
# Main
# --------------------------
if __name__ == "__main__":
    rospy.init_node("offb_controller")

    last_time = rospy.Time.now().to_sec()

    # Subscribers
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/odom", Odometry, pose_cb)
    rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, summit_odom_cb)
    rospy.Subscriber("/robot/move_base_simple/goal", PoseStamped, nav_goal_cb)
    rospy.Subscriber("incoming/cmd_vel", Twist, vel_cb)

    # Publishers
    drone_vel_pub = rospy.Publisher("desired/cmd_vel", Twist, queue_size=10)
    drone_vel_pub_start = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    final_vel = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    # Services
    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Warm up
    rate = rospy.Rate(rate_hz)
    zero_vel = TwistStamped()
    for _ in range(50):
        drone_vel_pub_start.publish(zero_vel)
        rate.sleep()

    offboard_enabled = False
    armed = False
    last_req = rospy.Time.now()
    change_state(STATE_TAKEOFF)
    prev_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        dt = now - prev_time
        prev_time = now

        # OFFBOARD + ARM
        if not offboard_enabled and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            try:
                resp = set_mode_client(custom_mode="OFFBOARD")
                if getattr(resp, 'mode_sent', False):
                    rospy.loginfo("OFFBOARD enabled")
                    offboard_enabled = True
            except:
                pass
            last_req = rospy.Time.now()

        if not armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            try:
                resp = arming_client(True)
                if getattr(resp, 'success', False):
                    rospy.loginfo("Vehicle armed")
                    armed = True
            except:
                pass
            last_req = rospy.Time.now()

        # Summit tracking
        update_summit_speed()
        auto_state_switch()

        # State machine - compute desired velocity and send to CBF
        if controller_state == STATE_TAKEOFF:
            vx, vy, vz, yaw_rate = compute_takeoff_velocity()
            publish_to_cbf(vx, vy, vz, yaw_rate)
            # During takeoff, publish directly to MAVROS (bypass CBF for takeoff)
            vel_msg = TwistStamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = vz
            vel_msg.twist.angular.z = 0.0
            drone_vel_pub_start.publish(vel_msg)
            
            if abs(target_alt - current_altitude) < 0.15:
                rospy.loginfo("Reached altitude -> HOLD")
                change_state(STATE_HOLD)

        elif controller_state == STATE_HOLD:
            # Compute desired velocity for HOLD state
            vx, vy, vz, yaw_rate = compute_hold_velocity_desired()
            # Send to CBF for filtering
            publish_to_cbf(vx, vy, vz, yaw_rate)
            # Publish CBF-filtered velocity to MAVROS (velocity feedthrough, no lag)
            publish_to_mavros()

        elif controller_state == STATE_SINUSOID:
            # Compute desired velocity for SINUSOID state
            t = time.time() - state_start_time
            vx, vy, vz, yaw_rate = compute_sinusoid_velocity_desired(t)
            # Send to CBF for filtering
            publish_to_cbf(vx, vy, vz, yaw_rate)
            # Publish CBF-filtered velocity to MAVROS (velocity feedthrough, no lag)
            publish_to_mavros()

        rate.sleep()