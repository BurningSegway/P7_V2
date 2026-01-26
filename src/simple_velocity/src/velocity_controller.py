#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from gazebo_msgs.msg import ModelStates

# --------------------------
# Globals
# --------------------------
current_state = None
current_yaw = 0.0
current_altitude = 0.0
current_drone_x = 0.0
current_drone_y = 0.0

latest_safe_cmd = Twist()

# Summit data (from its own odometry)
summit_pose_x = 0.0
summit_pose_y = 0.0
summit_heading = 0.0
last_time = None
last_x = 0.0
last_y = 0.0
summit_speed = 0.0

# Gazebo world frame positions
gazebo_summit_x = 0.0
gazebo_summit_y = 0.0
gazebo_drone_x = 0.0
gazebo_drone_y = 0.0
gazebo_data_received = False

# Model names - Will be set based on namespace
SUMMIT_MODEL_NAME = "robot"
DRONE_MODEL_NAME = "drone1"
DRONE_NAMESPACE = ""

SIDE = 1.0  # +1 for right, -1 for left

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
#state_start_time = rospy.Time.now()  # Initialize with ROS time

# Controller parameters
target_alt = 3.0
Kp_alt = 1.2
A = 1.3
omega = 1.0
forward_speed = 1.5
HOLD_OFFSET = 3.5
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
    """Drone's local position from MAVROS"""
    global current_yaw, current_altitude, current_drone_x, current_drone_y
    current_drone_x = msg.pose.pose.position.x
    current_drone_y = msg.pose.pose.position.y
    current_altitude = msg.pose.pose.position.z

    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)

def summit_odom_cb(msg):
    """Summit odometry - mainly for heading"""
    global summit_pose_x, summit_pose_y, summit_heading
    summit_pose_x = msg.pose.pose.position.x
    summit_pose_y = msg.pose.pose.position.y

    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    summit_heading = math.atan2(siny_cosp, cosy_cosp)

def gazebo_model_states_cb(msg):
    """
    Get true world positions from Gazebo.
    This is the ground truth in Gazebo's global frame.
    """
    global gazebo_summit_x, gazebo_summit_y
    global gazebo_drone_x, gazebo_drone_y
    global gazebo_data_received
    
    try:
        # Find summit position
        if SUMMIT_MODEL_NAME in msg.name:
            summit_idx = msg.name.index(SUMMIT_MODEL_NAME)
            gazebo_summit_x = msg.pose[summit_idx].position.x
            gazebo_summit_y = msg.pose[summit_idx].position.y
        
        # Find drone position
        if DRONE_MODEL_NAME in msg.name:
            drone_idx = msg.name.index(DRONE_MODEL_NAME)
            gazebo_drone_x = msg.pose[drone_idx].position.x
            gazebo_drone_y = msg.pose[drone_idx].position.y
        
        gazebo_data_received = True
        
    except (ValueError, IndexError) as e:
        rospy.logwarn_throttle(5.0, f"Could not find models in gazebo: {e}")

def nav_goal_cb(msg):
    global goal_x, goal_y, goal_received
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    goal_received = True
    rospy.loginfo(f"[{DRONE_NAMESPACE}] ✓ GOAL RECEIVED: ({goal_x:.2f}, {goal_y:.2f})")

def vel_cb(msg):
    global latest_safe_cmd
    latest_safe_cmd = msg

# --------------------------
# Helper functions
# --------------------------
def change_state(new_state):
    global controller_state, state_start_time
    controller_state = new_state
    state_start_time = rospy.Time.now()  # Use ROS time (respects /clock for simulation)

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
        rospy.loginfo_throttle(2.0, 
            f"[{DRONE_NAMESPACE}] Goal tracking: dist={dist_to_goal:.2f}m, "
            f"summit_speed={summit_speed:.2f}m/s, state={controller_state}")

    if summit_speed < SUMMIT_SPEED_THRESHOLD or (dist_to_goal is not None and dist_to_goal < GOAL_THRESHOLD):
        if controller_state != STATE_HOLD and abs(target_alt - current_altitude) < 0.15:
            rospy.loginfo(f"[{DRONE_NAMESPACE}] Summit stopped or near goal -> HOLD")
            change_state(STATE_HOLD)
    else:
        if goal_received and controller_state != STATE_SINUSOID:
            rospy.loginfo(f"[{DRONE_NAMESPACE}] Summit moving towards goal -> SINUSOID")
            change_state(STATE_SINUSOID)
        elif not goal_received and summit_speed >= SUMMIT_SPEED_THRESHOLD:
            rospy.logwarn_throttle(5.0, 
                f"[{DRONE_NAMESPACE}] Summit moving but NO GOAL received! "
                f"Is topic /robot/move_base_simple/goal publishing?")

def publish_to_cbf(vx_body, vy_body, vz_body, yaw_rate):
    """Publish velocity command to CBF filter."""
    vel_msg = Twist()
    vel_msg.linear.x = vx_body
    vel_msg.linear.y = vy_body
    vel_msg.linear.z = vz_body
    vel_msg.angular.z = yaw_rate
    drone_vel_pub.publish(vel_msg)

def publish_to_mavros():
    """Publish CBF-filtered velocity directly to MAVROS."""
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

def publish_velocity_direct(vx, vy, vz, yaw_rate=0.0):
    """Publish world-frame velocities directly to MAVROS."""
    vel_msg = TwistStamped()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.twist.linear.x = vx
    vel_msg.twist.linear.y = vy
    vel_msg.twist.linear.z = vz
    vel_msg.twist.angular.z = yaw_rate
    final_vel.publish(vel_msg)

def compute_takeoff_velocity():
    """Compute velocity for takeoff phase."""
    if current_altitude < 2.0:
        vz = 2.0
    else:
        vz = max(min(Kp_alt * (target_alt - current_altitude), max_vz), -max_vz)
    
    return 0.0, 0.0, vz, 0.0

# --------------------------
# GAZEBO-BASED HOLD CONTROLLER
# --------------------------
def compute_hold_velocity_gazebo():
    """
    Compute HOLD velocity using Gazebo world frame positions.
    This is the cleanest approach - everything in one consistent frame!
    
    Steps:
    1. Get Summit position in Gazebo world frame
    2. Get Summit heading (from odometry)
    3. Calculate perpendicular target position in world frame
    4. Calculate offset between Gazebo world and drone's local frame
    5. Transform target to drone's local frame
    6. Generate velocity command
    """
    
    if not gazebo_data_received:
        rospy.logwarn_throttle(1.0, "Waiting for Gazebo model states...")
        return 0.0, 0.0, Kp_alt * (target_alt - current_altitude), 0.0
    
    # STEP 1 & 2: Summit position and heading in world frame
    # gazebo_summit_x, gazebo_summit_y are already in world frame
    # summit_heading from odometry
    
    # STEP 3: Calculate perpendicular direction from Summit heading
    forward_x = math.cos(summit_heading)
    forward_y = math.sin(summit_heading)
    
    # Right perpendicular (90° clockwise from forward)
    perp_x = forward_y
    perp_y = -forward_x
    
    # Calculate target position in Gazebo world frame
    target_x_world = gazebo_summit_x + SIDE * HOLD_OFFSET * perp_x
    target_y_world = gazebo_summit_y + SIDE * HOLD_OFFSET * perp_y
    
    # STEP 4: Calculate frame offset
    # Offset = where drone thinks it is (local) vs where it actually is (gazebo)
    frame_offset_x = gazebo_drone_x - current_drone_x
    frame_offset_y = gazebo_drone_y - current_drone_y
    
    # STEP 5: Transform target from world frame to drone's local frame
    target_x_local = target_x_world - frame_offset_x
    target_y_local = target_y_world - frame_offset_y
    
    # STEP 6: Calculate position error in drone's local frame
    dx_err = target_x_local - current_drone_x
    dy_err = target_y_local - current_drone_y
    dz_err = target_alt - current_altitude
    
    # STEP 7: Proportional control
    Kp_xy = 1.5
    vx = Kp_xy * dx_err
    vy = Kp_xy * dy_err
    vz = Kp_alt * dz_err
    
    # STEP 8: Velocity limiting - reduced to 1.0 m/s max
    max_xy_speed = 1.5
    speed_xy = math.sqrt(vx**2 + vy**2)
    if speed_xy > max_xy_speed:
        scale = max_xy_speed / speed_xy
        vx *= scale
        vy *= scale
    
    # Debug output
    error = math.sqrt(dx_err**2 + dy_err**2)
    rospy.loginfo_throttle(2.0, 
        f"HOLD [Gazebo]: "
        f"Summit@({gazebo_summit_x:.2f},{gazebo_summit_y:.2f}) "
        f"Drone@({gazebo_drone_x:.2f},{gazebo_drone_y:.2f}) "
        f"Target@({target_x_world:.2f},{target_y_world:.2f}) "
        f"Error:{error:.3f}m | "
        f"Heading:{math.degrees(summit_heading):.1f}°"
    )
    
    return vx, vy, vz, 0.0

def compute_sinusoid_velocity_desired(t):
    """Compute DESIRED velocity for SINUSOID state."""
    vec_x = goal_x - summit_pose_x
    vec_y = goal_y - summit_pose_y
    norm = math.sqrt(vec_x**2 + vec_y**2)
    if norm < 1e-3:
        norm = 1e-3
    dir_x = vec_x / norm
    dir_y = vec_y / norm

    perp_x = -dir_y
    perp_y = dir_x
    phase_offset = math.pi if SIDE < 0 else 0.0
    sine = A * math.sin(omega * t + phase_offset)

    vx = forward_speed * dir_x + sine * perp_x
    vy = forward_speed * dir_y + sine * perp_y
    vz = Kp_alt * (target_alt - current_altitude)

    # Velocity limiting - cap XY speed at 1.0 m/s
    max_xy_speed = 1.5
    speed_xy = math.sqrt(vx**2 + vy**2)
    if speed_xy > max_xy_speed:
        scale = max_xy_speed / speed_xy
        vx *= scale
        vy *= scale

    return vx, vy, vz, 0.0

# --------------------------
# Namespace Configuration
# --------------------------
def configure_namespace():
    """Configure namespaces and model names based on ROS namespace."""
    global SIDE, DRONE_MODEL_NAME, DRONE_NAMESPACE
    
    namespace = rospy.get_namespace()
    DRONE_NAMESPACE = namespace.strip('/')
    
    rospy.loginfo(f"Detected namespace: {namespace}")
    
    # Extract drone number from namespace
    if 'drone1' in namespace.lower():
        SIDE = 1.0  # Right
        DRONE_MODEL_NAME = 'drone1'
        rospy.loginfo("Configured as DRONE1 (RIGHT side)")
    elif 'drone2' in namespace.lower():
        SIDE = -1.0  # Left
        DRONE_MODEL_NAME = 'drone2'
        rospy.loginfo("Configured as DRONE2 (LEFT side)")
    else:
        rospy.logwarn(f"Namespace '{namespace}' doesn't contain 'drone1' or 'drone2'")
        # Fallback to parameters
        SIDE = rospy.get_param('~side', 1.0)
        SIDE = 1.0 if float(SIDE) >= 0 else -1.0
        DRONE_MODEL_NAME = rospy.get_param('~drone_model_name', 'drone1')
        rospy.loginfo(f"Using parameter-based config: side={SIDE}, model={DRONE_MODEL_NAME}")

def get_prefixed_topic(topic_name):
    """
    Generate a topic name with namespace prefix.
    
    Examples:
        'mavros/state' -> '/drone1/mavros/state'
        'desired/cmd_vel' -> '/drone1/desired/cmd_vel'
    """
    ns = rospy.get_namespace()
    if ns == '/':
        return f"/{topic_name}"
    return f"{ns}{topic_name}"

# --------------------------
# Main
# --------------------------
if __name__ == "__main__":
    rospy.init_node("offb_controller")
    
    # Configure namespace and model names
    configure_namespace()
    
    # Get additional parameters
    SUMMIT_MODEL_NAME = rospy.get_param('~summit_model_name', 'robot')
    target_alt = rospy.get_param('~target_altitude', 3.0)
    HOLD_OFFSET = rospy.get_param('~hold_offset', 3.5)
    rate_hz = rospy.get_param('~rate_hz', 20)
    
    rospy.loginfo("="*60)
    rospy.loginfo("Gazebo-based Hold Controller Starting")
    rospy.loginfo(f"Drone namespace: {DRONE_NAMESPACE}")
    rospy.loginfo(f"Drone side: {'RIGHT' if SIDE > 0 else 'LEFT'}")
    rospy.loginfo(f"Drone model name: {DRONE_MODEL_NAME}")
    rospy.loginfo(f"Summit model name: {SUMMIT_MODEL_NAME}")
    rospy.loginfo(f"Hold offset: {HOLD_OFFSET}m")
    rospy.loginfo(f"Target altitude: {target_alt}m")
    rospy.loginfo("="*60)

    # Subscribers with proper namespace handling
    rospy.Subscriber(get_prefixed_topic("mavros/state"), State, state_cb)
    rospy.Subscriber(get_prefixed_topic("mavros/local_position/odom"), Odometry, pose_cb)
    rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, summit_odom_cb)
    rospy.Subscriber("/robot/move_base_simple/goal", PoseStamped, nav_goal_cb)  # GLOBAL - not namespaced
    rospy.Subscriber(get_prefixed_topic("incoming/cmd_vel"), Twist, vel_cb)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_states_cb)

    # Publishers with proper namespace handling
    drone_vel_pub = rospy.Publisher(get_prefixed_topic("desired/cmd_vel"), Twist, queue_size=10)
    drone_vel_pub_start = rospy.Publisher(get_prefixed_topic("mavros/setpoint_velocity/cmd_vel"), TwistStamped, queue_size=10)
    final_vel = rospy.Publisher(get_prefixed_topic("mavros/setpoint_velocity/cmd_vel"), TwistStamped, queue_size=10)

    # Services with proper namespace handling
    rospy.wait_for_service(get_prefixed_topic("mavros/cmd/arming"))
    arming_client = rospy.ServiceProxy(get_prefixed_topic("mavros/cmd/arming"), CommandBool)
    rospy.wait_for_service(get_prefixed_topic("mavros/set_mode"))
    set_mode_client = rospy.ServiceProxy(get_prefixed_topic("mavros/set_mode"), SetMode)

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

    while not rospy.is_shutdown():
        # OFFBOARD + ARM
        if not offboard_enabled and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            try:
                resp = set_mode_client(custom_mode="OFFBOARD")
                if getattr(resp, 'mode_sent', False):
                    rospy.loginfo(f"[{DRONE_NAMESPACE}] OFFBOARD enabled")
                    offboard_enabled = True
            except:
                pass
            last_req = rospy.Time.now()

        if not armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            try:
                resp = arming_client(True)
                if getattr(resp, 'success', False):
                    rospy.loginfo(f"[{DRONE_NAMESPACE}] Vehicle armed")
                    armed = True
            except:
                pass
            last_req = rospy.Time.now()

        # Summit tracking
        update_summit_speed()
        auto_state_switch()

        # State machine
        if controller_state == STATE_TAKEOFF:
            vx, vy, vz, yaw_rate = compute_takeoff_velocity()
            vel_msg = TwistStamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.twist.linear.z = vz
            drone_vel_pub_start.publish(vel_msg)
            
            if abs(target_alt - current_altitude) < 0.15:
                rospy.loginfo(f"[{DRONE_NAMESPACE}] Reached altitude -> HOLD")
                change_state(STATE_HOLD)

        elif controller_state == STATE_HOLD:
            # Use Gazebo-based controller
            vx, vy, vz, yaw_rate = compute_hold_velocity_gazebo()
            publish_to_cbf(vx, vy, vz, yaw_rate)
            publish_to_mavros()

        elif controller_state == STATE_SINUSOID:
            elapsed_time = (rospy.Time.now() - state_start_time).to_sec()  # ROS time, respects /clock
            vx, vy, vz, yaw_rate = compute_sinusoid_velocity_desired(elapsed_time)
            publish_to_cbf(vx, vy, vz, yaw_rate)
            publish_to_mavros()

        rate.sleep()