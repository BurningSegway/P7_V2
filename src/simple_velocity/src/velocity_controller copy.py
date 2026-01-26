#! /usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger, TriggerResponse

# --------------------------
# Globals
# --------------------------
current_state = State()
current_yaw = 0.0
latest_body_cmd = Twist()   # body-frame Twist
current_altitude = 0.0
heading_map = {
    'i': 0.0,
    'j': math.pi/2,
    'k': math.pi,
    'l': 3*math.pi/2
}

current_heading = 0.0  # initial heading along +X (same as 'i')
dx = math.cos(current_heading)
dy = math.sin(current_heading)



# --------------------------
# Callbacks
# --------------------------
def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_yaw, current_altitude
    # altitude
    current_altitude = msg.pose.position.z

    # yaw from quaternion (z-yaw typical)
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)

def vel_cb(msg):
    """
    Accepts geometry_msgs/Twist from teleop or local generator.
    If you want to accept TwistStamped, modify accordingly.
    """
    global latest_body_cmd
    latest_body_cmd = msg

# --------------------------
# World-frame velocity publisher
# --------------------------
def publish_velocity():
    global latest_body_cmd, current_yaw

    vx_body = latest_body_cmd.linear.x
    vy_body = latest_body_cmd.linear.y
    vz_body = latest_body_cmd.linear.z
    yaw_rate = latest_body_cmd.angular.z

    # rotate into world frame:
    vx_world = vx_body * math.cos(current_yaw) - vy_body * math.sin(current_yaw)
    vy_world = vx_body * math.sin(current_yaw) + vy_body * math.cos(current_yaw)

    vel_msg = TwistStamped()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.header.frame_id = "map"   # world frame

    vel_msg.twist.linear.x = vx_world
    vel_msg.twist.linear.y = vy_world
    vel_msg.twist.linear.z = vz_body
    vel_msg.twist.angular.z = yaw_rate

    drone_vel_pub.publish(vel_msg)


def change_state_service(req):
    global controller_state
    if controller_state == STATE_HOLD:
        rospy.loginfo("Service request: Changing to SINUSOID")
        change_state(STATE_SINUSOID)
        return TriggerResponse(success=True, message="State changed to SINUSOID")
    elif controller_state == STATE_SINUSOID:
        rospy.loginfo("Service request: Changing to HOLD")
        change_state(STATE_HOLD)
        return TriggerResponse(success=True, message="State changed to HOLD")
    else:
        return TriggerResponse(success=False, message="Cannot change state from current state")

def set_heading_service(key):
    global current_heading, dx, dy, heading_map
    current_heading = heading_map[key]
    dx = math.cos(current_heading)
    dy = math.sin(current_heading)
    rospy.loginfo(f"Heading set to {key} ({current_heading:.2f} rad)")
    return TriggerResponse(success=True, message=f"Heading set to {key}")



# ============================================================
# Controller state machine constants
# ============================================================
STATE_INIT     = 0
STATE_TAKEOFF  = 1
STATE_HOLD     = 2
STATE_SINUSOID = 3

controller_state = STATE_INIT
state_start_time = time.time()

# controller params
target_alt = 3.0
Kp_alt = 1.2

# sinusoid params
A = 0.5
omega = 0.6         # rad/sec if used directly, below uses math.sin(omega * t)
direction_angle = 0
dx = math.cos(direction_angle)
dy = math.sin(direction_angle)

def change_state(new_state):
    global controller_state, state_start_time
    controller_state = new_state
    state_start_time = time.time()

# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    rospy.init_node("offb_controller")

    # Params (can override via rosparam)
    target_alt = rospy.get_param("~target_alt", target_alt)
    Kp_alt = rospy.get_param("~altitude_kp", Kp_alt)
    A = rospy.get_param("~amplitude", A)
    omega = rospy.get_param("~omega", omega)    # treat as rad/s (or set to 2*pi*freq)
    direction_angle = rospy.get_param("~direction_angle", direction_angle)
    dx = math.cos(direction_angle)
    dy = math.sin(direction_angle)
    rate_hz = rospy.get_param("~rate", 20)
    max_vz = rospy.get_param("~max_vz", 0.6)

    # Subscribers (correct types)
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)  # pose -> PoseStamped
    #rospy.Subscriber("incoming/cmd_vel", Twist, vel_cb)# teleop/body commands (Twist)
    

    # Publishers
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    drone_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    # Services
    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.Service("change_state", Trigger, change_state_service)
    rospy.loginfo("State-Change service ready")

    rospy.Service("/set_heading_i", Trigger, lambda req: set_heading_service('i'))
    rospy.Service("/set_heading_j", Trigger, lambda req: set_heading_service('j'))
    rospy.Service("/set_heading_k", Trigger, lambda req: set_heading_service('k'))
    rospy.Service("/set_heading_l", Trigger, lambda req: set_heading_service('l'))



    rate = rospy.Rate(rate_hz)
    zero_vel = TwistStamped()

    for _ in range(50):
        # also publish zero velocity so px4 receives continuous setpoints
        drone_vel_pub.publish(zero_vel)
        rate.sleep()

    # Setup offboard/arming state trackers
    offboard_enabled = False
    armed = False
    last_req = rospy.Time.now()

    change_state(STATE_TAKEOFF)
    start_time = rospy.Time.now().to_sec()

    # Control loop
    while not rospy.is_shutdown():

        # Periodically request OFFBOARD
        if not offboard_enabled and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            try:
                resp = set_mode_client(custom_mode="OFFBOARD")
                if getattr(resp, 'mode_sent', False):
                    rospy.loginfo("OFFBOARD enabled")
                    offboard_enabled = True
                else:
                    rospy.logwarn("OFFBOARD request returned no mode_sent")
            except rospy.ServiceException as e:
                rospy.logwarn("Set mode service call failed: %s" % str(e))
            last_req = rospy.Time.now()

        # Periodically request ARM
        if not armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            try:
                resp = arming_client(True)
                if getattr(resp, 'success', False):
                    rospy.loginfo("Vehicle armed")
                    armed = True
                else:
                    rospy.logwarn("Arm request returned no success")
            except rospy.ServiceException as e:
                rospy.logwarn("Arming service call failed: %s" % str(e))
            last_req = rospy.Time.now()

        # --- State machine ---
        if controller_state == STATE_TAKEOFF:
            error = target_alt - current_altitude
            vz = Kp_alt * error
            vz = max(min(vz, max_vz), -max_vz)

            # body-frame command: only vertical velocity
            latest_body_cmd.linear.x = 0.0
            latest_body_cmd.linear.y = 0.0
            latest_body_cmd.linear.z = vz
            latest_body_cmd.angular.z = 0.0

            publish_velocity()

            if abs(error) < 0.15:
                rospy.loginfo("Reached altitude -> HOLD")
                change_state(STATE_HOLD)

        elif controller_state == STATE_HOLD:
            error = target_alt - current_altitude
            vz = Kp_alt * error
            vz = max(min(vz, max_vz), -max_vz)

            latest_body_cmd.linear.x = 0.0
            latest_body_cmd.linear.y = 0.0
            latest_body_cmd.linear.z = vz
            latest_body_cmd.angular.z = 0.0

            publish_velocity()

            rospy.wait_for_service("change_state")

            # # short settle then start sinusoid
            # if time.time() - state_start_time > 1.0:
            #     rospy.loginfo("Starting sinusoidal motion")
            #     change_state(STATE_SINUSOID)
            #     # reset start time used for sinusoid phase
            #     state_start_time = time.time()


        # # Chattens sinus
        # elif controller_state == STATE_SINUSOID:
        #     t = time.time() - state_start_time
        #     sine = A * math.sin(omega * t)

        #     error = target_alt - current_altitude
        #     vz = Kp_alt * error
        #     vz = max(min(vz, max_vz), -max_vz)

        #     # sinusoid in body-frame then rotated to world in publish_velocity()
        #     latest_body_cmd.linear.x = sine * dx
        #     latest_body_cmd.linear.y = sine * dy
        #     latest_body_cmd.linear.z = vz
        #     latest_body_cmd.angular.z = 0.0

        #     publish_velocity()

        elif controller_state == STATE_SINUSOID:
            t = time.time() - state_start_time

            # --- Heading-aware sinusoid ---
            # Use current_heading from service
            dx = math.cos(current_heading)
            dy = math.sin(current_heading)

            # perpendicular for side-to-side oscillation
            perp_x = -dy
            perp_y = dx

            sine = A * math.sin(omega * t)

            # vertical control
            error = target_alt - current_altitude
            vz = Kp_alt * error
            vz = max(min(vz, max_vz), -max_vz)

            # forward + sinusoid perpendicular to heading
            latest_body_cmd.linear.x = 1 * dx + sine * perp_x  # 0.5 m/s forward along heading
            latest_body_cmd.linear.y = 1 * dy + sine * perp_y
            latest_body_cmd.linear.z = vz
            latest_body_cmd.angular.z = 0.0

            publish_velocity()




        # loop rate
        rate.sleep()
