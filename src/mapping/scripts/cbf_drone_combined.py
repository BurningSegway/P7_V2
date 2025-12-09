#!/usr/bin/env python

import rospy
import numpy as np
from cvxopt import matrix, solvers
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs

# Suppress cvxopt output
solvers.options['show_progress'] = False

class CBFCommRangeFilter:
    def __init__(self):
        rospy.init_node('cbf_comm_range_filter', anonymous=True)
        
        # Drone state (from TF tree)
        self.drone_x = 0.0
        self.drone_y = 0.0
        
        # Other drone state (subscribed)
        self.other_drone_x = 0.0
        self.other_drone_y = 0.0
        self.other_drone_vx = 0.0  # Other drone velocity X
        self.other_drone_vy = 0.0  # Other drone velocity Y
        self.other_drone_received = False
        
        # Ground robot state (from TF tree)
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # CBF parameters
        self.alpha = rospy.get_param('~comm_radius', 1.0)  # Communication radius [m]
        self.safety_margin = rospy.get_param('~safety_margin', 0.2)  # Safety margin before boundary [m]
        self.gamma_comm = rospy.get_param('~gamma_comm', 10.0)  # Communication CBF parameter
        self.gamma_coll = rospy.get_param('~gamma_coll', 10.0)  # Collision CBF parameter
        self.drone_radius = rospy.get_param('~drone_radius', 0.5)  # Drone collision radius [m]
        self.P_slack = rospy.get_param('~P_slack', 1e6)  # Slack variable penalty
        
        # Control limits (velocity saturation for drone)
        self.lb = np.array([-2.0, -2.0])  # [Vx, Vy] lower bounds
        self.ub = np.array([2.0, 2.0])    # [Vx, Vy] upper bounds
        
        # Nominal control from teleop
        self.u_nom_x = 0.0
        self.u_nom_y = 0.0
        self.teleop_msg = Twist()  # Store full teleop message
        
        # Flags
        self.drone_received = False
        self.robot_received = False
        self.teleop_received = False
        
        # TF buffer for frame transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.global_frame = rospy.get_param('~global_frame', 'global_map')
        self.drone_frame = rospy.get_param('~drone_frame', 'drone1/base_link')
        self.robot_frame = rospy.get_param('~robot_frame', 'robot_base_link')
        self.other_drone_frame = rospy.get_param('~other_drone_frame', 'drone2/base_link')
        self.sub_topic = rospy.get_param('~sub_topic', 'drone1/desired/cmd_vel')
        self.pub_topic = rospy.get_param('~pub_topic', 'drone1/incoming/cmd_vel')
        self.other_drone_cmd_topic = rospy.get_param('~other_drone_cmd_topic', 'drone2/desired/cmd_vel')
        
        # Subscribers
        self.teleop_sub = rospy.Subscriber(self.sub_topic, Twist, self.teleop_callback)
        self.other_drone_cmd_sub = rospy.Subscriber(self.other_drone_cmd_topic, Twist, self.other_drone_cmd_callback)
        
        # Timer for control loop - also queries TF
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.cmd_callback)
        
        # Publisher for filtered commands
        self.safe_cmd_pub = rospy.Publisher(self.pub_topic, Twist, queue_size=10)
        
        rospy.loginfo("CBF Communication Range Filter Initialized")
        rospy.loginfo("Communication radius: {}m, Safety margin: {}m, Gamma_comm: {}".format(
            self.alpha, self.safety_margin, self.gamma_comm))
        rospy.loginfo("Collision avoidance: Drone radius: {}m, Gamma_coll: {}".format(
            self.drone_radius, self.gamma_coll))
        rospy.loginfo("Using global frame: {}".format(self.global_frame))
        rospy.loginfo("Drone frame: {}, Robot frame: {}".format(self.drone_frame, self.robot_frame))
        
    def update_poses(self):
        """Query TF tree to get current drone and robot positions"""
        try:
            # Get drone position
            drone_transform = self.tf_buffer.lookup_transform(self.global_frame, self.drone_frame, rospy.Time(0))
            self.drone_x = drone_transform.transform.translation.x
            self.drone_y = drone_transform.transform.translation.y
            self.drone_received = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "Failed to get drone pose: {}".format(str(e)))
            self.drone_received = False
        
        try:
            # Get robot position
            robot_transform = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time(0))
            self.robot_x = robot_transform.transform.translation.x
            self.robot_y = robot_transform.transform.translation.y
            self.robot_received = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "Failed to get robot pose: {}".format(str(e)))
            self.robot_received = False
        
        try:
            # Get other drone position
            other_drone_transform = self.tf_buffer.lookup_transform(self.global_frame, self.other_drone_frame, rospy.Time(0))
            self.other_drone_x = other_drone_transform.transform.translation.x
            self.other_drone_y = other_drone_transform.transform.translation.y
            self.other_drone_received = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "Failed to get other drone pose: {}".format(str(e)))
            self.other_drone_received = False
        
    def teleop_callback(self, msg):
        """Update nominal drone control from teleop"""
        self.u_nom_x = msg.linear.x
        self.u_nom_y = msg.linear.y
        self.teleop_msg = msg  # Store full message
        self.teleop_received = True
        
    def other_drone_cmd_callback(self, msg):
        """Subscribe to other drone's desired command velocity"""
        self.other_drone_vx = msg.linear.x
        self.other_drone_vy = msg.linear.y
        
    def barrier_function(self, x, y, x0, y0, alpha):
        """
        Communication radius barrier function with safety margin (single integrator)
        h = (alpha - safety_margin)^2 - (x - x0)^2 - (y - y0)^2
        h >= 0 means drone is within safe communication range (with margin)
        """
        effective_radius = alpha - self.safety_margin
        return effective_radius**2 - (x - x0)**2 - (y - y0)**2
    
    def Lf_h(self, x, y, x0, y0, alpha):
        """
        Lie derivative with respect to drift (f = 0 for single integrator)
        Lf_h = 0 (since f = 0)
        """
        return 0.0
    
    def Lg_h(self, x, y, x0, y0, alpha):
        """
        Lie derivative of communication barrier w.r.t. control input
        dh/dx = -2(x - x0)
        dh/dy = -2(y - y0)
        Lg_h = [dh/dx, dh/dy] = [-2(x - x0), -2(y - y0)]
        """
        dh_dx = -2.0 * (x - x0)
        dh_dy = -2.0 * (y - y0)
        return np.array([dh_dx, dh_dy])
    
    def collision_barrier_function(self, x1, y1, x2, y2):
        """
        Inter-drone collision barrier function
        h = ||p1 - p2||^2 - (2*Rd)^2
        h >= 0 means drones are sufficiently separated
        """
        dx = x1 - x2
        dy = y1 - y2
        return dx**2 + dy**2 - (2.0 * (self.drone_radius+0.2))**2
    
    def Lg_h_collision(self, x1, y1, x2, y2):
        """
        Lie derivative of collision barrier w.r.t. u1 (control of drone 1)
        dh/dx1 = 2(x1 - x2)
        dh/dy1 = 2(y1 - y2)
        Lg_h = [2(x1 - x2), 2(y1 - y2)]
        """
        dx = x1 - x2
        dy = y1 - y2
        return np.array([2.0 * dx, 2.0 * dy])
    
    def Lf_h_collision(self, x1, y1, x2, y2, u2_x, u2_y):
        """
        Lie derivative contribution from other drone's velocity
        dh/dt due to drone 2's motion: 2(dx * u2_x + dy * u2_y)
        where dx = x1 - x2, dy = y1 - y2
        """
        dx = x1 - x2
        dy = y1 - y2
        return 2.0 * (dx * u2_x + dy * u2_y)
    
    def compute_distance(self):
        """Compute distance between drone and robot"""
        dx = self.drone_x - self.robot_x
        dy = self.drone_y - self.robot_y
        return np.sqrt(dx**2 + dy**2)
    
    def apply_cbf_filter(self, u_nom):
        """
        Apply Control Barrier Function QP with TWO slack variables using cvxopt
        Accounts for other drone's velocity in collision CBF
        
        Decision variables: [Vx, Vy, s_comm, s_coll]
        minimize: ||u - u_nom||^2 + P_slack * (s_comm^2 + s_coll^2)
        subject to: Communication CBF: -Lg_h_comm * u - s_comm <= gamma_comm * h_comm
                   Collision CBF:      -Lg_h_coll * u - s_coll <= gamma_coll * h_coll + Lf_coll
                   lb <= u <= ub
                   s_comm >= 0, s_coll >= 0
        """
        
        # Compute communication barrier and Lie derivatives
        h_comm = self.barrier_function(self.drone_x, self.drone_y, self.robot_x, 
                                      self.robot_y, self.alpha)
        Lg_h_comm = self.Lg_h(self.drone_x, self.drone_y, self.robot_x, 
                             self.robot_y, self.alpha)
        
        # Compute collision barrier and Lie derivatives
        h_coll = self.collision_barrier_function(self.drone_x, self.drone_y, 
                                               self.other_drone_x, self.other_drone_y)
        Lg_h_coll = self.Lg_h_collision(self.drone_x, self.drone_y, 
                                       self.other_drone_x, self.other_drone_y)
        
        # Compute Lf contribution from other drone's velocity
        Lf_coll = self.Lf_h_collision(self.drone_x, self.drone_y, 
                                      self.other_drone_x, self.other_drone_y, 
                                      self.other_drone_vx, self.other_drone_vy)
        
        # If far from both boundaries, return nominal control
        if h_comm > 2.0 and h_coll > 2.0:
            return u_nom, 0.0, 0.0
        
        try:
            # P matrix for [Vx, Vy, s_comm, s_coll]: quadratic cost
            # ||u - u_nom||^2 + P_slack * (s_comm^2 + s_coll^2)
            P = matrix(np.diag([2.0, 2.0, 2.0 * self.P_slack, 2.0 * self.P_slack]))
            
            # q vector for linear cost
            q = matrix(np.array([-2.0*u_nom[0], -2.0*u_nom[1], 0.0, 0.0]))
            
            # Inequality constraints G*x <= h (cvxopt convention)
            G_list = [
                [-Lg_h_comm[0], -Lg_h_comm[1], -1.0, 0.0],   # Communication CBF
                [-Lg_h_coll[0], -Lg_h_coll[1], 0.0, -1.0],   # Collision CBF
                [-1.0, 0.0, 0.0, 0.0],                        # Vx >= lb[0]
                [0.0, -1.0, 0.0, 0.0],                        # Vy >= lb[1]
                [1.0, 0.0, 0.0, 0.0],                         # Vx <= ub[0]
                [0.0, 1.0, 0.0, 0.0],                         # Vy <= ub[1]
                [0.0, 0.0, -1.0, 0.0],                        # s_comm >= 0
                [0.0, 0.0, 0.0, -1.0]                         # s_coll >= 0
            ]
            
            h_list = [
                self.gamma_comm * h_comm,                  # Communication CBF
                self.gamma_coll * h_coll + Lf_coll,        # Collision CBF with other drone velocity
                -self.lb[0],                               # Vx lower bound
                -self.lb[1],                               # Vy lower bound
                self.ub[0],                                # Vx upper bound
                self.ub[1],                                # Vy upper bound
                0.0,                                       # s_comm >= 0
                0.0                                        # s_coll >= 0
            ]
            
            G = matrix(G_list).T
            h_vec = matrix(h_list)
            
            # Solve QP
            sol = solvers.qp(P, q, G, h_vec)
            
            if sol['status'] == 'optimal':
                x_sol = np.array(sol['x']).flatten()
                u_safe = x_sol[0:2]
                s_comm = x_sol[2]
                s_coll = x_sol[3]
                return u_safe, s_comm, s_coll
            else:
                rospy.logerr("CBF optimization failed: {}".format(sol['status']))
                return u_nom, 0.0, 0.0
                
        except Exception as e:
            rospy.logerr("CBF filter error: {}".format(e))
            return u_nom, 0.0, 0.0
    
    def cmd_callback(self, event):
        """Apply CBF safety filter to drone velocity commands"""
        # Update poses from TF tree
        self.update_poses()
        
        if not self.drone_received or not self.robot_received:
            rospy.logwarn_throttle(5, "Waiting for drone and robot pose data...")
            return
        
        if not self.other_drone_received:
            rospy.logwarn_throttle(5, "Waiting for other drone pose data...")
            return
        
        if not self.teleop_received:
            rospy.logwarn_throttle(5, "Waiting for teleop commands...")
            return
        
        # Get nominal control from teleop
        u_nom = np.array([self.u_nom_x, self.u_nom_y])
        
        # Apply CBF safety filter
        u_safe, slack_comm, slack_coll = self.apply_cbf_filter(u_nom)
        
        # Convert to Twist message - copy all fields from teleop, only modify x and y
        safe_cmd = Twist()
        safe_cmd.linear.x = u_safe[0]
        safe_cmd.linear.y = u_safe[1]
        safe_cmd.linear.z = self.teleop_msg.linear.z
        safe_cmd.angular.x = self.teleop_msg.angular.x
        safe_cmd.angular.y = self.teleop_msg.angular.y
        safe_cmd.angular.z = self.teleop_msg.angular.z
        
        # Publish safe command
        self.safe_cmd_pub.publish(safe_cmd)
        
        # Log if command was modified
        if np.linalg.norm(u_safe - u_nom) > 0.01 or slack_comm > 0.01 or slack_coll > 0.01:
            rospy.logwarn("CBF active!")
            if slack_comm > 0.01:
                rospy.logwarn("  Communication: Distance to robot: {:.3f}m, h_comm: {:.3f}".format(
                    np.sqrt((self.drone_x - self.robot_x)**2 + (self.drone_y - self.robot_y)**2),
                    self.barrier_function(self.drone_x, self.drone_y, self.robot_x, self.robot_y, self.alpha)))
            if slack_coll > 0.01:
                rospy.logwarn("  Collision: Distance to other drone: {:.3f}m, h_coll: {:.3f}".format(
                    np.sqrt((self.drone_x - self.other_drone_x)**2 + (self.drone_y - self.other_drone_y)**2),
                    self.collision_barrier_function(self.drone_x, self.drone_y, self.other_drone_x, self.other_drone_y)))
            rospy.logwarn("  Desired: Vx={:.2f}, Vy={:.2f} -> Safe: Vx={:.2f}, Vy={:.2f}".format(
                u_nom[0], u_nom[1], u_safe[0], u_safe[1]))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CBFCommRangeFilter()
        node.run()
    except rospy.ROSInterruptException:
        pass