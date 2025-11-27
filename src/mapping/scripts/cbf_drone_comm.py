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
        
        # Ground robot state (from TF tree)
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # CBF parameters
        self.alpha = rospy.get_param('~comm_radius', 1.0)  # Communication radius [m]
        self.gamma = rospy.get_param('~gamma', 1.0)  # CBF class-K function parameter
        self.P_slack = rospy.get_param('~P_slack', 1e6)  # Slack variable penalty
        
        # Control limits (velocity saturation for drone)
        self.lb = np.array([-1.5, -1.5])  # [Vx, Vy] lower bounds
        self.ub = np.array([1.5, 1.5])    # [Vx, Vy] upper bounds
        
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
        
        # Subscribers (only need teleop now)
        self.teleop_sub = rospy.Subscriber('drone1/desired/cmd_vel', Twist, self.teleop_callback) #tag teleop, eller controller
        
        # Timer for control loop - also queries TF
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.cmd_callback)
        
        # Publisher for filtered commands
        self.safe_cmd_pub = rospy.Publisher('drone1/incoming/cmd_vel', Twist, queue_size=10) #smid de in i offboard controller
        
        rospy.loginfo("CBF Communication Range Filter Initialized")
        rospy.loginfo("Communication radius: {}m, Gamma: {}".format(self.alpha, self.gamma))
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
        
    def teleop_callback(self, msg):
        """Update nominal drone control from teleop"""
        self.u_nom_x = msg.linear.x
        self.u_nom_y = msg.linear.y
        self.teleop_msg = msg  # Store full message
        self.teleop_received = True
        
    def barrier_function(self, x, y, x0, y0, alpha):
        """
        Communication radius barrier function (single integrator)
        h = alpha^2 - (x - x0)^2 - (y - y0)^2
        h >= 0 means drone is within communication range
        """
        return (alpha-0.3)**2 - (x - x0)**2 - (y - y0)**2
    
    def Lf_h(self, x, y, x0, y0, alpha):
        """
        Lie derivative with respect to drift (f = 0 for single integrator)
        Lf_h = dh/dx * f_x + dh/dy * f_y = 0 (since f = 0)
        """
        return 0.0
    
    def Lg_h(self, x, y, x0, y0, alpha):
        """
        Lie derivative with respect to control input
        dh/dx = -2(x - x0)
        dh/dy = -2(y - y0)
        Lg_h = [dh/dx, dh/dy] = [-2(x - x0), -2(y - y0)]
        """
        dh_dx = -2.0 * (x - x0)
        dh_dy = -2.0 * (y - y0)
        return np.array([dh_dx, dh_dy])
    
    def compute_distance(self):
        """Compute distance between drone and robot"""
        dx = self.drone_x - self.robot_x
        dy = self.drone_y - self.robot_y
        return np.sqrt(dx**2 + dy**2)
    
    def apply_cbf_filter(self, u_nom):
        """
        Apply Control Barrier Function QP with slack variable using cvxopt
        
        Decision variables: [Vx, Vy, s]
        minimize: ||u - u_nom||^2 + P_slack * s^2
        subject to: Lg_h * u - s <= Lf_h + gamma * h
                   lb <= u <= ub
                   s >= 0
        """
        
        # Compute barrier function and Lie derivatives
        h_val = self.barrier_function(self.drone_x, self.drone_y, self.robot_x, 
                                     self.robot_y, self.alpha)
        Lf_h_val = self.Lf_h(self.drone_x, self.drone_y, self.robot_x, 
                            self.robot_y, self.alpha)
        Lg_h_val = self.Lg_h(self.drone_x, self.drone_y, self.robot_x, 
                            self.robot_y, self.alpha)
        
        # If far from boundary, return nominal control
        if h_val > 10.0:
            return u_nom, 0.0
        
        try:
            # P matrix for [Vx, Vy, s]: quadratic cost
            # ||u - u_nom||^2 + P_slack * s^2
            P = matrix(np.diag([2.0, 2.0, 2.0 * self.P_slack]))
            
            # q vector for linear cost
            q = matrix(np.array([-2.0*u_nom[0], -2.0*u_nom[1], 0.0]))
            
            # Inequality constraints G*x <= h (cvxopt convention)
            # CBF constraint: Lg_h * u + gamma * h >= -s
            # Rearranged: Lg_h * u + s >= -gamma * h
            # In cvxopt form: -Lg_h * u - s <= gamma * h
            
            G_list = [
                [-Lg_h_val[0], -Lg_h_val[1], -1.0],   # -Lg_h*u - s <= gamma*h
                [-1.0, 0.0, 0.0],                      # Vx >= lb[0]
                [0.0, -1.0, 0.0],                      # Vy >= lb[1]
                [1.0, 0.0, 0.0],                       # Vx <= ub[0]
                [0.0, 1.0, 0.0],                       # Vy <= ub[1]
                [0.0, 0.0, -1.0]                       # s >= 0
            ]
            
            h_list = [
                self.gamma * h_val,   # CBF: gamma*h
                -self.lb[0],          # Vx lower bound
                -self.lb[1],          # Vy lower bound
                self.ub[0],           # Vx upper bound
                self.ub[1],           # Vy upper bound
                0.0                   # s >= 0
            ]
            
            G = matrix(G_list).T
            h_vec = matrix(h_list)
            
            # Solve QP
            sol = solvers.qp(P, q, G, h_vec)
            
            if sol['status'] == 'optimal':
                x_sol = np.array(sol['x']).flatten()
                u_safe = x_sol[0:2]
                s_val = x_sol[2]
                return u_safe, s_val
            else:
                rospy.logerr("CBF optimization failed: {}".format(sol['status']))
                return u_nom, 0.0
                
        except Exception as e:
            rospy.logerr("CBF filter error: {}".format(e))
            return u_nom, 0.0
    
    def cmd_callback(self, event):
        """Apply CBF safety filter to drone velocity commands"""
        # Update poses from TF tree
        self.update_poses()
        
        if not self.drone_received or not self.robot_received:
            rospy.logwarn_throttle(5, "Waiting for drone and robot pose data...")
            return
        
        if not self.teleop_received:
            rospy.logwarn_throttle(5, "Waiting for teleop commands...")
            return
        
        # Get nominal control from teleop
        u_nom = np.array([self.u_nom_x, self.u_nom_y])
        
        # Apply CBF safety filter
        u_safe, slack = self.apply_cbf_filter(u_nom)
        
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
        if np.linalg.norm(u_safe - u_nom) > 0.01 or slack > 0.01:
            rospy.logwarn("CBF active! Distance: {:.3f}m, Barrier h: {:.3f}".format(
                self.compute_distance(), 
                self.barrier_function(self.drone_x, self.drone_y, self.robot_x, 
                                    self.robot_y, self.alpha)))
            rospy.logwarn("Desired: Vx={:.2f}, Vy={:.2f} -> Safe: Vx={:.2f}, Vy={:.2f}, Slack={:.4f}".format(
                u_nom[0], u_nom[1], u_safe[0], u_safe[1], slack))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CBFCommRangeFilter()
        node.run()
    except rospy.ROSInterruptException:
        pass