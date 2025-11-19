#!/usr/bin/env python

import rospy
import numpy as np
from cvxopt import matrix, solvers
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

# Suppress cvxopt output
solvers.options['show_progress'] = False

class CBFSafetyFilter:
    def __init__(self):
        rospy.init_node('cbf_safety_filter', anonymous=True)
        
        # Robot parameters
        self.b = rospy.get_param('~wheel_base', 0.4)  # Wheel base width [m]
        
        # CBF parameters
        self.Rg = rospy.get_param('~safety_radius', 0.7)  # Safety radius [m]
        self.gamma = rospy.get_param('~gamma', 1.0)  # CBF class-K function parameter
        self.k_dir = rospy.get_param('~k_dir', 0.3)  # Directional barrier weight
        self.P_slack = rospy.get_param('~P_slack', 1000.0)  # Slack variable penalty
        
        # Control limits
        self.v_max = rospy.get_param('~max_linear_vel', 2.0)
        self.omega_max = rospy.get_param('~max_angular_vel', 2.0)
        
        # Convert to wheel velocity limits
        #self.Vr_max = self.v_max + (self.b/2)*self.omega_max
        #self.Vl_max = self.v_max + (self.b/2)*self.omega_max
        self.lb = np.array([-2.0, -2.0])  # [Vr, Vl] lower bounds
        self.ub = np.array([2.0, 2.0])    # [Vr, Vl] upper bounds
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.closest_distance = float('inf')
        self.closest_x0 = 0.0
        self.closest_y0 = 0.0
        self.closest_angle = 0.0
        self.desired_cmd = Twist()
        self.scan_data = None
        self.odom_received = False
        self.desired_cmd = Twist()
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/robot/front_laser/scan', LaserScan, self.scan_callback)
        self.cmd_sub = rospy.Subscriber('desired/cmd_vel', Twist, self.vel_callback)
        self.odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.cmd_callback)

        
        # Publisher for filtered commands
        self.safe_cmd_pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("CBF Safety Filter Node Initialized")
        rospy.loginfo("Safety radius: {}m, Gamma: {}, k_dir: {}".format(self.Rg, self.gamma, self.k_dir))
        rospy.loginfo("Wheel base: {}m".format(self.b))
        
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #print("Odom received: x={:.3f}, y={:.3f}".format(self.x, self.y))
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.theta = yaw
        self.odom_received = True
        
    def scan_callback(self, msg):
        """Process LiDAR scan to find closest point"""
        self.scan_data = msg
        
        # Convert scan to cartesian coordinates and find closest point
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        
        if not np.any(valid_indices):
            rospy.logwarn("No valid LiDAR readings!")
            self.closest_distance = float('inf')
            return
        
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Find closest point
        min_idx = np.argmin(valid_ranges)
        self.closest_distance = valid_ranges[min_idx]
        self.closest_angle = valid_angles[min_idx]
        
        # Convert to global frame (obstacle position relative to robot)
        x_rel = self.closest_distance * np.cos(self.closest_angle)
        y_rel = self.closest_distance * np.sin(self.closest_angle)
        
        # Transform to global coordinates
        self.closest_x0 = self.x + x_rel * np.cos(self.theta) - y_rel * np.sin(self.theta)
        self.closest_y0 = self.y + x_rel * np.sin(self.theta) + y_rel * np.cos(self.theta)
        
    def barrier_function(self, x, y, theta, x0, y0):
        """
        Combined radial + directional barrier function
        h = (x-x0)^2 + (y-y0)^2 - Rg^2 + k_dir*((x-x0)*cos(theta) + (y-y0)*sin(theta))
        """
        radial = (x - x0)**2 + (y - y0)**2 - self.Rg**2
        directional = self.k_dir * ((x - x0)*np.cos(theta) + (y - y0)*np.sin(theta))
        return radial + directional
    
    def Lg_h(self, x, y, theta, x0, y0):
        """    
        dh/dx = 2(x-x0) + k_dir*cos(theta)
        dh/dy = 2(y-y0) + k_dir*sin(theta)
        dh/dtheta = k_dir*(-(x-x0)*sin(theta) + (y-y0)*cos(theta))
        
        g = [0.5*cos(theta), 0.5*cos(theta);
             0.5*sin(theta), 0.5*sin(theta);
             1/b,            -1/b]
        """
        dx = x - x0
        dy = y - y0
        
        dh_dx = 2*dx + self.k_dir*np.cos(theta)
        dh_dy = 2*dy + self.k_dir*np.sin(theta)
        dh_dtheta = self.k_dir*(-dx*np.sin(theta) + dy*np.cos(theta))
        
        # Lg_h = [dh/dx, dh/dy, dh/dtheta] * g
        # For Vr (first column of g):
        Lg_h_Vr = dh_dx * 0.5*np.cos(theta) + dh_dy * 0.5*np.sin(theta) + dh_dtheta * (1/self.b)
        
        # For Vl (second column of g):
        Lg_h_Vl = dh_dx * 0.5*np.cos(theta) + dh_dy * 0.5*np.sin(theta) + dh_dtheta * (-1/self.b)
        
        return np.array([Lg_h_Vr, Lg_h_Vl])
    
    def vel_callback(self, msg):
        self.desired_cmd = msg
        
    def cmd_callback(self, event):
        """Receive desired velocity commands and apply CBF safety filter"""
        msg = self.desired_cmd
        
        if self.scan_data is None or not self.odom_received:
            rospy.logwarn("Waiting for LiDAR and odometry data...")
            self.safe_cmd_pub.publish(msg)
            return
        
        # Convert desired twist to wheel velocities (nominal control)
        v_nom = msg.linear.x
        omega_nom = msg.angular.z
        Vr_nom = v_nom + (self.b/2)*omega_nom
        Vl_nom = v_nom - (self.b/2)*omega_nom
        
        # Clip to bounds
        Vr_nom = np.clip(Vr_nom, self.lb[0], self.ub[0])
        Vl_nom = np.clip(Vl_nom, self.lb[1], self.ub[1])
        u_nom = np.array([Vr_nom, Vl_nom])
        
        # Apply CBF safety filter
        u_safe, slack = self.apply_cbf_filter(u_nom)
        
        # Convert back to twist
        safe_cmd = Twist()
        v_safe = (u_safe[0] + u_safe[1]) / 2.0
        omega_safe = (u_safe[0] - u_safe[1]) / self.b
        safe_cmd.linear.x = v_safe
        safe_cmd.angular.z = omega_safe
        
        # Publish safe command
        self.safe_cmd_pub.publish(safe_cmd)
        
        # Log if command was modified
        if np.linalg.norm(u_safe - u_nom) > 0.01 or slack > 0.01:
            rospy.logwarn("CBF active!")
            rospy.logwarn("Desired: Vr={:.2f}, Vl={:.2f} -> Safe: Vr={:.2f}, Vl={:.2f}".format(
                         Vr_nom, Vl_nom, u_safe[0], u_safe[1]))
    
    def apply_cbf_filter(self, u_nom):
        """
        Apply Control Barrier Function QP with slack variable using cvxopt
        
        minimize: ||u - u_nom||^2 + P_slack * s^2
        subject to: -Lg_h * u + s <= Lf_h + gamma * h
                   lb <= u <= ub
                   s >= 0
        """
        
        # Compute barrier function and Lie derivatives
        h_val = self.barrier_function(self.x, self.y, self.theta, 
                                      self.closest_x0, self.closest_y0)
        Lg_h_val = self.Lg_h(self.x, self.y, self.theta, 
                             self.closest_x0, self.closest_y0)
        
        if h_val > 5.0:
            return u_nom, 0.0
        
        try:
            # Decision variables: [Vr, Vl, s]
            # QP form: minimize (1/2)*x'*P*x + q'*x
            # subject to: G*x <= h, A*x = b (we don't have equality constraints)
            
            # P matrix for [Vr, Vl, s]:
            # ||u - u_nom||^2 + P_slack * s^2
            P = matrix(np.diag([2.0, 2.0, 2.0 * self.P_slack]))
            
            # q vector
            q = matrix(np.array([-2.0*u_nom[0], -2.0*u_nom[1], 0.0]))
            
            # Inequality constraints G*x <= h
            # 1. CBF constraint: -Lg_h * u + s <= gamma * h
            #    [-Lg_h[0], -Lg_h[1], 1] * [Vr, Vl, s] <= gamma * h
            # 2. Lower bounds: -u <= -lb  =>  [-1, 0, 0]*x <= -lb[0], [0, -1, 0]*x <= -lb[1]
            # 3. Upper bounds: u <= ub   =>  [1, 0, 0]*x <= ub[0], [0, 1, 0]*x <= ub[1]
            # 4. Slack constraint: -s <= 0  =>  [0, 0, -1]*x <= 0
            
            G_list = [
                [-Lg_h_val[0], -Lg_h_val[1], 1.0],  # CBF constraint
                [-1.0, 0.0, 0.0],                    # Vr >= lb[0]
                [0.0, -1.0, 0.0],                    # Vl >= lb[1]
                [1.0, 0.0, 0.0],                     # Vr <= ub[0]
                [0.0, 1.0, 0.0],                     # Vl <= ub[1]
                [0.0, 0.0, -1.0]                     # s >= 0
            ]
            
            h_list = [
                self.gamma * h_val,  # CBF
                -self.lb[0],                     # Vr lower bound
                -self.lb[1],                     # Vl lower bound
                self.ub[0],                      # Vr upper bound
                self.ub[1],                      # Vl upper bound
                0.0                              # s >= 0
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
                # Emergency: stop the robot
                return np.array([0.0, 0.0]), 0.0
                
        except Exception as e:
            rospy.logerr("CBF filter error: {}".format(e))
            # Emergency: stop the robot
            return np.array([0.0, 0.0]), 0.0
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CBFSafetyFilter()
        node.run()
    except rospy.ROSInterruptException:
        pass