#!/usr/bin/env python
"""
Simple Costmap-Based Controller
Reads goal from move_base_simple/goal
Plans using costmap
Publishes cmd_vel directly
Foundation for barrier function integration
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

class CostmapController:
    def __init__(self):
        rospy.init_node('costmap_controller')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Parameters
        self.robot_frame = rospy.get_param('~robot_frame', 'robot_base_link')
        self.map_frame = rospy.get_param('~map_frame', 'robot_map')
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 1.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)
        
        # State
        self.goal = None
        self.costmap = None
        self.has_goal = False
        self.has_costmap = False
        
        # Subscribers
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('combined_costmap', OccupancyGrid, 
                        self.costmap_callback)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        
        # Control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Costmap Controller initialized")
        rospy.loginfo("Robot frame: %s, Map frame: %s", self.robot_frame, self.map_frame)

    def goal_callback(self, msg):
        """Callback for goal"""
        self.goal = msg
        self.has_goal = True
        rospy.loginfo("New goal received: (%.2f, %.2f)", 
                     msg.pose.position.x, msg.pose.position.y)

    def costmap_callback(self, msg):
        """Callback for costmap"""
        self.costmap = msg
        self.has_costmap = True

    def control_loop(self, event):
        """Main control loop"""
        if not self.has_goal or not self.has_costmap:
            return

        # Get robot position in map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0), 
                rospy.Duration(0.1))
        except tf2_ros.TransformException as ex:
            rospy.logwarn_throttle(5, "Transform lookup failed: %s", ex)
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        # Goal position
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y

        # Distance to goal
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

        # Check if goal reached
        if dist_to_goal < self.goal_tolerance:
            rospy.loginfo("Goal reached!")
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.has_goal = False
            return

        # Simple control: move toward goal
        # Direction to goal
        angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)

        # Get robot's current orientation
        q = transform.transform.rotation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Angle error
        angle_error = angle_to_goal - yaw

        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Simple proportional control using tanh for smooth saturation
        linear_vel = self.max_linear_vel * math.tanh(dist_to_goal / 2.0)
        angular_vel = self.max_angular_vel * math.tanh(angle_error)

        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

        rospy.logdebug("Robot: (%.2f, %.2f), Goal: (%.2f, %.2f), Dist: %.2f",
                      robot_x, robot_y, goal_x, goal_y, dist_to_goal)

    def run(self):
        """Spin the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = CostmapController()
        controller.run()
    except rospy.ROSInterruptException:
        pass