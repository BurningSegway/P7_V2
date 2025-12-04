#!/usr/bin/env python
"""
Ground Truth Odometry Publisher for Gazebo
Reads Gazebo ground truth odometry and broadcasts clean TF frames.
Works with gazebo_groundtruth_plugin from the drone SDF.
Publishes: global_map -> drone_X/ground_truth_odom -> drone_X/odom (your existing chain)
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class GroundTruthPublisher:
    def __init__(self, drone_ns, drone_id):
        self.drone_ns = drone_ns
        self.drone_id = drone_id
        self.br = tf2_ros.TransformBroadcaster()
        
        # Subscribe to Gazebo ground truth odometry
        odom_topic = '/{}/ground_truth/state'.format(drone_ns)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        
        rospy.loginfo("Ground truth publisher for %s listening on %s", drone_ns, odom_topic)
    
    def odom_callback(self, msg):
        """Broadcast ground truth as TF transform"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "global_map"
        t.child_frame_id = "{}/ground_truth_odom".format(self.drone_ns)
        
        # Position from Gazebo ground truth
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Orientation from Gazebo ground truth
        t.transform.rotation = msg.pose.pose.orientation
        
        self.br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('ground_truth_publishers')
    
    # Create publishers for each drone
    gt_drone1 = GroundTruthPublisher('drone1', 1)
    gt_drone2 = GroundTruthPublisher('drone2', 2)
    
    rospy.loginfo("Ground truth publishers started for drone1 and drone2")
    rospy.spin()