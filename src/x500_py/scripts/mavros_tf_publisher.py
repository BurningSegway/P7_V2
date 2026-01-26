#!/usr/bin/env python
"""
Ground Truth TF Publisher
Replaces mavros/local_position/pose with /ground_truth/state for accurate odometry.
Maintains the exact same TF chain: drone_X/odom -> drone_X/base_link
robot_state_publisher handles the static sensor transforms.
"""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def ground_truth_callback(msg):
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    
    # Get tf_prefix from parameter
    tf_prefix = rospy.get_param('~tf_prefix', '')
    
    # Apply namespace prefix to frame names 
    if tf_prefix:
        t.header.frame_id = tf_prefix + "/odom"
        t.child_frame_id = tf_prefix + "/base_link"
    else:
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
    
    # Use ground truth position and orientation (accurate!)
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('mavros_tf_publisher')
    br = tf2_ros.TransformBroadcaster()
    
    # Get namespace for logging
    tf_prefix = rospy.get_param('~tf_prefix', '')
    if tf_prefix:
        rospy.loginfo("TF Publisher started - broadcasting %s/odom -> %s/base_link (GROUND TRUTH)", tf_prefix, tf_prefix)
    else:
        rospy.loginfo("TF Publisher started - broadcasting odom -> base_link (GROUND TRUTH)")
    
    # Subscribe to Gazebo ground truth instead of MAVROS pose
    odom_topic = '/{}/ground_truth/state'.format(tf_prefix) if tf_prefix else '/ground_truth/state'
    rospy.loginfo("Subscribing to: %s", odom_topic)
    rospy.Subscriber(odom_topic, Odometry, ground_truth_callback)
    rospy.spin()