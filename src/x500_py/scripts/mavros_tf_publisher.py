#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

def pose_callback(msg):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    
    # Get tf_prefix from parameter (set by robot_state_publisher or launch file)
    tf_prefix = rospy.get_param('~tf_prefix', '')
    
    # Apply namespace prefix to frame names
    if tf_prefix:
        t.header.frame_id = tf_prefix + "/odom"
        t.child_frame_id = tf_prefix + "/base_link"
    else:
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
    
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation = msg.pose.orientation
    
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('mavros_tf_publisher')
    br = tf2_ros.TransformBroadcaster()
    
    # Get namespace for logging
    tf_prefix = rospy.get_param('~tf_prefix', '')
    if tf_prefix:
        rospy.loginfo("TF Publisher started - broadcasting %s/odom -> %s/base_link", tf_prefix, tf_prefix)
    else:
        rospy.loginfo("TF Publisher started - broadcasting odom -> base_link")
    
    # Subscribe using relative topic (no namespace prefix needed when launched in namespace)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.spin()