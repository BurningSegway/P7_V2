#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

def pose_callback(msg):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation = msg.pose.orientation
    #print(msg.pose.position.z)
    
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('mavros_tf_publisher')
    
    br = tf2_ros.TransformBroadcaster()
    
    # Subscribe using relative topic (no namespace prefix needed when launched in namespace)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_callback)
    
    rospy.loginfo("TF Publisher started - broadcasting odom -> base_link")
    rospy.spin()