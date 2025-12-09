#!/usr/bin/env python
"""
Ground Level Frame Publisher

Subscribes to drone ground truth position (base_link) and publishes a new frame
at ground level (z=0) with the same X, Y coordinates as the drone.

This frame can be used as target_frame for pointcloud_to_laserscan to ensure
horizontal slicing regardless of drone tilt.

Python 2 compatible for ROS Melodic
"""

import rospy
import tf
import math
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class GroundLevelFramePublisher:
    def __init__(self, drone_namespace="drone1"):
        self.drone_namespace = drone_namespace.strip('/')
        self.br = tf.TransformBroadcaster()
        
        # Frame names
        self.drone_base_frame = "{}/base_link".format(self.drone_namespace)
        self.ground_level_frame = "{}/ground_level".format(self.drone_namespace)
        self.reference_frame = "{}/odom".format(self.drone_namespace)  # or "world" depending on your setup
        
        # Subscribe to the ground truth transform
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Ground Level Frame Publisher initialized")
        rospy.loginfo("  Drone base frame: {}".format(self.drone_base_frame))
        rospy.loginfo("  Ground level frame: {}".format(self.ground_level_frame))
        rospy.loginfo("  Reference frame: {}".format(self.reference_frame))
        
    def publish_ground_level_frame(self):
        """
        Get drone position and publish a frame at ground level with same X, Y
        """
        try:
            # Get the latest transform from reference frame to drone base_link
            (trans, rot) = self.tf_listener.lookupTransform(
                self.reference_frame,
                self.drone_base_frame,
                rospy.Time(0)
            )
            
            # Extract X, Y and set Z to 0
            x, y, z = trans
            ground_level_x = x
            ground_level_y = y
            ground_level_z = 0.0
            
            # Keep orientation level (no roll/pitch, only yaw)
            # Extract yaw from drone's quaternion
            qx, qy, qz, qw = rot
            
            # Calculate yaw from quaternion
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Create level quaternion (roll=0, pitch=0, yaw=yaw)
            level_quat = quaternion_from_euler(0, 0, yaw)
            
            # Publish transform
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.reference_frame
            t.child_frame_id = self.ground_level_frame
            
            t.transform.translation.x = ground_level_x
            t.transform.translation.y = ground_level_y
            t.transform.translation.z = ground_level_z
            
            t.transform.rotation.x = level_quat[0]
            t.transform.rotation.y = level_quat[1]
            t.transform.rotation.z = level_quat[2]
            t.transform.rotation.w = level_quat[3]
            
            self.br.sendTransformMessage(t)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "TF lookup failed: {}".format(e))

if __name__ == "__main__":
    rospy.init_node("ground_level_frame_publisher")
    
    # Get drone namespace from ROS parameters or default
    drone_namespace = rospy.get_param("~drone_namespace", "drone1")
    reference_frame = rospy.get_param("~reference_frame", None)
    
    publisher = GroundLevelFramePublisher(drone_namespace=drone_namespace)
    
    # If reference_frame is specified as parameter, override it
    if reference_frame:
        publisher.reference_frame = reference_frame
        rospy.loginfo("Using custom reference frame: {}".format(reference_frame))
    
    rate = rospy.Rate(30)  # 30 Hz to keep TF tree updated
    
    while not rospy.is_shutdown():
        publisher.publish_ground_level_frame()
        rate.sleep()