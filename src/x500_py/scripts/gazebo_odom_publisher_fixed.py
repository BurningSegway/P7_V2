#!/usr/bin/env python
"""
Ground truth odometry from Gazebo model states
Publishes world base_link transform directly (not through odom)
This lets SLAM handle map odom independently
"""
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState

class GazeboOdomPublisher:
    def __init__(self, drone_name):
        self.drone_name = drone_name  # e.g., "drone1"
        self.model_name = drone_name  # Gazebo model name
        
        # Get tf_prefix from param (set by launch file)
        self.tf_prefix = rospy.get_param('~tf_prefix', drone_name)
        
        # Publishers - use ground_truth/* topics to avoid MAVROS conflicts
        self.odom_pub = rospy.Publisher(
            'ground_truth/odom', 
            Odometry, 
            queue_size=10
        )
        self.pose_pub = rospy.Publisher(
            'ground_truth/pose',
            PoseStamped,
            queue_size=10
        )
        self.twist_pub = rospy.Publisher(
            'ground_truth/velocity_local',
            TwistStamped,
            queue_size=10
        )
        
        # TF broadcaster for world base_link (bypasses odom to avoid conflict with SLAM)
        self.br = tf2_ros.TransformBroadcaster()
        
        # Service client for Gazebo model state
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state',
            GetModelState
        )
        
        rospy.loginfo("Ground truth odom publisher for %s initialized", self.drone_name)
        rospy.loginfo("TF prefix: %s", self.tf_prefix)
        rospy.loginfo("Broadcasting global_map  %s/base_link_gt (ground truth only, separate from SLAM)", self.tf_prefix)
    
    def get_model_state_gazebo(self):
        """Query Gazebo for current model state"""
        try:
            resp = self.get_model_state(self.model_name, 'world')
            return resp
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", str(e))
            return None
    
    def publish_odom(self):
        """Main loop - publish ground truth odometry"""
        rate = rospy.Rate(50)  # 50 Hz to match typical MAVROS rate
        
        while not rospy.is_shutdown():
            state = self.get_model_state_gazebo()
            if state is None:
                rate.sleep()
                continue
            
            current_time = rospy.Time.now()
            
            # Extract position
            x = state.pose.position.x
            y = state.pose.position.y
            z = state.pose.position.z
            
            # Extract orientation (already quaternion from Gazebo)
            qx = state.pose.orientation.x
            qy = state.pose.orientation.y
            qz = state.pose.orientation.z
            qw = state.pose.orientation.w
            
            # Extract linear velocity
            vx = state.twist.linear.x
            vy = state.twist.linear.y
            vz = state.twist.linear.z
            
            # Extract angular velocity
            avx = state.twist.angular.x
            avy = state.twist.angular.y
            avz = state.twist.angular.z
            
            # --- Publish Odometry message ---
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "global_map"
            odom_msg.child_frame_id = self.tf_prefix + "/base_link"
            
            # Position
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = z
            
            # Orientation
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            
            # Velocity (from Gazebo twist)
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.linear.z = vz
            
            odom_msg.twist.twist.angular.x = avx
            odom_msg.twist.twist.angular.y = avy
            odom_msg.twist.twist.angular.z = avz
            
            # Covariance (small for ground truth)
            odom_msg.pose.covariance[0] = 0.01
            odom_msg.pose.covariance[7] = 0.01
            odom_msg.pose.covariance[14] = 0.01
            odom_msg.twist.covariance[0] = 0.01
            odom_msg.twist.covariance[7] = 0.01
            odom_msg.twist.covariance[14] = 0.01
            
            self.odom_pub.publish(odom_msg)
            
            # --- Publish PoseStamped ---
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "global_map"
            pose_msg.pose = odom_msg.pose.pose
            self.pose_pub.publish(pose_msg)
            
            # --- Publish TwistStamped ---
            twist_msg = TwistStamped()
            twist_msg.header.stamp = current_time
            twist_msg.header.frame_id = self.tf_prefix + "/base_link"
            twist_msg.twist = odom_msg.twist.twist
            self.twist_pub.publish(twist_msg)
            
            # --- Broadcast TF: global_map base_link_gt (ground truth only, separate from SLAM) ---
            # SLAM will handle map odom base_link normally
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time
            tf_msg.header.frame_id = "global_map"
            tf_msg.child_frame_id = self.tf_prefix + "/base_link_gt"
            
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z
            
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            
            self.br.sendTransform(tf_msg)
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gazebo_odom_publisher')
    
    drone_name = rospy.get_param('~drone_name', 'drone1')
    publisher = GazeboOdomPublisher(drone_name)
    
    rospy.loginfo("Starting ground truth odometry for %s", drone_name)
    publisher.publish_odom()