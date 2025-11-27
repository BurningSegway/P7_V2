#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point, Vector3
import math

class TwistArrowVisualizer:
    def __init__(self):
        rospy.init_node('twist_arrow_visualizer')
        
        # Subscribe to twist topic
        twist_topic = rospy.get_param('~twist_topic', 'drone1/incoming/cmd_vel')
        rospy.Subscriber(twist_topic, Twist, self.twist_callback)
        
        # Create publisher for markers
        self.marker_pub = rospy.Publisher('drone1/visualization_marker', Marker, queue_size=10)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'drone1/base_link')
        self.arrow_scale = rospy.get_param('~arrow_scale', 1.0)
        
        # Current twist values
        self.current_twist = Twist()
        
        rospy.loginfo("Twist Arrow Visualizer started, listening to: %s" % twist_topic)
    
    def twist_callback(self, msg):
        """Store incoming twist message and publish visualization"""
        self.current_twist = msg
        self.publish_arrows()
    
    def create_arrow_marker(self, marker_id, start_point, end_point, color_r, color_g, color_b):
        """Create an arrow marker from start to end point"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set arrow start and end points
        marker.points = [start_point, end_point]
        
        # Arrow appearance
        marker.scale.x = 0.05  # Arrow shaft diameter
        marker.scale.y = 0.1  # Arrow head diameter
        marker.scale.z = 0.1  # Arrow head length
        
        marker.color.r = color_r
        marker.color.g = color_g
        marker.color.b = color_b
        marker.color.a = 0.8
        
        # Set identity quaternion
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        return marker
    
    def publish_arrows(self):
        """Publish arrow markers for x and y velocity components"""
        start = Point(x=0.0, y=0.0, z=0.0)
        
        # X-axis arrow (forward/backward) - Red
        x_end = Point(
            x=self.current_twist.linear.x * self.arrow_scale,
            y=0.0,
            z=0.0
        )
        arrow_x = self.create_arrow_marker(0, start, x_end, 1.0, 0.0, 0.0)
        self.marker_pub.publish(arrow_x)
        
        # Y-axis arrow (left/right) - Green
        y_end = Point(
            x=0.0,
            y=self.current_twist.linear.y * self.arrow_scale,
            z=0.0
        )
        arrow_y = self.create_arrow_marker(1, start, y_end, 0.0, 1.0, 0.0)
        self.marker_pub.publish(arrow_y)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        visualizer = TwistArrowVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass