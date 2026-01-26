#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class RobotCircleVisualizer:
    def __init__(self):
        rospy.init_node('robot_circle_visualizer')
        # Create publisher for markers
        vis_topic = rospy.get_param('~vis_topic', 'robot/visualization_marker/radius')
        self.marker_pub = rospy.Publisher(vis_topic, Marker, queue_size=10)
        
        # Parameters
        self.circle_radius = rospy.get_param('~circle_radius', 5.0)
        self.circle_segments = rospy.get_param('~circle_segments', 50)
        self.frame_id = rospy.get_param('~frame_id', 'robot_base_link')
        self.update_rate = rospy.get_param('~update_rate', 10)
        
        # Color parameters
        self.color_r = rospy.get_param('~color_r', 0.0)
        self.color_g = rospy.get_param('~color_g', 1.0)
        self.color_b = rospy.get_param('~color_b', 0.0)
        self.color_a = rospy.get_param('~color_a', 0.5)
        
        # Line width
        self.line_width = rospy.get_param('~line_width', 0.05)
        
        # Publishing rate
        self.rate = rospy.Rate(self.update_rate)

    def create_circle_marker(self):
        """Create a circle marker using LINE_STRIP"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = self.line_width
        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.color.a = self.color_a
        
        # Generate circle points
        marker.points = []
        for i in range(self.circle_segments + 1):
            angle = (2.0 * math.pi * i) / self.circle_segments
            x = self.circle_radius * math.cos(angle)
            y = self.circle_radius * math.sin(angle)
            z = 0.0
            marker.points.append(Point(x=x, y=y, z=z))
        
        # Set position (at robot's center)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        return marker

    def run(self):
        """Main loop to publish markers"""
        rospy.loginfo("Robot Circle Visualizer started")
        rospy.loginfo("Publishing circle with radius: %.2f m" % self.circle_radius)
        rospy.loginfo("Color: R=%.2f, G=%.2f, B=%.2f, A=%.2f" % (self.color_r, self.color_g, self.color_b, self.color_a))
        rospy.loginfo("Line width: %.2f" % self.line_width)
        
        while not rospy.is_shutdown():
            marker = self.create_circle_marker()
            self.marker_pub.publish(marker)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = RobotCircleVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass