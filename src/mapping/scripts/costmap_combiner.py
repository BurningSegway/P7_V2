#!/usr/bin/env python
"""
Fixed-Size Costmap Combiner with TF Support
Creates a fixed global costmap and layers robot maps into it
Uses TF transforms to properly align maps from different robots
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from tf import TransformListener
import numpy as np

class FixedCostmapCombiner:
    def __init__(self):
        rospy.init_node('costmap_combiner')
        
        # Parameters
        self.robot_namespaces = rospy.get_param('~robot_namespaces', ['robot', 'drone1'])
        self.output_topic = rospy.get_param('~output_topic', 'combined_costmap')
        self.global_size = rospy.get_param('~global_size', 100)  # cells
        self.resolution = rospy.get_param('~resolution', 0.05)  # meters
        self.global_frame = rospy.get_param('~global_frame', 'global_map')
        
        # TF listener
        self.tf_listener = TransformListener()
        rospy.sleep(0.5)  # Give TF time to stabilize
        
        # Storage
        self.maps = {}
        self.global_costmap = None
        self.publisher = rospy.Publisher(self.output_topic, OccupancyGrid, queue_size=1)
        
        # Initialize global costmap (centered at origin)
        self.init_global_costmap()
        
        # Subscribe to individual maps
        for robot_ns in self.robot_namespaces:
            map_topic = '/{}/map'.format(robot_ns)
            rospy.loginfo("Subscribing to %s", map_topic)
            rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, 
                           callback_args=robot_ns)
        
        # Timer to publish combined map
        rospy.Timer(rospy.Duration(0.1), self.publish_combined)
        
        rospy.loginfo("Costmap Combiner initialized")
        rospy.loginfo("Global size: %dx%d cells, Resolution: %.3f m/cell", 
                     self.global_size, self.global_size, self.resolution)
        rospy.loginfo("Publishing to: %s with frame: %s", self.output_topic, self.global_frame)

    def init_global_costmap(self):
        """Initialize fixed-size global costmap centered at origin"""
        self.global_costmap = OccupancyGrid()
        self.global_costmap.header.frame_id = self.global_frame
        self.global_costmap.info.resolution = self.resolution
        self.global_costmap.info.width = self.global_size
        self.global_costmap.info.height = self.global_size
        
        # Origin at center of map
        self.global_costmap.info.origin.position.x = -(self.global_size * self.resolution) / 2.0
        self.global_costmap.info.origin.position.y = -(self.global_size * self.resolution) / 2.0
        self.global_costmap.info.origin.position.z = 0
        self.global_costmap.info.origin.orientation.w = 1.0
        
        # Initialize with unknown (-1)
        self.global_costmap.data = [-1] * (self.global_size * self.global_size)

    def map_callback(self, msg, robot_ns):
        """Store incoming map"""
        self.maps[robot_ns] = msg
        rospy.logdebug("Received map from %s, size: %dx%d", robot_ns, msg.info.width, msg.info.height)

    def get_transform(self, robot_ns):
        """Get transform from robot's map frame to global frame using TF"""
        try:
            robot_map_frame = '{}_map'.format(robot_ns)
            # Get transform from robot_map to global_map
            self.tf_listener.waitForTransform(self.global_frame, robot_map_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform(self.global_frame, robot_map_frame, rospy.Time(0))
            rospy.logdebug("Transform for %s: translation=(%.2f, %.2f)", robot_ns, trans[0], trans[1])
            return (trans[0], trans[1])
        except Exception as e:
            rospy.logwarn("Could not get transform for %s: %s", robot_ns, str(e))
            return (0.0, 0.0)

    def world_to_global_index(self, world_x, world_y):
        """Convert world coordinates to global costmap index"""
        # Relative to origin
        rel_x = world_x - self.global_costmap.info.origin.position.x
        rel_y = world_y - self.global_costmap.info.origin.position.y
        
        # To cell coordinates
        cell_x = int(rel_x / self.resolution)
        cell_y = int(rel_y / self.resolution)
        
        # Check bounds
        if (cell_x < 0 or cell_x >= self.global_size or
            cell_y < 0 or cell_y >= self.global_size):
            return None
        
        # To linear index
        return cell_y * self.global_size + cell_x

    def publish_combined(self, event):
        """Combine all maps into fixed global costmap"""
        if len(self.maps) == 0:
            return
        
        # Reset global costmap to unknown
        self.global_costmap.data = [-1] * (self.global_size * self.global_size)
        self.global_costmap.header.stamp = rospy.Time.now()
        
        # Layer each robot map
        for robot_ns in self.robot_namespaces:
            if robot_ns not in self.maps or self.maps[robot_ns] is None:
                rospy.logdebug("No map received from %s", robot_ns)
                continue
            
            robot_map = self.maps[robot_ns]
            
            # Get transform for this robot from TF
            transform_x, transform_y = self.get_transform(robot_ns)
            
            # Get the map's origin
            map_origin_x = robot_map.info.origin.position.x
            map_origin_y = robot_map.info.origin.position.y
            map_resolution = robot_map.info.resolution
            
            rospy.logdebug("Processing %s: map_origin=(%.2f, %.2f), tf_transform=(%.2f, %.2f)", 
                          robot_ns, map_origin_x, map_origin_y, transform_x, transform_y)
            
            # Layer each cell from robot's map into global costmap
            for cell_idx in range(len(robot_map.data)):
                if robot_map.data[cell_idx] < 0:  # Skip unknown
                    continue
                
                # Convert cell index to coordinates in robot's map
                cell_y = cell_idx // robot_map.info.width
                cell_x = cell_idx % robot_map.info.width
                
                # Convert to world coordinates relative to robot's map origin
                local_world_x = map_origin_x + cell_x * map_resolution
                local_world_y = map_origin_y + cell_y * map_resolution
                
                # Apply TF transform to get global coordinates
                global_world_x = local_world_x + transform_x
                global_world_y = local_world_y + transform_y
                
                # Convert to global costmap index
                global_idx = self.world_to_global_index(global_world_x, global_world_y)
                
                if global_idx is not None:
                    # Take maximum occupancy
                    current = self.global_costmap.data[global_idx]
                    if current < 0:
                        self.global_costmap.data[global_idx] = robot_map.data[cell_idx]
                    else:
                        self.global_costmap.data[global_idx] = max(current, robot_map.data[cell_idx])
        
        # Publish
        self.publisher.publish(self.global_costmap)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        combiner = FixedCostmapCombiner()
        combiner.run()
    except rospy.ROSInterruptException:
        pass