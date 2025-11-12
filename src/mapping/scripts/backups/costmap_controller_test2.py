#!/usr/bin/env python
"""
A* Path Planner
Subscribes to occupancy grid and goal
Plans path using A* algorithm
Publishes path for visualization
Publishes velocity commands to follow the path
Replans periodically on a timer
"""

import rospy
import math
import numpy as np
import time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
import heapq

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Parameters
        self.robot_frame = rospy.get_param('~robot_frame', 'robot_base_link')
        self.map_frame = rospy.get_param('~map_frame', 'robot_map')
        self.occupancy_threshold = rospy.get_param('~occupancy_threshold', 50)
        self.robot_radius = rospy.get_param('~robot_radius', 0.5)  # meters
        self.replan_interval = rospy.get_param('~replan_interval', 5.0)  # seconds
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 2.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 2.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.3)
        
        # State
        self.goal = None
        self.costmap = None
        self.has_goal = False
        self.has_costmap = False
        self.current_path = None
        self.current_waypoint_idx = 0
        
        # Subscribers
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('combined_costmap', OccupancyGrid, self.costmap_callback)
        
        # Publishers
        self.path_pub = rospy.Publisher('planned_path', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        
        # Control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        # Replanning loop
        self.replan_timer = rospy.Timer(rospy.Duration(self.replan_interval), 
                                       self.replan_callback)
        
        rospy.loginfo("A* Planner initialized")
        rospy.loginfo("Occupancy threshold: %d, Robot radius: %.2f m", 
                     self.occupancy_threshold, self.robot_radius)
        rospy.loginfo("Replan interval: %.1f seconds", self.replan_interval)

    def goal_callback(self, msg):
        """Callback for new goal"""
        self.goal = msg
        self.has_goal = True
        self.current_waypoint_idx = 0
        rospy.loginfo("New goal received: (%.2f, %.2f)", 
                     msg.pose.position.x, msg.pose.position.y)
        #self.plan_path()

    def costmap_callback(self, msg):
        """Callback for costmap updates - just store the map"""
        self.costmap = msg
        self.has_costmap = True

    def replan_callback(self, event):
        """Timer callback for periodic replanning"""
        if self.has_goal and self.has_costmap:
            rospy.loginfo("Replanning (periodic)...")
            self.plan_path()

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.costmap is None:
            return None, None
        
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.costmap is None:
            return None, None
        
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution
        
        x = origin_x + (grid_x + 0.5) * resolution
        y = origin_y + (grid_y + 0.5) * resolution
        
        return x, y

    def is_valid_cell(self, grid_x, grid_y):
        """Check if grid cell is valid and free"""
        if self.costmap is None:
            return False
        
        width = self.costmap.info.width
        height = self.costmap.info.height
        
        # Check bounds
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
        
        # Check occupancy
        index = grid_y * width + grid_x
        if index >= len(self.costmap.data):
            return False
        
        occupancy = self.costmap.data[index]
        
        # -1 = unknown, 0 = free, 100 = occupied
        # Treat unknown cells as valid for exploration
        if occupancy == -1:
            return True
        
        # Free cells are valid
        return occupancy < self.occupancy_threshold

    def get_neighbors(self, grid_x, grid_y):
        """Get valid neighboring cells (8-connected)"""
        neighbors = []
        
        # 8-connected grid
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = grid_x + dx, grid_y + dy
                
                if self.is_valid_cell(nx, ny):
                    # Cost is sqrt(2) for diagonal, 1 for cardinal
                    cost = math.sqrt(dx*dx + dy*dy)
                    neighbors.append((nx, ny, cost))
        
        return neighbors

    def heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def astar(self, start_x, start_y, goal_x, goal_y):
        """A* path planning algorithm"""
        # Priority queue: (f_score, g_score, x, y)
        open_set = []
        heapq.heappush(open_set, (0, 0, start_x, start_y))
        
        # Dictionaries for tracking
        came_from = {}
        g_score = {(start_x, start_y): 0}
        
        nodes_expanded = 0
        
        while open_set:
            _, current_g, current_x, current_y = heapq.heappop(open_set)
            nodes_expanded += 1
            
            # Check if goal reached
            if current_x == goal_x and current_y == goal_y:
                # Reconstruct path
                path = []
                current = (current_x, current_y)
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_x, start_y))
                path.reverse()
                
                rospy.loginfo("Path found! Nodes expanded: %d, Path length: %d", 
                            nodes_expanded, len(path))
                return path
            
            # Explore neighbors
            for next_x, next_y, edge_cost in self.get_neighbors(current_x, current_y):
                tentative_g = current_g + edge_cost
                
                if (next_x, next_y) not in g_score or tentative_g < g_score[(next_x, next_y)]:
                    came_from[(next_x, next_y)] = (current_x, current_y)
                    g_score[(next_x, next_y)] = tentative_g
                    f_score = tentative_g + self.heuristic(next_x, next_y, goal_x, goal_y)
                    heapq.heappush(open_set, (f_score, tentative_g, next_x, next_y))
        
        rospy.logwarn("No path found! Nodes expanded: %d", nodes_expanded)
        return None

    def plan_path(self):
        """Plan path from current robot position to goal"""
        if not self.has_goal or not self.has_costmap:
            rospy.logdebug("Cannot plan: missing goal or costmap")
            return
        
        # Get robot position from TF
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0), 
                rospy.Duration(0.1))
            start_x = transform.transform.translation.x
            start_y = transform.transform.translation.y
        except tf2_ros.TransformException as ex:
            rospy.logwarn_throttle(5, "Transform lookup failed, using (0,0): %s", ex)
            start_x = 0.0
            start_y = 0.0
        
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        
        # Convert to grid coordinates
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.world_to_grid(goal_x, goal_y)
        
        if start_grid_x is None or goal_grid_x is None:
            rospy.logerr("Failed to convert coordinates to grid")
            return
        
        rospy.loginfo("Planning from grid (%d, %d) to (%d, %d)", 
                     start_grid_x, start_grid_y, goal_grid_x, goal_grid_y)
        
        # Check if start and goal are valid
        if not self.is_valid_cell(start_grid_x, start_grid_y):
            rospy.logerr("Start position is not valid!")
            return
        
        if not self.is_valid_cell(goal_grid_x, goal_grid_y):
            rospy.logerr("Goal position is not valid!")
            return
        
        # Time the planning
        start_time = time.time()
        
        # Run A*
        grid_path = self.astar(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y)
        
        planning_time = time.time() - start_time
        rospy.loginfo("Planning time: %.4f seconds", planning_time)
        
        if grid_path is None:
            rospy.logerr("Path planning failed!")
            self.current_path = None
            return
        
        # Convert path to world coordinates and publish
        self.publish_path(grid_path)
        self.current_waypoint_idx = 0

    def publish_path(self, grid_path):
        """Convert grid path to Path message and publish"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame
        
        for grid_x, grid_y in grid_path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            
            if world_x is None:
                continue
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.current_path = path_msg
        
        rospy.loginfo("Published path with %d waypoints", len(path_msg.poses))

    def control_loop(self, event):
        """Main control loop - follow the planned path"""
        if not self.has_goal or self.current_path is None:
            return
        
        if len(self.current_path.poses) == 0:
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

        # Get robot's current orientation
        q = transform.transform.rotation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Final goal position
        final_goal_x = self.goal.pose.position.x
        final_goal_y = self.goal.pose.position.y
        dist_to_final_goal = math.sqrt((final_goal_x - robot_x)**2 + 
                                       (final_goal_y - robot_y)**2)

        # Check if final goal reached
        if dist_to_final_goal < self.goal_tolerance:
            rospy.loginfo("Final goal reached!")
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.has_goal = False
            self.current_path = None
            return

        # Get current waypoint
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.current_waypoint_idx = len(self.current_path.poses) - 1

        waypoint = self.current_path.poses[self.current_waypoint_idx]
        waypoint_x = waypoint.pose.position.x
        waypoint_y = waypoint.pose.position.y

        # Distance to current waypoint
        dist_to_waypoint = math.sqrt((waypoint_x - robot_x)**2 + 
                                     (waypoint_y - robot_y)**2)

        # If close enough to waypoint, advance to next
        if dist_to_waypoint < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.current_path.poses):
                #rospy.loginfo("Reached waypoint %d/%d", 
                #            self.current_waypoint_idx, 
                #            len(self.current_path.poses))
                waypoint = self.current_path.poses[self.current_waypoint_idx]
                waypoint_x = waypoint.pose.position.x
                waypoint_y = waypoint.pose.position.y
                dist_to_waypoint = math.sqrt((waypoint_x - robot_x)**2 + 
                                            (waypoint_y - robot_y)**2)

        # Direction to current waypoint
        angle_to_waypoint = math.atan2(waypoint_y - robot_y, waypoint_x - robot_x)

        # Angle error
        angle_error = angle_to_waypoint - yaw

        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Simple proportional control using tanh for smooth saturation
        linear_vel = self.max_linear_vel * math.tanh(dist_to_waypoint / 2.0)
        angular_vel = self.max_angular_vel * math.tanh(angle_error)

        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

        rospy.logdebug("Robot: (%.2f, %.2f), Waypoint: (%.2f, %.2f), Dist: %.2f",
                      robot_x, robot_y, waypoint_x, waypoint_y, dist_to_waypoint)

    def run(self):
        """Spin the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass


    """This is a good start. I think i would like inflation also, i see that i can put in the robot radius. Another thing i noticed on long paths, it takes some time to plan, and the robot will try to follow the path, but after replanning begins spinning around and never catches the path again, is it because it drives a bit and replans that it ends up far away from the first waypoint? Once replanned, can the first waypoint it navigates to be the closest, or what do you think is an appropriate approach. It is quite a lot of points to go through for planning, that each cell in the map is  a node on the graph. Is there a way to optimise that?"""