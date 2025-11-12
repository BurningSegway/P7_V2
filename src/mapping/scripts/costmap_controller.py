#!/usr/bin/env python
"""
A* Path Planner with Inflation
Subscribes to occupancy grid and goal
Plans path using A* algorithm
Publishes path for visualization and velocity commands
Simple and robust approach
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
from scipy import ndimage

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
        self.robot_radius = rospy.get_param('~robot_radius', 1.0)  # meters
        self.inflation_radius = rospy.get_param('~inflation_radius', 1.0)  # meters
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 2.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 3.0)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.3)
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 1.0)  # meters
        
        # State
        self.goal = None
        self.costmap = None
        self.inflated_costmap = None
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
        
        rospy.loginfo("A* Planner initialized")
        rospy.loginfo("Robot radius: %.2f m, Inflation radius: %.2f m", 
                     self.robot_radius, self.inflation_radius)
        rospy.loginfo("Lookahead distance: %.2f m", self.lookahead_distance)

    def goal_callback(self, msg):
        """Callback for new goal"""
        self.goal = msg
        self.has_goal = True
        self.current_waypoint_idx = 0
        rospy.loginfo("New goal received: (%.2f, %.2f)", 
                     msg.pose.position.x, msg.pose.position.y)
        self.plan_path()

    def costmap_callback(self, msg):
        """Callback for costmap updates - store and inflate"""
        self.costmap = msg
        self.has_costmap = True
        self.inflate_costmap()

    def inflate_costmap(self):
        """Inflate obstacles by inflation radius"""
        if self.costmap is None:
            return
        
        width = self.costmap.info.width
        height = self.costmap.info.height
        resolution = self.costmap.info.resolution
        
        # Convert to 2D array
        data = np.array(self.costmap.data).reshape((height, width))
        
        # Calculate inflation radius in cells
        inflation_cells = int(self.inflation_radius / resolution)
        
        # Create binary obstacle map (occupied cells)
        obstacle_map = (data >= self.occupancy_threshold).astype(np.uint8)
        
        # Dilate obstacles (inflation)
        if inflation_cells > 0:
            structure = ndimage.generate_binary_structure(2, 2)  # 8-connected
            inflated = ndimage.binary_dilation(obstacle_map, 
                                              structure=structure, 
                                              iterations=inflation_cells)
            inflated = inflated.astype(np.int8)
        else:
            inflated = obstacle_map
        
        # Keep unknown cells (-1) as passable, mark inflated obstacles as 100
        self.inflated_costmap = np.where(data == -1, -1, 
                                         np.where(inflated > 0, 100, 0))

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

    def is_valid_cell(self, grid_x, grid_y, costmap_data, width, height):
        """Check if grid cell is valid and free"""
        # Check bounds
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
        
        # Check occupancy
        occupancy = costmap_data[grid_y, grid_x]
        
        # -1 = unknown, 0 = free, 100 = occupied
        # Treat unknown cells as valid for exploration
        if occupancy == -1:
            return True
        
        # Free cells are valid
        return occupancy < self.occupancy_threshold

    def get_neighbors(self, grid_x, grid_y, costmap_data, width, height):
        """Get valid neighboring cells (8-connected)"""
        neighbors = []
        
        # 8-connected grid
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = grid_x + dx, grid_y + dy
                
                if self.is_valid_cell(nx, ny, costmap_data, width, height):
                    # Cost is sqrt(2) for diagonal, 1 for cardinal
                    cost = math.sqrt(dx*dx + dy*dy)
                    neighbors.append((nx, ny, cost))
        
        return neighbors

    def heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def astar(self, start_x, start_y, goal_x, goal_y, costmap_data, width, height):
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
            for next_x, next_y, edge_cost in self.get_neighbors(current_x, current_y, 
                                                                costmap_data, width, height):
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
        if not self.has_goal or not self.has_costmap or self.inflated_costmap is None:
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
        
        height, width = self.inflated_costmap.shape
        
        rospy.loginfo("Planning from grid (%d, %d) to (%d, %d)", 
                     start_grid_x, start_grid_y, goal_grid_x, goal_grid_y)
        
        # Check if start and goal are valid
        if not self.is_valid_cell(start_grid_x, start_grid_y, self.inflated_costmap, 
                                  width, height):
            rospy.logerr("Start position is not valid!")
            return
        
        if not self.is_valid_cell(goal_grid_x, goal_grid_y, self.inflated_costmap, 
                                  width, height):
            rospy.logerr("Goal position is not valid!")
            return
        
        # Time the planning
        start_time = time.time()
        
        # Run A*
        grid_path = self.astar(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y,
                              self.inflated_costmap, width, height)
        
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

    def get_lookahead_waypoint(self, robot_x, robot_y):
        """Get waypoint at lookahead distance along the path"""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return None
        
        # Start from current waypoint
        for i in range(self.current_waypoint_idx, len(self.current_path.poses)):
            wx = self.current_path.poses[i].pose.position.x
            wy = self.current_path.poses[i].pose.position.y
            dist = math.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            
            # Return first waypoint at or beyond lookahead distance
            if dist >= self.lookahead_distance:
                return i, wx, wy
        
        # If no waypoint is far enough, return the last one
        last_idx = len(self.current_path.poses) - 1
        wx = self.current_path.poses[last_idx].pose.position.x
        wy = self.current_path.poses[last_idx].pose.position.y
        return last_idx, wx, wy

    def control_loop(self, event):
        """Main control loop - follow the planned path with lookahead"""
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

        # Get lookahead waypoint
        result = self.get_lookahead_waypoint(robot_x, robot_y)
        if result is None:
            return
        
        target_idx, target_x, target_y = result

        # Update current waypoint index (for tracking progress)
        # Find closest waypoint behind the lookahead point
        for i in range(self.current_waypoint_idx, min(target_idx + 1, len(self.current_path.poses))):
            wx = self.current_path.poses[i].pose.position.x
            wy = self.current_path.poses[i].pose.position.y
            dist = math.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            if dist < self.waypoint_tolerance:
                self.current_waypoint_idx = i + 1

        # Direction to lookahead target
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)

        # Angle error
        angle_error = angle_to_target - yaw

        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Distance to target
        dist_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        # Simple proportional control using tanh for smooth saturation
        # Reduce speed when angle error is large
        speed_factor = 1.0 - 0.5 * abs(angle_error) / math.pi
        linear_vel = self.max_linear_vel * math.tanh(dist_to_target / 2.0) * speed_factor
        angular_vel = self.max_angular_vel * math.tanh(angle_error)

        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

        rospy.logdebug("Robot: (%.2f, %.2f), Target[%d]: (%.2f, %.2f), Dist: %.2f, Angle: %.2f",
                      robot_x, robot_y, target_idx, target_x, target_y, 
                      dist_to_target, angle_error)

    def run(self):
        """Spin the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass