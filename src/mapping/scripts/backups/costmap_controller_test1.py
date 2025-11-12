#!/usr/bin/env python
"""
A* planner + simple waypoint follower using nav_msgs/OccupancyGrid.

- Subscribes to:
    - move_base_simple/goal (PoseStamped)
    - <~costmap_topic> (nav_msgs/OccupancyGrid)
- Publishes:
    - /robot/cmd_vel (geometry_msgs/Twist)
    - nav_msgs/Path (optional)
- Replans automatically when the map or goal updates.
- Logs total planning time.

Suitable as a baseline controller for integrating Control Barrier Functions (CBF).
"""

import rospy
import math
import numpy as np
import heapq
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion


# ---------- A* Planner Class ----------

class AStarPlanner:
    def __init__(self, occupancy_grid, occ_threshold=50, unknown_as_occupied=False, robot_radius=0.0):
        """
        occupancy_grid: nav_msgs/OccupancyGrid
        occ_threshold: cells >= occ_threshold are considered obstacles
        unknown_as_occupied: treat -1 as obstacle if True
        robot_radius: inflation radius (meters)
        """
        self.grid = occupancy_grid
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.resolution = occupancy_grid.info.resolution
        self.origin = occupancy_grid.info.origin
        self.data = np.array(occupancy_grid.data, dtype=np.int8).reshape((self.height, self.width))
        self.occ_threshold = occ_threshold
        self.unknown_as_occupied = unknown_as_occupied

        # Inflation
        self.inflation_cells = int(math.ceil(robot_radius / self.resolution)) if robot_radius > 0 else 0
        if self.inflation_cells > 0:
            self.obstacle_mask = self._compute_inflated_obstacles()
        else:
            self.obstacle_mask = self._compute_free_mask()

    def _compute_free_mask(self):
        mask = np.ones((self.height, self.width), dtype=np.bool_)
        mask[self.data >= self.occ_threshold] = False
        if self.unknown_as_occupied:
            mask[self.data == -1] = False
        return mask

    def _compute_inflated_obstacles(self):
        from scipy.ndimage import binary_dilation
        base_obstacle = np.zeros((self.height, self.width), dtype=np.bool_)
        base_obstacle[self.data >= self.occ_threshold] = True
        if self.unknown_as_occupied:
            base_obstacle[self.data == -1] = True
        structure = np.ones((2 * self.inflation_cells + 1, 2 * self.inflation_cells + 1), dtype=np.bool_)
        inflated = binary_dilation(base_obstacle, structure=structure)
        free_mask = np.logical_not(inflated)
        return free_mask

    def world_to_map(self, x, y):
        ox, oy = self.origin.position.x, self.origin.position.y
        ix = int(math.floor((x - ox) / self.resolution))
        iy = int(math.floor((y - oy) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return None
        return ix, iy

    def map_to_world(self, ix, iy):
        ox, oy = self.origin.position.x, self.origin.position.y
        x = ox + (ix + 0.5) * self.resolution
        y = oy + (iy + 0.5) * self.resolution
        return x, y

    def is_free(self, ix, iy):
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return False
        return self.obstacle_mask[iy, ix]

    def neighbors(self, ix, iy):
        nbrs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and self.is_free(nx, ny):
                    cost = math.hypot(dx, dy)
                    nbrs.append((nx, ny, cost))
        return nbrs

    def heuristic(self, a_ix, a_iy, b_ix, b_iy):
        dx = (a_ix - b_ix) * self.resolution
        dy = (a_iy - b_iy) * self.resolution
        return math.hypot(dx, dy)

    def plan(self, start_w, goal_w, max_expansions=200000):
        start_cell = self.world_to_map(*start_w)
        goal_cell = self.world_to_map(*goal_w)
        if start_cell is None or goal_cell is None:
            rospy.logwarn("Start or goal outside map bounds")
            return None
        sx, sy = start_cell
        gx, gy = goal_cell
        if not self.is_free(gx, gy):
            rospy.logwarn("Goal cell is occupied.")
            return None

        open_set = []
        g_score = np.full((self.height, self.width), np.inf, dtype=np.float32)
        f_score = np.full((self.height, self.width), np.inf, dtype=np.float32)
        came_from = {}
        visited = np.zeros((self.height, self.width), dtype=np.bool_)

        g_score[sy, sx] = 0.0
        f_score[sy, sx] = self.heuristic(sx, sy, gx, gy)
        heapq.heappush(open_set, (f_score[sy, sx], (sx, sy)))

        expansions = 0
        while open_set:
            _, (cx, cy) = heapq.heappop(open_set)
            if visited[cy, cx]:
                continue
            visited[cy, cx] = True
            expansions += 1
            if expansions > max_expansions:
                rospy.logwarn("A* exceeded max expansions.")
                break

            if (cx, cy) == (gx, gy):
                # reconstruct
                path_cells = [(cx, cy)]
                while (cx, cy) != (sx, sy):
                    cx, cy = came_from[(cx, cy)]
                    path_cells.append((cx, cy))
                path_cells.reverse()
                return [self.map_to_world(ix, iy) for ix, iy in path_cells]

            for nx, ny, move_cost in self.neighbors(cx, cy):
                tentative_g = g_score[cy, cx] + move_cost * self.resolution
                if tentative_g < g_score[ny, nx]:
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[ny, nx] = tentative_g
                    f = tentative_g + self.heuristic(nx, ny, gx, gy)
                    f_score[ny, nx] = f
                    heapq.heappush(open_set, (f, (nx, ny)))

        return None


# ---------- Controller Node ----------

class CostmapAStarController:
    def __init__(self):
        rospy.init_node('costmap_astar_controller')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters
        self.robot_frame = rospy.get_param('~robot_frame', 'robot_base_link')
        self.map_frame = rospy.get_param('~map_frame', 'robot_map')
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 1.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.25)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.1)
        self.replan_rate = rospy.get_param('~replan_rate', 0.5)
        self.control_rate = rospy.get_param('~control_rate', 10.0)
        self.occ_threshold = rospy.get_param('~occ_threshold', 50)
        self.unknown_as_occupied = rospy.get_param('~unknown_as_occupied', False)  
        self.robot_radius = rospy.get_param('~robot_radius', 0.2)
        self.costmap_topic = rospy.get_param('~costmap_topic', 'combined_costmap')
        self.path_topic = rospy.get_param('~path_topic', 'robot/path')

        # State
        self.goal = None
        self.occupancy = None
        self.has_goal = False
        self.has_map = False
        self.planner = None
        self.current_path = None
        self.current_waypoint_index = 0

        # ROS I/O
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.costmap_cb)
        self.cmd_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1) if self.path_topic else None

        rospy.Timer(rospy.Duration(1.0 / self.replan_rate), self.replan_timer_cb)
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop_cb)

        rospy.loginfo("Costmap A* Controller ready (unknown cells treated as free).")

    # ---------------------- Callbacks ----------------------

    def goal_cb(self, msg):
        goal = msg
        if msg.header.frame_id != self.map_frame:
            try:
                goal = self.tf_buffer.transform(msg, self.map_frame, rospy.Duration(0.5))
            except Exception as e:
                rospy.logwarn("Failed to transform goal: %s", e)
                return
        self.goal = goal
        self.has_goal = True
        self.current_path = None
        self.current_waypoint_index = 0
        rospy.loginfo("New goal: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y)

    def costmap_cb(self, msg):
        self.occupancy = msg
        self.has_map = True
        self.planner = None
        self.current_path = None
        self.current_waypoint_index = 0

    def replan_timer_cb(self, event):
        if not self.has_goal or not self.has_map:
            return

        if self.planner is None:
            self.planner = AStarPlanner(
                self.occupancy,
                occ_threshold=self.occ_threshold,
                unknown_as_occupied=self.unknown_as_occupied,
                robot_radius=self.robot_radius,
            )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.2))
        except Exception as e:
            rospy.logwarn_throttle(5, "TF lookup failed: %s", e)
            return

        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        start = (rx, ry)
        goal_xy = (self.goal.pose.position.x, self.goal.pose.position.y)

        # Always replan when map or goal updates
        rospy.loginfo("Planning path using A*...")
        start_time = rospy.get_time()
        path = self.planner.plan(start, goal_xy)
        end_time = rospy.get_time()
        plan_time = end_time - start_time

        if path is not None:
            rospy.loginfo("A* planning complete: %d waypoints in %.3f s", len(path), plan_time)
            self.current_path = path
            self.current_waypoint_index = 0
            self.publish_path(path)
        else:
            rospy.logwarn("A* failed to find a path (%.3f s)", plan_time)
            self.current_path = None
            self.current_waypoint_index = 0

    def publish_path(self, path):
        if not self.path_pub:
            return
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()
        for (x, y) in path:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def find_lookahead_point(self, robot_x, robot_y, lookahead_dist=0.5):
        if not self.current_path or len(self.current_path) == 0:
            return 0
        dists = [math.hypot(px - robot_x, py - robot_y) for px, py in self.current_path]
        for i, d in enumerate(dists):
            if d > lookahead_dist:
                return i
        return len(self.current_path) - 1

    def control_loop_cb(self, event):
        """Main control loop: follow current path safely"""
        if not self.has_goal or not self.has_map:
            return

        # Make sure planner exists
        if self.planner is None:
            rospy.logwarn_throttle(5, "Planner not ready yet")
            return

        # Get robot pose in map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0),
                rospy.Duration(0.1))
        except tf2_ros.TransformException as ex:
            rospy.logwarn_throttle(5, "Transform lookup failed: %s", ex)
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        q = transform.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_x, self.robot_y, self.robot_yaw = robot_x, robot_y, yaw

        # If path not available, try to plan
        if not self.current_path or len(self.current_path) == 0:
            try:
                start_time = rospy.get_time()
                self.current_path = self.planner.plan(
                    (robot_x, robot_y),
                    (self.goal.pose.position.x, self.goal.pose.position.y)
                )
                end_time = rospy.get_time()
                rospy.loginfo("A* planning took %.3f s, %d waypoints",
                            end_time - start_time,
                            len(self.current_path))
            except Exception as e:
                rospy.logerr("Planning failed: %s", e)
                return

        # Find lookahead waypoint
        target_idx = self.find_lookahead_point(robot_x, robot_y, lookahead_dist=0.5)
        target_x, target_y = self.current_path[target_idx]

        # Compute distance and angle to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist_to_target = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)

        # Angle error normalization
        angle_error = angle_to_target - yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Proportional control with threshold
        angle_threshold = math.radians(15)
        if abs(angle_error) > angle_threshold:
            linear_vel = 0.0  # rotate in place
        else:
            linear_vel = self.max_linear_vel * math.tanh(dist_to_target / 2.0)

        angular_vel = self.max_angular_vel * math.tanh(angle_error)

        # Stop if goal reached
        goal_dx = self.goal.pose.position.x - robot_x
        goal_dy = self.goal.pose.position.y - robot_y
        dist_to_goal = math.hypot(goal_dx, goal_dy)
        if dist_to_goal < self.goal_tolerance:
            rospy.loginfo("Goal reached!")
            self.cmd_pub.publish(Twist())  # stop
            self.has_goal = False
            self.current_path = []
            return

        # Publish velocity
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)


    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CostmapAStarController()
        node.run()
    except rospy.ROSInterruptException:
        pass
