// lpa_star_planner.cpp
// Lifelong Planning A* implementation for dynamic replanning
// Fixed version with proper queue management and safety checks

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>

// Hash function for grid cells
struct CellHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

// Key comparison for LPA*
struct Key {
    double first;
    double second;
    
    Key(double f = 0, double s = 0) : first(f), second(s) {}
    
    bool operator<(const Key& other) const {
        if (std::abs(first - other.first) > 1e-5)
            return first < other.first;
        return second < other.second;
    }
    
    bool operator==(const Key& other) const {
        return std::abs(first - other.first) < 1e-5 && 
               std::abs(second - other.second) < 1e-5;
    }
};

// Priority queue element for LPA*
struct PQElement {
    Key key;
    std::pair<int, int> cell;
    
    bool operator>(const PQElement& other) const {
        return other.key < key;  // Min-heap
    }
};

class LPAStarPlanner {
public:
    LPAStarPlanner() : tf_listener_(tf_buffer_), has_goal_(false), has_costmap_(false) {
        ros::NodeHandle nh;
        ros::NodeHandle nh_private("~");
        
        // Parameters
        nh_private.param("robot_frame", robot_frame_, std::string("robot_base_link"));
        nh_private.param("map_frame", map_frame_, std::string("robot_map"));
        nh_private.param("occupancy_threshold", occupancy_threshold_, 50);
        nh_private.param("inflation_radius", inflation_radius_, 1.0);
        nh_private.param("max_linear_vel", max_linear_vel_, 2.5);
        nh_private.param("max_angular_vel", max_angular_vel_, 3.0);
        nh_private.param("goal_tolerance", goal_tolerance_, 0.2);
        nh_private.param("lookahead_distance", lookahead_distance_, 1.0);
        nh_private.param("max_planning_iterations", max_iterations_, 50000);
        
        // Subscribers
        goal_sub_ = nh.subscribe("move_base_simple/goal", 1, 
                                 &LPAStarPlanner::goalCallback, this);
        costmap_sub_ = nh.subscribe("combined_costmap", 1, 
                                    &LPAStarPlanner::costmapCallback, this);
        
        // Publishers
        path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1);
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
        
        // Control timer
        control_timer_ = nh.createTimer(ros::Duration(0.1), 
                                       &LPAStarPlanner::controlLoop, this);
        
        ROS_INFO("LPA* Planner initialized");
        ROS_INFO("Inflation radius: %.2f m, Lookahead: %.2f m", 
                 inflation_radius_, lookahead_distance_);
        ROS_INFO("Max planning iterations: %d", max_iterations_);
    }
    
private:
    // ROS
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber goal_sub_, costmap_sub_;
    ros::Publisher path_pub_, cmd_vel_pub_;
    ros::Timer control_timer_;
    
    // Parameters
    std::string robot_frame_, map_frame_;
    int occupancy_threshold_;
    int max_iterations_;
    double inflation_radius_;
    double max_linear_vel_, max_angular_vel_;
    double goal_tolerance_, lookahead_distance_;
    
    // State
    bool has_goal_, has_costmap_;
    geometry_msgs::PoseStamped goal_;
    nav_msgs::OccupancyGrid costmap_;
    std::vector<int8_t> inflated_costmap_;
    nav_msgs::Path current_path_;
    size_t current_waypoint_idx_;
    
    // LPA* data structures
    std::pair<int, int> start_cell_, goal_cell_;
    std::unordered_map<std::pair<int, int>, double, CellHash> g_values_;
    std::unordered_map<std::pair<int, int>, double, CellHash> rhs_values_;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_list_;
    std::unordered_map<std::pair<int, int>, Key, CellHash> keys_in_queue_;
    
    static constexpr double INF = std::numeric_limits<double>::infinity();
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_ = *msg;
        has_goal_ = true;
        current_waypoint_idx_ = 0;
        
        ROS_INFO("New goal: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
        
        if (has_costmap_) {
            planPath();
        }
    }
    
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        bool first_costmap = !has_costmap_;
        costmap_ = *msg;
        has_costmap_ = true;
        
        // Inflate costmap
        inflateCostmap();
        
        if (has_goal_) {
            if (first_costmap) {
                // First plan - initialize LPA*
                planPath();
            } else {
                // Replan - use LPA* incremental update
                replan();
            }
        }
    }
    
    void inflateCostmap() {
        int width = costmap_.info.width;
        int height = costmap_.info.height;
        double resolution = costmap_.info.resolution;
        
        inflated_costmap_.resize(width * height);
        
        int inflation_cells = static_cast<int>(inflation_radius_ / resolution);
        
        // First pass: mark occupied cells
        std::vector<bool> occupied(width * height, false);
        for (int i = 0; i < width * height; ++i) {
            occupied[i] = (costmap_.data[i] >= occupancy_threshold_);
        }
        
        // Second pass: inflate obstacles
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                
                if (costmap_.data[idx] == -1) {
                    inflated_costmap_[idx] = -1;
                    continue;
                }
                
                bool in_inflated_region = false;
                
                // Check if within inflation radius of any obstacle
                for (int dy = -inflation_cells; dy <= inflation_cells && !in_inflated_region; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                            continue;
                        
                        int nidx = ny * width + nx;
                        
                        if (occupied[nidx]) {
                            double dist = std::sqrt(dx * dx + dy * dy);
                            if (dist <= inflation_cells) {
                                in_inflated_region = true;
                                break;
                            }
                        }
                    }
                }
                
                inflated_costmap_[idx] = in_inflated_region ? 100 : 0;
            }
        }
    }
    
    std::pair<int, int> worldToGrid(double x, double y) const {
        double ox = costmap_.info.origin.position.x;
        double oy = costmap_.info.origin.position.y;
        double res = costmap_.info.resolution;
        
        int gx = static_cast<int>((x - ox) / res);
        int gy = static_cast<int>((y - oy) / res);
        
        return {gx, gy};
    }
    
    std::pair<double, double> gridToWorld(int gx, int gy) const {
        double ox = costmap_.info.origin.position.x;
        double oy = costmap_.info.origin.position.y;
        double res = costmap_.info.resolution;
        
        double x = ox + (gx + 0.5) * res;
        double y = oy + (gy + 0.5) * res;
        
        return {x, y};
    }
    
    bool isValidCell(int x, int y) const {
        int width = costmap_.info.width;
        int height = costmap_.info.height;
        
        if (x < 0 || x >= width || y < 0 || y >= height)
            return false;
        
        int idx = y * width + x;
        int8_t occupancy = inflated_costmap_[idx];
        
        // Unknown cells are valid
        if (occupancy == -1)
            return true;
        
        return occupancy < occupancy_threshold_;
    }
    
    std::vector<std::pair<int, int>> getNeighbors(int x, int y) const {
        std::vector<std::pair<int, int>> neighbors;
        
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0)
                    continue;
                
                int nx = x + dx;
                int ny = y + dy;
                
                if (isValidCell(nx, ny)) {
                    neighbors.push_back({nx, ny});
                }
            }
        }
        
        return neighbors;
    }
    
    double edgeCost(const std::pair<int, int>& from, const std::pair<int, int>& to) const {
        int dx = to.first - from.first;
        int dy = to.second - from.second;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double heuristic(const std::pair<int, int>& cell) const {
        int dx = goal_cell_.first - cell.first;
        int dy = goal_cell_.second - cell.second;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double getG(const std::pair<int, int>& cell) const {
        auto it = g_values_.find(cell);
        return (it != g_values_.end()) ? it->second : INF;
    }
    
    double getRHS(const std::pair<int, int>& cell) const {
        auto it = rhs_values_.find(cell);
        return (it != rhs_values_.end()) ? it->second : INF;
    }
    
    Key calculateKey(const std::pair<int, int>& cell) const {
        double g = getG(cell);
        double rhs = getRHS(cell);
        double min_val = std::min(g, rhs);
        
        return Key(min_val + heuristic(cell), min_val);
    }
    
    void updateVertex(const std::pair<int, int>& cell) {
        // If not goal, update rhs
        if (cell != goal_cell_) {
            double min_rhs = INF;
            
            for (const auto& neighbor : getNeighbors(cell.first, cell.second)) {
                double g_neighbor = getG(neighbor);
                double cost = edgeCost(cell, neighbor);
                min_rhs = std::min(min_rhs, g_neighbor + cost);
            }
            
            rhs_values_[cell] = min_rhs;
        }
        
        // Remove from queue if present
        keys_in_queue_.erase(cell);
        
        // If locally inconsistent, add to queue
        double g = getG(cell);
        double rhs = getRHS(cell);
        
        if (std::abs(g - rhs) > 1e-5) {
            Key key = calculateKey(cell);
            open_list_.push({key, cell});
            keys_in_queue_[cell] = key;
        }
    }
    
    void computeShortestPath() {
        int iterations = 0;
        
        while (!open_list_.empty() && iterations < max_iterations_) {
            // Get top element
            PQElement top = open_list_.top();
            open_list_.pop();
            
            // Check if this element is outdated
            auto it = keys_in_queue_.find(top.cell);
            if (it == keys_in_queue_.end()) {
                // This cell was removed, skip it
                continue;
            }
            
            // Check if key matches (element might be outdated)
            if (!(it->second == top.key)) {
                // Outdated element, skip
                continue;
            }
            
            // Remove from tracking map
            keys_in_queue_.erase(top.cell);
            
            Key top_key = top.key;
            Key start_key = calculateKey(start_cell_);
            
            double g_start = getG(start_cell_);
            double rhs_start = getRHS(start_cell_);
            
            // Termination condition
            if (!(top_key < start_key) && std::abs(g_start - rhs_start) < 1e-5) {
                break;
            }
            
            iterations++;
            
            if (iterations % 1000 == 0) {
                ROS_DEBUG("LPA* iteration %d, queue size: %zu", 
                         iterations, open_list_.size());
            }
            
            auto current = top.cell;
            double g = getG(current);
            double rhs = getRHS(current);
            
            if (g > rhs) {
                // Overconsistent
                g_values_[current] = rhs;
            } else {
                // Underconsistent or consistent
                g_values_[current] = INF;
                updateVertex(current);
            }
            
            // Update all neighbors
            for (const auto& neighbor : getNeighbors(current.first, current.second)) {
                updateVertex(neighbor);
            }
        }
        
        if (iterations >= max_iterations_) {
            ROS_WARN("LPA* reached max iterations (%d)!", max_iterations_);
        } else {
            ROS_INFO("LPA* converged in %d iterations", iterations);
        }
    }
    
    void initializeLPAStar() {
        g_values_.clear();
        rhs_values_.clear();
        keys_in_queue_.clear();
        
        // Clear priority queue
        while (!open_list_.empty())
            open_list_.pop();
        
        // Initialize goal
        rhs_values_[goal_cell_] = 0.0;
        Key key = calculateKey(goal_cell_);
        open_list_.push({key, goal_cell_});
        keys_in_queue_[goal_cell_] = key;
        
        ROS_INFO("LPA* initialized: goal at (%d, %d)", goal_cell_.first, goal_cell_.second);
    }
    
    void planPath() {
        if (!has_goal_ || !has_costmap_)
            return;
        
        // Get robot position
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_, robot_frame_, 
                                                  ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5, "Transform failed: %s", ex.what());
            return;
        }
        
        double start_x = transform.transform.translation.x;
        double start_y = transform.transform.translation.y;
        double goal_x = goal_.pose.position.x;
        double goal_y = goal_.pose.position.y;
        
        start_cell_ = worldToGrid(start_x, start_y);
        goal_cell_ = worldToGrid(goal_x, goal_y);
        
        if (!isValidCell(start_cell_.first, start_cell_.second)) {
            ROS_ERROR("Start position invalid at (%d, %d)!", 
                     start_cell_.first, start_cell_.second);
            return;
        }
        
        if (!isValidCell(goal_cell_.first, goal_cell_.second)) {
            ROS_ERROR("Goal position invalid at (%d, %d)!", 
                     goal_cell_.first, goal_cell_.second);
            return;
        }
        
        ROS_INFO("Planning from (%d, %d) to (%d, %d)", 
                 start_cell_.first, start_cell_.second,
                 goal_cell_.first, goal_cell_.second);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Initialize LPA*
        initializeLPAStar();
        
        ROS_INFO("Starting computeShortestPath...");
        
        // Compute path
        computeShortestPath();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        ROS_INFO("Initial planning time: %ld ms", duration.count());
        
        // Extract and publish path
        extractAndPublishPath();
    }
    
    void replan() {
        if (!has_goal_)
            return;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Update start position
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_, robot_frame_, 
                                                  ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            return;
        }
        
        double start_x = transform.transform.translation.x;
        double start_y = transform.transform.translation.y;
        start_cell_ = worldToGrid(start_x, start_y);
        
        // Recompute (LPA* will reuse previous results)
        computeShortestPath();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        ROS_INFO("Replanning time: %ld ms", duration.count());
        
        extractAndPublishPath();
    }
    
    void extractAndPublishPath() {
        // Check if path exists
        double g_start = getG(start_cell_);
        if (g_start >= INF - 1) {
            ROS_ERROR("No path found! g(start) = inf");
            current_path_.poses.clear();
            return;
        }
        
        // Extract path by following gradient
        std::vector<std::pair<int, int>> path;
        auto current = start_cell_;
        path.push_back(current);
        
        int max_path_length = 10000;
        
        while (current != goal_cell_ && path.size() < max_path_length) {
            auto neighbors = getNeighbors(current.first, current.second);
            
            if (neighbors.empty()) {
                ROS_ERROR("No neighbors available during path extraction!");
                break;
            }
            
            // Find neighbor with minimum g + edge cost
            std::pair<int, int> best_neighbor = neighbors[0];
            double best_cost = INF;
            
            for (const auto& neighbor : neighbors) {
                double g_neighbor = getG(neighbor);
                double cost = g_neighbor + edgeCost(current, neighbor);
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_neighbor = neighbor;
                }
            }
            
            if (best_cost >= INF - 1) {
                ROS_ERROR("Path extraction failed - no valid successor");
                break;
            }
            
            current = best_neighbor;
            path.push_back(current);
        }
        
        if (current != goal_cell_) {
            ROS_WARN("Path extraction stopped before reaching goal!");
        }
        
        // Convert to ROS path
        current_path_.header.stamp = ros::Time::now();
        current_path_.header.frame_id = map_frame_;
        current_path_.poses.clear();
        
        for (const auto& cell : path) {
            auto world_coords = gridToWorld(cell.first, cell.second);
            double wx = world_coords.first;
            double wy = world_coords.second;
            
            geometry_msgs::PoseStamped pose;
            pose.header = current_path_.header;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            current_path_.poses.push_back(pose);
        }
        
        path_pub_.publish(current_path_);
        current_waypoint_idx_ = 0;
        
        ROS_INFO("Published path with %zu waypoints", current_path_.poses.size());
    }
    
    void controlLoop(const ros::TimerEvent&) {
        if (!has_goal_ || current_path_.poses.empty())
            return;
        
        // Get robot pose
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_, robot_frame_, 
                                                  ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            return;
        }
        
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        
        tf::Quaternion q(transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Check if goal reached
        double dx = goal_.pose.position.x - robot_x;
        double dy = goal_.pose.position.y - robot_y;
        double dist_to_goal = std::sqrt(dx * dx + dy * dy);
        
        if (dist_to_goal < goal_tolerance_) {
            ROS_INFO("Goal reached!");
            geometry_msgs::Twist cmd;
            cmd_vel_pub_.publish(cmd);
            has_goal_ = false;
            return;
        }
        
        // Find lookahead point
        size_t target_idx = current_waypoint_idx_;
        double target_x, target_y;
        
        for (size_t i = current_waypoint_idx_; i < current_path_.poses.size(); ++i) {
            double wx = current_path_.poses[i].pose.position.x;
            double wy = current_path_.poses[i].pose.position.y;
            double dist = std::sqrt((wx - robot_x) * (wx - robot_x) + 
                                   (wy - robot_y) * (wy - robot_y));
            
            if (dist >= lookahead_distance_) {
                target_idx = i;
                target_x = wx;
                target_y = wy;
                break;
            }
            
            if (i == current_path_.poses.size() - 1) {
                target_idx = i;
                target_x = wx;
                target_y = wy;
            }
        }
        
        // Update waypoint index
        for (size_t i = current_waypoint_idx_; i <= target_idx && i < current_path_.poses.size(); ++i) {
            double wx = current_path_.poses[i].pose.position.x;
            double wy = current_path_.poses[i].pose.position.y;
            double dist = std::sqrt((wx - robot_x) * (wx - robot_x) + 
                                   (wy - robot_y) * (wy - robot_y));
            if (dist < 0.3) {
                current_waypoint_idx_ = i + 1;
            }
        }
        
        // Calculate control
        double angle_to_target = std::atan2(target_y - robot_y, target_x - robot_x);
        double angle_error = angle_to_target - yaw;
        
        // Normalize angle
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        
        double dist_to_target = std::sqrt((target_x - robot_x) * (target_x - robot_x) + 
                                         (target_y - robot_y) * (target_y - robot_y));
        
        double speed_factor = 1.0 - 0.5 * std::abs(angle_error) / M_PI;
        double linear_vel = max_linear_vel_ * std::tanh(dist_to_target / 2.0) * speed_factor;
        double angular_vel = max_angular_vel_ * std::tanh(angle_error);
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_vel;
        cmd.angular.z = angular_vel;
        cmd_vel_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lpa_star_planner");
    
    LPAStarPlanner planner;
    
    ros::spin();
    
    return 0;
}