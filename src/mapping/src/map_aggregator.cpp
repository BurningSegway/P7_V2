/*
 * Map Aggregator Node
 * Subscribes to multiple robot maps and publishes an aggregated map
 * Useful for multi-robot systems where move_base needs access to all maps
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <mutex>

class MapAggregator
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  std::string aggregated_map_topic_;
  std::string global_frame_;
  std::vector<std::string> robot_namespaces_;
  
  ros::Publisher aggregated_map_pub_;
  std::map<std::string, nav_msgs::OccupancyGridConstPtr> robot_maps_;
  std::map<std::string, ros::Subscriber> map_subscribers_;
  std::mutex maps_mutex_;
  
  ros::Timer publish_timer_;

public:
  MapAggregator() : tf_listener_(tf_buffer_), private_nh_("~")
  {
    // Get parameters
    private_nh_.param("aggregated_map_topic", aggregated_map_topic_, 
                     std::string("aggregated_map"));
    private_nh_.param("global_frame", global_frame_, std::string("robot_map"));
    
    // Get robot namespaces
    robot_namespaces_ = {"robot", "robot_b"};
    private_nh_.getParam("robot_namespaces", robot_namespaces_);
    
    ROS_INFO("Map Aggregator initialized");
    ROS_INFO("Global frame: %s", global_frame_.c_str());
    ROS_INFO("Output topic: %s", aggregated_map_topic_.c_str());
    
    // Publisher for aggregated map
    aggregated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
        aggregated_map_topic_, 1, true);
    
    // Subscribe to individual robot maps
    for (const auto& robot_ns : robot_namespaces_) {
      std::string map_topic = "/" + robot_ns + "/map";
      ROS_INFO("Subscribing to %s", map_topic.c_str());
      
      // Create a copy of robot_ns for the lambda
      std::string ns_copy = robot_ns;
      map_subscribers_[robot_ns] = nh_.subscribe<nav_msgs::OccupancyGrid>(
          map_topic, 10,
          [this, ns_copy](const nav_msgs::OccupancyGridConstPtr& msg) {
            this->mapCallback(msg, ns_copy);
          });
      
      ROS_INFO("Subscription created for %s", map_topic.c_str());
    }
    
    // Timer to publish aggregated map periodically
    publish_timer_ = nh_.createTimer(ros::Duration(0.5),
                                     &MapAggregator::publishAggregatedMap, this);
  }

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg,
                   const std::string& robot_ns)
  {
    ROS_INFO("Received map from %s: size %dx%d", robot_ns.c_str(), 
             msg->info.width, msg->info.height);
    std::lock_guard<std::mutex> lock(maps_mutex_);
    robot_maps_[robot_ns] = msg;
  }

  void publishAggregatedMap(const ros::TimerEvent& event)
  {
    std::lock_guard<std::mutex> lock(maps_mutex_);
    
    if (robot_maps_.empty()) {
      return;
    }

    // Use the first map as the base for dimensions
    auto base_map = robot_maps_.begin()->second;
    if (!base_map) {
      return;
    }

    // Create aggregated map with same properties as base map
    // Start with all unknown cells (-1)
    nav_msgs::OccupancyGrid aggregated;
    aggregated.header.frame_id = global_frame_;
    aggregated.header.stamp = ros::Time::now();
    aggregated.info = base_map->info;
    aggregated.data.resize(base_map->data.size(), -1);  // Initialize with unknown (-1)

    // Merge all maps
    for (auto it = robot_maps_.begin(); it != robot_maps_.end(); ++it) {
      auto current_map = it->second;
      if (!current_map) {
        continue;
      }

      // For the first map, no transform needed
      if (it == robot_maps_.begin()) {
        // Copy first map directly
        aggregated.data = current_map->data;
      } else {
        // Try to get transform from this map to base map
        geometry_msgs::TransformStamped transform;
        try {
          std::string current_frame = it->first + "_map";
          transform = tf_buffer_.lookupTransform(
              global_frame_, current_frame, ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
          ROS_WARN_THROTTLE(5, "Could not get transform for %s: %s",
                           it->first.c_str(), ex.what());
          continue;
        }

        // Merge this map into aggregated map
        mergeMapData(aggregated, current_map, transform);
      }
    }

    aggregated_map_pub_.publish(aggregated);
  }

  void mergeMapData(nav_msgs::OccupancyGrid& aggregated,
                    const nav_msgs::OccupancyGridConstPtr& other_map,
                    const geometry_msgs::TransformStamped& transform)
  {
    double offset_x = transform.transform.translation.x;
    double offset_y = transform.transform.translation.y;
    double resolution = aggregated.info.resolution;

    int offset_x_cells = static_cast<int>(offset_x / resolution);
    int offset_y_cells = static_cast<int>(offset_y / resolution);

    int aggregated_width = aggregated.info.width;
    int aggregated_height = aggregated.info.height;
    int other_width = other_map->info.width;
    int other_height = other_map->info.height;

    // Merge cells from other_map into aggregated map
    for (int y = 0; y < other_height; ++y) {
      for (int x = 0; x < other_width; ++x) {
        int other_idx = y * other_width + x;
        int8_t other_value = other_map->data[other_idx];

        // Skip unknown cells
        if (other_value < 0) {
          continue;
        }

        // Calculate position in aggregated map
        int agg_x = x + offset_x_cells;
        int agg_y = y + offset_y_cells;

        // Check if within bounds
        if (agg_x < 0 || agg_x >= aggregated_width ||
            agg_y < 0 || agg_y >= aggregated_height) {
          continue;
        }

        int agg_idx = agg_y * aggregated_width + agg_x;

        // Merge: take maximum occupancy value
        if (other_value > aggregated.data[agg_idx]) {
          aggregated.data[agg_idx] = other_value;
        }
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_aggregator");
  MapAggregator aggregator;
  ros::spin();
  return 0;
}