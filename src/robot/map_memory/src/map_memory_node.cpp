#include "map_memory_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {}

class MappingNode : public rclcpp::Node {
public:
    MappingNode()
        : Node("mapping_node"), last_x_(0.0), last_y_(0.0), distance_threshold_(1.5) {
        // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));

        // Initialize global map
        global_map_.info.resolution = 0.1; 
        global_map_.info.width = 100;      
        global_map_.info.height = 100;     
        global_map_.info.origin.position.x = 0.0;
        global_map_.info.origin.position.y = 0.0;
        global_map_.info.origin.position.z = 0.0;
        global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1); // Unknown cells
    }

private:
    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x_, last_y_;
    const double distance_threshold_;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
        if (distance >= distance_threshold_) {
            last_x_ = x;
            last_y_ = y;
            should_update_map_ = true;
        }
    }

    // Timer-based map update
    void updateMap() {
        if (should_update_map_ && costmap_updated_) {
            integrateCostmap();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
        }
    }

    // Integrate the latest costmap into the global map
    void integrateCostmap() {
        // Transform and merge 
        for (size_t y = 0; y < latest_costmap_.info.height; ++y) {
            for (size_t x = 0; x < latest_costmap_.info.width; ++x) {
                size_t index = y * latest_costmap_.info.width + x;

                if (latest_costmap_.data[index] != -1) { // Known cell
                    size_t global_index = transformToGlobalIndex(x, y);
                    global_map_.data[global_index] = latest_costmap_.data[index];
                }
            }
        }
    }

    // Transform local costmap coordinates to global map index
    size_t transformToGlobalIndex(size_t x, size_t y) {
      
        size_t global_x = x + static_cast<size_t>(global_map_.info.origin.position.x / global_map_.info.resolution);
        size_t global_y = y + static_cast<size_t>(global_map_.info.origin.position.y / global_map_.info.resolution);
        return global_y * global_map_.info.width + global_x;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
