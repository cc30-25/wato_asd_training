#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {}

class BasicCostmapNode : public rclcpp::Node
{
public:
    BasicCostmapNode() : Node("basic_costmap_node")
    {
        // Subscriber to '/lidar' topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10, std::bind(&BasicCostmapNode::laserCallback, this, std::placeholders::_1));

        // Publisher to '/costmap' topic
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

        // Initialize 
        resolution_ = 0.1; // 0.1 meters per cell
        width_ = 100;     // 10 meters wide
        height_ = 100;    // 10 meters tall
        origin_x_ = -5.0; // Origin offset
        origin_y_ = -5.0;
        inflation_radius_ = 1.0; // 1 meter
        max_cost_ = 100;

        initializeCostmap();
    }

private:
    void initializeCostmap()
    {
        costmap_.resize(width_ * height_, 0); // Initialize all cells to 0 
    }

    void convertToGrid(double range, double angle, int &x_grid, int &y_grid)
    {
        // Convert polar coordinates to Cartesian coordinates
        double x = range * cos(angle);
        double y = range * sin(angle);

        // Transform to grid indices
        x_grid = static_cast<int>((x - origin_x_) / resolution_);
        y_grid = static_cast<int>((y - origin_y_) / resolution_);
    }

    void markObstacle(int x_grid, int y_grid)
    {
        if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_)
        {
            int index = y_grid * width_ + x_grid;
            costmap_[index] = max_cost_; // Mark as obstacle
        }
    }

    void inflateObstacles()
    {
        std::vector<int> inflated_costmap = costmap_;

        for (int y = 0; y < height_; ++y)
        {
            for (int x = 0; x < width_; ++x)
            {
                int index = y * width_ + x;
                if (costmap_[index] == max_cost_)
                {
                    for (int dy = -inflation_radius_ / resolution_; dy <= inflation_radius_ / resolution_; ++dy)
                    {
                        for (int dx = -inflation_radius_ / resolution_; dx <= inflation_radius_ / resolution_; ++dx)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
                            {
                                double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                                if (distance <= inflation_radius_)
                                {
                                    int n_index = ny * width_ + nx;
                                    int inflated_cost = static_cast<int>(max_cost_ * (1 - distance / inflation_radius_));
                                    inflated_costmap[n_index] = std::max(inflated_costmap[n_index], inflated_cost);
                                }
                            }
                        }
                    }
                }
            }
        }

        costmap_ = inflated_costmap;
    }

    void publishCostmap()
    {
        auto costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        costmap_msg->header.stamp = this->now();
        costmap_msg->header.frame_id = "map";

        costmap_msg->info.resolution = resolution_;
        costmap_msg->info.width = width_;
        costmap_msg->info.height = height_;
        costmap_msg->info.origin.position.x = origin_x_;
        costmap_msg->info.origin.position.y = origin_y_;
        costmap_msg->info.origin.position.z = 0.0;
        costmap_msg->info.origin.orientation.w = 1.0;

        costmap_msg->data = costmap_;

        costmap_publisher_->publish(*costmap_msg);
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        initializeCostmap();

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            double angle = scan->angle_min + i * scan->angle_increment;
            double range = scan->ranges[i];

            if (range < scan->range_max && range > scan->range_min)
            {
                int x_grid, y_grid;
                convertToGrid(range, angle, x_grid, y_grid);
                markObstacle(x_grid, y_grid);
            }
        }

        inflateObstacles();
        publishCostmap();
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

    std::vector<int8_t> costmap_;
    double resolution_;
    int width_, height_;
    double origin_x_, origin_y_;
    double inflation_radius_;
    int max_cost_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
