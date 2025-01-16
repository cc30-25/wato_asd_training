#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{
class CostmapCore {
public:
    
    explicit CostmapCore(const rclcpp::Logger& logger);

    
    void configureCostmap(
        double resolution,
        int width,
        int height,
        geometry_msgs::msg::Pose origin,
        double obstacle_inflation_radius
    );
void refreshCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laser_data) const;

   
    void expandObstacleArea(int origin_x, int origin_y) const;

    nav_msgs::msg::OccupancyGrid::SharedPtr fetchCostmapData() const;

private:
   
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    rclcpp::Logger logger_;

    double inflation_radius_;
    int inflation_cells_;
};

}
