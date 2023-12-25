#ifndef LOCAL_CONTROLLER_HPP
#define LOCAL_CONTROLLER_HPP

#include <vector>
#include <array>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Define your Pose structure here if it's not already defined

class LocalController : public rclcpp::Node {
public:
    LocalController();

    std::vector<std::vector<double>> DWAcontroller();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    geometry_msgs::msg::Twist twist;

    // Member variables for parameters
    double v_min;
    double v_max;
    double w_min;
    double w_max;
    double v_a;
    double w_a;
    double v_resolution;
    double w_resolution;
    double robot_r;
    double dt;
    double predict_time;
    double alpha_goal_coef;
    double beta_velocity_coef;
    double gamma_obstacle_coef;
    std::vector<double> best_u;
    std::vector<std::vector<double>> best_Trajectory;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void getObstacles();
    void getPath(const nav_msgs::msg::Path& path_msg);
    void getGoal();
    double goalCost(const geometry_msgs::msg::Pose& pose, double goal_theta);
    double velocityCost(double v, double u);
    double obstacleCost(const std::vector<geometry_msgs::msg::Pose>& poses, const std::vector<geometry_msgs::msg::Pose>& obstacles);
    std::vector<double> vwRange(double v, double w);
    std::vector<double> motion(const std::vector<double>& x, const std::vector<double>& u);
    std::vector<std::vector<double>> Trajectory_Calculate(const std::vector<double>& x, const std::vector<double>& u);

};

#endif  // LOCAL_CONTROLLER_HPP