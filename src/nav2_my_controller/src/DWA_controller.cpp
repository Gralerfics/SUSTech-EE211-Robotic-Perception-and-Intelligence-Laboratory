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

class LocalController : public rclcpp::Node {
public:
    LocalController() : Node("dwa_controller") {
        costmap_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", 10,
            std::bind(&LocalController::costmapCallback, this, std::placeholders::_1));

        odom_sub = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
            std::bind(&LocalController::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        twist = geometry_msgs::msg::Twist();
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    geometry_msgs::msg::Twist twist;
    // Add member variables for parameters
    double v_min = -0.1;
    double v_max = 0.3;
    double w_min = -50 / 180.0 * M_PI;
    double w_max = 50 / 180.0 * M_PI;
    double v_a = 0.05;
    double w_a = 30 / 180.0 * M_PI;
    double v_resolution = 0.01;
    double w_resolution = 1 / 180.0 * M_PI;
    double robot_r = 0.2;
    double dt = 0.05;
    double predict_time = 1;
    double alpha_goal_coef = 1;
    double beta_velocity_coef = 1;
    double gamma_obstacle_coef = 1;
    std::vector<double> best_u = {0.0, 0.0};
    std::vector<std::vector<double>> best_Trajectory;
};

    // Add methods for subscribing to topics and initializing callbacks

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        localmap_width = msg->info.width;
        localmap_height = msg->info.height;
        localmap_resolution = msg->info.resolution;
        localmap_origin_x = msg->info.origin.position.x;
        localmap_origin_y = msg->info.origin.position.y;

        costmap = msg->data;
        getObstacles();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        angle = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        vx = msg->twist.twist.linear.x;
        vy = msg->twist.twist.linear.y;
        v = std::sqrt(vx * vx + vy * vy);
        w = msg->twist.twist.angular.z;

    void getObstacles() {
        for (int y = 0; y < localmap_height; ++y) {
            for (int x = 0; x < localmap_width; ++x) {
                int index = x + y * localmap_width;
                if (costmap[index] != 0) {   // Occupied cell
                    // Convert map index to world coordinates
                    double world_x = localmap_origin_x + (x + 0.5) * localmap_resolution;
                    double world_y = localmap_origin_y + (y + 0.5) * localmap_resolution;
                    obstacle_list.push_back(std::make_pair(world_x, world_y));
                }
            }
        }
    }

    void getPath(const nav_msgs::Path& path_msg) {
        for (auto pose : path_msg.poses) {
            PathPose path_pose;
            path_pose.x = pose.pose.position.x;
            path_pose.y = pose.pose.position.y;
            
            // 获取姿态的欧拉角
            tf2::Quaternion quaternion;
            tf2::fromMsg(pose.pose.orientation, quaternion);
            tf2::Matrix3x3 mat(quaternion);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            path_pose.theta = yaw; // 这里使用yaw角作为theta
            
            path.push_back(path_pose);
        }
        goal_count = 0;
        goal_numbers = path.size();
    }

    void getGoal() {
        if (goal_count < goal_numbers) {
            PathPose goal_pose = path[goal_count];
            // 使用 goal_pose.x, goal_pose.y, goal_pose.theta 即可获取目标位置和姿态
            // 这里的 goal_pose 是一个 PathPose 结构体，包含了 x、y、theta 三个字段
            goal_count++;
        }
    }

    double goalCost(const Pose& pose, double goal_theta) {
        return M_PI - goal_theta;
    }

    double velocityCost(double v, double u) {
        return v_max - u;
    }

    double obstacleCost(const std::vector<Pose>& poses, const std::vector<Pose>& obstacles) {
        double min_distance = std::numeric_limits<double>::infinity();
        for (const auto& pose : poses) {
            for (const auto& obstacle : obstacles) {
                double current_distance = std::sqrt(std::pow(pose.x - obstacle.x, 2) + std::pow(pose.y - obstacle.y, 2));
                if (current_distance <= robot_r) {
                    return std::numeric_limits<double>::infinity();
                }
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                }
            }
        }
        return 1.0 / min_distance;
    }

    std::vector<double> vwRange(double v, double w) {
        double v_min_actual = std::max(v_min, v - dt * v_a);
        double v_max_actual = std::max(v_max, v + dt * v_a);
        double w_min_actual = std::max(w_min, w - dt * w_a);
        double w_max_actual = std::max(w_max, w + dt * w_a);
        return {v_min_actual, v_max_actual, w_min_actual, w_max_actual};
    }

    std::vector<double> motion(const std::vector<double>& x, const std::vector<double>& u) {
        std::vector<double> new_x = x;
        new_x[0] += u[0] * dt * std::cos(x[2]);
        new_x[1] += u[0] * dt * std::sin(x[2]);
        new_x[2] += u[1] * dt;
        new_x[3] = u[0];
        new_x[4] = u[1];
        return new_x;
    }

    std::vector<std::vector<double>> Trajectory_Calculate(const std::vector<double>& x, const std::vector<double>& u) {
        std::vector<std::vector<double>> trajectory;
        trajectory.push_back(x);

        std::vector<double> x_new = x;
        double time = 0;
        while (time <= predict_time) {
            x_new = motion(x_new, u);
            trajectory.push_back(x_new);
            time += dt;
        }
        return trajectory;
    }

    std::vector<std::vector<double>> DWAcontroller() {
        std::vector<std::vector<double>> trajectory;
        std::vector<double> best_trajectory;
        double min_score = std::numeric_limits<double>::max();

        for (double v = v_min; v <= v_max; v += v_resolution) {
            for (double w = w_min; w <= w_max; w += w_resolution) {
                std::vector<double> u = {v, w};
                std::vector<std::vector<double>> Trajectory = Trajectory_Calculate(x, u);
                double goal_score = Goal_Cost(Trajectory);
                double vel_score = Velocity_Cost(u);
                double obs_score = Obstacle_Cost(Trajectory);

                double score = alpha_goal_coef * goal_score + beta_velocity_coef * vel_score + gamma_obstacle_coef * obs_score;
                if (score <= min_score) {
                    min_score = score;
                    best_u = u;
                    best_Trajectory = Trajectory;
                }
            }
        }
        return best_Trajectory;
    }

    // void goToGoal(const Path& path) {
    //     if (!path.empty()) {
    //         get_path(path);
    //         for (int i = 0; i < goal_numbers; ++i) {
    //             get_goal();
    //             DWAcontroller();
    //             if (!best_Trajectory.empty()) {
    //                 std::cout << "Heading to the " << i << " / " << goal_numbers << " goal! " << std::endl;
    //                 while (std::abs(x - goal_pos[0]) >= 0.05 || std::abs(y - goal_pos[1]) >= 0.05) {
    //                     // Perform control based on best_u values to navigate to the goal
    //                     // Use appropriate ROS functions to publish twist commands
    //                 }
    //             }
    //         }
    //     }
    // }
};
