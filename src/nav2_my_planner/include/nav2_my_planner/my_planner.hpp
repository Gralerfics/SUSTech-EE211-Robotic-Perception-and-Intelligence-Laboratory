#ifndef NAV2_MY_PLANNER__MY_PLANNER_HPP_
#define NAV2_MY_PLANNER__MY_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_my_planner/astar_planner.hpp"

namespace nav2_my_planner {

class AStar: public nav2_core::GlobalPlanner {
public:
	AStar();
	~AStar();
	
	void configure(
		const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
		std::string name,
		std::shared_ptr<tf2_ros::Buffer> tf,
		std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
	) override;

	void cleanup() override;
	void activate() override;
	void deactivate() override;

	nav_msgs::msg::Path createPlan(
		const geometry_msgs::msg::PoseStamped & start,
		const geometry_msgs::msg::PoseStamped & goal
	) override;

protected:
	bool makePlan(
		const geometry_msgs::msg::Pose & start,
		const geometry_msgs::msg::Pose & goal, double tolerance,
		nav_msgs::msg::Path & plan
	);

	bool computePotential(const geometry_msgs::msg::Point & world_point);

	bool getPlanFromPotential(
		const geometry_msgs::msg::Pose & goal,
		nav_msgs::msg::Path & plan
	);

	bool getPlan(
		const geometry_msgs::msg::Pose & goal,
		nav_msgs::msg::Path & plan
	);

	void smoothApproachToGoal(
		const geometry_msgs::msg::Pose & goal,
		nav_msgs::msg::Path & plan
	);

  	double getPointPotential(const geometry_msgs::msg::Point & world_point);

	inline double squared_distance(
		const geometry_msgs::msg::Pose & p1,
		const geometry_msgs::msg::Pose & p2
	) {
		double dx = p1.position.x - p2.position.x;
		double dy = p1.position.y - p2.position.y;
		return dx * dx + dy * dy;
	}

	bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
	void mapToWorld(double mx, double my, double & wx, double & wy);

  	bool isPlannerOutOfDate();

	std::unique_ptr<AStarPlanner> planner_;

	std::shared_ptr<tf2_ros::Buffer> tf_;

	rclcpp::Clock::SharedPtr clock_;

	rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};

	nav2_costmap_2d::Costmap2D * costmap_;

 	std::string global_frame_, name_;

  	double tolerance_;

  	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
};

}

#endif