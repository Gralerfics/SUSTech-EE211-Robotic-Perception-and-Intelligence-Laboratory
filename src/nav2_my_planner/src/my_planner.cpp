#include "nav2_my_planner/my_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_my_planner/astar_planner.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_my_planner {

AStar::AStar(): tf_(nullptr), costmap_(nullptr) {
	RCLCPP_INFO(logger_, "AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar, AStar ~");
}

AStar::~AStar() {
  	RCLCPP_INFO(logger_, "Destroying plugin %s of type AStar", name_.c_str());
}

void AStar::configure(
	const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
	std::string name,
	std::shared_ptr<tf2_ros::Buffer> tf,
	std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
	tf_ = tf;
	name_ = name;
	costmap_ = costmap_ros->getCostmap();
	global_frame_ = costmap_ros->getGlobalFrameID();

	node_ = parent;
	auto node = parent.lock();
	clock_ = node->get_clock();
	logger_ = node->get_logger();

	RCLCPP_INFO(logger_, "Configuring plugin %s of type AStar", name_.c_str());

	nav2_util::declare_parameter_if_not_declared(node, name_ + ".tolerance", rclcpp::ParameterValue(0.3));
	node->get_parameter(name_ + ".tolerance", tolerance_);

	planner_ = std::make_unique<AStarPlanner>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
}

void AStar::activate() {
	RCLCPP_INFO(logger_, "Activating plugin %s of type AStar.", name_.c_str());
}

void AStar::deactivate() {
	RCLCPP_INFO(logger_, "Deactivating plugin %s of type AStar.", name_.c_str());
}

void AStar::cleanup() {
	RCLCPP_INFO(logger_, "Cleaning up plugin %s of type AStar.", name_.c_str());
	planner_.reset();
}

nav_msgs::msg::Path AStar::createPlan(
	const geometry_msgs::msg::PoseStamped & start,
	const geometry_msgs::msg::PoseStamped & goal
) {	
	unsigned int mx_start, my_start, mx_goal, my_goal;
	if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
		RCLCPP_ERROR(logger_, "The start position (%f, %f) is off the map.", start.pose.position.x, start.pose.position.y);
	}
	if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
		RCLCPP_ERROR(logger_, "The goal position (%f, %f) is off the map.", goal.pose.position.x, goal.pose.position.y);
	}
	if (tolerance_ == 0 && costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
		RCLCPP_ERROR(logger_, "The goal (%f, %f) was in a lethal cost.", goal.pose.position.x, goal.pose.position.y);
	}

	if (isPlannerOutOfDate()) planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

	nav_msgs::msg::Path path;

	if (start.pose.position.x == goal.pose.position.x && start.pose.position.y == goal.pose.position.y) {
		path.header.stamp = clock_->now();
		path.header.frame_id = global_frame_;
		geometry_msgs::msg::PoseStamped pose;
		pose.header = path.header;
		pose.pose.position.z = 0.0;
		pose.pose = start.pose;
		if (start.pose.orientation != goal.pose.orientation && !false) {
			pose.pose.orientation = goal.pose.orientation;
		}
		path.poses.push_back(pose);
		return path;
	}

	if (!makePlan(start.pose, goal.pose, tolerance_, path)) RCLCPP_ERROR(logger_, "Failed to create plan with tolerance of: %f", tolerance_);

	return path;
}

bool AStar::isPlannerOutOfDate() {
	return (!planner_.get() || planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) || planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()));
}

bool AStar::makePlan(
	const geometry_msgs::msg::Pose& start,
	const geometry_msgs::msg::Pose& goal,
	double tolerance,
	nav_msgs::msg::Path& plan
) {
	plan.poses.clear();
	plan.header.stamp = clock_->now();
	plan.header.frame_id = global_frame_;

	unsigned int mx, my;

	worldToMap(start.position.x, start.position.y, mx, my);
	int map_start[2];
	map_start[0] = mx;
	map_start[1] = my;

    planner_->setGoal(map_start);

	costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
	std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
	planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
	planner_->setCostmap(costmap_->getCharMap(), true, true);
	lock.unlock();

//	worldToMap(goal.position.x, goal.position.y, mx, my);
//	int map_goal[2];
//	map_goal[0] = mx;
//	map_goal[1] = my;
//
//	planner_->setStart(map_goal);
//	planner_->setGoal(map_start);
//	planner_->calcAStarPlanner();

    if (getPlan(goal, plan)) {
        smoothApproachToGoal(goal, plan);
    } else {
        RCLCPP_ERROR(logger_, "Failed to create a plan from potential when a legal potential was found. This shouldn't happen.");
    }

//	double resolution = costmap_->getResolution();
//	geometry_msgs::msg::Pose p, best_pose;
//
//	bool found_legal = false;
//
//	p = goal;
//	double potential = getPointPotential(p.position);
//	if (potential < POT_HIGH) {
//		// 已经到达
//		best_pose = p;
//		found_legal = true;
//	} else {
//		// Goal is not reachable. Trying to find nearest to the goal
//		// reachable point within its tolerance region
//		double best_sdist = std::numeric_limits<double>::max();
//
//		p.position.y = goal.position.y - tolerance;
//		while (p.position.y <= goal.position.y + tolerance) {
//			p.position.x = goal.position.x - tolerance;
//			while (p.position.x <= goal.position.x + tolerance) {
//				potential = getPointPotential(p.position);
//				double sdist = squared_distance(p, goal);
//				if (potential < POT_HIGH && sdist < best_sdist) {
//					best_sdist = sdist;
//					best_pose = p;
//					found_legal = true;
//				}
//				p.position.x += resolution;
//			}
//			p.position.y += resolution;
//		}
//	}

//	if (found_legal) {
//		// extract the plan
//		if (getPlanFromPotential(best_pose, plan)) {
//			smoothApproachToGoal(best_pose, plan);
//		} else {
//			RCLCPP_ERROR(logger_, "Failed to create a plan from potential when a legal potential was found. This shouldn't happen.");
//		}
//	}

	return !plan.poses.empty();
}

bool AStar::getPlan(
        const geometry_msgs::msg::Pose & goal,
        nav_msgs::msg::Path & plan
) {
    // clear the plan, just in case
    plan.poses.clear();

    // Goal should be in global frame
    double wx = goal.position.x;
    double wy = goal.position.y;

    // the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    worldToMap(wx, wy, mx, my);

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->setupAStarPlanner(true);

    const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ? (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

    int path_len = planner_->calcPath(max_cycles);
    if (path_len == 0) {
        return false;
    }

//    auto cost = planner_->getLastPathCost();
//    RCLCPP_DEBUG(logger_, "Path found, %d steps, %f cost\n", path_len, cost);

    RCLCPP_DEBUG(logger_, "Path found, %d steps\n", path_len);

    // extract the plan
    float * x = planner_->getPathX();
    float * y = planner_->getPathY();
    int len = planner_->getPathLen();

    for (int i = len - 1; i >= 0; --i) {
        // convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(x[i], y[i], world_x, world_y);

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.poses.push_back(pose);
    }

    return !plan.poses.empty();
}

void AStar::smoothApproachToGoal(
        const geometry_msgs::msg::Pose & goal,
        nav_msgs::msg::Path & plan
) {
    // Replace the last pose of the computed path if it's actually further away
    // to the second to last pose than the goal pose.
    if (plan.poses.size() >= 2) {
        auto second_to_last_pose = plan.poses.end()[-2];
        auto last_pose = plan.poses.back();
        if (squared_distance(last_pose.pose, second_to_last_pose.pose) > squared_distance(goal, second_to_last_pose.pose)) {
            plan.poses.back().pose = goal;
            return;
        }
    }
    geometry_msgs::msg::PoseStamped goal_copy;
    goal_copy.pose = goal;
    plan.poses.push_back(goal_copy);
}

//bool AStar::getPlanFromPotential(
//	const geometry_msgs::msg::Pose & goal,
//	nav_msgs::msg::Path & plan
//) {
//	// clear the plan, just in case
//	plan.poses.clear();
//
//	// Goal should be in global frame
//	double wx = goal.position.x;
//	double wy = goal.position.y;
//
//	// the potential has already been computed, so we won't update our copy of the costmap
//	unsigned int mx, my;
//	worldToMap(wx, wy, mx, my);
//
//	int map_goal[2];
//	map_goal[0] = mx;
//	map_goal[1] = my;
//
//	planner_->setStart(map_goal);
//
//	const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ? (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);
//
//	int path_len = planner_->calcPath(max_cycles);
//	if (path_len == 0) {
//		return false;
//	}
//
//	auto cost = planner_->getLastPathCost();
//	RCLCPP_DEBUG(logger_, "Path found, %d steps, %f cost\n", path_len, cost);
//
//	// extract the plan
//	float * x = planner_->getPathX();
//	float * y = planner_->getPathY();
//	int len = planner_->getPathLen();
//
//	for (int i = len - 1; i >= 0; --i) {
//		// convert the plan to world coordinates
//		double world_x, world_y;
//		mapToWorld(x[i], y[i], world_x, world_y);
//
//		geometry_msgs::msg::PoseStamped pose;
//		pose.pose.position.x = world_x;
//		pose.pose.position.y = world_y;
//		pose.pose.position.z = 0.0;
//		pose.pose.orientation.x = 0.0;
//		pose.pose.orientation.y = 0.0;
//		pose.pose.orientation.z = 0.0;
//		pose.pose.orientation.w = 1.0;
//		plan.poses.push_back(pose);
//	}
//
//	return !plan.poses.empty();
//}

//double AStar::getPointPotential(const geometry_msgs::msg::Point & world_point) {
//	unsigned int mx, my;
//	if (!worldToMap(world_point.x, world_point.y, mx, my)) {
//		return std::numeric_limits<double>::max();
//	}
//
//	unsigned int index = my * planner_->nx + mx;
//	return planner_->potarr[index];
//}

bool AStar::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
	if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) return false;

	mx = static_cast<int>(std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
	my = static_cast<int>(std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));
	if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) return true;

	RCLCPP_ERROR(logger_, "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my, costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
	return false;
}

void AStar::mapToWorld(double mx, double my, double& wx, double& wy) {
	wx = costmap_->getOriginX() + mx * costmap_->getResolution();
	wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_my_planner::AStar, nav2_core::GlobalPlanner)