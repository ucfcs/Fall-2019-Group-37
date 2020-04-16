#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include "environment_dimensions.h"

//Environment dimensions in metersS
#define X_DIMENSION 10
#define Y_DIMENSION 25 //Dimension for the internal competition
// #define Y_DIMENSION 50

//How close the rover has to be to reach the goal in meters
#define GOAL_ACCURACY_X 0.5
#define GOAL_ACCURACY_Y 0.5

#define MOVEMENT_UNIT_Y 2
#define MOVEMENT_UNIT_X 0.01 //For Prototype
// #define MOVEMENT_UNIT_X 0.5 //TODO update for more accurate overlap for rover

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner
{

GlobalPlanner::GlobalPlanner()
{
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
	initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{

	geometry_msgs::PoseStamped end_goal = start;
	end_goal.pose.position.x += X_DIMENSION;

	plan.push_back(start);

	while (!reachedGoal())
	{
		bool flip = false;
		for (double y = 0.0; y < Y_DIMENSION; y += MOVEMENT_UNIT_Y)
		{
			for (double x = 0.0; x < X_DIMENSION; x += MOVEMENT_UNIT_X)
			{
				geometry_msgs::PoseStamped new_goal = goal;
				tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

				if (flip)
				{
					new_goal.pose.position.x = start.pose.position.x + x;
					new_goal.pose.position.y = start.pose.position.y + y;
				}
				else
				{
					new_goal.pose.position.x = start.pose.position.x + x;
					new_goal.pose.position.y = start.pose.position.y + Y_DIMENSION - y;
				}

				new_goal.pose.orientation.x = goal_quat.x();
				new_goal.pose.orientation.y = goal_quat.y();
				plan.push_back(new_goal);
			}
			flip = !flip;
		}
		return true;
	}

	bool GlobalPlanner::reachedGoal(const geometry_msgs::PoseStamped &end_goal, const const geometry_msgs::PoseStamped &current)
	{
		if (std
			: abs(end_goal.pose.orientation.x - current.pose.orientation.x) <= GOAL_ACCURACY_X && std
			: abs(end_goal.pose.orientation.y - current.pose.orientation.y) <= GOAL_ACCURACY_Y)
			return true;
		return false;
	}
};