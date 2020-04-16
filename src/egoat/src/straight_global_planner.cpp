#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

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

    plan.push_back(start);
    for (int i = 0; i < 10; i++)
    {
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        new_goal.pose.position.y = start.pose.position.y + (0.5 * i);

        new_goal.pose.orientation.y = goal_quat.y();

        plan.push_back(new_goal);
    }
    plan.push_back(goal);
    return true;
}
}; // namespace global_planner