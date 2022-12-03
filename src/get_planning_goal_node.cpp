#include "route_converter/get_planning_goal.hpp"

int main(int argc, char ** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "get_planning_goal_node");
    route_converter::GetPlanningGoal get_planning_goal;
    ros::spin();

    return 0;
}