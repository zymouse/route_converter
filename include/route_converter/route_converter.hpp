#pragma once

#include <string>
#include <vector>
// ros
#include <ros/ros.h>

// autoware
#include "autoware_planning_msgs/Route.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_planning_msgs/TrajectoryPoint.h"
#include "autoware_lanelet2_msgs/MapBin.h"
#include "lanelet2_extension/utility/message_conversion.h"
#include "lanelet2_extension/utility/query.h"
#include "lanelet2_extension/utility/utilities.h"

namespace route_converter
{
class route_converter
{
private:
    std::string sub_map_topic_;
    std::string sub_route_topic_;
    std::string pub_traj_topic_;
    double lat_origin_;
    double lon_origin_;
    double alt_origin_;
    ros::NodeHandle nh_;

    ros::Subscriber sub_route_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_traj_;

    bool is_map_loaded_ = false;

    lanelet::LaneletMapPtr global_lanelet_map_ptr_;
    lanelet::ConstLanelets all_lanelets_;
    void reSubscriber();
    void onLanelet2Map(autoware_lanelet2_msgs::MapBin msg);
    void onRoute(const autoware_planning_msgs::RouteConstPtr &msg);
    
public:
    route_converter(/* args */);
};

}