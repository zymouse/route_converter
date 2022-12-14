#pragma once

#include <string>
#include <vector>
#include <iostream>

// ros
#include <ros/ros.h>

// ROS msg 
#include "autoware_planning_msgs/Route.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_planning_msgs/TrajectoryPoint.h"
#include "autoware_lanelet2_msgs/MapBin.h"
#include "std_msgs/Int8.h"

// lanelet2 API
#include "lanelet2_extension/utility/message_conversion.h"
#include "lanelet2_extension/utility/query.h"
#include "lanelet2_extension/utility/utilities.h"

#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/LaneletMap.h"


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
    void onRoute(const std_msgs::Int8ConstPtr &msg);
    

    lanelet::ConstLanelet m_lanelet;

public:
    route_converter(/* args */);
};

}