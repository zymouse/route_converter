#pragma once

// 内置库
#include <string>
#include <vector>

// ros
#include <ros/ros.h>


// lanelet API
#include "autoware_lanelet2_msgs/MapBin.h"
#include "lanelet2_extension/utility/message_conversion.h"
#include "lanelet2_extension/utility/query.h"
#include "lanelet2_extension/utility/utilities.h"

// ROS srv
#include "route_converter/GetPrimitiveLength_srv.h"


namespace route_converter
{
struct PostPosition
{
   double  x=0;
   double  y=0;
   double  z=0;
   bool operator == (const PostPosition & value) const;
};

class GetPrimitiveLength
{
private:   // ROS 
    ros::NodeHandle nh_;

    // ROS sublisher
    std::string sub_map_topic_;                  // 订阅的话题
    ros::Subscriber sub_map_;                           // 订阅地图
    void onLanelet2Map(autoware_lanelet2_msgs::MapBin msg);
    lanelet::LaneletMapPtr m_global_lanelet_map_ptr_;   // 存储地图队对象
    bool is_map_loaded_ = false;                        // 地图对象是否有存在

    // ROS service
    std::string get_primitive_lenght_topic_name_;
    ros::ServiceServer get_primitive_lenght_servce_;
    bool onGetPrimitiveLenght(route_converter::GetPrimitiveLength_srvRequest& req,\
                              route_converter::GetPrimitiveLength_srvResponse& resp);

private:  // 工具函数
    std::vector<PostPosition> point_xyz_list_;        // 存储车道居中线所有点xyz 或者 某区域内所有遍历线点xyz 的列表

    /**
     * @brief   获取车道居中线所有点
     * @output  this->point_xyz_list_
     * 
     * @param   area_id     车道ID
     * @return  bool  false 失败 true 成功
     */
    bool get_lanlet_centerLane_point_list(int8_t area_id);

    /**
     * @brief   描述-获取polygon区域的遍历线所有点
     * @output  this->point_xyz_list_
     * 
     * @param   area_id     polygonID
     * @return  bool        false 失败 true 成功
     */
    bool get_coverage_lane_point_list(int8_t area_id);

public:
    GetPrimitiveLength(/* args */);
    ~GetPrimitiveLength();


};
    
}