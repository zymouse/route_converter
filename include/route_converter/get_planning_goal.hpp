#pragma once
#include "autoware_planning_msgs/Mission.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <string>
#include <vector>
#include <cmath>

// ros
#include <ros/ros.h>
#include "tf/tf.h"

// ros msg - autoware 
#include "autoware_planning_msgs/Route.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_planning_msgs/TrajectoryPoint.h"
#include "autoware_lanelet2_msgs/MapBin.h"

#include "std_msgs/Int8.h"

// lanelet API
#include "lanelet2_extension/utility/message_conversion.h"
#include "lanelet2_extension/utility/query.h"
#include "lanelet2_extension/utility/utilities.h"

// ros service 
#include "route_converter/GetPlanningGoal_srv.h"
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

struct QuaternionOrientation
{
   double  qx=0;
   double  qy=0;
   double  qz=0;
   double  qw=0;
   bool operator == (const QuaternionOrientation & value) const;

};

class GetPlanningGoal
{
private:
    ros::NodeHandle m_nh;

    // ROS publisher
    ros::Publisher pub_mision_;
    geometry_msgs::PoseStamped pub_mision_msg;


    // ROS sublisher
    std::string m_sub_map_topic_param;                  // 订阅的话题
    ros::Subscriber sub_map_;                           // 订阅地图
    void onLanelet2Map(autoware_lanelet2_msgs::MapBin msg);
    lanelet::LaneletMapPtr m_global_lanelet_map_ptr_;   // 存储地图队对象
    bool is_map_loaded_ = false;                        // 地图对象是否有存在

    /**
    *  输入lanelet ID 计算车道长度 或者 polygon ID 计算多边形区域遍历清扫长度
    *  发布者的方式发布出去
    *  
    */
    // std::string sub_primitive_length_topic_;                  // 订阅的话题名称
    // ros::Subscriber sub_primitive_length_;                    // lanelet原语 遍历线原语长度   
    // void onGetPrimitiveLength(std_msgs::Int8ConstPtr& msg);


    // 服务--计算合理的任务点
    std::string m_ser_getMission_topic_param;          // 服务的话题
    ros::ServiceServer m_getMission_server;            // 服务器
    bool onGetMissionMsg(route_converter::GetPlanningGoal_srv::Request& req,\
                        route_converter::GetPlanningGoal_srv::Response& resp);
    autoware_planning_msgs::Mission m_mission_msg;     // 计算的结果值

    // 计算--车道长度， 遍历清扫长度，指定地点所有lanelet车道长度
    std::string get_primitive_lenght_topic_name_;
    ros::ServiceServer get_primitive_lenght_servce_;
    bool onGetPrimitiveLenght(route_converter::GetPrimitiveLength_srvRequest& req,\
                              route_converter::GetPrimitiveLength_srvResponse& resp);

    // osm 原语对象 
    lanelet::ConstLanelet m_lanelet_obj;               // 车道对象
    lanelet::ConstLineString3d m_lineString_obj;       // 特殊线串对象--遍历线 泊车线 出库线 

    // 车道方向--排序 开头两个点
    PostPosition point_left_start_1;
    PostPosition point_left_start_2;
    PostPosition point_right_start_1;
    PostPosition point_right_start_2;
    double lanelet_start_dist=0;

    // 车道方向--排序 结尾两个点
    PostPosition point_left_end_1;
    PostPosition point_left_end_2;
    PostPosition point_right_end_1;
    PostPosition point_right_end_2;
    double lanelet_end_dist=0;

    // 特殊线串   开头两个点
    PostPosition point_lineString_start_1;
    PostPosition point_lineString_start_2;

    // 特殊线串   结尾两个点
    PostPosition point_lineString_end_1;
    PostPosition point_lineString_end_2;
    
    /*获取车道方向 左右边界 车道起点-两组点*/
    void get_laneletbound_start_points();
    /*获取车道方向 左右边界 车道终点-两组点*/
    void get_laneletbound_end_points();
    
    /*获取线串前-两组点*/
    void get_lingString_start_points();
    /*获取线串后-两组点*/
    void get_lingString_end_points();


private:  // 私有函数 -- 工具函数
    /**
     * @brief   a到b点 偏移offset_dist距离视为点c，求点c距离
     * @output  this->m_mission_msg.goal.position
     * @param   a 点a
     * @param   b 点b
     * @param   k           offset_dist/a到b距离
     * @return PostPosition 点c坐标
     */

    void  calculationxyz(const PostPosition a, const PostPosition b, float k);
    /**
     * @brief   通过两个3D点，计算(1,0,0)到向量(start_point-end_point)的orientation
     * @output this->m_mission_msg.goal.orientation
     *
     * @param start_point 
     * @param end_point 
     * @return false 是同一个点 true 计算成功
     */
    bool get_orientation(const PostPosition start_point, const PostPosition end_point);
    /**
     * @brief   根据车道两边的端点，获取想要的规划position--点c
     *                 ^
     *                 |
     *                 |
     *          a<-----c---------->b
     *          |                  |
     *          |                  |
     * @param   left_point  左边界端点
     * @param   rigth_point 右边界端点
     * @param   offset_dist 编译距离
     * @param   is_center 是否是左右边界
     * @return  bool  true 成功 flase 失败
     */
    bool get_position(const PostPosition left_point, const PostPosition rigth_point, float offset_dist, bool is_center);
    
    /**
     * @brief   根据两点线 叉乘 z轴 得出position--点c
     *                  |
     *                 c|--------->
     *                  |
     *          a>--------------->b
     * @param   start_point 开始点     
     * @param   end_point   结束点
     * @param   width       宽度
     * @return  bool true 成功 flase 失败
     */
    bool from_parkingLine_position(const PostPosition start_point, const PostPosition end_point, float width);

    /**
     * @brief 根据osm地图ID获取两点
     * 
     * 
     * @param region_id        OSM地图中的ID
     * @param sweeping_mode    清扫模式
     * @param offset_dist      偏移距离 不能大于车道1/2宽
     * @param travelDirection   是否逆行 false 正向行驶 true 逆向行驶
     *   1 居中无清扫 2 居中清扫  3 右贴边清扫 4 左贴边清扫
     *   5 遍历清扫   6 巡检清扫  7 注水  8 充电 9 出库 10 倒垃圾
     * @return 0 该区域号下-清扫模式不存在 -1 获取失败区域号不存在 -2 地图没有打开 -3 偏移距离大于车道的1/2宽度 1 获取成功 
     */
    int8_t from_osm_two_points(int8_t region_id, int8_t sweeping_mode, float offset_dist, bool travelDirection);
   
    /**
     * @brief   判断超过限制距离
     * @param limit_dist       限制距离 
     * @param offset_dist      偏移距离
     * @return   -3 偏移距离大于车道的1/2宽度 1 ok 
     */
    int8_t is_offset_dist_limit(float offset_dist);

private:  // 工具函数 -- 求原语长度
    std::vector<PostPosition> point_xyz_list_;        // 存储车道居中线所有点xyz 或者 某区域内所有遍历线点xyz 的列表

    bool point_in_polygon(const PostPosition& p , const std::vector<PostPosition> &ptPolygon);
    
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
    GetPlanningGoal(/* args */);
    ~GetPlanningGoal();

    /**
     * @brief   获取任务下发的mission
     * 
     * 
     * @param[in] region_id 区域号
     * @param[in] sweeping_mode 任务模式
     * @param offset_dist      偏移距离 不能大于车道1/2宽
     * @param travelDirection   是否逆行 false 正向行驶 true 逆向行驶
     * @return 获取的状态：0 该区域号下-清扫模式不存在 -3 偏移距离大于车道的1/2宽度 1 获取成功 -4 泊车线 原语 不正确 -1 获取失败区域号不存在 -2 地图没有打开
     */
    int8_t get_planning_mission(int8_t region_id, int8_t sweeping_mode, float offset_dist, bool travelDirection);
    
};
}