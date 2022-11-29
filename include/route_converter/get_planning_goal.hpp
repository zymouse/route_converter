#include "autoware_planning_msgs/Mission.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

namespace route_converter
{
    /*
    * @brief 根据服务器下发的线段ID，获取规划的mission
    * 1.0 根据点对position -- 计算orientation
    * 1.1 根据两组点对 -- 判断点对的同向还是异向
    * 2.0 根据区域Lanelet_ID -- 获取车道左边界 右边界 居中线--对于车道的开头两个点和最后两个点
    * 3.0 根据区域polygon_ID  -- 获取遍历清扫线的起点和终点 泊车线的两个点
    * 4.0 
    */
class GetPlanningGoal
{
private:
    autoware_planning_msgs::Mission m_mission_msg;  // 计算的结果值

    /**
     * @brief   通过两个3D点，计算
     * 
     *
     * @param start_point 
     * @param end_point 
     */
    void get_orientation(geometry_msgs::Point start_point, geometry_msgs::Point end_point);
public:
    GetPlanningGoal(/* args */);
    ~GetPlanningGoal();
};


}