#include "route_converter/route_converter_test.hpp"
namespace route_converter
{
route_converter::route_converter():nh_("~")
{
    ROS_INFO("route_converter start run ....");
    // node params
    {
        nh_.param<std::string>("sub_map_topic", sub_map_topic_, "lanelet2_map");
        nh_.param<std::string>("sub_route_topic", sub_route_topic_, "route");
        nh_.param<std::string>("pub_traj_topic", pub_traj_topic_, "trajectory");

        nh_.param<double>("lat_origin", lat_origin_, 0.0);
        nh_.param<double>("lon_origin", lon_origin_, 0.0);
        nh_.param<double>("alt_origin", alt_origin_, 0.0);
    }

    // subscriber
    {
        // /map/vector_map
        sub_map_ = nh_.subscribe("/map/vector_map", 1, &route_converter::onLanelet2Map, this);
        // /planning/mission_planning/route
        sub_route_ = nh_.subscribe("/fleet/osm_test/order", 1, &route_converter::onRoute, this);
    }
    
    // publisher
    {
        pub_traj_ = nh_.advertise<autoware_planning_msgs::Trajectory>(pub_traj_topic_, 1, false);
    }

    sleep(3);
}

void route_converter::reSubscriber()
{

}

void route_converter::onLanelet2Map(autoware_lanelet2_msgs::MapBin msg)
{
    ROS_INFO("lanelet2 map sub success....");
    lanelet::LaneletMapPtr lanelet_map_ptr(new lanelet::LaneletMap);
    lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_ptr);
    global_lanelet_map_ptr_ = lanelet_map_ptr;
    is_map_loaded_ = true;

}


void route_converter::onRoute(const std_msgs::Int8ConstPtr &msg)
{
    // 如果map地图数据获取成功
    if(!is_map_loaded_)
    {
        return;
    }


    if(global_lanelet_map_ptr_->polygonLayer.exists(msg->data)){/*没有处理内容*/}

    if(global_lanelet_map_ptr_->pointLayer.exists(msg->data)){/*暂时没有处理内容*/}

    if(global_lanelet_map_ptr_->laneletLayer.exists(msg->data)){
        // 取出左边界 中线 右边界 - 开始和结束两个点
        // lanelet::ConstLanelet temp_lanelet = global_lanelet_map_ptr_->laneletLayer.get(msg->data);
        this->m_lanelet = global_lanelet_map_ptr_->laneletLayer.get(msg->data);
        
        auto start_point =  this->m_lanelet.leftBound3d().basicBegin();

        {
            std::cout << "x=" << start_point->x() << std::endl;
            std::cout << "y=" << start_point->y() << std::endl;
            std::cout << "z=" << start_point->z() << std::endl;
            std::cout << "----------------------" << std::endl;
        }
        start_point++;
        {
            std::cout << "x=" << start_point->x() << std::endl;
            std::cout << "y=" << start_point->y() << std::endl;
            std::cout << "z=" << start_point->z() << std::endl;
            std::cout << "----------------------" << std::endl;
        }
        
        auto end_point =  this->m_lanelet.leftBound3d().basicEnd();
        end_point--;
        {
            std::cout << "x=" << end_point->x() << std::endl;
            std::cout << "y=" << end_point->y() << std::endl;
            std::cout << "z=" << end_point->z() << std::endl;
            std::cout << "----------------------" << std::endl;
        }
        end_point--;
        {
            std::cout << "x=" << end_point->x() << std::endl;
            std::cout << "y=" << end_point->y() << std::endl;
            std::cout << "z=" << end_point->z() << std::endl;
            std::cout << "----------------------" << std::endl;
        }
        // for(auto start_point =  temp_lanelet.leftBound3d().basicBegin(); start_point!=temp_lanelet.leftBound3d().basicEnd(); start_point++)
        // {
        //     std::cout << "x=" << start_point->x() << std::endl;
        //     std::cout << "y=" << start_point->y() << std::endl;
        //     std::cout << "z=" << start_point->z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }

        // for(auto start_point =  temp_lanelet.rightBound3d().basicBegin(); start_point!=temp_lanelet.rightBound3d().basicEnd(); start_point++)
        // {
        //     std::cout << "x=" << start_point->x() << std::endl;
        //     std::cout << "y=" << start_point->y() << std::endl;
        //     std::cout << "z=" << start_point->z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }
        // for(const auto point:temp_lanelet.leftBound3d().basicLineString()){
            
        //     std::cout << "x=" << point.x() << std::endl;
        //     std::cout << "y=" << point.y() << std::endl;
        //     std::cout << "z=" << point.z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }
        
        // for(const auto point:temp_lanelet.rightBound3d().basicLineString()){
            
        //     std::cout << "x=" << point.x() << std::endl;
        //     std::cout << "y=" << point.y() << std::endl;
        //     std::cout << "z=" << point.z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }

    

        // std::for_each(temp_lanelet.leftBound3d().basicLineString().begin(), temp_lanelet.leftBound3d().basicLineString().end(), [&](lanelet::Point3d point)
        // {
        // std::cout << "x=" << point.x() << std::endl;
        // std::cout << "y=" << point.y() << std::endl;
        // std::cout << "z=" << point.z() << std::endl;
        // std::cout << "id=" << point.id() << std::endl;
        // std::cout << "----------------------" << std::endl;
        // });

        
    }

    if(global_lanelet_map_ptr_->lineStringLayer.exists(msg->data)){
        // 判断特殊线段 -- 获取两个点 -- 出库 入库-倒垃圾 入库--充电注水
        lanelet::ConstLineString3d temp_lanelet = global_lanelet_map_ptr_->lineStringLayer.get(msg->data);
        auto start_point =  temp_lanelet.basicBegin();
        
        std::cout << "attribute-" << temp_lanelet.attribute(lanelet::AttributeName::Type) << std::endl;
        std::cout << "attribute-" << temp_lanelet.attribute(lanelet::AttributeName::Subtype) << std::endl;
        if(temp_lanelet.hasAttribute("width"))
        {
            double f1 = temp_lanelet.attribute("width").asDouble().value();
            std::cout << "attribute-width" << f1  << std::endl;
        }

        if(temp_lanelet.attribute(lanelet::AttributeName::Type)=="coverage_path"){
            std::cout << "这是遍历线段" << std::endl;
        }
        // {
        //     std::cout << "x=" << start_point->x() << std::endl;
        //     std::cout << "y=" << start_point->y() << std::endl;
        //     std::cout << "z=" << start_point->z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }
        // start_point++;
        // {
        //     std::cout << "x=" << start_point->x() << std::endl;
        //     std::cout << "y=" << start_point->y() << std::endl;
        //     std::cout << "z=" << start_point->z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }
        
        // auto end_point =  temp_lanelet.basicEnd();
        // end_point--;
        // {
        //     std::cout << "x=" << end_point->x() << std::endl;
        //     std::cout << "y=" << end_point->y() << std::endl;
        //     std::cout << "z=" << end_point->z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }
        // end_point--;
        // {
        //     std::cout << "x=" << end_point->x() << std::endl;
        //     std::cout << "y=" << end_point->y() << std::endl;
        //     std::cout << "z=" << end_point->z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // }
    }

    ROS_INFO("lanelet2 ID sub success....");
}

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "route_converter_test");
    route_converter::route_converter route_converter_node;
    ros::spin();

    return 0;
}