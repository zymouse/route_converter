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

    if(global_lanelet_map_ptr_->polygonLayer.exists(msg->data))
    {
        auto temp_polygon = global_lanelet_map_ptr_->polygonLayer.get(msg->data);
        
        // auto temp_lineString = global_lanelet_map_ptr_->lineStringLayer.search(lanelet::BoundingBox2d(lanelet::BasicPoint2d(-6.2121, -5.1093), lanelet::BasicPoint2d(19.8427, -26.756)));
        
        // if(temp_lineString.empty())
        // {   
        //     std::cout << "该对象为空" << std::endl;
        // }

        // for(auto object1=temp_lineString.begin(); object1!=temp_lineString.end();object1)
        // {
            
        //     std::cout << "id=" << object1->id() << std::endl;
        // }

        // std::for_each(temp_polygon->, temp_lineString.end(), [&](lanelet::Point3d point)
        // {
        //     std::cout << "x=" << point.x() << std::endl;
        //     std::cout << "y=" << point.y() << std::endl;
        //     std::cout << "z=" << point.z() << std::endl;
        //     std::cout << "----------------------" << std::endl;
        // });  
        
        lanelet::Polygon2d  polygon2d_obj;
        std::for_each(temp_polygon.begin(), temp_polygon.end(), [&](lanelet::Point2d point)
        {
            std::cout << "x=" << point.x() << std::endl;
            std::cout << "y=" << point.y() << std::endl;
            std::cout << "----------------------" << std::endl;
            polygon2d_obj.push_back(point);
        });


        auto temp_laneString = global_lanelet_map_ptr_->lineStringLayer.search(lanelet::geometry::boundingBox2d(polygon2d_obj));

        if(temp_laneString.empty())
        {   
            std::cout << "该对象为空" << std::endl;
        }

        for(auto object1=temp_laneString.begin(); object1!=temp_laneString.end(); object1++)
        {
            
            std::cout << "id=" << object1->id() << std::endl;
        }

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