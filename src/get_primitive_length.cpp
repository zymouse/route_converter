#include "get_primitive_length.hpp"

namespace route_converter
{
     bool PostPosition::operator == (const PostPosition & value) const
    {
            if(this->x==value.x && this->y==value.y && this->z==value.z)
            {
                return true;
            }else{
                return false;
            }
    }

    GetPrimitiveLength::GetPrimitiveLength(/* args */)
    {
        // ROS params
        {
            this->nh_.param<std::string>("/fleet/param/sub_map_topic", this->sub_map_topic_, "/map/vector_map");
            this->nh_.param<std::string>("/fleet/param/get_primitive_lenght_topic_name", this->get_primitive_lenght_topic_name_, "/fleet/service/getPrimitiveLenght");
        }

        // ROS service
        {
            this->get_primitive_lenght_servce_ = this->nh_.advertiseService(this->get_primitive_lenght_topic_name_, &GetPrimitiveLength::onGetPrimitiveLenght, this);
        }

        // ROS sublisher
        {   
            this->sub_map_ = this->nh_.subscribe(this->sub_map_topic_, 1, &GetPrimitiveLength::onLanelet2Map, this);
        }
    }
    
    GetPrimitiveLength::~GetPrimitiveLength()
    {
    }


    // ----------工具函数-------------------------


    bool GetPrimitiveLength::get_lanlet_centerLane_point_list(int8_t area_id)
    {
        // 获取车道对象
        if(this->m_global_lanelet_map_ptr_->laneletLayer.exists(area_id))
        {
            ROS_WARN("ID为%d的车道lanlet--不存在");
            return false;
        }
        this->point_xyz_list_.clear();
        PostPosition xyz;

        auto temp_lanelet = this->m_global_lanelet_map_ptr_->laneletLayer.get(area_id);
        for(const auto point : temp_lanelet.centerline3d().basicLineString())
        {
            xyz.x = point.x();
            xyz.y = point.y();
            xyz.z = point.z();
            this->point_xyz_list_.push_back(xyz);
        }
        return true;
    }

    bool GetPrimitiveLength::get_coverage_lane_point_list(int8_t area_id)
    {
        if(this->m_global_lanelet_map_ptr_->pointLayer.exists(area_id))
        {
            ROS_WARN("ID为%d的车道lanlet--不存在");
            return false;
        }
        this->point_xyz_list_.clear();
        PostPosition xyz;

        this->m_global_lanelet_map_ptr_->lineStringLayer.search()

        return true;
    }

    // ----------回调函数-------------------------
    void GetPrimitiveLength::onLanelet2Map(autoware_lanelet2_msgs::MapBin msg)
    {
        ROS_INFO("lanelet2 map sub success....");
        lanelet::LaneletMapPtr lanelet_map_ptr(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_ptr);
        this->m_global_lanelet_map_ptr_ = lanelet_map_ptr;
        this->is_map_loaded_ = true;
    }

    bool GetPrimitiveLength::onGetPrimitiveLenght(route_converter::GetPrimitiveLength_srvRequest& req, route_converter::GetPrimitiveLength_srvResponse& resp)
    {
        // 如果map地图数据获取成功
        if(!this->is_map_loaded_)
        {
            ROS_ERROR("地图没有打开，请启动自动驾驶系统后在执行任务");
            return false;
        }



    }

}