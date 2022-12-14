#include "route_converter/get_planning_goal.hpp"

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

    bool QuaternionOrientation::operator == (const QuaternionOrientation & value) const
    {
            if(this->qx==value.qx && this->qy==value.qy && this->qz==value.qz && this->qw==value.qw)
            {
                return true;
            }else{
                return false;
            }
    }


    GetPlanningGoal::GetPlanningGoal(/* args */)
    {
            // init Member variable
            // node params
            {
                this->m_nh.param<std::string>("/fleet/param/m_sub_map_topic_param", this->m_sub_map_topic_param, "/map/vector_map");
                this->m_nh.param<std::string>("/fleet/param/m_ser_getMission_topic_param", this->m_ser_getMission_topic_param, "/fleet/service/getMission");
                this->m_nh.param<std::string>("/fleet/param/get_primitive_lenght_topic_name", this->get_primitive_lenght_topic_name_, "/fleet/service/getPrimitiveLenght");

            }
            // publisher
            {
                this->pub_mision_ = this->m_nh.advertise<geometry_msgs::PoseStamped>("/fleet/service/getMission_test", 1, true);
            }
            // subscriber
            {
                this->sub_map_ = this->m_nh.subscribe(this->m_sub_map_topic_param, 1, &GetPlanningGoal::onLanelet2Map, this);
                // this->sub_primitive_length_ = this->m_nh.subscribe(this->sub_primitive_length_topic_, 1, &GetPlanningGoal::onGetPrimitiveLength, this);
            }

            // service 
            {
                this->m_getMission_server = this->m_nh.advertiseService(this->m_ser_getMission_topic_param, &GetPlanningGoal::onGetMissionMsg, this);
                this->get_primitive_lenght_servce_ = this->m_nh.advertiseService(this->get_primitive_lenght_topic_name_, &GetPlanningGoal::onGetPrimitiveLenght, this);

            }

            sleep(3);
    }
   
    GetPlanningGoal::~GetPlanningGoal()
    {
    }

   // --------------------????????????------------------
    void GetPlanningGoal::calculationxyz(const PostPosition a, const PostPosition b, float k)
    {  
        this->m_mission_msg.goal.position.x = (b.x-a.x)*k + a.x;
        this->m_mission_msg.goal.position.y = (b.y-a.y)*k + a.y;
        this->m_mission_msg.goal.position.z = (b.z-a.z)*k + a.z;
    }
   
    bool GetPlanningGoal::get_position(const PostPosition left_point, const PostPosition rigth_point, float offset_dist, bool is_center)
    {   
        // left_point ??? rigth_point???????????????
        if(left_point==rigth_point){
            ROS_WARN("left_point ??? rigth_point???????????????");
            return false;
        }

        float points_dist = hypot(left_point.x-rigth_point.x, left_point.y-rigth_point.y);

        // ?????????????????????
        if(!is_center){

            // ?????????offset_dist?????????????????????
            if(offset_dist>0){offset_dist=points_dist-offset_dist;}
            // offset_dist<0 ?????????
            if(offset_dist<0){offset_dist=abs(offset_dist);}
            this->calculationxyz(left_point, rigth_point, offset_dist/points_dist);
            return true;

        }else{// ???????????????
            // ?????????????????????
            if(offset_dist==0){            
                this->calculationxyz(left_point, rigth_point, 1.0/2.0);
                return true;
            }
            offset_dist=points_dist/2 -offset_dist;
            this->calculationxyz(left_point, rigth_point, offset_dist/points_dist);
            return true;
        }
        return true;
    }
    
    bool GetPlanningGoal::from_parkingLine_position(const PostPosition start_point, const PostPosition end_point, float width)
    {
        // ????????????????????????
        if(start_point==end_point){
            ROS_WARN("????????????????????????%f-%f-%f", start_point.x, start_point.y, start_point.z);
            return false;
        }

        PostPosition c1;  // ??????
        c1.x = (start_point.x+end_point.x)/2;
        c1.y = (start_point.y+end_point.y)/2;
        c1.z = (start_point.z+end_point.z)/2;

        // z??????????????????
        tf::Vector3 start_V(0, 0, 1);
        tf::Vector3 end_v(
            (end_point.x-start_point.x),
            (end_point.y-start_point.y),
            (end_point.z-start_point.z)
        );

        tf::Vector3 v3 = tf::tfCross(start_V, end_v).normalized();   // ??????????????????

        PostPosition c2;  // c???

        c2.x = v3.x()*width + c1.x;
        c2.y = v3.y()*width + c1.y;
        c2.z = v3.z()*width + c1.z;
        
        this->m_mission_msg.goal.position.x = (c1.x+c2.x)/2;
        this->m_mission_msg.goal.position.y = (c1.y+c2.y)/2;
        this->m_mission_msg.goal.position.z = (c1.z+c2.z)/2;
        return true;
    }

    bool GetPlanningGoal::get_orientation(PostPosition start_point, PostPosition end_point)
    {   
        // ????????????????????????
        if(start_point==end_point){
            ROS_WARN("????????????????????????%f-%f-%f", start_point.x, start_point.y, start_point.z);
            return false;
        }

        // x??????????????????
        tf::Vector3 start_V(1, 0, 0);
        tf::Vector3 end_v(
            (end_point.x-start_point.x),
            (end_point.y-start_point.y),
            (end_point.z-start_point.z)
        );

        tf::Vector3 v3 = tf::tfCross(start_V, end_v).normalized();   // ??????????????????
        // ?????????????????????
        auto angle_v = tf::tfAngle(start_V, end_v);

        // ??????????????????
        if(angle_v==0){
            this->m_mission_msg.goal.orientation.x = 0;
            this->m_mission_msg.goal.orientation.y = 0;
            this->m_mission_msg.goal.orientation.z = 0;
            this->m_mission_msg.goal.orientation.w = 1;
            return true;
        }
        // ???????????????
        tf::Quaternion orientation;
        orientation.setRotation(v3, angle_v);
        this->m_mission_msg.goal.orientation.x = orientation.x();
        this->m_mission_msg.goal.orientation.y = orientation.y();
        this->m_mission_msg.goal.orientation.z = orientation.z();
        this->m_mission_msg.goal.orientation.w = orientation.w();
        return true;

    }

    int8_t GetPlanningGoal::is_offset_dist_limit(float offset_dist)
    {
        float limit_dist = (this->lanelet_start_dist+this->lanelet_end_dist)/2;
        if(abs(offset_dist)>limit_dist){
            ROS_WARN("??????????????????");
            return -3;
        }
        return 1;
    }


    void GetPlanningGoal::get_laneletbound_start_points()
    {
        // 1-->2----------1-->2
        // ?????????
        auto left_start_point =  this->m_lanelet_obj.leftBound3d().basicBegin();
        this->point_left_start_1.x = left_start_point->x();
        this->point_left_start_1.y = left_start_point->y();
        this->point_left_start_1.z = left_start_point->z();

        left_start_point++;
        this->point_left_start_2.x = left_start_point->x();
        this->point_left_start_2.y = left_start_point->y();
        this->point_left_start_2.z = left_start_point->z();

        // ?????????
        auto rigth_start_point =  this->m_lanelet_obj.rightBound3d().basicBegin();
        this->point_right_start_1.x = rigth_start_point->x();
        this->point_right_start_1.y = rigth_start_point->y();
        this->point_right_start_1.z = rigth_start_point->z();

        rigth_start_point++;
        this->point_right_start_2.x = rigth_start_point->x();
        this->point_right_start_2.y = rigth_start_point->y();
        this->point_right_start_2.z = rigth_start_point->z();
        this->lanelet_start_dist = hypot(this->point_left_start_1.x-this->point_right_start_1.x,\
                                           this->point_left_start_1.y-this->point_right_start_1.y);
        
    }

    void GetPlanningGoal::get_laneletbound_end_points()
    {
        // 1-->2----------1-->2
        // ?????????
        auto left_end_point =  this->m_lanelet_obj.leftBound3d().basicEnd();

        left_end_point--;
        this->point_left_end_2.x =  left_end_point->x();
        this->point_left_end_2.y =  left_end_point->y();
        this->point_left_end_2.z =  left_end_point->z();

        left_end_point--;
        this->point_left_end_1.x =  left_end_point->x();
        this->point_left_end_1.y =  left_end_point->y();
        this->point_left_end_1.z =  left_end_point->z();

        // ?????????
        auto rigth_end_point =  this->m_lanelet_obj.rightBound3d().basicEnd();

        rigth_end_point--;
        this->point_right_end_2.x = rigth_end_point->x();
        this->point_right_end_2.y = rigth_end_point->y();
        this->point_right_end_2.z = rigth_end_point->z();

        rigth_end_point--;
        this->point_right_end_1.x = rigth_end_point->x();
        this->point_right_end_1.y = rigth_end_point->y();
        this->point_right_end_1.z = rigth_end_point->z();

        this->lanelet_end_dist = hypot(this->point_left_end_2.x-this->point_right_end_2.x,\
                                        this->point_left_end_2.y-point_right_end_2.y);
    }


    void GetPlanningGoal::get_lingString_start_points()
    {
        // 1-->2----------1-->2
        auto start_point = this->m_lineString_obj.basicBegin();
        this->point_lineString_start_1.x = start_point->x();
        this->point_lineString_start_1.y = start_point->y();
        this->point_lineString_start_1.z = start_point->z();

        start_point++;
        this->point_lineString_start_2.x = start_point->x();
        this->point_lineString_start_2.y = start_point->y();
        this->point_lineString_start_2.z = start_point->z();
    }

    void GetPlanningGoal::get_lingString_end_points()
    {
        // 1-->2----------1-->2
        auto end_point = this->m_lineString_obj.basicEnd();

        end_point--;
        this->point_lineString_end_2.x = end_point->x();
        this->point_lineString_end_2.y = end_point->y();
        this->point_lineString_end_2.z = end_point->z();

        end_point--;
        this->point_lineString_end_1.x = end_point->x();
        this->point_lineString_end_1.y = end_point->y();
        this->point_lineString_end_1.z = end_point->z();
    }


    int8_t GetPlanningGoal::from_osm_two_points(int8_t region_id, int8_t sweeping_mode, float offset_dist, bool travelDirection)
    {
        // ??????map????????????????????????
        if(!this->is_map_loaded_)
        {
            ROS_ERROR("??????????????????????????????????????????????????????????????????");
            return -2;
        }

        // ID?????????points
        if(this->m_global_lanelet_map_ptr_->pointLayer.exists(region_id))
        {
            ROS_WARN("???????????????????????????????????????");
            return 0;
        }
        
        // ID?????????polygons
        if(this->m_global_lanelet_map_ptr_->polygonLayer.exists(region_id))
        {
            ROS_WARN("???????????????????????????????????????");
            return 0;
        }

        // lanelet ??????ID
        if(this->m_global_lanelet_map_ptr_->laneletLayer.exists(region_id)){
            // ??????????????? ?????? ????????? - ????????????????????????
            this->m_lanelet_obj = this->m_global_lanelet_map_ptr_->laneletLayer.get(region_id);

            // ??????????????? ?????? ????????????
            if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDriving ||\
            sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDrivingWithSweeping)
            {   
                int8_t ret = this->is_offset_dist_limit(offset_dist);
                // ??????????????????????????????
                if(!ret){return ret;}

                // ???????????? -- ???????????????
                if(travelDirection){
                    // ??????????????????
                    this->get_laneletbound_start_points();
                    if(!this->get_position(this->point_right_start_1, this->point_left_start_1, offset_dist, true))
                    {
                        return -3;
                    }
                    if(!this->get_orientation(this->point_left_start_2,this->point_left_start_1))
                    {
                        return -3;
                    }

                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDriving){
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDriving;
                    }

                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDrivingWithSweeping){
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDrivingWithSweeping;
                    }
                    
                    this->m_mission_msg.free_space_sweeping_mode = 0;

                    return ret;

                }else{// ???????????? -- ???????????????
                    // ??????????????????
                    this->get_laneletbound_end_points();
                    if(!this->get_position(this->point_left_end_2, this->point_right_end_2, offset_dist, true))
                    {
                        return -3;
                    }
                    if(!this->get_orientation(this->point_left_end_1,this->point_left_end_2))
                    {
                        return -3;
                    }

                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDriving){
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDriving;
                    }

                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDrivingWithSweeping){
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDrivingWithSweeping;
                    }

                    this->m_mission_msg.free_space_sweeping_mode = 0;

                    
                    return ret;
                }
            }

            // ???????????????
            if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDrivingWithLeftBoundarySweeping)
            {
                int8_t ret = this->is_offset_dist_limit(offset_dist);
                // ??????????????????????????????
                if(!ret || offset_dist>0){return ret;}
                if(offset_dist==0){offset_dist = -0.5;}

                // ??????????????????
                // ???????????? -- ???????????????
                if(travelDirection){
                    // ??????????????????
                    this->get_laneletbound_start_points();
                    if(!this->get_position(this->point_right_start_1, this->point_left_start_1, offset_dist, false))
                    {
                        return -3;
                    }
                    if(!this->get_orientation(this->point_left_start_2,this->point_left_start_1))
                    {
                        return -3;
                    }
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDrivingWithLeftBoundarySweeping;
                    this->m_mission_msg.free_space_sweeping_mode = 0;
                    return ret;

                }else{// ???????????? -- ???????????????
                    // ??????????????????
                    this->get_laneletbound_end_points();
                    if(!this->get_position(this->point_left_end_2, this->point_right_end_2, offset_dist, false))
                    {
                        return -3;
                    }
                    if(!this->get_orientation(this->point_left_end_1, this->point_left_end_2))
                    {
                        return -3;
                    }
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDrivingWithLeftBoundarySweeping;
                    this->m_mission_msg.free_space_sweeping_mode = 0;
                    return ret;
                }
                
            }

            // ???????????????
            if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::LaneDrivingWithRightBoundarySweeping)
            {
                int8_t ret = this->is_offset_dist_limit(offset_dist);
                // ??????????????????????????????
                if(!ret || offset_dist<0){return ret;}
                if(offset_dist==0){offset_dist = 0.5;}
                // ??????????????????

                // ???????????? -- ???????????????
                if(travelDirection){
                    // ??????????????????
                    this->get_laneletbound_start_points();
                    if(!this->get_position(this->point_right_start_1, this->point_left_start_1,  offset_dist, false))
                    {
                        return -3;
                    }
                    if(!this->get_orientation(this->point_right_start_2,this->point_right_start_1))
                    {
                        return -3;
                    }
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDrivingWithRightBoundarySweeping;
                    this->m_mission_msg.free_space_sweeping_mode = 0;
                    return ret;

                }else{// ???????????? -- ???????????????
                    // ??????????????????
                    this->get_laneletbound_end_points();
                    if(!this->get_position(this->point_left_end_2, this->point_right_end_2, offset_dist, false))
                    {
                        return -3;
                    }
                    if(!this->get_orientation(this->point_right_end_1, this->point_right_end_2))
                    {
                        return -3;
                    }
                    this->m_mission_msg.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDrivingWithRightBoundarySweeping;
                    this->m_mission_msg.free_space_sweeping_mode = 0;
                    return ret;
                }
            }
            ROS_INFO("lanelet2 ID sub success...."); 
            return 1;  
        }

        // ?????? lineString ID -- 
        if(this->m_global_lanelet_map_ptr_->lineStringLayer.exists(region_id)){
            // ?????????????????? -- ??????????????? -- ?????? ??????-????????? ??????--????????????
            this->m_lineString_obj = this->m_global_lanelet_map_ptr_->lineStringLayer.get(region_id);
            
            /*
            * lanelet oneway=yes/on
            * ???????????? Type=coverage_path
            * ????????????
            *           Type = parking_space 
            *           Subtype=pour_trash ?????????
            *           Subtype=reload     ??????/??????
            *           Subtype=solid      ??????
            *           width ??????
            * ???????????? false ???????????? true ???????????? 
            * false ???????????????????????????  true ???????????????????????????
            * false ??????????????? ture ???????????????
            */
            this->m_mission_msg.lane_driving_sweeping_mode=0;
            // 1.0 ????????????????????? goal??????
            if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::CoverageSweeping)
            {   
                this->m_mission_msg.free_space_sweeping_mode=autoware_planning_msgs::Mission::CoverageSweeping;

                if(this->m_lineString_obj.attribute(lanelet::AttributeName::Type)=="coverage_path_start" || this->m_lineString_obj.attribute(lanelet::AttributeName::Type)=="coverage_path_end"){
                    // ????????????????????????
                    if(travelDirection){
                        this->get_lingString_start_points();
                        // ?????????
                        if(!this->get_orientation(this->point_lineString_start_1, this->point_lineString_start_2))
                        {
                            return -3;
                        }
                        this->m_mission_msg.goal.position.x = this->point_lineString_start_1.x;
                        this->m_mission_msg.goal.position.y = this->point_lineString_start_1.y;
                        this->m_mission_msg.goal.position.z = this->point_lineString_start_1.z;
                        return 1;
                    }else{ //???????????????????????????
                        this->get_lingString_end_points();
                        // ?????????
                        if(!this->get_orientation(this->point_lineString_end_1, this->point_lineString_end_2))
                        {
                            return -3;
                        }
                        this->m_mission_msg.goal.position.x = (this->point_lineString_end_2.x + this->point_lineString_end_1.x)/2;
                        this->m_mission_msg.goal.position.y = (this->point_lineString_end_2.y + this->point_lineString_end_2.y)/2;
                        this->m_mission_msg.goal.position.z = (this->point_lineString_end_2.z + this->point_lineString_end_2.z/2);
                        return 1;
                    }
                    return 1;
                }else{
                    ROS_WARN("????????????,?????????????????????");
                    return 0;
                }

            }
            
            
            // 2.0 ?????????  goal?????? ??????width????????????????????????
            if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::SearchingSweeping|| \
               sweeping_mode==route_converter::GetPlanningGoal_srv::Request::Water || \
               sweeping_mode==route_converter::GetPlanningGoal_srv::Request::Charge || \
               sweeping_mode==route_converter::GetPlanningGoal_srv::Request::warehouse || \
               sweeping_mode==route_converter::GetPlanningGoal_srv::Request::DumpingTrash)
            {   // ???????????????--width??????
                if(this->m_lineString_obj.hasAttribute("width")){
                    this->get_lingString_start_points();

                    if(!this->get_orientation(this->point_lineString_start_1, this->point_lineString_start_2))
                    {
                        return -3;
                    }
                    if(!this->from_parkingLine_position(this->point_lineString_start_1, this->point_lineString_start_2, this->m_lineString_obj.attribute("width").asDouble().value()))
                    {
                        return -3;
                    } 

                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::SearchingSweeping)
                    {
                        this->m_mission_msg.free_space_sweeping_mode = autoware_planning_msgs::Mission::SearchingSweeping;
                    }
                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::Water|| \
                       sweeping_mode==route_converter::GetPlanningGoal_srv::Request::Charge
                       )
                    {
                        this->m_mission_msg.free_space_sweeping_mode = autoware_planning_msgs::Mission::Reloading;
                    }
                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::warehouse)
                    {
                        this->m_mission_msg.free_space_sweeping_mode = autoware_planning_msgs::Mission::NavigationMPC;
                    }
                    if(sweeping_mode==route_converter::GetPlanningGoal_srv::Request::DumpingTrash)
                    {
                        this->m_mission_msg.free_space_sweeping_mode = autoware_planning_msgs::Mission::DumpingTrash;
                    }

                    return 1;

                }else{
                    ROS_INFO("laneString?????????width??????"); 
                    return -4;
                }
                ROS_INFO("lanelet2 ID sub success...."); 
                return 1;  
            }else{
                return 0;
                ROS_WARN("????????????????????????");
            }
             
        }

        return -1;
    }

    // ------------??????????????????----------------------------------------
    bool GetPlanningGoal::point_in_polygon(const PostPosition &p, const std::vector<PostPosition> &ptPolygon)
    {
        int nCount = ptPolygon.size();
        //  number of cross point
        int nCross = 0;  
        for (int i = 0; i < nCount; i++)   
        {  
            PostPosition p1 = ptPolygon[i];  
            PostPosition p2 = ptPolygon[(i + 1) % nCount];

            if ( p1.y == p2.y )  
                continue;  
            if ( p.y < std::min(p1.y, p2.y) )  
                continue;  
            if ( p.y >= std::max(p1.y, p2.y) )  
                continue;  

            double x = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;  

            if ( x > p.x )  
            {  
                nCross++;  
            }  
        }  
        if ((nCross % 2) == 1)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    bool GetPlanningGoal::get_lanlet_centerLane_point_list(int8_t area_id)
    {
        // ??????????????????
        if(this->m_global_lanelet_map_ptr_->laneletLayer.exists(area_id))
        {
            ROS_WARN("ID???%d?????????lanlet--?????????");
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

    bool GetPlanningGoal::get_coverage_lane_point_list(int8_t area_id)
    {
        if(this->m_global_lanelet_map_ptr_->pointLayer.exists(area_id))
        {
            ROS_WARN("ID???%d?????????lanlet--?????????");
            return false;
        }
        this->point_xyz_list_.clear();
        PostPosition xyz;

        auto temp_laneStrings = this->m_global_lanelet_map_ptr_->lineStringLayer.get(area_id);
        for(auto temp_laneString=temp_laneStrings.begin(); temp_laneString!=temp_laneStrings.end(); temp_laneString++)
        {
            // for(){}
        }

        return true;
    }
   // --------------------????????????------------------
    void GetPlanningGoal::onLanelet2Map(autoware_lanelet2_msgs::MapBin msg)
    {
        ROS_INFO("lanelet2 map sub success....");
        lanelet::LaneletMapPtr lanelet_map_ptr(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_ptr);
        this->m_global_lanelet_map_ptr_ = lanelet_map_ptr;
        this->is_map_loaded_ = true;
    }

    bool GetPlanningGoal::onGetMissionMsg(route_converter::GetPlanningGoal_srv::Request& req,\
                        route_converter::GetPlanningGoal_srv::Response& resp)
    {
        if(req.two_point)
        {
            resp.status = 1;
            resp.current_mission.lane_driving_sweeping_mode = autoware_planning_msgs::Mission::LaneDriving;
            resp.current_mission.free_space_sweeping_mode = autoware_planning_msgs::Mission::Empty;
            resp.current_mission.mission_order = req.missoin_order;


            route_converter::PostPosition start_pose;
            route_converter::PostPosition end_pose;
            
            start_pose.x = req.start_point.x;
            start_pose.y = req.start_point.y;
            start_pose.z = req.start_point.z;

            end_pose.x = req.end_point.x;
            end_pose.y = req.end_point.y;
            end_pose.z = req.end_point.z;

            this->get_position(start_pose, end_pose, 0, true);
            resp.current_mission.goal.position.x =  this->m_mission_msg.goal.position.x;
            resp.current_mission.goal.position.y =  this->m_mission_msg.goal.position.y;
            resp.current_mission.goal.position.z =  this->m_mission_msg.goal.position.z;

            this->get_orientation(start_pose, end_pose);
            resp.current_mission.goal.orientation.x = this->m_mission_msg.goal.orientation.x;
            resp.current_mission.goal.orientation.y = this->m_mission_msg.goal.orientation.y;
            resp.current_mission.goal.orientation.z = this->m_mission_msg.goal.orientation.z;
            resp.current_mission.goal.orientation.w = this->m_mission_msg.goal.orientation.w;

            this->pub_mision_.publish(this->pub_mision_msg);
            return true;
        }


        ROS_INFO("????????????????????????id: %d", req.region_id);
        int ret = this->from_osm_two_points(req.region_id, req.sweeping_mode, req.offset_dist, req.travelDirection);
        this->m_mission_msg.mission_order = req.missoin_order;

        ROS_INFO("??????????????????Goal??????");
        resp.current_mission = this->m_mission_msg;
        resp.status = ret;
        this->pub_mision_msg.header.frame_id="map";
        this->pub_mision_msg.header.stamp = ros::Time::now();
        this->pub_mision_msg.pose=this->m_mission_msg.goal;
        this->pub_mision_.publish(this->pub_mision_msg);
        return true;
        
    }

    bool GetPlanningGoal::onGetPrimitiveLenght(route_converter::GetPrimitiveLength_srvRequest& req,\
                              route_converter::GetPrimitiveLength_srvResponse& resp)
    {
        
    }
   
}
