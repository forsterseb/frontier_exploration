#include <frontier_exploration/bounded_explore_layer.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <frontier_msgs/msg/frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <frontier_exploration/frontier_search.hpp>
#include <frontier_exploration/geometry_tools.hpp>
#include <utility>



namespace frontier_exploration
{

    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;
    using nav2_costmap_2d::FREE_SPACE;
    using rcl_interfaces::msg::ParameterType;

    BoundedExploreLayer::BoundedExploreLayer(){}

    BoundedExploreLayer::~BoundedExploreLayer(){
        polygonService_.reset();
        frontierService_.reset();
        dyn_params_handler_.reset();
    }

    void BoundedExploreLayer::onInitialize(){

        configured_ = false;
        marked_ = false;

        declareParameter("explore_clear_space", rclcpp::ParameterValue(true));
        declareParameter("resize_to_boundary", rclcpp::ParameterValue(true));
        declareParameter("frontier_travel_point", rclcpp::ParameterValue(std::string("closest")));
        declareParameter("enabled", rclcpp::ParameterValue(true));

        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node bounded_explore_layer"};
        }

        node->get_parameter("explore_clear_space", explore_clear_space_);
        node->get_parameter("resize_to_boundary", resize_to_boundary_);
        node->get_parameter("frontier_travel_point", frontier_travel_point_);
        node->get_parameter("enabled", enabledLayer_);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Enable parameter is" << enabledLayer_);
        // TODO. Fix later
        explore_clear_space_ = false;
        resize_to_boundary_ = false;
        frontier_travel_point_ = "initial";
        enabledLayer_ = true;
        frontierDetectRadius_ = 2.0;

        if(explore_clear_space_){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        frontier_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("frontiers", custom_qos);
        frontier_cloud_pub->on_activate();
        polygonService_ = node->create_service<frontier_msgs::srv::UpdateBoundaryPolygon>(
            "update_boundary_polygon",
            std::bind(&BoundedExploreLayer::updateBoundaryPolygonService, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        frontierService_ = node->create_service<frontier_msgs::srv::GetNextFrontier>(
            "get_next_frontier",
            std::bind(&BoundedExploreLayer::getNextFrontierService, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &BoundedExploreLayer::dynamicParametersCallback,
            this, std::placeholders::_1));
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

        matchSize();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Bounded explore layer Initialized with enabled as: " << enabledLayer_);
    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }

    rcl_interfaces::msg::SetParametersResult BoundedExploreLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    rcl_interfaces::msg::SetParametersResult result;

    for (auto parameter : parameters) {
        const auto & param_type = parameter.get_type();
        const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_BOOL) {
        if (param_name == "explore_clear_space") {
            explore_clear_space_ = parameter.as_bool();
        } else if (param_name == "resize_to_boundary") {
            resize_to_boundary_ = parameter.as_bool();
        } else if (param_name == "enabled") {
            enabledLayer_ = parameter.as_bool();
        }
    }
    
    else if(param_type == ParameterType::PARAMETER_STRING) {
        if(param_name == "frontier_travel_point") {
            frontier_travel_point_ = parameter.as_string();
        }
    }

    }

    result.successful = true;
    return result;
    }

    void BoundedExploreLayer::getNextFrontierService(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res){
        //wait for costmap to get marked with boundary
        // TODO
        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Get Next Frontier Service called");
        rclcpp::Rate r(10);
        while (!marked_)
        {
            // auto n = std::dynamic_pointer_cast<rclcpp::Node>(node->get_node_base_interface());
            // rclcpp::spin(node);
            r.sleep();
        }
        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Waiting to be marked done");

        if(req->start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            std::string tf_error;
            if(!tf_buffer_->canTransform(layered_costmap_->getGlobalFrameID(), req->start_pose.header.frame_id,tf2::TimePointZero, &tf_error)) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Couldn't transform from map "<<" to "<< req->start_pose.header.frame_id.c_str());
                res->success = false;
            }
            geometry_msgs::msg::PoseStamped temp_pose = req->start_pose;
            tf_buffer_->transform(temp_pose,req->start_pose,layered_costmap_->getGlobalFrameID(),tf2::durationFromSec(10));
        }

        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Transformation done");

        //initialize frontier search implementation
        frontier_exploration::FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()));
        //get list of frontiers from search implementation
        std::list<frontier_msgs::msg::Frontier> frontier_list = frontierSearch.searchFrom(req->start_pose.pose.position);

        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Frontier_search done");

        //create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        if(frontier_list.size() == 0){
            RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"), "No frontiers found, exploration complete");
            res->success = false;
            res->next_frontier.pose.position = selected.centroid;
            return;
        }

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);

        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Publishing routine started");

        // FRONTIER SELECTION - CHANGE TO NEW FUNCTION IF YOU WISH TO ADD INTELLIGENCE.
        bool frontierSelectionFlag = false;
        for (const auto& frontier : frontier_list) {
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.middle.x;
            frontier_point_viz.y = frontier.middle.y;
            frontier_cloud_viz.push_back(frontier_point_viz);

            //check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if(frontier.min_distance > frontierDetectRadius_){
                if (frontier.min_distance < selected.min_distance){
                    selected = frontier;
                    max = frontier_cloud_viz.size()-1;
                    frontierSelectionFlag = true;
                }
            }
        }
        if(frontierSelectionFlag == false){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"FrontierSelection Flag is: " << frontierSelectionFlag);
            res->success = false;
            return;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Current selected frontier is: x:" << selected.middle.x << ", y: " << selected.middle.y);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Frontier cloud size is: " << frontier_cloud_viz.size());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Max is: " << max);
        if(frontier_cloud_viz.size() > 0) {
            frontier_cloud_viz[max].intensity = 100;
        }
        // RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Color set");

        //publish visualization point cloud
        sensor_msgs::msg::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        // RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"costmap frame id to set");
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        // RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Header frame ID set to global frame id.");
        frontier_viz_output.header.stamp = rclcpp::Clock().now();
        frontier_cloud_pub->publish(frontier_viz_output);
        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Get Next Frontier Service called and published new frontier for visualization");

        //set goal pose to next frontier
        res->next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        res->next_frontier.header.stamp = rclcpp::Clock().now();

        //
        if(frontier_travel_point_ == "closest"){
            res->next_frontier.pose.position = selected.initial;
        }else if(frontier_travel_point_ == "middle"){
            res->next_frontier.pose.position = selected.middle;
        }else if(frontier_travel_point_ == "centroid"){
            res->next_frontier.pose.position = selected.centroid;
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("bounded_explore_layer"),"Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
            res->next_frontier.pose.position = selected.initial;
        }

        res->next_frontier.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(req->start_pose.pose.position, res->next_frontier.pose.position) );
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"The result to be sent is x: " << res->next_frontier.pose.position.x << ", y:" << res->next_frontier.pose.position.y);
        res->success = true;

        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Bounded explore layer Get Next Frontier Service Call completed");

    }

    geometry_msgs::msg::Quaternion BoundedExploreLayer::createQuaternionMsgFromYaw(double yaw)
    {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
    }

    void BoundedExploreLayer::updateBoundaryPolygonService(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Request> req, std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Response> res){
        //clear existing boundary, if any
        polygon_.points.clear();
        std::string tf_error;
        //error if no transform available between polygon and costmap
        if(!tf_buffer_->canTransform(layered_costmap_->getGlobalFrameID(), req->explore_boundary.header.frame_id,tf2::TimePointZero, &tf_error)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Couldn't transform from " << layered_costmap_->getGlobalFrameID().c_str() << " to "<< req->explore_boundary.header.frame_id.c_str());
            res-> success = false;
        }

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::msg::PointStamped in; 
        geometry_msgs::msg::PointStamped out;
        in.header = req->explore_boundary.header;
        for (const auto& point32 : req->explore_boundary.polygon.points) {
            in.point = nav2_costmap_2d::toPoint(point32);
            tf_buffer_->transform(in,out,layered_costmap_->getGlobalFrameID(),tf2::durationFromSec(10));
            polygon_.points.push_back(nav2_costmap_2d::toPoint32(out.point));
        }

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::msg::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x_polygon = std::numeric_limits<double>::infinity();
            double min_y_polygon = std::numeric_limits<double>::infinity();
            double max_x_polygon = -std::numeric_limits<double>::infinity();
            double max_y_polygon = -std::numeric_limits<double>::infinity();

            for (const auto& point : polygon_.points) {
                min_x_polygon = std::min(min_x_polygon,(double)point.x);
                min_y_polygon = std::min(min_y_polygon,(double)point.y);
                max_x_polygon = std::max(max_x_polygon,(double)point.x);
                max_y_polygon = std::max(max_y_polygon,(double)point.y);
            }

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x_polygon - min_x_polygon, max_y_polygon - min_y_polygon, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x_polygon, min_y_polygon);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        res->success = true;
        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Update boundary polygon service finished");
    }

    void BoundedExploreLayer::reset(){

        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Reset function called");

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabledLayer_ || !configured_){ return; }

        //update the whole costmap
        // *min_x = getOriginX();
        // *min_y = getOriginY();
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer")," Update bounds Min X and Min Y: " << *min_x << "," << *min_y);
        // *max_x = getSizeInMetersX()+getOriginX();
        // *max_y = getSizeInMetersY()+getOriginY();

        *min_x = *min_x;
        *max_x = *max_x;
        *min_y = *min_y;
        *max_y = *max_y;

        RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Bounded explore layer Updated bounds");

    }

    void BoundedExploreLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Marked is set as: " << marked_);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Configured is set as" << configured_);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Enabled is set as" << enabledLayer_);
        if (!enabledLayer_ || !configured_){ 
            return; 
        }

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){
            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);

        // RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Bounded explore layer updated costs");

    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabledLayer_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("bounded_explore_layer"),"Values:" << max_j << ","<<max_i);
        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;

            }
        }
        marked_ = true;
        // RCLCPP_INFO(rclcpp::get_logger("bounded_explore_layer"),"Update costs function called, marked set to true");
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, nav2_costmap_2d::Layer)

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
    
//     // Create an instance of your class
//     frontier_exploration::BoundedExploreLayer bel;

//     // You can now access the node using the class instance
//     // For example, call a member function that uses the node

//     // auto n = std::dynamic_pointer_cast<rclcpp::Node>(bel.node->get_node_base_interface());
//     // rclcpp::spin(n); // Spin the node to handle callbacks

//     rclcpp::shutdown();
//     return 0;
// }