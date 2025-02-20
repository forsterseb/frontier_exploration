#ifndef BOUNDED_EXPLORE_LAYER_H_
#define BOUNDED_EXPLORE_LAYER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/quaternion.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <frontier_msgs/msg/frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

namespace frontier_exploration
{

/**
 * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
 * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
 * and processes costmap to find next frontier to explore.
 */
class BoundedExploreLayer : public nav2_costmap_2d::Layer, public nav2_costmap_2d::Costmap2D
{
public:
    BoundedExploreLayer();
    ~BoundedExploreLayer();

    /**
     * @brief Loads default values and initialize exploration costmap.
     */
    virtual void onInitialize();

    /**
     * @brief Calculate bounds of costmap window to update
     */
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* polygon_min_x, double* polygon_min_y, double* polygon_max_x,
                              double* polygon_max_y);

    /**
     * @brief Update requested costmap window
     */
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    
    bool isDiscretized()
    {
        return true;
    }

    /**
     * @brief Match dimensions and origin of parent costmap
     */
    void matchSize() override;

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() {return false;}

    /**
     * @brief Reset exploration progress
     */
    virtual void reset();

    // std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("bounded_explore_layer");

    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
    
    rclcpp::Service<frontier_msgs::srv::UpdateBoundaryPolygon>::SharedPtr polygonService_;
    rclcpp::Service<frontier_msgs::srv::GetNextFrontier>::SharedPtr frontierService_;

    // rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("bounded_explore_layer");

protected:

    /**
     * @brief ROS Service wrapper for updateBoundaryPolygon
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    /**
     * @brief Load polygon boundary to draw on map with each update
     * @param polygon_stamped polygon boundary
     * @return True if polygon successfully loaded, false otherwise
     */
    void updateBoundaryPolygonService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Request> req, 
        std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Response> res);

    /**
     * @brief ROS Service wrapper for getNextFrontier
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    /**
     * @brief Search the costmap for next reachable frontier to explore
     * @param start_pose Pose from which to start search
     * @param next_frontier Pose of found frontier
     * @return True if a reachable frontier was found, false otherwise
     */
    void getNextFrontierService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req,
        std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res);

private:

    /**
     * @brief Update the map with exploration boundary data
     * @param master_grid Reference to master costmap
     */
    void mapUpdateKeepObstacles(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    geometry_msgs::msg::Polygon polygon_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub;

    bool enabledLayer_;
    bool configured_, marked_;
    bool explore_clear_space_;
    bool resize_to_boundary_;
    std::string frontier_travel_point_;
    int max;
    double frontierDetectRadius_;

    // Dynamic parameters handler
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}
#endif
