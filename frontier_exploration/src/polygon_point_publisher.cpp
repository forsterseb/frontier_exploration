#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class PointPublisherNode : public rclcpp::Node
{
public:
  PointPublisherNode() : Node("point_publisher_node"), count_(1)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", 10);
    count_ = 1;
    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&PointPublisherNode::publishPoint, this));
    squareHalfSideLength_ = 25.0;
  }

private:
  void publishPoint()
  {
    auto point_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
    point_msg->header.stamp = this->now();
    point_msg->header.frame_id = "map"; // Modify the frame_id according to your needs
    point_msg->point.x = -squareHalfSideLength_;
    point_msg->point.y = squareHalfSideLength_;
    point_msg->point.z = 0.002;

    switch (count_)
    {
      case 1:
        point_msg->point.x = -squareHalfSideLength_;
        point_msg->point.y = squareHalfSideLength_;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        break;
      case 2:
        point_msg->point.x = squareHalfSideLength_;
        point_msg->point.y = squareHalfSideLength_;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        break;
      case 3:
        point_msg->point.x = squareHalfSideLength_;
        point_msg->point.y = -squareHalfSideLength_;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        break;
      case 4:
        point_msg->point.x = -squareHalfSideLength_;
        point_msg->point.y = -squareHalfSideLength_;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        break;
      case 5:
        point_msg->point.x = -squareHalfSideLength_;
        point_msg->point.y = squareHalfSideLength_;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        break;
      case 6:
        point_msg->point.x = 0.0;
        point_msg->point.y = 0.0;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        break;
      default:
        // Reset the counter if it exceeds 4
        count_ = 0;
        timer_->cancel();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("polygon_point_publisher"), "Point: " << count_);
        rclcpp::shutdown();
        break;
    }

    publisher_->publish(*point_msg);
    count_++;
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
  double squareHalfSideLength_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointPublisherNode>());
  rclcpp::shutdown();
  return 0;
}