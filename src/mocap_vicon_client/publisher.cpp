#include "mocap_vicon_client/publisher.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.frame_id = p.translation_type;
    msg->header.stamp = rclcpp::Clock().now(); // TODO: use timestamp from Vicon
    msg->pose.position.x = p.translation[0]; 
    msg->pose.position.y = p.translation[1]; 
    msg->pose.position.z = p.translation[2]; 
    msg->pose.orientation.x = p.rotation[0];
    msg->pose.orientation.y = p.rotation[1];
    msg->pose.orientation.z = p.rotation[2];
    msg->pose.orientation.w = p.rotation[3];
    position_publisher_->publish(*msg);
}
