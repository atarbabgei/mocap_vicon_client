#include "mocap_vicon_client/publisher.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node, tf2_ros::TransformBroadcaster* tf_broadcaster)
    : tf_broadcaster_(tf_broadcaster)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.frame_id = p.translation_type;
    msg->header.stamp = p.stamp;
    msg->pose.position.x = p.translation[0];
    msg->pose.position.y = p.translation[1];
    msg->pose.position.z = p.translation[2];
    msg->pose.orientation.x = p.rotation[0];
    msg->pose.orientation.y = p.rotation[1];
    msg->pose.orientation.z = p.rotation[2];
    msg->pose.orientation.w = p.rotation[3];
    position_publisher_->publish(*msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = p.stamp;
    tf_msg.header.frame_id = p.translation_type;
    tf_msg.child_frame_id = p.child_frame_id;
    tf_msg.transform.translation.x = p.translation[0];
    tf_msg.transform.translation.y = p.translation[1];
    tf_msg.transform.translation.z = p.translation[2];
    tf_msg.transform.rotation.x = p.rotation[0];
    tf_msg.transform.rotation.y = p.rotation[1];
    tf_msg.transform.rotation.z = p.rotation[2];
    tf_msg.transform.rotation.w = p.rotation[3];
    tf_broadcaster_->sendTransform(tf_msg);
}
