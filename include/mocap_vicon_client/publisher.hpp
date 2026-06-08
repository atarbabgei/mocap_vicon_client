#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// Struct used to hold segment data to transmit to the Publisher class.
struct PositionStruct
{
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;   // parent TF frame id (also used for PoseStamped.header.frame_id)
    std::string child_frame_id;     // sanitized child TF frame id, "<subject>_<segment>"
    unsigned int frame_number;
    rclcpp::Time stamp;             // Vicon-latency-corrected sample time
};

// Class that allows segment data to be published in a ROS2 topic.
class Publisher
{
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
    tf2_ros::TransformBroadcaster* tf_broadcaster_;  // non-owning; lifetime held by Communicator

public:
    bool is_ready = false;

    Publisher(std::string topic_name, rclcpp::Node* node, tf2_ros::TransformBroadcaster* tf_broadcaster);

    // Publishes the given position in the ROS2 topic whose name is indicated in
    // the constructor, and broadcasts the same pose as a TF transform.
    void publish(PositionStruct p);
};

#endif
