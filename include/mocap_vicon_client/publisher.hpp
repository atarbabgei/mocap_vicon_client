#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <unistd.h>
#include <array>
#include <deque>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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

// Settings for the finite-difference velocity estimator. Built once by Communicator from ROS
// params and copied into every Publisher, so all segments share one configuration.
struct VelocityConfig
{
    bool         enabled       = true;
    size_t       window        = 5;     // samples in the least-squares slope fit; 2 == raw difference
    unsigned int max_gap       = 3;     // drop the history when more than this many frames go missing
    double       frame_rate_hz = 0.0;   // Vicon frame rate; 0 means "fall back to ROS stamps"
};

// Class that allows segment data to be published in a ROS2 topic.
class Publisher
{
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    tf2_ros::TransformBroadcaster* tf_broadcaster_;  // non-owning; lifetime held by Communicator

    VelocityConfig vel_cfg_;

    // Sliding window of (sample time [s], position [m]) used for the slope fit, newest at the back.
    std::deque<std::pair<double, std::array<double, 3>>> history_;
    unsigned int last_frame_number_ = 0;
    bool have_last_frame_ = false;

    // Ordinary-least-squares slope of each axis against time, in m/s.
    // Returns false when there is not enough history yet to define a slope.
    bool fit_velocity(double out[3]) const;

public:
    bool is_ready = false;

    // topic_base is a namespace, not a topic: the pose goes to "<topic_base>/pose" and the
    // velocity to "<topic_base>/twist". Keeping the base free of any topic of its own is what
    // stops a topic from also being the parent of another topic.
    Publisher(std::string topic_base, rclcpp::Node* node,
              tf2_ros::TransformBroadcaster* tf_broadcaster,
              const VelocityConfig& vel_cfg);

    // Publishes the pose on "<topic_base>/pose", broadcasts the same pose as a TF transform,
    // and (when enabled) publishes the derived linear velocity on "<topic_base>/twist".
    void publish(PositionStruct p);
};

#endif
