#include "mocap_vicon_client/publisher.hpp"

Publisher::Publisher(std::string topic_base, rclcpp::Node* node,
                     tf2_ros::TransformBroadcaster* tf_broadcaster,
                     const VelocityConfig& vel_cfg)
    : tf_broadcaster_(tf_broadcaster), vel_cfg_(vel_cfg)
{
    position_publisher_ =
        node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_base + "/pose", 10);
    if (vel_cfg_.enabled)
    {
        velocity_publisher_ =
            node->create_publisher<geometry_msgs::msg::TwistStamped>(topic_base + "/twist", 10);
    }
    is_ready = true;
}

bool Publisher::fit_velocity(double out[3]) const
{
    const size_t n = history_.size();
    if (n < 2)
        return false;

    // Fit position against the *actual* sample times rather than the sample index. That is what
    // makes a window containing a dropout still correct: a missing frame widens the spacing
    // instead of silently pretending the samples were evenly spaced.
    double t_mean = 0.0;
    for (const auto& s : history_)
        t_mean += s.first;
    t_mean /= static_cast<double>(n);

    double denom = 0.0;
    for (const auto& s : history_)
    {
        const double dt = s.first - t_mean;
        denom += dt * dt;
    }
    if (denom <= 0.0)
        return false; // every sample shares one timestamp; slope is undefined

    for (size_t axis = 0; axis < 3; ++axis)
    {
        double p_mean = 0.0;
        for (const auto& s : history_)
            p_mean += s.second[axis];
        p_mean /= static_cast<double>(n);

        double num = 0.0;
        for (const auto& s : history_)
            num += (s.first - t_mean) * (s.second[axis] - p_mean);

        out[axis] = num / denom;
    }
    return true;
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

    if (!velocity_publisher_)
        return;

    // The same Vicon frame can be served twice: in ClientPull mode GetFrame() re-delivers the
    // buffered frame when we poll faster than the system runs. It carries no new information,
    // but it arrives with a fresh wall-clock stamp, so letting it into the fit would add a
    // sample pair separated by ~50 us of stamp and 0 m of motion.
    if (have_last_frame_ && p.frame_number == last_frame_number_)
        return;

    // Vicon frame numbers only ever advance. Any decrease means the stream restarted, and the
    // unsigned subtraction below wraps to a huge value, which trips the same reset path.
    if (have_last_frame_ && (p.frame_number - last_frame_number_) > vel_cfg_.max_gap)
    {
        // Across a long dropout the endpoints are not usefully related: the slope would report
        // the average displacement over the gap rather than a velocity. Start over instead.
        history_.clear();
    }
    last_frame_number_ = p.frame_number;
    have_last_frame_ = true;

    // Prefer the Vicon frame counter as the time base. It is exact and uniform by construction,
    // whereas the ROS stamp is a poll-time reading minus a per-frame latency estimate and so
    // carries jitter (and, on a congested link, can even step backwards). frame_rate_hz is 0
    // only if the system did not report a rate, in which case the stamp is all we have.
    const double t = (vel_cfg_.frame_rate_hz > 0.0)
                         ? static_cast<double>(p.frame_number) / vel_cfg_.frame_rate_hz
                         : p.stamp.seconds();

    std::array<double, 3> pos = {{p.translation[0], p.translation[1], p.translation[2]}};
    history_.push_back(std::make_pair(t, pos));
    while (history_.size() > vel_cfg_.window)
        history_.pop_front();

    double v[3];
    if (!fit_velocity(v))
        return; // still filling the window after a start or a reset

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = p.stamp;
    twist.header.frame_id = p.translation_type;
    twist.twist.linear.x = v[0];
    twist.twist.linear.y = v[1];
    twist.twist.linear.z = v[2];
    // Angular velocity is intentionally left zero: it is not derived from the segment rotation.
    velocity_publisher_->publish(twist);
}
