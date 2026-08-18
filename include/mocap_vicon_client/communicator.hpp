#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <memory>
#include <chrono>
#include <string>
#include <unistd.h>
#include <boost/thread.hpp>

using namespace std;

// Main Node class
class Communicator : public rclcpp::Node
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;
    string server;
    unsigned int buffer_size;
    string ns_name;
    string parent_frame_;
    map<string, Publisher> pub_map;
    boost::mutex mutex;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    VelocityConfig velocity_cfg_;
    bool frame_rate_known_ = false;   // GetFrameRate() needs a fetched frame, so it is queried lazily

    // Rolling stream-health accounting, driven by gaps in the Vicon frame counter.
    unsigned int last_stream_frame_ = 0;
    bool have_stream_frame_ = false;
    unsigned long long frames_delivered_ = 0;
    unsigned long long frames_missed_ = 0;
    double loss_window_start_s_ = 0.0;   // seconds, not rclcpp::Time, to dodge clock-type mismatches

public:
    Communicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the
    // received segment data to the Publisher class.
    void get_frame();

    // functions to create a segment publisher in a new thread
    void create_publisher(const string subject_name, const string segment_name, const string topic_base);
    void create_publisher_thread(const string subject_name, const string segment_name, const string topic_base);

    // Replaces characters outside [A-Za-z0-9_] with '_' so the string is a valid tf2 frame id.
    // tf2 silently rejects invalid frames in RViz, so this is load-bearing for visualization.
    static std::string sanitize_frame(const std::string& name);
};

#endif // COMMUNICATOR_HPP
