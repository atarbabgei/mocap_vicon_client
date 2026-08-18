#include "mocap_vicon_client/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

namespace {
// How often to summarise stream health, and the loss fraction worth complaining about. A healthy
// wired link measures 0.00% loss, so anything above a fraction of a percent is a real signal.
constexpr double kStreamCheckPeriodSec = 5.0;
constexpr double kStreamLossWarnPercent = 1.0;
// A jump larger than this is a stream restart, not dropped frames; counting it as loss would
// report a nonsense percentage.
constexpr unsigned int kStreamRestartFrames = 1000;
}

Communicator::Communicator() : Node("vicon")
{
    // Declare parameters without default values
    this->declare_parameter<std::string>("server");
    this->declare_parameter<int>("buffer_size");
    this->declare_parameter<std::string>("namespace");
    this->declare_parameter<std::string>("parent_frame", "map");

    // Check if parameters are set
    if (!this->get_parameter("server", server)) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'server' is not set");
        throw std::runtime_error("Parameter 'server' is not set");
    }

    if (!this->get_parameter("buffer_size", buffer_size)) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'buffer_size' is not set");
        throw std::runtime_error("Parameter 'buffer_size' is not set");
    }

    if (!this->get_parameter("namespace", ns_name)) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'namespace' is not set");
        throw std::runtime_error("Parameter 'namespace' is not set");
    }

    parent_frame_ = this->get_parameter("parent_frame").as_string();

    // Velocity estimator settings. A window of N samples is a least-squares slope fit whose
    // noise falls as ~N^-1.5 while its group delay grows as (N-1)/2 frames, so this is the
    // one knob that trades smoothness against lag. N == 2 is a plain backward difference.
    this->declare_parameter<bool>("publish_velocity", false);
    this->declare_parameter<int>("velocity_window", 5);
    this->declare_parameter<int>("velocity_max_gap_frames", 3);

    velocity_cfg_.enabled = this->get_parameter("publish_velocity").as_bool();

    const int window = static_cast<int>(this->get_parameter("velocity_window").as_int());
    if (window < 2) {
        RCLCPP_WARN(this->get_logger(),
            "velocity_window=%d is below the minimum of 2 (a two-sample fit is already the "
            "plain backward difference); using 2", window);
        velocity_cfg_.window = 2;
    } else {
        velocity_cfg_.window = static_cast<size_t>(window);
    }

    const int max_gap = static_cast<int>(this->get_parameter("velocity_max_gap_frames").as_int());
    velocity_cfg_.max_gap = static_cast<unsigned int>(max_gap < 1 ? 1 : max_gap);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

std::string Communicator::sanitize_frame(const std::string& name)
{
    std::string out;
    out.reserve(name.size());
    for (char c : name) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '_') {
            out.push_back(c);
        } else {
            out.push_back('_');
        }
    }
    return out;
}

bool Communicator::connect()
{
    // connect to server
    std::string msg = "Connecting to " + server + " ...";
    std::cout << msg << std::endl;
    int counter = 0;
    while (!vicon_client.IsConnected().Connected)
    {
        bool ok = (vicon_client.Connect(server).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            std::cout << msg << std::endl;
            sleep(1);
        }
    }
    msg = "Connection successfully established with " + server;
    std::cout << msg << std::endl;

    // perform further initialization
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableMarkerRayData();
    vicon_client.EnableDeviceData();
    vicon_client.EnableDebugData();

    vicon_client.SetStreamMode(StreamMode::ClientPull);
    vicon_client.SetBufferSize(buffer_size);

    msg = "Initialization complete";
    std::cout << msg << std::endl;

    return true;
}

bool Communicator::disconnect()
{
    if (!vicon_client.IsConnected().Connected)
        return true;
    sleep(1);
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();
    std::string msg = "Disconnecting from " + server + "...";
    std::cout << msg << std::endl;
    vicon_client.Disconnect();
    msg = "Successfully disconnected";
    std::cout << msg << std::endl;
    if (!vicon_client.IsConnected().Connected)
        return true;
    return false;
}

void Communicator::get_frame()
{
    // GetFrame()'s result is load-bearing: if it fails, every subsequent Get*() call returns
    // Result::NoFrame with stale or zeroed data, which we would otherwise happily publish.
    Output_GetFrame frame = vicon_client.GetFrame();
    if (frame.Result != Result::Success) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GetFrame() failed (Result=%d); skipping frame",
            static_cast<int>(frame.Result));
        rclcpp::sleep_for(std::chrono::milliseconds(1)); // don't busy-spin a core on failure
        return;
    }

    // Vicon-latency-corrected stamp: subtract camera->client pipeline latency from now().
    // Standard approach used by ethz-asl/vicon_bridge. Note that a system reporting no latency
    // samples is NOT an error -- the SDK returns Success with Total == 0.0 in that case
    // (see DataStreamClient.h:2217), so the only real failures here are NotConnected/NoFrame.
    Output_GetLatencyTotal lat = vicon_client.GetLatencyTotal();
    rclcpp::Time stamp;
    if (lat.Result == Result::Success) {
        stamp = this->now() - rclcpp::Duration::from_seconds(lat.Total);
        if (lat.Total == 0.0) {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "Vicon reports no latency samples (total 0.0 s); stamps are uncorrected wall clock");
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(),
                "Vicon latency reporting active: %.2f ms total across %u samples",
                lat.Total * 1000.0, vicon_client.GetLatencySampleCount().Count);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GetLatencyTotal() failed (Result=%d); falling back to wall clock stamp "
            "(no latency correction)",
            static_cast<int>(lat.Result));
        stamp = this->now();
    }

    // The Vicon frame counter is the time base for the velocity fit, so we need the rate that
    // converts it to seconds. GetFrameRate() requires a frame to have been fetched, hence the
    // lazy query here rather than in connect(). A system that reports no rate leaves this at 0,
    // and the estimator silently falls back to the (jittery) ROS stamps.
    if (!frame_rate_known_) {
        Output_GetFrameRate rate = vicon_client.GetFrameRate();
        if (rate.Result == Result::Success && rate.FrameRateHz > 0.0) {
            velocity_cfg_.frame_rate_hz = rate.FrameRateHz;
            RCLCPP_INFO(this->get_logger(),
                "Vicon frame rate %.2f Hz (%.4f ms/frame); using the frame counter as the "
                "velocity time base", rate.FrameRateHz, 1000.0 / rate.FrameRateHz);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "GetFrameRate() unavailable (Result=%d, %.2f Hz); velocity will be timed from "
                "ROS stamps instead", static_cast<int>(rate.Result), rate.FrameRateHz);
        }
        frame_rate_known_ = true;
    }

    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    // Stream health. The frame counter increments once per Vicon frame regardless of how fast we
    // poll, so a step greater than 1 is proof that frames went missing between our reads -- much
    // more direct than inferring it from arrival times, which carry their own jitter. A step of 0
    // is the same frame served twice (ClientPull re-delivery), which is not loss.
    if (frame_number.Result == Result::Success) {
        if (have_stream_frame_) {
            const unsigned int step = frame_number.FrameNumber - last_stream_frame_;
            if (step > kStreamRestartFrames) {
                RCLCPP_WARN(this->get_logger(),
                    "Vicon frame counter jumped by %u; treating as a stream restart and resetting "
                    "stream statistics", step);
                frames_delivered_ = 0;
                frames_missed_ = 0;
            } else if (step > 0) {
                frames_delivered_ += 1;
                frames_missed_ += (step - 1);
            }
        }
        last_stream_frame_ = frame_number.FrameNumber;
        have_stream_frame_ = true;
    }

    const double now_s = this->now().seconds();
    if (loss_window_start_s_ == 0.0) {
        loss_window_start_s_ = now_s;
    } else if (now_s - loss_window_start_s_ >= kStreamCheckPeriodSec) {
        const double elapsed = now_s - loss_window_start_s_;
        const unsigned long long expected = frames_delivered_ + frames_missed_;
        if (expected > 0) {
            const double loss_pct = 100.0 * static_cast<double>(frames_missed_) /
                                    static_cast<double>(expected);
            if (loss_pct >= kStreamLossWarnPercent) {
                RCLCPP_WARN(this->get_logger(),
                    "Vicon stream degraded: %.1f Hz delivered vs %.1f Hz expected, %.1f%% of "
                    "frames lost (%llu of %llu) over the last %.0f s. Check the network link -- a "
                    "wired connection typically loses none.%s",
                    static_cast<double>(frames_delivered_) / elapsed,
                    velocity_cfg_.frame_rate_hz > 0.0
                        ? velocity_cfg_.frame_rate_hz
                        : static_cast<double>(expected) / elapsed,
                    loss_pct, frames_missed_, expected, elapsed,
                    velocity_cfg_.enabled
                        ? " Velocity is derived from these frames, so its accuracy suffers too."
                        : "");
            }
        }
        frames_delivered_ = 0;
        frames_missed_ = 0;
        loss_window_start_s_ = now_s;
    }

    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    std::map<std::string, Publisher>::iterator pub_it;

    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // get the subject name
        std::string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // count the number of segments
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            // get the segment name
            std::string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            // get position of segment
            PositionStruct current_position;
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion rot =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

            // An occluded segment is not merely low quality -- Vicon zeroes it. The translation
            // comes back as [0,0,0] and the quaternion as all zeros, which is not a unit
            // quaternion at all. Publishing that teleports the subject to the origin and, once
            // differentiated, yields spikes of >100 m/s on both the entry and the exit of every
            // dropout. Emit nothing for this segment instead; the gap then trips the velocity
            // estimator's reset, so the fit never spans the dropout.
            if (trans.Result != Result::Success || rot.Result != Result::Success ||
                trans.Occluded || rot.Occluded)
            {
                RCLCPP_WARN_ONCE(this->get_logger(),
                    "segment '%s/%s' is occluded; skipping occluded frames (pose, TF and "
                    "velocity are simply not published while a segment is not visible)",
                    subject_name.c_str(), segment_name.c_str());
                continue;
            }

            for (size_t i = 0; i < 4; i++)
            {
                if (i < 3)
                    current_position.translation[i] = trans.Translation[i] / 1000.0; // convert to meters
                current_position.rotation[i] = rot.Rotation[i];
            }
            current_position.segment_name = segment_name;
            current_position.subject_name = subject_name;
            current_position.translation_type = parent_frame_;
            current_position.frame_number = frame_number.FrameNumber;
            current_position.stamp = stamp;

            // Naming convention. The common case is a single-segment rigid body, where the
            // Vicon segment name duplicates the subject name (e.g. FlapperDrone/FlapperDrone).
            // For that case we use clean names: namespace "<ns>/<subject>", child frame
            // "<subject>_link". When a subject has more than one segment we fall back to
            // including the segment so the topics/frames don't collide.
            //
            // Note this is a topic *namespace*, never a topic itself -- Publisher hangs
            // "<base>/pose" and "<base>/twist" underneath it. Publishing the pose directly on
            // the base would make one topic the parent of another, which is legal in ROS 2 but
            // renders an ambiguous topic tree.
            std::string topic_base;
            std::string raw_child;
            if (segment_count == 1) {
                topic_base = ns_name + "/" + subject_name;
                raw_child = subject_name + "_link";
            } else {
                topic_base = ns_name + "/" + subject_name + "/" + segment_name;
                raw_child = subject_name + "_" + segment_name + "_link";
                RCLCPP_WARN_ONCE(this->get_logger(),
                    "subject '%s' has %u segments; including segment name in topic/frame to avoid collisions",
                    subject_name.c_str(), segment_count);
            }

            current_position.child_frame_id = sanitize_frame(raw_child);
            if (current_position.child_frame_id != raw_child) {
                RCLCPP_WARN_ONCE(this->get_logger(),
                    "tf frame name sanitized: '%s' -> '%s' (non-alphanumeric chars replaced with '_')",
                    raw_child.c_str(), current_position.child_frame_id.c_str());
            }

            // send position to publisher
            boost::mutex::scoped_try_lock lock(mutex);

            if (lock.owns_lock())
            {
                // get publisher
                pub_it = pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        pub.publish(current_position);
                    }
                }
                else
                {
                    // create publisher if not already available
                    lock.unlock();
                    create_publisher(subject_name, segment_name, topic_base);
                }
            }
        }
    }
}

void Communicator::create_publisher(const std::string subject_name, const std::string segment_name, const std::string topic_base)
{
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name, topic_base);
}

void Communicator::create_publisher_thread(const std::string subject_name, const std::string segment_name, const std::string topic_base)
{
    std::string key = subject_name + "/" + segment_name;

    std::string msg = "Creating publisher for segment " + segment_name + " from subject " + subject_name;
    std::cout << msg << std::endl;

    // create publisher
    boost::mutex::scoped_lock lock(mutex);
    pub_map.insert(std::map<std::string, Publisher>::value_type(
        key, Publisher(topic_base, this, tf_broadcaster_.get(), velocity_cfg_)));

    // we don't need the lock anymore, since rest is protected by is_ready
    lock.unlock();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    try {
        node->connect();
    } catch (const std::runtime_error& e) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize the node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    while (rclcpp::ok()){
        node->get_frame();
    }

    node->disconnect();
    rclcpp::shutdown();
    return 0;
}
