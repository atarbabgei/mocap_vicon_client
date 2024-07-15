#include "mocap_vicon_client/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

Communicator::Communicator() : Node("vicon")
{
    // Declare parameters without default values
    this->declare_parameter<std::string>("server");
    this->declare_parameter<int>("buffer_size");
    this->declare_parameter<std::string>("namespace");

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
    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

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
            
            for (size_t i = 0; i < 4; i++)
            {
                if (i < 3)
                    current_position.translation[i] = trans.Translation[i] / 1000.0; // convert to meters
                current_position.rotation[i] = rot.Rotation[i];
            }
            current_position.segment_name = segment_name;
            current_position.subject_name = subject_name;
            current_position.translation_type = "map";
            current_position.frame_number = frame_number.FrameNumber;

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
                    create_publisher(subject_name, segment_name);
                }
            }
        }
    }
}

void Communicator::create_publisher(const std::string subject_name, const std::string segment_name)
{
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name);
}

void Communicator::create_publisher_thread(const std::string subject_name, const std::string segment_name)
{
    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    std::string msg = "Creating publisher for segment " + segment_name + " from subject " + subject_name;
    std::cout << msg << std::endl;

    // create publisher
    boost::mutex::scoped_lock lock(mutex);
    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));

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
