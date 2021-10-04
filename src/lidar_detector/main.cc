#include "lidar_node.hpp"
#include "pclbuff.hpp"
#include <ros/ros.h>
#include <signal.h>
#include <mutex>
#include <thread>
#include <condition_variable>

bool run = true;

// Signal Handler for SIGINT
void sigint_handler(int sig_num)
{
    std::cout << "Ctrl+C pressed, exiting program..." << std::endl;
    run = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_detector_node");
    ros::NodeHandle param_nh("~");
    signal(SIGINT, sigint_handler);

    // Read settings from launch file
    bool debug;
    double loop_rate_hz;
    std::string config_path;
    param_nh.param<bool>("debug", debug, true);
    param_nh.param<double>("loop_rate_hz", loop_rate_hz, 1.0);
    param_nh.param<std::string>("config_path", config_path, "/app/config/bootstrap.yaml");


    // Possibly change logger lever to debug
    if (debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO_STREAM("Configurations:" << std::endl
        << "  debug: "   << (debug == 1 ? "true" : "false")
    );

    // Create class instance with desired settings
    ROS_INFO_STREAM("Loading bootstrap config:" << config_path);
    YAML::Node config = YAML::LoadFile(config_path); // DB_CONF
    // Decide between TRT and Onnx files
    std::string pfe_file, backbone_file; 
    if(config["UseOnnx"].as<bool>()) {
    pfe_file = config["PfeOnnx"].as<std::string>();
    backbone_file = config["BackboneOnnx"].as<std::string>();
    }else {
    pfe_file = config["PfeTrt"].as<std::string>();
    backbone_file = config["BackboneTrt"].as<std::string>();
    }
    std::cout << "Backbone: "<< backbone_file << std::endl;

    PclBuff data_buffer(param_nh);
    LidarNode lidar_node(param_nh, debug, config, pfe_file, backbone_file);
    ROS_INFO("Node initialization complete!");


    // Start lidar data buffering process
    std::thread (&PclBuff::start, data_buffer).detach();

    ros::Rate loop_rate(loop_rate_hz);
    float* data = nullptr;
    size_t point_threshold = 1000;

    while (run)
    {
        size_t point_count = data_buffer.getData(&data);
        if ((point_count > point_threshold) && (data != nullptr))
        {
            std::vector<int> out_labels;
            std::vector<float> out_detections, out_scores;
            size_t detection_count = lidar_node.detect(data, point_count, out_detections, out_labels, out_scores);
            
            if (detection_count > 0)
            {
                lidar_node.publishDetectionMsg(out_detections);
            }
        }

        data_buffer.markDone();
    }

    data_buffer.stop(); // free buffering process resources

    return EXIT_SUCCESS;
}
