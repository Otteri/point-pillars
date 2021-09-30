#include "lidar_node.hpp"
#include <ros/ros.h>
#include <signal.h>

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
    param_nh.param<bool>("debug", debug, true);
    param_nh.param<double>("loop_rate_hz", loop_rate_hz, 1.0);

    // Possibly change logger lever to debug
    if (debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO_STREAM("Configurations:" << std::endl
        << "  debug: "   << (debug == 1 ? "true" : "false")
    );

    // Create class instance with desired settings
    const std::string DB_CONF = "/app/PointPillars/lidar_detector/bootstrap.yaml";
    YAML::Node config = YAML::LoadFile(DB_CONF);
    // Decide between TRT and Onnx files
    std::string pfe_file, backbone_file; 
    if(config["UseOnnx"].as<bool>()) {
    pfe_file = config["PfeOnnx"].as<std::string>();
    backbone_file = config["BackboneOnnx"].as<std::string>();
    }else {
    pfe_file = config["PfeTrt"].as<std::string>();
    backbone_file = config["BackboneTrt"].as<std::string>();
    }
    std::cout << backbone_file << std::endl;
    LidarNode lidar_node(param_nh, debug, config, pfe_file, backbone_file);

    ROS_INFO("Node initialization complete!");

    // Run
    ros::Rate loop_rate(loop_rate_hz);
    while (run)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
