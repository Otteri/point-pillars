#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "../pointpillars/pointpillars.h"


class LidarNode
{
public:
    LidarNode(const ros::NodeHandle& nh, const bool debug, YAML::Node config, std::string pfe_file, std::string backbone_file);

    void lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_cloud);

    int detect(float* const points_array, size_t in_num_points, std::vector<float>& out_detections);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher poincloud_pub_;
    ros::Publisher detection_pub_;

    bool run_;
    bool debug_;
    YAML::Node config_;
    std::string frame_;
    double loop_rate_hz_;

    PointPillars pp_;
};
