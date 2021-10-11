#pragma once

#include <ros/ros.h>
#include "pointpillars/pointpillars.h"

class LidarNode
{
public:
    LidarNode(const ros::NodeHandle& nh, const bool debug, YAML::Node config, std::string pfe_file, std::string backbone_file);

    int detect(float* const points_array, size_t in_num_points, std::vector<float>& out_detections, std::vector<int>&out_labels, std::vector<float>& out_scores);
    void publishDetectionMsg(std::vector<float>& out_detections, std::vector<int>& out_labels, std::vector<float>& out_scores);

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
