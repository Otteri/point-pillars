#include "lidar_node.hpp"

#include <iostream>
#include <math.h>

#include <mbilly_msgs/MBillyDetection3D.h>
#include <mbilly_msgs/MBillyDetection3DArray.h>


LidarNode::LidarNode(const ros::NodeHandle& nh, const bool debug, YAML::Node config, std::string pfe_file, std::string backbone_file)
    : debug_(debug)
    , run_(true)
    , config_(config)
    , pp_(config["ScoreThreshold"].as<float>(),
          config["NmsOverlapThreshold"].as<float>(),
          config["UseOnnx"].as<bool>(),
          pfe_file,
          backbone_file,
          config["ModelConfig"].as<std::string>()
         )
{
    std::string detection_topic;
    nh.param<std::string>("detection_topic",   detection_topic,   "/lidar/detections");
    nh.param<std::string>("frame",             frame_,            "/base_link");

    nh.param<double>("loop_rate_hz",  loop_rate_hz_, 40.0);

    // Publisher for lidar detections
    detection_pub_ = nh_.advertise<mbilly_msgs::MBillyDetection3DArray>(detection_topic, 5);
}

// A helper function that gives quaternion orientation from yaw (euler angle). 
// pich (Y) and roll (Z) angles are ignored: we don't care about these in this context.
// Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// C++ source code, but with flipped signs due to XYZ representation instead of ZYX!
geometry_msgs::Quaternion getOrientation(float yaw)
{
    geometry_msgs::Quaternion q;
    q.w = cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw * 0.5);
    return q;
}

int LidarNode::detect(float* const pPoints, size_t in_num_points, std::vector<float>& out_detections, std::vector<int>&out_labels, std::vector<float>& out_scores)
{
    float* points_array = pPoints;

    cudaDeviceSynchronize();
    pp_.DoInference(points_array, in_num_points, &out_detections, &out_labels, &out_scores);
    cudaDeviceSynchronize();

    int BoxFeature = 7; // make this class param
    int num_objects = out_detections.size() / BoxFeature;

    return num_objects;
};

void LidarNode::publishDetectionMsg(std::vector<float>& out_detections)
{
    mbilly_msgs::MBillyDetection3DArray detection_array;
    for (size_t i = 0; i < out_detections.size(); i=i+7)
    {
        // boxes: (n,7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw)   
        mbilly_msgs::MBillyDetection3D detection;
        detection.pose.position.x = out_detections[i];
        detection.pose.position.y = out_detections[i+1];
        detection.pose.position.z = out_detections[i+2];

        detection.size.x = out_detections[i+3]; 
        detection.size.y = out_detections[i+4];
        detection.size.z = out_detections[i+5];

        float yaw = out_detections[i+6]; // radians
        detection.pose.orientation = getOrientation(yaw);

        detection.categories.push_back("Asdsad");
        detection.category_confidences.push_back(1.0f);

        detection_array.header.stamp = ros::Time::now();
        detection_array.header.frame_id = frame_;
        detection_array.detections.push_back(detection);
    }

    std::cout << ">>> publishing " << out_detections.size() / 7 << " detections" << std::endl;
    detection_pub_.publish(detection_array);
}
