#include "lidar_node.hpp"

#include <iostream>
#include <math.h>

#include "rviz_detections/Detection3D.h"
#include "rviz_detections/Detection3DArray.h"


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
    detection_pub_ = nh_.advertise<rviz_detections::Detection3DArray>(detection_topic, 5);
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

// Map labels that the model is capable of detecting (see: cbgs_pp_multihead.yaml)
// All possible labels are listed here: https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/instructions_nuscenes.md
std::string intLabelToString(int label)
{
    switch (label)
    {
        case 0: return "Car";
        case 1: return "Truck";
        case 2: return "Construction vehicle";
        case 3: return "Bus";
        case 4: return "Trailer";
        case 5: return "Barrier";
        case 6: return "Motorcycle";
        case 7: return "Bicycle";
        case 8: return "Pedestrian";
        case 9: return "Traffic cone";
        default: return "Unknown";
    }
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

void LidarNode::publishDetectionMsg(std::vector<float>& out_detections, std::vector<int>& out_labels, std::vector<float>& out_scores)
{
    rviz_detections::Detection3DArray detection_array;

    for (size_t i = 0; i < out_detections.size() / 7; i++)
    {
        // boxes: (n,7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw)   
        rviz_detections::Detection3D detection;
        size_t j = i*7; // packed data width is 7
        detection.pose.position.x = out_detections[j];
        detection.pose.position.y = out_detections[j+1];
        detection.pose.position.z = out_detections[j+2];

        detection.size.x = out_detections[j+3]; 
        detection.size.y = out_detections[j+4];
        detection.size.z = out_detections[j+5];

        float yaw = out_detections[j+6]; // radians
        detection.pose.orientation = getOrientation(yaw);

        detection.categories.push_back(intLabelToString(out_labels[i]));
        detection.category_confidences.push_back(out_scores[i]);

        detection_array.header.stamp = ros::Time::now();
        detection_array.header.frame_id = frame_;
        detection_array.detections.push_back(detection);
    }

    std::cout << ">>> publishing " << out_detections.size() / 7 << " detections" << std::endl;
    detection_pub_.publish(detection_array);
}
