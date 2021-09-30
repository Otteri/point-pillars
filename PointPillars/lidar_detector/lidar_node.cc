#include "lidar_node.hpp"

#include <iostream>
#include <math.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <odts_msgs/Detection3D.h>
#include <odts_msgs/Detection3DArray.h>


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
    std::string pcl_topic, pointcloud_topic, detection_topic;
    nh.param<std::string>("lidar_topic",       pcl_topic,         "/lidar_points_rectified");
    nh.param<std::string>("pointcloud_topic",  pointcloud_topic,  "/lidar/pointcloud");
    nh.param<std::string>("detection_topic",   detection_topic,   "/lidar/detections");
    nh.param<std::string>("frame",             frame_,            "/front_velodyne");

    int input_queue_size;
    nh.param<int>("input_queue_size", input_queue_size,  1);
    nh.param<double>("loop_rate_hz",  loop_rate_hz_, 40.0);

    // Subscriber for camera messages
    pcl_sub_ = nh_.subscribe(
        pcl_topic, input_queue_size, &LidarNode::lidarDataCallback, this, ros::TransportHints().tcpNoDelay());

    // Publisher for camera ego-motion messages
    poincloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, input_queue_size);
    detection_pub_ = nh_.advertise<odts_msgs::Detection3DArray>(detection_topic, input_queue_size);
}

// A helper function that gives quaternion orientation from yaw (euler angle). 
// pich (Y) and roll (Z) angles are ingnored, we don't care about these in this context.
// Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// C++ source code, but with flipped signs due to XYZ reoresentation and not ZYX!
geometry_msgs::Quaternion getOrientation(float yaw)
{

    geometry_msgs::Quaternion q;
    q.w = cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw * 0.5);
    return q;
}

void LidarNode::lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_cloud)
{
    sensor_msgs::PointCloud out_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*pcl_cloud, out_pointcloud);

    // Allocate buffer for point data
    size_t feature_count = 5; // [x,y,z,?,?]
    size_t point_count = pcl_cloud->width * pcl_cloud->height;
    size_t buffer_size = point_count * feature_count;
    float* const data = new float[buffer_size]; // ptr to beginning of the data array
    float* pPoints = data;

    // Collect points
    for(size_t i = 0 ; i < out_pointcloud.points.size(); i++)
    {
        *pPoints++ = out_pointcloud.points[i].x;
        *pPoints++ = out_pointcloud.points[i].y;
        *pPoints++ = out_pointcloud.points[i].z;
        *pPoints++ = 0.0;
        *pPoints++ = 0.0;
    }

    std::vector<float> out_detections;
    size_t box_count = detect(data, point_count, out_detections);

    odts_msgs::Detection3DArray detection_array;

    for (size_t i = 0; i < out_detections.size(); i=i+7)
    {
        // boxes: (n,7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw)   
        odts_msgs::Detection3D detection;
        detection.center.x = out_detections[i];
        detection.center.y = out_detections[i+1];
        detection.center.z = out_detections[i+2];

        detection.size.x = out_detections[i+3]; 
        detection.size.y = out_detections[i+4];
        detection.size.z = out_detections[i+5];
        float yaw = out_detections[i+6]; // radians
        detection.orientation = getOrientation(yaw);

        detection_array.header.stamp = pcl_cloud->header.stamp; //ros::Time::now();
        detection_array.header.frame_id = frame_;
        detection_array.detections.push_back(detection);
    }

    if (box_count > 0)
    {
        std::cout << ">>> publishing " << box_count << " detections" << std::endl;
        detection_pub_.publish(detection_array);

        if (debug_)
        {   // This is heavy, but helps to associate detections to raw data
            poincloud_pub_.publish(pcl_cloud);
        }
    }

    delete data;
}


int LidarNode::detect(float* const pPoints, size_t in_num_points, std::vector<float>& out_detections)
{
    std::vector<int> out_labels; // Make arg
    std::vector<float> out_scores; // Make arg
    float* points_array = pPoints;

    cudaDeviceSynchronize();
    pp_.DoInference(points_array, in_num_points, &out_detections, &out_labels , &out_scores);
    cudaDeviceSynchronize();

    int BoxFeature = 7;
    int num_objects = out_detections.size() / BoxFeature;

    return num_objects;
};
