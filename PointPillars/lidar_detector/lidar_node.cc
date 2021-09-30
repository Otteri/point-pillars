#include "lidar_node.hpp"
#include "detector.hpp"

#include <iostream>
#include <math.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vision_msgs/BoundingBox3D.h>
#include <odts_msgs/Detection3D.h>
#include <odts_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Static keep multiple callback data in memory
// size_t constexpr buffer_size = 9999999;
// float points[buffer_size];
// size_t callback_counter = 0;
// float* pPoints = points;
// size_t point_count = 0.0;


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

void LidarNode::lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_cloud)
{


    // sensor_msgs::PointCloud out_pointcloud;
    // sensor_msgs::convertPointCloud2ToPointCloud(*pcl_cloud, out_pointcloud);

    //size_t constexpr buffer_size = 999999;
    //float points[buffer_size];
    //float* pPoints = points;
    size_t point_count = 0.0;

    // Collect points
    // for(size_t i = 0 ; i < out_pointcloud.points.size() && point_count < buffer_size; i++)
    // {
    //     *pPoints++ = out_pointcloud.points[i].x;
    //     *pPoints++ = out_pointcloud.points[i].y;
    //     *pPoints++ = out_pointcloud.points[i].z;
    //     *pPoints++ = 0.0;
    //     *pPoints++ = 0.0;
    //     point_count++;
    // }
    // std::cout << "Point count:" << point_count << std::endl;

    // Convert to PCL data type
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcl_cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pt_cloud);

    std::cout << "Allocating memory for " << pt_cloud->points.size() << " points" << std::endl;
    float* const points = new float[pt_cloud->points.size()*5]; // ptr to beginning of the data array
    float* pPoints = points;
    std::cout << "points: " << hex << points << std::endl;
    std::cout << "pPoints: " << hex << pPoints << std::endl;

    std::cout << "Allocating memory for " << pt_cloud->points.size() << " points" << std::endl;


    for (auto p : pt_cloud->points)
    {
        *pPoints++ = p.x;
        *pPoints++ = p.y;
        *pPoints++ = p.z;
        *pPoints++ = 0.0;
        *pPoints++ = 0.0;
        point_count++;
    }
    std::cout << "Point count:" << point_count << "data len: " << 5*point_count << std::endl;
    std::cout << "2. points: " << hex << points << std::endl;
    std::cout << "2. pPoints: " << hex << pPoints << std::endl;

    // if (callback_counter % 5 == 0)
    // {
        std::cout << "Detecting, point count: " << point_count << std::endl;
        std::vector<float> out_detections;
        size_t box_count = detect(points, point_count, out_detections);

        odts_msgs::Detection3DArray detection_array;

        std::cout << "box count: " << box_count << std::endl;
        size_t publish_box_count = 0.0;
        for (size_t i = 0; i < out_detections.size(); i=i+7)
        {
            // cloud: (N, 4)  [x, y, z, intensity]
            // boxes: (n,7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw)   
            odts_msgs::Detection3D detection;
            detection.center.x = out_detections[i];
            detection.center.y = out_detections[i+1]; // flip y-axis, due /front_velocyne frame
            detection.center.z = out_detections[i+2];

            detection.size.x = out_detections[i+3]; 
            detection.size.y = out_detections[i+4]; // flip y-axis, due /front_velocyne frame
            detection.size.z = out_detections[i+5];
            float yaw = out_detections[i+6];
            // Direction arrow:
            float dir_x = detection.size.x / 2.0 * cos(yaw);
            float dir_y = detection.size.x / 2.0 * sin(yaw);
            // float arrow_origin = [x - dir_x, y - dir_y, z];
            // float arrow_end = [x + dir_x, y + dir_y, z];

            detection.orientation.x = detection.size.x * sin(yaw/2);
            detection.orientation.y = detection.size.y * sin(yaw/2);
            detection.orientation.z = detection.size.z * sin(yaw/2);
            detection.orientation.w = cos(yaw/2);

            //std::cout << "BBox message constructed!" << std::endl;
            detection_array.header.stamp = pcl_cloud->header.stamp; //ros::Time::now();
            detection_array.header.frame_id = frame_;
            detection_array.detections.push_back(detection);

            publish_box_count++;
            
            // Zero
            //pPoints = points;
            //point_count = 0.0;
        }

        if (box_count > 0)
        {
            std::cout << "publishing " << publish_box_count << "bboxes" << std::endl;
            detection_pub_.publish(detection_array);

            // Republish the input pointcloud. We haven't done any processing
            poincloud_pub_.publish(pcl_cloud);
        }
    //}
    delete points;
}

void Boxes2Txt( std::vector<float> boxes , string file_name , int num_feature = 7)
{
    ofstream ofFile;
    ofFile.open(file_name , std::ios::out );  
    if (ofFile.is_open()) {
        for (int i = 0 ; i < boxes.size() / num_feature ; ++i) {
            for (int j = 0 ; j < num_feature ; ++j) {
                ofFile << boxes.at(i * num_feature + j) << " ";
            }
            ofFile << "\n";
        }
    }
    ofFile.close();
    return ;
};

int Arrary2Txt(float* &points_array, size_t point_count, uint num_feature, string file_name)
{
  ofstream file;
  float* ptr = points_array;
  file.open(file_name.data());
  assert(file.is_open());

  for (size_t i = 1; i < point_count + 1; i++)
    {
        file << *ptr++ << " ";
        if (i % num_feature == 0)
        {
            file << "\n";
        }
    }

  std::cout << "wrote points_array contents\n";
  file.close();  

};

int LidarNode::detect(float* const pPoints, size_t in_num_points, std::vector<float>& out_detections)
{
    //std::vector<float> out_detections;
    std::vector<int> out_labels;
    std::vector<float> out_scores;
    float* points_array = pPoints;

    std::cout << "starting detecting" << std::endl;
    // Arrary2Txt(points_array, in_num_points, 5, "/app/model/input_point_cloud.txt");

    cudaDeviceSynchronize();
    pp_.DoInference(points_array, in_num_points, &out_detections, &out_labels , &out_scores);
    cudaDeviceSynchronize();
    int BoxFeature = 7;
    int num_objects = out_detections.size() / BoxFeature;
    std::cout << "Doing inference..." << std::endl;

    std::string boxes_file_name = config_["OutputFile"].as<std::string>();
    Boxes2Txt(out_detections , boxes_file_name );

    return num_objects;
};


/// 
//OLD LOGIC


    // // if (callback_counter % 5 == 0)
    // // {
    //     std::cout << "Detecting, point count: " << point_count << std::endl;
    //     std::vector<float> out_detections;
    //     size_t box_count = detect(pPoints, point_count, out_detections);

    //     odts_msgs::Detection3DArray detection_array;

    //     std::cout << "box count: " << box_count << std::endl;
    //     size_t publish_box_count = 0.0;
    //     for (size_t i = 0; i < out_detections.size(); i=i+7)
    //     {
    //         // cloud: (N, 4)  [x, y, z, intensity]
    //         // boxes: (n,7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw)   
    //         odts_msgs::Detection3D detection;
    //         detection.center.x = out_detections[i];
    //         detection.center.y = out_detections[i+1]; // flip y-axis, due /front_velocyne frame
    //         detection.center.z = out_detections[i+2];

    //         detection.size.x = out_detections[i+3]; 
    //         detection.size.y = out_detections[i+4]; // flip y-axis, due /front_velocyne frame
    //         detection.size.z = out_detections[i+5];
    //         float yaw = out_detections[i+6];
    //         // Direction arrow:
    //         float dir_x = detection.size.x / 2.0 * cos(yaw);
    //         float dir_y = detection.size.x / 2.0 * sin(yaw);
    //         // float arrow_origin = [x - dir_x, y - dir_y, z];
    //         // float arrow_end = [x + dir_x, y + dir_y, z];

    //         detection.orientation.x = detection.size.x * sin(yaw/2);
    //         detection.orientation.y = detection.size.y * sin(yaw/2);
    //         detection.orientation.z = detection.size.z * sin(yaw/2);
    //         detection.orientation.w = cos(yaw/2);

    //         //std::cout << "BBox message constructed!" << std::endl;
    //         detection_array.header.stamp = pcl_cloud->header.stamp; //ros::Time::now();
    //         detection_array.header.frame_id = frame_;
    //         detection_array.detections.push_back(detection);

    //         publish_box_count++;
            
    //         // Zero
    //         //pPoints = points;
    //         //point_count = 0.0;
    //     }

    //     if (box_count > 0)
    //     {
    //         std::cout << "publishing " << publish_box_count << "bboxes" << std::endl;
    //         detection_pub_.publish(detection_array);

    //         // Republish the input pointcloud. We haven't done any processing
    //         poincloud_pub_.publish(pcl_cloud);
    //     }
    // //}
