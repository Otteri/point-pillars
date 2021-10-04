#include "pclbuff.hpp"

#include <iostream>
#include <sensor_msgs/point_cloud_conversion.h>

// Notice that it takes a second to produce first processed pointcloud,
// so the default value should be good as well because program tries to use these.
PclBuff::PclBuff(const ros::NodeHandle& nh)
: storage1{nullptr, 0, true}
, storage2{nullptr, 0, true}
, storage(storage1)
, run_(true)
{
    storage = storage1;
    feature_count = 5; // [x,y,z,w,i]

    std::string pcl_topic;
    nh.param<std::string>("lidar_topic", pcl_topic, "/lidar_points_rectified");

    int input_queue_size;
    nh.param<int>("input_queue_size", input_queue_size,  1);

    nh.param<double>("loop_rate_hz", loop_rate_hz_, 40.0);

    // Subscriber for lidar messages
    pcl_sub_ = nh_.subscribe(
        pcl_topic, input_queue_size, &PclBuff::lidarDataCallback, this, ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("PclBuff initialized and listening: '" << pcl_topic << "'" << std::endl);
}

void PclBuff::start()
{
    ros::Rate loop_rate(loop_rate_hz_);

    // This process just runs continuously.
    // With loop rate, we can set how often ROS messages are checked
    // and how often lidarDataCallback and its logic can be executed.
    while (run_)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PclBuff::stop()
{
    run_ = false;
}

void PclBuff::lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
{
    sensor_msgs::PointCloud pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*pcl_msg, pointcloud);

    storage = selectStorage();

    // Allocate buffer for point data
    size_t point_count = pcl_msg->width * pcl_msg->height;
    storage.size = point_count * feature_count;
    storage.data = new float[storage.size];

    processData(storage, pointcloud);

    //delete data;

    // if (storage.is_available)
    // {
    //     clearStorage(storage); // storage.clear() if it was class...
    // }
}

void PclBuff::processData(Storage storage, sensor_msgs::PointCloud pcl)
{
    float* pPoints = storage.data;

    // Collect points
    for(size_t i = 0 ; i < pcl.points.size(); i++)
    {
        *pPoints++ = pcl.points[i].x;
        *pPoints++ = pcl.points[i].y;
        *pPoints++ = pcl.points[i].z;
        *pPoints++ = 0.0;
        *pPoints++ = 0.0;
    }
}

// Currently storage selection logic is bit dumb,
// Should be made smarter
Storage& PclBuff::selectStorage()
{
    // clear storage here?
    // static count: i % 2
    //return storage1.data;

    return (storage1.is_available) ? storage1 : storage2;
}

// Storage refers to most fresh storage
// Returns number of items in the storage
size_t PclBuff::getData(float** data)
{
    storage.is_available = false;
    *data = storage.data;
    size_t point_count = storage.size / feature_count;
    return point_count;
}

void PclBuff::markDone()
{
    if (!storage1.is_available && storage1.size > 0)
    {
        storage1.clear();
        storage1.is_available = true;
    }
    else if(!storage2.is_available && storage2.size > 0)
    {
        storage2.clear();
        storage2.is_available = true;
    }
}

void Storage::clear()
{
    size = 0;
    delete data;
}
