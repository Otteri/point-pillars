#pragma once

#include <ros/ros.h>
#include <mutex>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

class Storage
{
public:
    float* data;
    size_t size;
    bool is_available;

    void clear();
};

class PclBuff
{
public:
    PclBuff(const ros::NodeHandle& nh);

    size_t getData(float** data); // We only update pointer, thus **
    void markDone();
    void clearStorage(Storage storage); // Priv

    void start();
    void stop();

    Storage& selectStorage();

private:
    void lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_cloud);

    void processData(Storage storage, sensor_msgs::PointCloud input);
    //float* const selectStorage();
    void deleteStorage();

    uint feature_count;

    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;

    Storage storage1;
    Storage storage2;
    Storage& storage; // active storage

    bool run_; // should this process run?
    double loop_rate_hz_; // How fast we would like it to run?
};
