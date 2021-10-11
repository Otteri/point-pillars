#pragma once

#include <ros/ros.h>
#include <mutex>
#include <chrono>
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
    double getCurrentRunRate();
    void markDone();

    void start();
    void stop();


private:
    void lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_cloud);
    void processData(Storage storage, sensor_msgs::PointCloud input);
    Storage& selectStorage();
    void deleteStorage();

    uint feature_count;

    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;

    Storage storage1;
    Storage storage2;
    Storage& storage; // active storage

    bool run_; // should this process run?
    double loop_rate_hz_; // How fast we would like it to run?
    double current_loop_rate_hz_; // How fast it actually runs

    std::chrono::high_resolution_clock::time_point previous_time_;
};
