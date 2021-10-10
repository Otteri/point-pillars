#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream>
#include "pclbuff.hpp"

class PclBuffTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        // We must use the same nh between all test cases!
        nh_ = new ros::NodeHandle("~");

        // We can also reuse the same buffer instance
        data_buffer_ = new PclBuff(*nh_);

        // Start lidar data buffering process
        std::thread(&PclBuff::start, data_buffer_).detach();

        // It takes roughly a second for ROS to connect to the lidar topic
        ros::Duration(1.0).sleep();
    }

public:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }

    static ros::NodeHandle* nh_;
    static PclBuff* data_buffer_;
};

// Static storages
ros::NodeHandle* PclBuffTest::nh_;  // ros::init() must be called before this, thus ptr
PclBuff* PclBuffTest::data_buffer_; // We need nh_ for construting, thus ptr


TEST_F(PclBuffTest, DoesPointCloudConversion)
{
    ros::Rate loop_rate(40);
    float* data = nullptr;

    for(size_t i = 0; i < 100; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
        size_t point_count = PclBuffTest::data_buffer_->getData(&data);
        EXPECT_GT(point_count, 0);
    }

    ASSERT_TRUE(data);
}

TEST_F(PclBuffTest, StaysOver3Hz)
{
    ros::Rate loop_rate(40);
    float* data = nullptr;
    for(size_t i = 0; i < 100; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
        size_t point_count = PclBuffTest::data_buffer_->getData(&data);
        double hz = data_buffer_->getCurrentRunRate();
        EXPECT_GT(hz, 3);
    }
}

TEST_F(PclBuffTest, OccasionallyOver20Hz)
{
    ros::Rate loop_rate(40);
    float* data = nullptr;
    double max_hz = 0.0;
    for(size_t i = 0; i < 100; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
        size_t point_count = PclBuffTest::data_buffer_->getData(&data);
        double hz = data_buffer_->getCurrentRunRate();
        max_hz = (hz > max_hz) ? hz : max_hz;
    }
        EXPECT_GT(max_hz, 20);
}

auto main(int argc, char **argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lidar_detector_test");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    return RUN_ALL_TESTS();
}
