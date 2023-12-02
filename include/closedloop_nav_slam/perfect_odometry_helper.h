/**
 * @file perfect_odometry_helper.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 12-01-2023
 * @copyright Copyright (c) 2023
 */

#ifndef CLOSEDLOOP_NAV_SLAM_H_
#define CLOSEDLOOP_NAV_SLAM_H_

#include <memory>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace cl
{
class PerfectOdometryHelper
{
public:
    /**
     * @brief Construct a new Perfect Odometry Helper object
     *
     * @param pnh
     */
    PerfectOdometryHelper(ros::NodeHandle& pnh);

    /**
     * @brief
     *
     */
    void Run();

private:
    /**
     * @brief
     *
     */
    void InitRos(ros::NodeHandle& pnh);

    /**
     * @brief
     *
     * @param msg
     */
    void OdomMsgCallback(const nav_msgs::OdometryConstPtr& msg);

    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_footprint"};

    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener>    tfL_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
    double           transform_timeout_        = 0.0;   // seconds
    double           transform_publish_period_ = 0.05;  // seconds;
    tf2::Transform   map_to_odom_;
    std_msgs::Header odom_header_;
    ros::Subscriber  odom_sub_;
};

}  // namespace cl

#endif