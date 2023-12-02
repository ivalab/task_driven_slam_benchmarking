/**
 * @file odometry_converter.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 11-05-2023
 * @copyright Copyright (c) 2023
 */

#ifndef CLOSED_LOOP_NAV_SLAM_MAP_TO_ODOM_PUBLISHER_H_
#define CLOSED_LOOP_NAV_SLAM_MAP_TO_ODOM_PUBLISHER_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// #include <Eigen/Core>
// #include <Eigen/Geometry>

namespace cl
{
class MapToOdomPublisher
{
public:
    /**
     * @brief Construct a new Message Converter object
     *
     */
    MapToOdomPublisher(ros::NodeHandle& pnh);

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
    void PoseMsgCallback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    /**
     * @brief
     *
     * @param msg
     */
    void OdomMsgCallback(const nav_msgs::OdometryConstPtr& msg);

    // Frame info.
    std::string source_msg_parent_frame_{""};
    std::string source_msg_child_frame_{""};
    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};

    std::string base_frame_{"base_footprint"};

    // ROS info.
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>
        pose_sub_;
    // oTb = oTp_ * pTc * cTb_; pTc is from source msg
    tf2::Transform mTp_;  // transform from map to source_msg_parent frame
    tf2::Transform cTb_;  // transform from source_msg_child to base frame
    bool           is_transform_valid_{false};

    // Tf info.
    // tf2_ros::StaticTransformBroadcaster static_br_;
    std::unique_ptr<tf2_ros::Buffer>               tf_;
    std::unique_ptr<tf2_ros::TransformListener>    tfL_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
    double transform_timeout_        = 0.0;   // seconds
    double transform_publish_period_ = 0.05;  // seconds;

    tf2::Transform   map_to_odom_;
    std_msgs::Header msg_header_;
};
}  // namespace cl

#endif