/**
 * @file odometry_converter.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 11-05-2023
 * @copyright Copyright (c) 2023
 */

#ifndef CLOSED_LOOP_NAV_SLAM_ODOMETRY_CONVERTER_H_
#define CLOSED_LOOP_NAV_SLAM_ODOMETRY_CONVERTER_H_

#include <memory>
#include <string>

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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// #include <Eigen/Core>
// #include <Eigen/Geometry>

namespace cl
{
class MessageConverter
{
public:
    /**
     * @brief Construct a new Message Converter object
     *
     */
    MessageConverter(ros::NodeHandle& pnh);

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

    /**
     * @brief
     *
     * @param msg_in
     * @param msg_out
     */
    void TransformPose(const nav_msgs::Odometry& msg_in,
                       nav_msgs::Odometry&       msg_out) const;

    void TransformTwist(const nav_msgs::Odometry& msg_in,
                        nav_msgs::Odometry&       msg_out) const;

    // Frame info.
    std::string source_frame_{""};
    std::string target_frame_{""};
    std::string base_frame_{"base_footprint"};
    std::string prefix_{"visual"};

    // ROS info.
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>
                   pose_sub_;
    ros::Publisher odom_pub_;
    bool           is_sTt_valid_{false};
    tf2::Transform sTt_;  // transform from source to target

    // Tf info.
    // tf2_ros::StaticTransformBroadcaster static_br_;
    std::unique_ptr<tf2_ros::Buffer>               tf_;
    std::unique_ptr<tf2_ros::TransformListener>    tfL_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
    double transform_timeout_ = 0.5;  // seconds
};
}  // namespace cl

#endif