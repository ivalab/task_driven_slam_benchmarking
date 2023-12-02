/**
 * @file perfect_odometry_helper.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 12-01-2023
 * @copyright Copyright (c) 2023
 */

#include <closedloop_nav_slam/perfect_odometry_helper.h>

namespace cl
{

PerfectOdometryHelper::PerfectOdometryHelper(ros::NodeHandle& nh)
{
    InitRos(nh);
}

void PerfectOdometryHelper::InitRos(ros::NodeHandle& pnh)
{
    // Init tf.
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(/*ros::Duration(tf_buffer_dur_)*/);
    tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // Init params.
    pnh.param("map_frame", map_frame_, std::string("map"));
    pnh.param("base_frame", base_frame_, std::string("base_footprint"));
    pnh.param("transform_timeout", transform_timeout_, 0.2);
    pnh.param("transform_publish_period", transform_publish_period_, 0.05);
    odom_sub_ =
        pnh.subscribe("odom", 1, &PerfectOdometryHelper::OdomMsgCallback, this);
}

void PerfectOdometryHelper::OdomMsgCallback(
    const nav_msgs::OdometryConstPtr& msg)
{
    tf2::Transform map_to_world, base_to_odom, world_to_base;
    try
    {
        geometry_msgs::TransformStamped tf_msg = tf_buffer_->lookupTransform(
            map_frame_, msg->header.frame_id, ros::Time(0));
        tf2::convert(tf_msg.transform, map_to_world);
        tf_msg =
            tf_buffer_->lookupTransform(base_frame_, odom_frame_, ros::Time(0));
        tf2::convert(tf_msg.transform, base_to_odom);

        {
            const geometry_msgs::Pose&       pose        = msg->pose.pose;
            const geometry_msgs::Point&      position    = pose.position;
            const geometry_msgs::Quaternion& orientation = pose.orientation;

            world_to_base.setOrigin(
                tf2::Vector3(position.x, position.y, position.z));
            world_to_base.setRotation(tf2::Quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w));
        }
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
        return;
    }
    map_to_odom_ = map_to_world * world_to_base * base_to_odom;
    odom_header_ = msg->header;
}

void PerfectOdometryHelper::Run()
{
    ros::Rate r(1.0 / transform_publish_period_);
    while (ros::ok())
    {
        if (odom_header_.stamp.toSec() > 0.0 && !odom_header_.frame_id.empty())
        {
            geometry_msgs::TransformStamped msg;
            tf2::convert(map_to_odom_, msg.transform);
            msg.child_frame_id  = odom_frame_;
            msg.header.frame_id = map_frame_;
            msg.header.stamp =
                odom_header_.stamp + ros::Duration(transform_timeout_);
            tfB_->sendTransform(msg);
        }
        ros::spinOnce();
        r.sleep();
    }
}

}  // namespace cl

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "perfect_odometry_helper");
    ros::NodeHandle nh("~");
    ros::spinOnce();

    cl::PerfectOdometryHelper helper(nh);
    helper.Run();
    return 0;
}
