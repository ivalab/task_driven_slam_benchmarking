/**
 * @file map_to_odom_publisher.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 11-26-2023
 * @copyright Copyright (c) 2023
 */

#include <closedloop_nav_slam/map_to_odom_publisher.h>

namespace cl
{

MapToOdomPublisher::MapToOdomPublisher(ros::NodeHandle& pnh)
{
    InitRos(pnh);
}

void MapToOdomPublisher::InitRos(ros::NodeHandle& pnh)
{
    // Init tf.
    tf_  = std::make_unique<tf2_ros::Buffer>(/*ros::Duration(tf_buffer_dur_)*/);
    tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // Init params.
    pnh.param("map_frame", map_frame_, std::string("map"));
    pnh.param("odom_frame", odom_frame_, std::string("odom"));
    pnh.param("base_frame", base_frame_, std::string("base_footprint"));
    pnh.param("transform_timeout", transform_timeout_, 0.5);
    pnh.param("transform_publish_period", transform_publish_period_, 0.05);

    std::string source_msg_type("odom");
    pnh.param("source_msg_type", source_msg_type, std::string("odom"));
    pnh.param("source_msg_parent_frame", source_msg_parent_frame_,
              std::string(""));
    pnh.param("source_msg_child_frame", source_msg_child_frame_,
              std::string(""));

    is_transform_valid_ = false;
    if (source_msg_parent_frame_.empty() || source_msg_child_frame_.empty())
    {
        mTp_.setIdentity();
        cTb_.setIdentity();
        is_transform_valid_ = true;
    }

    // Subscriber and publisher.
    int queue_size = 10;
    if ("pose" == source_msg_type)
    {
        pose_sub_.subscribe(pnh, "/source_msg_topic", queue_size);
        pose_sub_.registerCallback(
            boost::bind(&MapToOdomPublisher::PoseMsgCallback, this, _1));
    }
    else if ("odom" == source_msg_type)
    {
        odom_sub_.subscribe(pnh, "/source_msg_topic", queue_size);
        odom_sub_.registerCallback(
            boost::bind(&MapToOdomPublisher::OdomMsgCallback, this, _1));
    }
}

void MapToOdomPublisher::PoseMsgCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    static nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry());

    // Assign header.
    odom_msg->header = msg->header;
    // odom_msg->child_frame_id // undefined

    // Assign pose.
    odom_msg->pose = msg->pose;

    // Assign zero velocity.
    odom_msg->twist.twist.linear.x  = 0.0;
    odom_msg->twist.twist.linear.y  = 0.0;
    odom_msg->twist.twist.linear.z  = 0.0;
    odom_msg->twist.twist.angular.x = 0.0;
    odom_msg->twist.twist.angular.y = 0.0;
    odom_msg->twist.twist.angular.z = 0.0;

    // Make odom callback.
    this->OdomMsgCallback(odom_msg);
}

void MapToOdomPublisher::OdomMsgCallback(const nav_msgs::OdometryConstPtr& msg)
{
    if (!is_transform_valid_)
    {
        try
        {
            // Define map frame to align with base frame.
            geometry_msgs::TransformStamped tf_msg = tf_->lookupTransform(
                base_frame_, source_msg_parent_frame_, ros::Time(0), ros::Duration(0.1));
            tf2::convert(tf_msg.transform, mTp_);
            tf_msg = tf_->lookupTransform(source_msg_child_frame_, base_frame_,
                                          ros::Time(0), ros::Duration(0.1));
            tf2::convert(tf_msg.transform, cTb_);

            is_transform_valid_ = true;
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    // Compute map to base.
    tf2::Transform pTc;
    tf2::fromMsg(msg->pose.pose, pTc);
    const tf2::Transform mTb = mTp_ * pTc * cTb_;

    // Looking for base to odom
    tf2::Transform bTo;
    try
    {
        geometry_msgs::TransformStamped tf_msg = tf_->lookupTransform(
            base_frame_, odom_frame_, msg->header.stamp, ros::Duration(0.1));
        tf2::convert(tf_msg.transform, bTo);
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
        return;
    }

    // Compute map to odom;
    map_to_odom_ = mTb * bTo;
    msg_header_  = msg->header;
}

void MapToOdomPublisher::Run()
{
    ros::Rate r(1.0 / transform_publish_period_);
    while (ros::ok())
    {
        if (msg_header_.stamp.toSec() > 0.0 && !msg_header_.frame_id.empty())
        {
            geometry_msgs::TransformStamped msg;
            tf2::convert(map_to_odom_, msg.transform);
            msg.child_frame_id  = odom_frame_;
            msg.header.frame_id = map_frame_;
            msg.header.stamp =
                msg_header_.stamp + ros::Duration(transform_timeout_);
            tfB_->sendTransform(msg);
        }
        ros::spinOnce();
        r.sleep();
    }
}

}  // namespace cl

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "map_to_odom_publisher");
    ros::NodeHandle nh("~");
    ros::spinOnce();

    cl::MapToOdomPublisher m2o(nh);
    m2o.Run();
    return 0;
}
