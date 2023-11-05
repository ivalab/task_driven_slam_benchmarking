/**
 * @file odometry_converter.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 11-05-2023
 * @copyright Copyright (c) 2023
 */

#include <closedloop_nav_slam/odometry_converter.h>

namespace cl
{

geometry_msgs::TransformStamped OdomToTf(const nav_msgs::Odometry& odom,
                                         double transform_timeout = 0.0)
{
    geometry_msgs::TransformStamped transform;
    transform.header = odom.header;
    transform.header.stamp =
        odom.header.stamp + ros::Duration(transform_timeout);
    transform.child_frame_id = odom.child_frame_id;

    geometry_msgs::Vector3&     trans = transform.transform.translation;
    const geometry_msgs::Point& pos   = odom.pose.pose.position;

    trans.x = pos.x;
    trans.y = pos.y;
    trans.z = pos.z;

    transform.transform.rotation = odom.pose.pose.orientation;

    return transform;
}

tf2::Transform PoseToTf(const geometry_msgs::Pose& pose)
{
    return tf2::Transform{{
                              pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w,
                          },
                          {
                              pose.position.x,
                              pose.position.y,
                              pose.position.z,
                          }};
}

geometry_msgs::Pose TfToPose(const tf2::Transform& transform)
{
    geometry_msgs::Pose out;
    out.position.x    = transform.getOrigin().x();
    out.position.y    = transform.getOrigin().y();
    out.position.z    = transform.getOrigin().z();
    out.orientation.x = transform.getRotation().x();
    out.orientation.y = transform.getRotation().y();
    out.orientation.z = transform.getRotation().z();
    out.orientation.w = transform.getRotation().w();
    return out;
}

MessageConverter::MessageConverter(ros::NodeHandle& pnh)
{
    InitRos(pnh);
}

void MessageConverter::InitRos(ros::NodeHandle& pnh)
{
    // Init tf.
    tf_  = std::make_unique<tf2_ros::Buffer>(/*ros::Duration(tf_buffer_dur_)*/);
    tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // Init params.
    std::string source_msg_type("odom");
    pnh.param("source_msg_type", source_msg_type, std::string("odom"));

    pnh.param("prefix", prefix_, std::string("visual"));
    pnh.param("base_frame", base_frame_, std::string("base_footprint"));
    pnh.param("source_frame", source_frame_, std::string(""));
    pnh.param("target_frame", target_frame_, std::string(""));
    pnh.param("transform_timeout", transform_timeout_, 0.5);
    is_sTt_valid_ = false;
    if (source_frame_.empty() || target_frame_.empty())
    {
        is_sTt_valid_ = true;
        sTt_.setIdentity();  // identity
    }

    // Subscriber and publisher.
    int queue_size = 1000;
    if ("pose" == source_msg_type)
    {
        pose_sub_.subscribe(pnh, "/source_msg_topic", queue_size);
        pose_sub_.registerCallback(
            boost::bind(&MessageConverter::PoseMsgCallback, this, _1));
    }
    else if ("odom" == source_msg_type)
    {
        odom_sub_.subscribe(pnh, "/source_msg_topic", queue_size);
        odom_sub_.registerCallback(
            boost::bind(&MessageConverter::OdomMsgCallback, this, _1));
    }

    odom_pub_ =
        pnh.advertise<nav_msgs::Odometry>("/" + prefix_ + "/odom", queue_size);
}

void MessageConverter::PoseMsgCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    static nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry());
    odom_msg->header = msg->header;
    // odom_msg->child_frame_id
    odom_msg->pose                  = msg->pose;
    odom_msg->twist.twist.linear.x  = 0.0;
    odom_msg->twist.twist.linear.y  = 0.0;
    odom_msg->twist.twist.linear.z  = 0.0;
    odom_msg->twist.twist.angular.x = 0.0;
    odom_msg->twist.twist.angular.y = 0.0;
    odom_msg->twist.twist.angular.z = 0.0;
    this->OdomMsgCallback(odom_msg);
}

void MessageConverter::OdomMsgCallback(const nav_msgs::OdometryConstPtr& msg)
{
    if (!is_sTt_valid_)
    {
        try
        {
            const geometry_msgs::TransformStamped tf_msg = tf_->lookupTransform(
                source_frame_, target_frame_, ros::Time(0));
            tf2::convert(tf_msg.transform, sTt_);
            is_sTt_valid_ = true;
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
    }

    // copy from the original message
    nav_msgs::Odometry out;
    out.header          = msg->header;
    out.header.frame_id = "odom";
    out.child_frame_id  = "/" + prefix_ + "/" + base_frame_;

    TransformPose(*msg, out);
    TransformTwist(*msg, out);

    // TODO
    // transform the "cov" values
    // for now a simple linear model is assumed;
    // more rigourous impl can be found at mrpt package:
    // http://wiki.ros.org/pose_cov_ops
    out.pose.covariance  = msg->pose.covariance;
    out.twist.covariance = msg->twist.covariance;

    // Pub odom;
    odom_pub_.publish(out);

    // Convert odom to tf;
    const geometry_msgs::TransformStamped out_tf =
        OdomToTf(out, transform_timeout_);

    tfB_->sendTransform(out_tf);
}

void MessageConverter::TransformPose(const nav_msgs::Odometry& msg_in,
                                     nav_msgs::Odometry&       msg_out) const
{
    //
    const tf2::Transform input_pose  = PoseToTf(msg_in.pose.pose);
    const tf2::Transform output_pose = sTt_ * input_pose;
    // tf::Transformer::transformPose(odom_link_, input_pose, output_pose);

    msg_out.pose.pose = TfToPose(output_pose);
}

void MessageConverter::TransformTwist(const nav_msgs::Odometry& msg_in,
                                      nav_msgs::Odometry&       msg_out) const
{
    //
    tf2::Vector3 twist_rot(msg_in.twist.twist.angular.x,
                           msg_in.twist.twist.angular.y,
                           msg_in.twist.twist.angular.z);
    tf2::Vector3 twist_vel(msg_in.twist.twist.linear.x,
                           msg_in.twist.twist.linear.y,
                           msg_in.twist.twist.linear.z);

    tf2::Vector3 out_rot = sTt_.getBasis() * twist_rot;
    tf2::Vector3 out_vel =
        sTt_.getBasis() * twist_vel + sTt_.getOrigin().cross(out_rot);

    msg_out.twist.twist.linear.x  = out_vel.x();
    msg_out.twist.twist.linear.y  = out_vel.y();
    msg_out.twist.twist.linear.z  = out_vel.z();
    msg_out.twist.twist.angular.x = out_rot.x();
    msg_out.twist.twist.angular.y = out_rot.y();
    msg_out.twist.twist.angular.z = out_rot.z();
}

}  // namespace cl

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "odometry_converter");
    ros::NodeHandle nh("~");
    ros::spinOnce();

    cl::MessageConverter mc(nh);
    ros::spin();
    return 0;
}
