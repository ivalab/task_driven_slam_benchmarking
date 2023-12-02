/**
 * @file tf_to_pose_converter.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 12-02-2023
 * @copyright Copyright (c) 2023
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace cl
{
class TfToPoseConverter
{
public:
    /**
     * @brief Construct a new Tf To Pose Converter:: Tf To Pose Converter object
     *
     * @param pnh
     */
    TfToPoseConverter(ros::NodeHandle& pnh)
    {
        InitRos(pnh);
    }

    /**
     * @brief
     *
     */
    void Run()
    {
        if (source_frame_.empty() || target_frame_.empty())
        {
            ROS_INFO("Empty source or target frame, nothing to convert.");
            return;
        }

        ros::Rate r(1.0 / transform_publish_period_);
        while (ros::ok())
        {
            try
            {
                // Define map frame to align with base frame.
                geometry_msgs::TransformStamped tf_msg = tf_->lookupTransform(
                    source_frame_, target_frame_, ros::Time(0));

                geometry_msgs::PoseWithCovarianceStamped pose;
                pose.header                  = tf_msg.header;
                pose.pose.pose.position.x    = tf_msg.transform.translation.x;
                pose.pose.pose.position.y    = tf_msg.transform.translation.y;
                pose.pose.pose.position.z    = tf_msg.transform.translation.z;
                pose.pose.pose.orientation.x = tf_msg.transform.rotation.x;
                pose.pose.pose.orientation.y = tf_msg.transform.rotation.y;
                pose.pose.pose.orientation.z = tf_msg.transform.rotation.z;
                pose.pose.pose.orientation.w = tf_msg.transform.rotation.w;
                // Covariance is not assigned.

                pose_pub_.publish(pose);
            }
            catch (tf2::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    /**
     * @brief
     *
     * @param pnh
     */
    void InitRos(ros::NodeHandle& pnh)
    {
        // Init tf.
        tf_ = std::make_unique<tf2_ros::Buffer>(
            /*ros::Duration(tf_buffer_dur_)*/);
        tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
        tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();

        // Init params.
        pnh.param("source_frame", source_frame_, std::string(""));
        pnh.param("target_frame", target_frame_, std::string(""));
        pnh.param("transform_publish_period", transform_publish_period_, 0.05);

        pose_pub_ = pnh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/output_pose_topic", 10);
    }

    // Frame info.
    std::string source_frame_{""};
    std::string target_frame_{""};

    // Tf info.
    std::unique_ptr<tf2_ros::Buffer>               tf_;
    std::unique_ptr<tf2_ros::TransformListener>    tfL_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
    double transform_publish_period_ = 0.05;  // seconds;

    ros::Publisher pose_pub_;
};

}  // namespace cl

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "tf_to_pose_converter");
    ros::NodeHandle nh("~");
    ros::spinOnce();

    cl::TfToPoseConverter converter(nh);
    converter.Run();
    return 0;
}
