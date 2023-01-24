#include <string>
#include "odom_updater.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief The timer_callback method 
 *
 * This method connects the tf tree 
 *
 */
void odom_updater::timer_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    geometry_msgs::msg::TransformStamped t;

    std::string odom = "/robot1/odom";
    std::string footprint = "/robot1/base_footprint";

    /*******************************************
     * Broadcaster: "/robot1/odom" -> "/robot1/base_footprint
     *******************************************/
    t.header.stamp = this->get_clock()->now();

    //Declaring the parent and child frame
    t.header.frame_id = odom;
    t.child_frame_id = footprint;

    //Retrieving the position of the robot
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    //Retrieving the orientation of the robot
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    m_tf_broadcaster->sendTransform(t);

}

