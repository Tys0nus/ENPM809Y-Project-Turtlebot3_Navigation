#pragma once

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <string>
#include <sstream>


/**
 * @brief The Odom Updater class
 *
 * This class contains a non-static broadcaster and a subscriber to retrieve the robot pose
 *
 */

class odom_updater : public rclcpp::Node
{
public:
    odom_updater(std::string node_name) : Node(node_name)
    {
        // Initialize the transform broadcaster
        m_tf_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Subscribe to a /robot1/odom topic and call timer_callback
        m_tf_subscription = this->create_subscription<nav_msgs::msg::Odometry>
        ("/robot1/odom", 10, std::bind(&odom_updater::timer_callback, this, std::placeholders::_1));                                  
    }

private:
    // attributes

    // Shared pointer to a Subscription object for the /robot1/odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_tf_subscription;
    
    //Unique pointer to a transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster{nullptr};

    // methods
    void timer_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
};
