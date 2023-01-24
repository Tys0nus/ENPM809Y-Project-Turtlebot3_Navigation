#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

/**
 * @file target_reach.h
 * @author Sarin Ann Mathew (sarinann@umd.edu)
 * @author Aditi Bhoir      (abhoir@umd.edu)
 * @author Aditya Chaugule  (aditya97@umd.edu)
 * @brief Implementation of code to broadcast /robot1/base_footprint as a child of robot1/odom
 * @version 0.1
 * @date 2022-16-12
 */

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        /**
         * @brief Initializing the m_bot controller with shared pointer bot_controller
         */
        m_bot_controller = bot_controller;

        /**
         * @brief Initializing a boolean
         * 
         */
        i = false;
        
        /**
         * @brief Initializing the target position from the parameters from .yaml file
         * 
         */
        goal_x = this->declare_parameter<double>("aruco_target.x");
        goal_y = this->declare_parameter<double>("aruco_target.y");

        /**
         * @brief Initializing the frame of pose of the goal.
         * 
         */
        frame_id = this->declare_parameter<std::string>("final_destination.frame_id");

        /**
         * @brief Initializing the position of marker 0 from the .yaml file.
         * 
         */
        aruco_0_x = this->declare_parameter<double>("final_destination.aruco_0.x");
        aruco_0_y = this->declare_parameter<double>("final_destination.aruco_0.y");

        /**
         * @brief Initializing the position of marker 1 from the .yaml file.
         * 
         */
        aruco_1_x = this->declare_parameter<double>("final_destination.aruco_1.x");
        aruco_1_y = this->declare_parameter<double>("final_destination.aruco_1.y");

        /**
         * @brief Initializing the position of marker 2 from the .yaml file.
         * 
         */
        aruco_2_x = this->declare_parameter<double>("final_destination.aruco_2.x");
        aruco_2_y = this->declare_parameter<double>("final_destination.aruco_2.y");

        /**
         * @brief Initializing the position of marker 3 from the .yaml file.
         * 
         */
        aruco_3_x = this->declare_parameter<double>("final_destination.aruco_3.x");
        aruco_3_y = this->declare_parameter<double>("final_destination.aruco_3.y");

        /**
         * @brief Using set_goal method from bot_controller to set the goal.
         * 
         */
        m_bot_controller->set_goal(goal_x,goal_y);

        /**
         * @brief Initializing the publisher to publish the velocity to topic /robot1/cmd_vel.
         * 
         */
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        /**
         * @brief Initializing the subscriber to know if the goal is reached or not by subscribing to topic /goal_reached.
         * 
         */
        goal_reached_subscriber = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10, std::bind(&TargetReacher::timer_callback, this, std::placeholders::_1));

        /**
         * @brief Initizalizing the subscriber to know if the marker is found by subscribing to the topic /aruco_markers.
         * 
         */
        aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::check_marker, this, std::placeholders::_1));

        /**
         * @brief Initializing the static broadcaster to transform the goal.
         * 
         */
        final_destination_broadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        /**
         * @brief Creating a timer.
         * 
         */
        m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / 1.0)), std::bind(&TargetReacher::check_destination, this));

        /**
         * @brief Initizaling the buffer.
         * 
         */
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        /**
         * @brief Initializing the listener to listen to the transfrom.
         * 
         */
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);


    }

private:
    // attributes

    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> final_destination_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::string frame_id;
    

    //Defining the variables for parameters
    bool i;

    //Target
    double goal_x;
    double goal_y;

    //Marker 0
    double aruco_0_x;
    double aruco_0_y;

    //Marker 1
    double aruco_1_x;
    double aruco_1_y;

    //Marker 2
    double aruco_2_x;
    double aruco_2_y;

    //Marker 3
    double aruco_3_x;
    double aruco_3_y;

    //Method to transform the goal coordinates from the frame of the aruco 
    //to the frame of the robot through transform function
    void final_destination(int k);

    //Reading the marker_ids in the Aruco Interfaces and retrive the goal positions
    void check_marker(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> aruco);

    //Timer callback function to publish the twist velocities into robot1/cmd_vel
    void timer_callback(const std::shared_ptr<std_msgs::msg::Bool> msg);

    //Logging the movement of the robot using set_goal function while checking for error in TF tree
    void check_destination();
};