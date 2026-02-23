/*
    Created by: Aaron Rhim (aaronrhim)
    Created on: February 21, 2026
    purpose: to handle moveit control for new arm
*/

// ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "arm_constants_2.h"
#include "rover_msgs/msg/arm_command.hpp"

using namespace Rover2026Arm;
using std::placeholders::_1;

// path planning
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#define CARTESIAN_EE_FRAME 2
#define CARTESIAN_BASE_FRAME 3

class ArmMoveitControl2 : public rclcpp::Node {
    public:
        ArmMoveitControl2() : Node("Rover2026_control") {
            // initializations
            for (int i = 0; i < NUM_JOINTS; i++) {
                joint_states[i].zero_rad = ArmConstants::axis_zero_rads[i]; 
                joint_states[i].dir = ArmConstants::axis_dirs[i];
            }

            // Rowan: auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
            // path planning
            arm_publisher_ = this->create_publisher<rover_msgs::msg::ArmCommand>("joint_states", 3);

            // servo
            joint_cmd_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(ArmConstants::servo_fk_topic, 10);
            twist_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(ArmConstants::servo_ik_topic, 10);

            // the node instantly prints to moveit path planning and moveit servo
            joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
                ArmConstants::joy_topic, 3, std::bind(&ArmMoveitControl2::joyCallback, this, _1));
            servo_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                ArmConstants::servo_fk_topic, 3, std::bind(&ArmMoveitControl2::servoCallback, this, _1));
        }

    int count_ = 0;    
    void publishCommands();	
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    int joyControlMode = CARTESIAN_EE_FRAME;

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        struct JointStates {
            float position = 00.00;
            float velocity = 00.00;
            float zero_rad; // <-- this gets set in the loop
            int dir;        // <-- this gets set in the loop
        };
        JointStates joint_states[NUM_JOINTS];  // 6 axes on the arm + 1 for ee

        /* Instantiation */
        // path planning
        rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_publisher_;

        // servo
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_publisher_;

        // post-servo-callback
        rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr trajectory_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_subscriber;

        rclcpp::TimerBase::SharedPtr timer_;

        float radToDeg(float rad) {
            return (rad * 180.0) / PI;
        }

        float moveitToFirmwareOffset(float rad, int i) {
            float deg = (rad - joint_states[i].zero_rad) * joint_states[i].dir;
            return radToDeg(deg);
        }

        float moveitVelocityToFirmwareOffset(float rad, int i) {
            return radToDeg(rad) * joint_states[i].dir;
        }

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received joint state callback");
            rover_msgs::msg::ArmCommand target;
            target.positions.resize(NUM_JOINTS);
            target.cmd_type = 'P';

            for (int i = 0; i < NUM_JOINTS; i++) {
                target.positions[i] = moveitToFirmwareOffset(msg->position[i], i);
            }

            arm_publisher_->publish(target);
        }

        void jointTrajectoryCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received joint trajectory callback");
            rover_msgs::msg::ArmCommand target;
            target.velocities.resize(NUM_JOINTS_NO_EE);
            target.cmd_type = 'V';

            for (size_t i = 0; i < msg->output.velocities.size(); ++i) {
                target.velocities[i] = moveitVelocityToFirmwareOffset(msg->output.velocities[i], i);
            }
            arm_publisher_->publish(target);
        }

        void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received ArmCommand callback");
            for (size_t i = 0; i < msg->positions.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Joint %zu: Position = %f", i, msg->positions[i]);
            }
        }

        void servoCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {  
            RCLCPP_INFO(this->get_logger(), "Received servo callback");
            rover_msgs::msg::ArmCommand target;
            target.velocities.resize(NUM_JOINTS_NO_EE);
            target.cmd_type = 'V';
            
            for (int i = 0; i < NUM_JOINTS_NO_EE; i++) {
                joint_states[i].velocity = moveitVelocityToFirmwareOffset(msg->points[0].velocities[i], i);
            }

            arm_publisher_->publish(target); // publish to motors
        }
};