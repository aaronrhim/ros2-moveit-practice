/*
    Created by: Aaron Rhim (aaronrhim)
    Created on: February 21, 2026
    purpose: to handle moveit control for new arm
*/

// ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "arm_constants_2.h"

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
            // private publisher for the arm joints
            arm_publisher_ = this->create_publisher<std_msgs::msg::String>("joint_states", 3);
            // joint_cmd_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(ArmConstants::servo_fk_topic, 10);
            // twist_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(ArmConstants::servo_ik_topic, 10);

            // the node instantly prints to moveit path planning and moveit servo
            joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
                ArmConstants::joy_topic, 3, std::bind(&ArmMoveitControl2::joyCallback, this, _1));
            servo_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                ArmConstants::servo_topic, 3, std::bind(&ArmMoveitControl2::servoCallback, this, _1));
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

        void servoCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {            
            for (int i = 0; i < NUM_JOINTS_NO_EE; i++) {
                joint_states[i].velocity = moveitVelocityToFirmwareOffset(msg->points[0].velocities[i], i);
            }

            publisher_->publish(target); // publish to motors
        }
};