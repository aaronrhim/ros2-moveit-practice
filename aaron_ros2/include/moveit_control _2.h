/*
Created by: Aaron Rhim (aaronrhim)
Created on: February 21, 2026
purpose: to handle moveit control for new arm
*/

//ROS2
#include "rclcpp/rclcpp.hpp"

NUM_JOINTS = 6;
class ArmMoveitControl2 : public rclcpp::Node {
  public:
      ArmMoveitControl2() : Node("arm2_moveit_control") {
        // initializations
        for (int i = 0; i < NUM_JOINTS; i++) {
            axes[i].zero_rad = ArmConstants::axis_zero_rads[i]; 
            axes[i].dir = ArmConstants::axis_dirs[i];
        }

        // private publisher for the arm joints
        publisher_ = this->create_publisher<std_msgs::msg::String>("joint_states", 10);
      }

  private:
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

      struct Axis {
          float position = 00.00;
          float velocity = 00.00;
          float zero_rad;   // <-- this gets set in the loop
          int dir;           // <-- this gets set in the loop
      };
      Axis axes[NUM_JOINTS];  // 6 axes on the arm
};