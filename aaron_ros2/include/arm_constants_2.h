#define PI 3.1415
#define NUM_JOINTS 7
#define NUM_JOINTS_NO_EE 6

// "folder" to keep things distinct but could move structs outside
namespace Rover2026Arm {
    // globals
    static constexpr float base_max_accel = 3.0;
    static constexpr float max_accel[NUM_JOINTS] = {base_max_accel,  //A1 - base rotation
                                                    base_max_accel,  //A2 - shoulder
                                                    base_max_accel,  //A3 - elbow
                                                    base_max_accel,  //A4 - elbow rotation
                                                    base_max_accel,  //A5 - wrist pitch
                                                    base_max_accel,  //A6 - wrist roll
                                                    base_max_accel}; //A7 - end effector

    struct ArmConstants{
        static constexpr float axis_zero_rads[NUM_JOINTS] = { 
            -0.9608,
            -1.9390,
            -1.3460,
            -5.5523+PI,
            2.2060-PI/3,
            0,
            0
        };
        
        static constexpr int axis_dirs[NUM_JOINTS] = {
            1,
            1, 
            1, 
            1,
            -1,
            -1,
            1
        };

        // cannot use standard:
        // static constexpr std::string_view command_topic = "/arm/command";
        // because rclcpp expects a const char* (char[])

        // custom
        static constexpr char command_topic[] = "/arm/command";
        static constexpr char sim_ee_topic[] = "/arm/ee_command/sim";

        // existing for sim
        static constexpr char sim_command_topic[] = "/arm/sim_command";
        static constexpr char joint_states_topic[] = "/joint_states";
        static constexpr char joy_topic[] = "/joy";
        
        // moveit topics
        static constexpr char servo_ik_topic[] = "/arm_moveit_control/delta_twist_cmds"; //inverse kinematics
        static constexpr char servo_fk_topic[] = "/arm_moveit_control/delta_joint_cmds"; //forward kinematics (joint space)
    };
}
