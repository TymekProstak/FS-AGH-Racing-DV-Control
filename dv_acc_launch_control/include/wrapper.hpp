#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

#include <dv_interfaces/Control.h>
#include <dv_interfaces/DV_board.h>
#include <dv_interfaces/AccDebug_long.h>
#include <dv_interfaces/AccDebug_lat.h>
#include <dv_interfaces/Acc_handling_info.h>
#include <dv_interfaces/Imu.h>
#include <dv_interfaces/OdomDebug.h>
#include <dv_interfaces/EmergencyInfo.h>

#include "acc_types.hpp"
#include "ParamBank.hpp"
#include "pid.hpp"

#include "ltv_mpc_unbounded.hpp"
#include "offline_map_handler.hpp"
#include "math_utils.hpp"
#include "longitudal_utils.hpp"
#include "emergency_check.hpp"
#include "dv_control_common/load_transfer.hpp"

namespace acc_launch_control
{

// =============================================================================
//                              DEBUG STRUCTS
// =============================================================================

struct LongitudinalDebugInfo
{
    bool if_safe = true;

    double v_curr_mps = 0.0;
    double v_ref_mps = 0.0;
    double v_error_mps = 0.0;

    double torque_cmd_Nm = 0.0;
    double throttle_cmd_percent = 0.0;

    double s_curr_m = 0.0;
    double s_total_m = 0.0;
    double distance_left_to_end_m = 0.0;

    DrivePhase phase = DrivePhase::WAITING;

    std::string safety_reason = "OK";
    bool emergency_brake_requested = false;
};


struct LateralDebugInfo
{
    bool if_safe = true;

    double ey_m = 0.0;
    double epsi_rad = 0.0;

    double curr_steer_rad = 0.0;
    double ref_steer_rad = 0.0;
    double steer_error_rad = 0.0;

    double curr_yaw_rate_radps = 0.0;
    double ref_yaw_rate_radps = 0.0;
    double yaw_rate_error_radps = 0.0;

    double curr_vy_mps = 0.0;
    double ref_vy_mps = 0.0;
    double vy_error_mps = 0.0;
};


struct ControlCommand
{
    double steer_cmd_rad = 0.0;

    // Everything inside the controller is wheel-side torque [Nm].
    // No percent in the control pipeline.
    WheelValues wheel_torque_cmd_Nm;
    double total_torque_cmd_Nm = 0.0;

    // Final value written into dv_interfaces::Control::movement:
    //     movement = total_wheel_torque_Nm / gear_ratio
    // This is not percent.
    double movement_cmd_interface = 0.0;

    bool finished = false;
};


struct GlobalHandlingInfo
{
    bool has_valid_path_from_pp = false;
    bool has_received_first_dv_board_message = false;
    bool cube_mars_initialization_finished = false;
    bool has_odometry_message = false;
    bool has_received_imu_message = false;

    bool ready_to_start_drive = false;
    bool as_finished = false;
};


struct PhaseInfo
{
    DrivePhase phase = DrivePhase::WAITING;

    ros::Time ros_time_now_s;
    ros::Time phase_start_time_s = ros::Time(0.0);

    ros::Time launch_start_time_s = ros::Time(0.0);
    ros::Time brake_start_time_s = ros::Time(0.0);
    ros::Time emergency_brake_start_time_s = ros::Time(0.0);

    bool started_yet = false;
};

struct EmergencyCheckInputCache
{
    bool new_encoder_message_get = false;
    bool new_ins_message_get = false;
    bool new_path_planner_message_get = false;

    double encoder_position_rad = 0.0;
    double encoder_reference_position_rad = 0.0;

    Eigen::Vector2d ins_xy = Eigen::Vector2d::Zero();
    Eigen::Vector2d ins_velocity_body_xy_mps = Eigen::Vector2d::Zero();

    Eigen::Vector2d bolide_xy = Eigen::Vector2d::Zero();

    std::vector<Eigen::Vector2d> path_xy;

    bool spline_is_valid = false;
    bool path_is_track_closed = false;
};


// =============================================================================
//                              CONTROLLER
// =============================================================================

class Controller
{
public:
    Controller() = delete;
    Controller(ros::NodeHandle& nh, const ParamBank& param);
    ~Controller() = default;

    void pathCallback(const geometry_msgs::PoseArray& msg);

    // Ten callback jest głównym callbackiem sterowania.
    // Biorę z niego pose  oraz wołam główną pętlę sterowania, która na tej podstawie decyduje o fazie, i generuje komendy.
    void poseCallback(const nav_msgs::Odometry& msg);

    // opcjonalny callback zawierający informację dt. prędkości, w przypadku gdy 
    // w pose callback nie ma valid prędkości
    void odometryCallback(const nav_msgs::Odometry& msg);
    void dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg);

    void angleSensorCallback(const std_msgs::Float64& msg);
    void cubeMarsStatusCallback(const std_msgs::Bool& msg);


    void odometryDebugCallback(const dv_interfaces::OdomDebug::ConstPtr& msg);
    void imuCallback(const dv_interfaces::Imu::ConstPtr& msg);

private:
    // -------------------------------------------------------------------------
    // ROS
    // -------------------------------------------------------------------------

    ros::Subscriber path_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber dv_board_sub_;
    ros::Subscriber angle_sensor_sub_;
    ros::Subscriber cube_mars_status_sub_;
    ros::Subscriber odom_debug_sub_;
    ros::Subscriber imu_sub_;

    ros::Timer emergency_check_timer_;

    ros::Publisher pub_control_;
    ros::Publisher pub_acc_debug_long_;
    ros::Publisher pub_acc_debug_lat_;
    ros::Publisher pub_global_handling_info_;
    ros::Publisher pub_emergency_check_;
    ros::Publisher pub_ref_path_;
    ros::Publisher pub_ref_point_;

    // -------------------------------------------------------------------------
    // Params
    // -------------------------------------------------------------------------

    ParamBank param_;

    // -------------------------------------------------------------------------
    // Vehicle state
    // -------------------------------------------------------------------------

    State current_state_;

    double current_x_m_ = 0.0;
    double current_y_m_ = 0.0;
    double current_yaw_rad_ = 0.0;

    // -------------------------------------------------------------------------
    // Path from path planner
    // -------------------------------------------------------------------------

    Eigen::VectorXd X_last_from_pp_;
    Eigen::VectorXd Y_last_from_pp_;

    bool has_valid_path_from_pp_ = false;

    double path_start_x_m_ = 0.0;
    double path_start_y_m_ = 0.0;

    double path_dir_x_ = 1.0;
    double path_dir_y_ = 0.0;

    bool has_valid_path_axis_ = false;


   
    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    UnboundedLtvMpc ltv_mpc_unbounded_;

    PIDController pid_speed_hold_;
    dv_control_common::LoadTransferRelaxation load_transfer_relaxation_;

    Result lateral_controller_result_;

    LateralControllerType lateral_controller_type_ = LateralControllerType::NONE;

    // -------------------------------------------------------------------------
    // Launch runtime map handler
    // -------------------------------------------------------------------------
    acc_runtime_maps::LaunchRuntimeMapHandler runtime_map_handler;
    acc_runtime_maps::LaunchRuntimeCommand current_runtime_command_;
    acc_runtime_maps::LaunchPhaseEndInfo launch_end_info_;

    // -------------------------------------------------------------------------
    // Output / phase / global / debug
    // -------------------------------------------------------------------------

    ControlCommand control_output_;

    PhaseInfo phase_info_;
    GlobalHandlingInfo global_handling_info_;

    LongitudinalDebugInfo long_debug_;
    LateralDebugInfo lat_debug_;

    bool emergency_latched_ = false;
    std::string emergency_reason_ = "OK";


    EmergencyCheck emergency_checker_;
    EmergencyCheckInputCache emergency_check_input_;

    bool emergency_check_current_ = false;
    bool hard_emergency_check_current_ = false;

    std::string emergency_check_current_reason_ = "OK";
    std::string hard_emergency_check_current_reason_ = "OK";

private:
    // -------------------------------------------------------------------------
    // General mode
    // -------------------------------------------------------------------------
    // One longitudinal pipeline now:
    // WAITING -> LAUNCH -> ACCELERATION -> BRAKING -> COASTING -> FINISHED.
    // SPEED_HOLD is only a speed cap when vx exceeds general.vx_max.
    // Emergency is always hard-latched. It publishes Control.finished=true,
    // but it does NOT set handling_info.as_finished.
    void updateReadyFlag();
    void resetSpeedPid();
    void setPhase(DrivePhase new_phase);
    double getControlDt() const;
    void initializeSteeringStateIfNeeded();
    void updateSteeringStateForController();


    double integrateSteeringCommand(double u_delta_cmd);

    void StateMachinePhaseTransitionLogic();
    void emergencyCheckTimerCallback(const ros::TimerEvent& event); // REALY IMPORTANT -> this is defuelt funciton to handle all phase realted logic, and should be called inside main control loop


    // -------------------------------------------------------------------------
    // Phase commands
    // -------------------------------------------------------------------------

    double computeLaunchThrottleNm(const State& state, double dt_since_phase_start);

    double computeRuntimeAccelerationThrottleNm(const State& state,
                                                double dt_since_phase_start);

    double computeSpeedHoldThrottleNm(const State& state,
                                      double dt_since_phase_start);

    double computeRuntimeBrakeThrottleNm(const State& state,
                                         double dt_since_phase_start);

    double computeEmergencyBrakeThrottleNm(const State& state,
                                           double dt_since_phase_start);

    bool shouldStartBrake(const State& state) const;

    // --------------------------------------------------------------------------
    // Lateral control
    // -------------------------------------------------------------------------
    double computeLateralControlCommand();

    // -------------------------------------------------------------------------
    // Path
    // -------------------------------------------------------------------------

    void computePathErrorsAndUpdatState();

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------

    void publishControl(double steering_rad,
                        const WheelValues& wheel_torque_cmd_Nm,
                        bool finished);
    void publishZeroControl(bool finished);
    void publishZeroMovement(double steering_rad, bool finished);
    void publishZeroSteering(const WheelValues& wheel_torque_cmd_Nm,
                             bool finished);
    void publishLogInfoLongitudal();
    void publishLogInfoLateral();
    void publishGlobalHandlingInfo();
    void publishAllDebug();
    void publishEmergencyCheckDebug();
    // -------------------------------------------------------------------------
    // Visualization publishers
    void publishReferencePath(const Eigen::VectorXd& X,
                              const Eigen::VectorXd& Y);
    void publishReferencePoint();

    // -------------------------------------------------------------------------
    // Steering state
    // -------------------------------------------------------------------------
    /*
        Steering convention used by this ACC wrapper:

            delta_cmd_:
                absolute command published to /dv_board/control,
                obtained by integrating u_delta_cmd.

            current_state_.delta:
                measured physical steering angle from the encoder.

            delta_dot_model_:
                PT2 model steering-rate state returned as delta_dot_next
                by LTV MPC. The encoder is never numerically differentiated.

        PT2 is the actuator model used by the controller.
    */
    bool steering_state_initialized_ = false;

    double delta_cmd_ = 0.0;
    double delta_dot_model_ = 0.0;

    double delta_encoder_ = 0.0;
    bool has_delta_encoder_ = false;

};


} // namespace acc_launch_control
