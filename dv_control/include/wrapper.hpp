#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>

#include <nlohmann/json.hpp>

#include <dv_interfaces/DV_board.h>
#include <dv_interfaces/Control.h>
#include <dv_interfaces/Imu.h>
#include <dv_interfaces/OdomDebug.h>
#include <dv_interfaces/Path.h>
#include <dv_interfaces/ControlDebug_long.h>
#include <dv_interfaces/ControlDebug_lat.h>
#include <dv_interfaces/Controlhandling_info.h>
#include <dv_interfaces/EmergencyAutoX.h>

#include "ParamBank.hpp"
#include "control_types.hpp"
#include "path_processing.hpp"
#include "longitudal_utils.hpp"
#include "math_utilis.hpp"

#include "mpc_ltv_unbouded.hpp"
#include "pid.hpp"
#include "smooth_path_qp.hpp"
#include "emergency_check.hpp"
#include "4wheel_utilites.hpp"
#include "dv_control_common/load_transfer.hpp"

namespace dv_control
{

using json = nlohmann::json;

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

    /*
        Debug conversion for the throttle-like message field. The controller
        pipeline itself is Nm-only.
    */
    double throttle_cmd_percent = 0.0;

    double s_curr_m = 0.0;

    DrivePhase phase = DrivePhase::WAITING;

    std::string safety_reason = "OK";
    bool emergency_brake_requested = false;

    double ax_mps2 = 0.0;
    double ax_ref_mps2 = 0.0;
    double ax_error_mps2 = 0.0;

};


struct LateralDebugInfo
{
    bool if_safe = true;
    bool solver_failed = false;

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

    double ay_mps2 = 0.0;

    bool optimized_torque_vectoring_used = false;
    bool jaca_torque_vectoring_used = false;

    double tv_yaw_moment_Nm = 0.0;
    double tv_yaw_moment_raw_Nm = 0.0;

    /*
        Diagnostic only:
            yaw moment that JACA would request at the current vehicle state,
            regardless of which TV source is currently active.
    */
    double jaca_tv_yaw_moment_Nm = 0.0;
};


struct ControlCommand
{
    double steer_cmd_rad = 0.0;

    bool finished = false;

    /*
        Nm-only internal convention.

        base_torque_cmd_Nm:
            requested total longitudinal wheel-side torque [Nm].

        tv_yaw_moment_raw_Nm:
            raw yaw-moment request produced by the selected TV source.

        tv_yaw_moment_Nm:
            yaw-moment request passed to the allocator.

        torque_allocation:
            single source of truth for the final FL/FR/RL/RR wheel torques,
            allocator limits and allocator diagnostics.
    */
    double base_torque_cmd_Nm = 0.0;

    double tv_yaw_moment_Nm = 0.0;
    double tv_yaw_moment_raw_Nm = 0.0;

    bool optimized_torque_vectoring_used = false;
    bool jaca_torque_vectoring_used = false;

    SimpleTorqueAllocatorResult torque_allocation;
    bool torque_allocation_valid = false;
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

    bool final_brake_requested = false;

    int lap_count = 0;
};


struct PhaseInfo
{
    DrivePhase phase = DrivePhase::WAITING;

    ros::Time ros_time_now_s = ros::Time(0.0);
    ros::Time phase_start_time_s = ros::Time(0.0);

    ros::Time launch_start_time_s = ros::Time(0.0);
    ros::Time brake_start_time_s = ros::Time(0.0);
    ros::Time emergency_brake_start_time_s = ros::Time(0.0);

    bool started_yet = false;
    bool should_make_final_brake = false;
};


struct EmergencyCheckInput
{
    bool new_encoder_message = false;
    bool new_ins_message = false;

    double encoder_position_rad = 0.0;
    double encoder_reference_position_rad = 0.0;

    Eigen::Vector2d ins_xy =
        Eigen::Vector2d::Zero();

    /*
        Raw path points used only by EmergencyCheck for the closest-point
        distance check. Updated when a new path message arrives.
    */
    std::vector<Eigen::Vector2d> path_xy;
};

// =============================================================================
//                              CONTROLLER
// =============================================================================

class Controller
{
public:
    Controller() = delete;
    ~Controller() = default;

    Controller(ros::NodeHandle& nh, const ParamBank& param);

    void pathCallback(const dv_interfaces::Path& msg);
    void poseCallback(const nav_msgs::Odometry& msg);

    void dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg);
    void imuCallback(const dv_interfaces::Imu::ConstPtr& msg);
    void angleSensorCallback(const std_msgs::Float64& msg);
    void cubeMarsStatusCallback(const std_msgs::Bool& msg);
    void odometryDebugCallback(const dv_interfaces::OdomDebug::ConstPtr& msg);
    void lapCountCallback(const std_msgs::Int32& msg);

private:
    // -------------------------------------------------------------------------
    // ROS subscribers
    // -------------------------------------------------------------------------

    ros::Subscriber path_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber dv_board_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber angle_sensor_sub_;
    ros::Subscriber cube_mars_status_sub_;
    ros::Subscriber odom_debug_sub_;
    ros::Subscriber lap_count_sub_;

    // -------------------------------------------------------------------------
    // ROS publishers
    // -------------------------------------------------------------------------

    ros::Publisher pub_control_;
    ros::Publisher pub_debug_long_;
    ros::Publisher pub_debug_lat_;
    ros::Publisher pub_global_handling_info_;
    ros::Publisher pub_ref_path_;
    ros::Publisher pub_ref_point_;
    ros::Publisher pub_raw_path_from_pp_;
    ros::Publisher pub_emergency_check_;

    // -------------------------------------------------------------------------
    // Params / state / references
    // -------------------------------------------------------------------------

    ParamBank param_;

    State current_state_;

    LocalPlannerResult ref_path;

    /*
        Full raw path from path planning.

        There is no moving/local raw-path window. Open and closed paths both
        use the complete accepted geometry.
    */
    Eigen::VectorXd X_last_from_pp_;
    Eigen::VectorXd Y_last_from_pp_;

    /*
        Cached smoothed full path corresponding exactly to path_spline_.
        Recomputed only when pathCallback() receives genuinely changed path
        geometry or a changed open/closed flag.
    */
    Eigen::VectorXd X_plan_;
    Eigen::VectorXd Y_plan_;

    /*
        The spline cache belongs to the wrapper.

        path_processing.hpp receives this already-built spline and performs no
        persistent path caching on its own.
    */
    TrackSpline2D path_spline_;
    bool path_spline_valid_ = false;

    /*
        A rejected changed path does not erase the previous valid cache or
        previous ref_path. While this flag is active, poseCallback() keeps
        recalculating lateral control from the previous valid reference and
        uses recoverable EMERGENCY_BRAKE longitudinal control.
    */
    bool path_update_fault_active_ = false;
    std::string path_update_fault_reason_ = "OK";

    bool first_path_received_ = false;
    bool closed_path_received_ = false;

    /*
        BEFORE_START may be used only during the initial approach to an open
        path. The first fully usable planner result with before_bolide == false
        permanently disables that artificial mode for the rest of this
        Controller lifetime.

        This flag is passed into path_processing.hpp. Once false, path
        processing skips fillBeforeStartResult(); if the global classifier
        still reports BEFORE_START, the planner continues with the normal
        spline reference from s = 0.
    */
    bool allow_before_start_reference_ = true;

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    UnboundedLtvMpc mpc_unbounded_;

    PIDController pid_speed_hold_;

    Result lateral_controller_result_;

    LateralControllerType lateral_controller_type_ =
        LateralControllerType::NONE;

    LongitudinalControllerMode longitudinal_controller_mode_ =
        LongitudinalControllerMode::NONE;

    // -------------------------------------------------------------------------
    // Output / phase / global / debug
    // -------------------------------------------------------------------------

    ControlCommand control_output_;

    PhaseInfo phase_info_;
    GlobalHandlingInfo global_handling_info_;

    LongitudinalDebugInfo long_debug_;
    LateralDebugInfo lat_debug_;

    dv_control_common::LoadTransferRelaxation
        load_transfer_relaxation_;

    // -------------------------------------------------------------------------
    // Steering state flags
    // -------------------------------------------------------------------------

    bool steering_state_initialized_ = false;
    bool has_delta_encoder_ = false;

    /*
        PT2 steering-rate state carried between control iterations.

        delta itself is always refreshed from the physical encoder.
    */
    double delta_dot_model_ = 0.0;

    // -------------------------------------------------------------------------
    // Emergency check
    // -------------------------------------------------------------------------

    EmergencyCheck emergency_checker_;

    ros::Timer emergency_check_timer_;

    EmergencyCheckInput emergency_check_input_;

    /*
        The wrapper stores only the current result returned by EmergencyCheck.
        There is no normal/hard split and no checker latch.
    */
    bool emergency_check_is_safe_ = true;
    std::string emergency_check_reason_ = "OK";

private:
    // -------------------------------------------------------------------------
    // General helpers
    // -------------------------------------------------------------------------

    void updateReadyFlag();
    void resetSpeedPid();
    void setPhase(DrivePhase new_phase);

    double getControlDt() const;

    void initializeSteeringStateIfNeeded();
    void updateSteeringStateForController();
    double integrateSteeringCommand(double u_delta_cmd);

    void StateMachinePhaseTransitionLogic(bool recoverable_control_fault);
    void emergencyCheckTimerCallback(const ros::TimerEvent& event);

    // -------------------------------------------------------------------------
    // Publishing command
    // -------------------------------------------------------------------------

    void publishControl(double steering_rad,
                        double total_torque_cmd_Nm,
                        bool finished);

    void publishZeroControl(bool finished);
    void publishZeroMovement(double steering_rad, bool finished);

    // -------------------------------------------------------------------------
    // Visualization
    // -------------------------------------------------------------------------

    void publishReferencePath(const Eigen::VectorXd& X,
                              const Eigen::VectorXd& Y);

    void publishReferencePoint();

    void publishRawPathFromPP(const Eigen::VectorXd& X,
                              const Eigen::VectorXd& Y);

    // -------------------------------------------------------------------------
    // Debug publishers
    // -------------------------------------------------------------------------

    void publishControlDebugLongitudinal();
    void publishControlDebugLateral();
    void publishGlobalHandlingInfo();
    void publishAllDebug();
    void publishEmergencyCheckDebug();

    // -------------------------------------------------------------------------
    // Torque vectoring helpers
    // -------------------------------------------------------------------------

    bool useJacaTorqueVectoring() const;
    bool useOptimizedTorqueVectoring() const;

    /*
        Diagnostic only. Computes the current JACA yaw-moment request in Nm
        even when JACA is not the selected torque-vectoring source.
    */
    double calculateJacaTvYawMomentNmForDebug() const;

    /*
        Selects the yaw-moment request source:
            - JACA,
            - optimized LTV-MPC TV,
            - zero.
        It does not allocate wheel torques.
    */
    void updateTorqueVectoringCommandFromLateralResult();

    /*
        Stores the longitudinal total wheel-torque request in Nm.
        Final FL/FR/RL/RR allocation is performed once by
        allocateSimpleWheelTorques().
    */
    void setBaseTorqueCommandNm(double total_torque_cmd_Nm);
};

} // namespace dv_control
