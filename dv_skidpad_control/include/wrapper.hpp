#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include <dv_interfaces/Control.h>
#include <dv_interfaces/DV_board.h>
#include <dv_interfaces/SkidpadDebug_lat.h>
#include <dv_interfaces/SkidpadDebug_long.h>
#include <dv_interfaces/Skidpad_handling_info.h>
#include <dv_interfaces/EmergencyInfo.h>
#include <dv_interfaces/Imu.h>
#include <dv_interfaces/OdomDebug.h>

#include "ParamBank.hpp"

#include "control_enums.hpp"
#include "control_types.hpp"
#include "path_utils.hpp"
#include "path_spline.hpp"
#include "math_utils.hpp"
#include "longitudinal_utils.hpp"
#include "ros_msg_adapters.hpp"

#include "pid.hpp"
#include "ltv_mpc_unbounded.hpp"
#include "emergency_check.hpp"
#include "dv_control_common/load_transfer.hpp"

namespace skidpad_control
{

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
    ~Controller();

    void pathCallback(const geometry_msgs::PoseArray& msg);

    void poseCallback(const nav_msgs::Odometry& msg);
    void odometryCallback(const nav_msgs::Odometry& msg);
    void odometryDebugCallback(const dv_interfaces::OdomDebug::ConstPtr& msg);
    void imuCallback(const dv_interfaces::Imu::ConstPtr& msg);

    void dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg);
    void angleSensorCallback(const std_msgs::Float64& msg);
    void cubeMarsStatusCallback(const std_msgs::Bool& msg);

private:
    // -------------------------------------------------------------------------
    // Time / frequency helpers
    // -------------------------------------------------------------------------

    double getControlDt() const;

    void initializeSteeringStateIfNeeded();
    void updateSteeringStateForController();

    // -------------------------------------------------------------------------
    // Ready / phase helpers
    // -------------------------------------------------------------------------

    void updateReadyFlag();
    void resetSpeedPid();
    void setPhase(DrivePhase new_phase);
    void updatePhase();

    // -------------------------------------------------------------------------
    // Emergency checker
    // -------------------------------------------------------------------------

    void emergencyCheckTimerCallback(const ros::TimerEvent& event);
    void publishEmergencyCheckDebug();

    // -------------------------------------------------------------------------
    // Torque vectoring / 4-wheel allocation helpers
    // -------------------------------------------------------------------------

    bool useJacaTorqueVectoring() const;
    bool useOptimizedTorqueVectoring() const;

    double getTorqueVectoringTrackWidth() const;
    double getMaxTorqueVectoringYawMomentNm() const;

    double clampTorqueVectoringYawMomentNm(double yaw_moment_Nm) const;
    double yawMomentToTorqueDiffNm(double yaw_moment_Nm) const;
    double torqueDiffToYawMomentNm(double torque_diff_Nm) const;

    void updateTorqueVectoringCommandFromLateralResult();
    void applyTorqueVectoringToBaseTorque(double total_torque_cmd_Nm);

    // -------------------------------------------------------------------------
    // Steering command helper
    // -------------------------------------------------------------------------

    double integrateSteeringCommand(double u_delta_cmd);

    // -------------------------------------------------------------------------
    // Projection helper
    // -------------------------------------------------------------------------

    bool updatePathProjectionForCurrentController();

    // -------------------------------------------------------------------------
    // Path update guard
    // -------------------------------------------------------------------------

    inline bool isNewPathStart(double x, double y)
    {
        constexpr double kSameStartEpsM = 1.0e-6;

        if (!has_previous_path_start_)
        {
            previous_path_start_x_ = x;
            previous_path_start_y_ = y;
            has_previous_path_start_ = true;

            return true;
        }

        const double dx = x - previous_path_start_x_;
        const double dy = y - previous_path_start_y_;

        if (std::hypot(dx, dy) < kSameStartEpsM)
        {
            return false;
        }

        previous_path_start_x_ = x;
        previous_path_start_y_ = y;

        return true;
    }

    // -------------------------------------------------------------------------
    // Control publishing
    // -------------------------------------------------------------------------
    // Nm-only convention:
    //   total_torque_cmd_Nm is total wheel-side torque [Nm].
    //   Conversion to the vehicle interface is done only inside publishControl().
    // -------------------------------------------------------------------------

    void publishControl(double steering_rad,
                        double total_torque_cmd_Nm,
                        bool finished = false);

    void publishZeroControl(bool finished = false);
    void publishZeroMovement(double steering_rad, bool finished = false);
    void publishZeroSteering(double total_torque_cmd_Nm, bool finished = false);

    // -------------------------------------------------------------------------
    // Debug publishers
    // -------------------------------------------------------------------------

    void publishLogInfoLongitudal();
    void publishLogInfoLateral();
    void publishGlobalHandlingInfo();
    void publishAllDebug();

    // -------------------------------------------------------------------------
    // Visualization
    // -------------------------------------------------------------------------

    void publishReferencePath(const Eigen::VectorXd& X,
                              const Eigen::VectorXd& Y);

    void publishReferencePoint();

private:
    // -------------------------------------------------------------------------
    // ROS subscribers
    // -------------------------------------------------------------------------

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber odom_debug_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber dv_board_sub_;
    ros::Subscriber angle_sensor_sub_;
    ros::Subscriber cube_mars_status_sub_;
    ros::Subscriber pose_sub_;

    // -------------------------------------------------------------------------
    // ROS publishers
    // -------------------------------------------------------------------------

    ros::Publisher pub_control_;
    ros::Publisher pub_acc_debug_long_;
    ros::Publisher pub_acc_debug_lat_;
    ros::Publisher pub_global_handling_info_;
    ros::Publisher pub_ref_path_;
    ros::Publisher pub_ref_point_;
    ros::Publisher pub_emergency_check_;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------

    ParamBank param_;

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    UnboundedLtvMpc ltv_mpc_unbounded_;

    PIDController pid_speed_hold_;

    // -------------------------------------------------------------------------
    // Current vehicle state
    // -------------------------------------------------------------------------

    State current_state_;

    bool steering_state_initialized_ = false;
    bool has_delta_encoder_ = false;

    // -------------------------------------------------------------------------
    // Path from path planning
    // -------------------------------------------------------------------------

    Eigen::VectorXd X_last_from_pp_;
    Eigen::VectorXd Y_last_from_pp_;
    Eigen::VectorXd S_last_from_pp_;

    bool has_valid_path_from_pp_ = false;

    PathSpline path_spline_;
    bool has_valid_path_spline_ = false;

    double along_skidpad_ref_path_m_ = 0.0;
    double skidpad_entry_straight_length_m_ = 0.0;
    int last_projection_segment_index_ = 0;
    bool has_path_projection_ = false;

    PathProjection proj_;

    // -------------------------------------------------------------------------
    // Path update guard
    // -------------------------------------------------------------------------

    double previous_path_start_x_ = 0.0;
    double previous_path_start_y_ = 0.0;
    bool has_previous_path_start_ = false;

    // -------------------------------------------------------------------------
    // Control / debug / phase
    // -------------------------------------------------------------------------

    ControlCommand control_output_;
    Result lateral_controller_result_;

    GlobalHandlingInfo global_handling_info_;

    DrivePhase phase_ = DrivePhase::WAITING;

    bool final_brake_requested_ = false;

    LateralControllerType lateral_controller_type_ = LateralControllerType::NONE;

    // -------------------------------------------------------------------------
    // Emergency check
    // -------------------------------------------------------------------------

    EmergencyCheck emergency_checker_;

    ros::Timer emergency_check_timer_;

    EmergencyCheckInputCache emergency_check_input_;

    bool emergency_check_current_ = false;
    bool hard_emergency_check_current_ = false;
    bool hard_emergency_check_latched_ = false;

    std::string emergency_check_current_reason_ = "OK";
    std::string hard_emergency_check_current_reason_ = "OK";
    std::string hard_emergency_check_latched_reason_ = "OK";

    // -------------------------------------------------------------------------
    // Torque vectoring / 4-wheel allocation state
    // -------------------------------------------------------------------------

    double base_torque_cmd_Nm_ = 0.0;

    double tv_yaw_moment_Nm_ = 0.0;
    double tv_yaw_moment_raw_Nm_ = 0.0;
    double tv_torque_diff_Nm_ = 0.0;

    bool optimized_torque_vectoring_used_ = false;
    bool jaca_torque_vectoring_used_ = false;

    dv_control_common::LoadTransferRelaxation
        load_transfer_relaxation_;

    // Compatibility with older main / utilities if still linked somewhere.
    double torque_percetage_to_dv_board_format(double torque_percentage) const;
};

} // namespace skidpad_control
