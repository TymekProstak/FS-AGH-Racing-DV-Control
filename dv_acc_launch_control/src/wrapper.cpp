#include "wrapper.hpp"
#include <ros/service.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/package.h>



namespace   acc_launch_control
{

namespace
{

double last_allocator_total_limited_torque_Nm =
    0.0;

bool last_allocator_total_limited_torque_valid =
    false;


bool g_print_console_debug_info =
    true;

inline bool printConsoleDebugInfo()
{
    return g_print_console_debug_info;
}


bool callDvBoardEmergencyServiceSafe(
    const std::string& log_prefix
)
{
    /*
        Hard-emergency service call policy:

            1) caller publishes zero control first,
            2) then this calls /dv_board/emergency,
            3) if the service is not available or fails, try again every hard
               emergency control cycle,
            4) stop calling only after the service returns success=true.

        This intentionally does NOT rate-limit the service attempts.
        In hard emergency the safe behavior is to keep requesting the board
        emergency path until it is accepted.

        std_srvs/Trigger has an empty request.
    */
    static bool service_call_succeeded = false;

    if (service_call_succeeded)
    {
        return true;
    }

    if (!ros::service::exists(
            "/dv_board/emergency",
            false
        ))
    {
        if (printConsoleDebugInfo())
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                log_prefix
                    << " /dv_board/emergency service not available; retrying every hard emergency cycle"
            );
        }
        return false;
    }

    std_srvs::Trigger srv;

    if (!ros::service::call(
            "/dv_board/emergency",
            srv
        ))
    {
        if (printConsoleDebugInfo())
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                log_prefix
                    << " /dv_board/emergency service call failed; retrying every hard emergency cycle"
            );
        }
        return false;
    }

    if (!srv.response.success)
    {
        if (printConsoleDebugInfo())
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                log_prefix
                    << " /dv_board/emergency returned failure: "
                    << srv.response.message
                    << "; retrying every hard emergency cycle"
            );
        }
        return false;
    }

    service_call_succeeded =
        true;

    if (printConsoleDebugInfo())
    {
        ROS_ERROR_STREAM(
            log_prefix
                << " /dv_board/emergency accepted: "
                << srv.response.message
        );
    }

    return true;
}

/*
    Regulation launch hold.

    The car may enter AS DRIVE / controller-ready state, but the launch map
    must start only after this delay.  Therefore:
        phase LAUNCH starts immediately when the controller becomes ready,
        launch map time t=0 starts after kLaunchMapStartDelayAfterDriveS.
*/
constexpr double kLaunchMapStartDelayAfterDriveS =
    3.0;


std::string makePackagePath(const std::string& package_name,
                            const std::string& relative_path)
{
    const std::string package_path = ros::package::getPath(package_name);

    if (package_path.empty())
    {
        throw std::runtime_error(
            "[acc_launch_control] Cannot resolve ROS package path: " + package_name
        );
    }

    if (relative_path.empty())
    {
        return package_path;
    }

    if (relative_path.front() == '/')
    {
        return package_path + relative_path;
    }

    return package_path + "/" + relative_path;
}



WheelValues scaleWheelValuesLocal(const WheelValues& in,
                                  const double scale)
{
    WheelValues out;

    out.FL = in.FL * scale;
    out.FR = in.FR * scale;
    out.RL = in.RL * scale;
    out.RR = in.RR * scale;

    return out;
}


WheelValues absWheelValuesLocal(const WheelValues& in)
{
    WheelValues out;

    out.FL = std::abs(in.FL);
    out.FR = std::abs(in.FR);
    out.RL = std::abs(in.RL);
    out.RR = std::abs(in.RR);

    return out;
}



} // anonymous namespace

// =============================================================================
//                              CONTROLLER
// =============================================================================
Controller::Controller(ros::NodeHandle& nh, const ParamBank& param)
    : param_(param),
      ltv_mpc_unbounded_(param),
      pid_speed_hold_(makeSpeedPidParams(param)),
      runtime_map_handler(
          makePackagePath(
              "dv_acc_launch_control",
              param_.getString("launch_map.maps_root")
          ),
          param_.get("general.mu_x"),
          param_.get("map_generator.S")
      )
{
    g_print_console_debug_info =
        param_.getBool("general.print_console_debug_info");

    path_sub_ = nh.subscribe(
        "/path_planning/path",
        1,
        &Controller::pathCallback,
        this
    );

    pose_sub_ = nh.subscribe(
        "/ins/pose",
        1,
        &Controller::poseCallback,
        this
    );

    /*
        Velocity is read from /ins/pose Odometry twist in poseCallback().
        The optional /dv_odometry/odometry callback is intentionally not used
        in this ACC wrapper.
    */
    // odom_sub_ = nh.subscribe(
    //     "/dv_odometry/odometry",
    //     1,
    //     &Controller::odometryCallback,
    //     this
    // );

    odom_debug_sub_ = nh.subscribe(
        "/dv_odometry/odometry_debug",
        1,
        &Controller::odometryDebugCallback,
        this
    );

    imu_sub_ = nh.subscribe(
        "/dv_board/imu",
        1,
        &Controller::imuCallback,
        this
    );

    dv_board_sub_ = nh.subscribe(
        "/dv_board/data",
        1,
        &Controller::dvBoardCallback,
        this
    );

    angle_sensor_sub_ = nh.subscribe(
        "/servo_node/cubemars/encoder_absolute",
        1,
        &Controller::angleSensorCallback,
        this
    );

    cube_mars_status_sub_ = nh.subscribe(
        "/servo_node/cubemars/initialization_complete",
        1,
        &Controller::cubeMarsStatusCallback,
        this
    );

    pub_control_ = nh.advertise<dv_interfaces::Control>(
        "/dv_board/control",
        1
    );

    pub_acc_debug_long_ = nh.advertise<dv_interfaces::AccDebug_long>(
        "/acc_launch_control/acc_debug_long",
        1
    );

    pub_acc_debug_lat_ = nh.advertise<dv_interfaces::AccDebug_lat>(
        "/acc_launch_control/acc_debug_lat",
        1
    );

    pub_global_handling_info_ =
        nh.advertise<dv_interfaces::Acc_handling_info>(
            "/acc_launch_control/acc_handling_info",
            1
        );

    pub_emergency_check_ =
        nh.advertise<dv_interfaces::EmergencyInfo>(
            "/emergency_info",
            1
        );

    pub_ref_path_ = nh.advertise<visualization_msgs::Marker>(
        "/acc_launch_control/ref_path",
        1,
        true
    );

    pub_ref_point_ = nh.advertise<visualization_msgs::Marker>(
        "/acc_launch_control/ref_point",
        1,
        true
    );

    lateral_controller_type_ =
        LateralControllerType::LTV_MPC_UNBOUNDED;

    if (printConsoleDebugInfo())
    {
        ROS_INFO(
            "[acc_launch_control] Using the shared unbounded LTV MPC."
        );
    }

    launch_end_info_ =
        runtime_map_handler.getInitialLaunchEndInfo();

    emergency_checker_.param_bank =
        param_;

    emergency_check_timer_ =
        nh.createTimer(
            ros::Duration(0.01),
            &Controller::emergencyCheckTimerCallback,
            this
        );

    if (printConsoleDebugInfo())
    {
        ROS_INFO("[acc_launch_control] Emergency checker timer started at 100 Hz.");
    }

    if (printConsoleDebugInfo())
    {
        ROS_INFO("[acc_launch_control] Longitudinal pipeline enabled: WAITING -> LAUNCH -> ACCELERATION -> BRAKING -> COASTING -> FINISHED.");
    }
    if (printConsoleDebugInfo())
    {
        ROS_INFO("[acc_launch_control] Emergency behaviour: hard latch, straight steering, Control.finished=true.");
    }
    if (printConsoleDebugInfo())
    {
        ROS_INFO_STREAM("[acc_launch_control] Launch map path: " << runtime_map_handler.loadedPath());
    }

    if (launch_end_info_.valid)
    {
        if (printConsoleDebugInfo())
        {
            ROS_INFO_STREAM(
                "[acc_launch_control] Launch end from map: "
                << "vx_end=" << launch_end_info_.vx_mps << " m/s, "
                << "s_end=" << launch_end_info_.s_m << " m, "
                << "t_end=" << launch_end_info_.t_s << " s, "
                << "reason=" << launch_end_info_.reason
            );
        }
    }
    else
    {
        if (printConsoleDebugInfo())
        {
            ROS_WARN("[acc_launch_control] Launch end info invalid. Speed hold target will use general.vx_max.");
        }
    }
}


double Controller::getControlDt() const
{
    const double hz =
        param_.get("frequency.steer_cmd_loop_hz");

    if (!std::isfinite(hz) || hz <= 0.0)
    {
        throw std::runtime_error(
            "[acc_launch_control] frequency.steer_cmd_loop_hz must be finite and > 0"
        );
    }

    return 1.0 / hz;
}

void Controller::initializeSteeringStateIfNeeded()
{
    if (steering_state_initialized_)
    {
        return;
    }

    const double max_steer =
        param_.get("steering_limit.max_steer");

    const double initial_delta_raw =
        has_delta_encoder_
            ? delta_encoder_
            : control_output_.steer_cmd_rad;

    const double initial_delta =
        clampLocal(
            std::isfinite(initial_delta_raw)
                ? initial_delta_raw
                : 0.0,
            -max_steer,
            max_steer
        );

    /*
        Start the command from the measured physical position so that enabling
        control does not create an artificial command step.
    */
    delta_cmd_ =
        initial_delta;

    /*
        Steering rate is never obtained by differentiating the encoder.
        It is propagated by the PT2 state in the model.
    */
    delta_dot_model_ =
        0.0;

    steering_state_initialized_ =
        true;
}


void Controller::updateSteeringStateForController()
{
    initializeSteeringStateIfNeeded();

    const double max_steer =
        param_.get("steering_limit.max_steer");

    current_state_.delta_cmd =
        delta_cmd_;

    current_state_.delta_enc =
        has_delta_encoder_
            ? clampLocal(
                  std::isfinite(delta_encoder_)
                      ? delta_encoder_
                      : 0.0,
                  -max_steer,
                  max_steer
              )
            : delta_cmd_;

    /*
        Closed-loop actuator state:
            delta     = real encoder measurement,
            delta_dot = PT2 model state.

        The measured angle closes the actuator-state loop.
    */
    current_state_.delta =
        current_state_.delta_enc;

    current_state_.delta_dot =
        std::isfinite(delta_dot_model_)
            ? delta_dot_model_
            : 0.0;

    current_state_.delta_vehicle_used =
        current_state_.delta;
}


double Controller::integrateSteeringCommand(double u_delta_cmd)
{
    initializeSteeringStateIfNeeded();

    const double dt =
        std::max(
            getControlDt(),
            1.0e-9
        );

    const double max_steer =
        param_.get("steering_limit.max_steer");

    const double max_steer_rate =
        param_.get("steering_limit.max_steer_rate");

    if (!std::isfinite(u_delta_cmd))
    {
        u_delta_cmd =
            0.0;
    }

    const double limited_u_delta_cmd =
        clampLocal(
            u_delta_cmd,
            -max_steer_rate,
            max_steer_rate
        );

    lateral_controller_result_.rate_saturated =
        lateral_controller_result_.rate_saturated ||
        std::abs(limited_u_delta_cmd - u_delta_cmd) > 1.0e-12;

    const double delta_cmd_raw_next =
        delta_cmd_
        + limited_u_delta_cmd * dt;

    delta_cmd_ =
        clampLocal(
            delta_cmd_raw_next,
            -max_steer,
            max_steer
        );

    lateral_controller_result_.steering_angle_saturated =
        lateral_controller_result_.steering_angle_saturated ||
        std::abs(delta_cmd_ - delta_cmd_raw_next) > 1.0e-12;

    /*
        Carry only the PT2 steering-rate state between controller iterations.

        The angle state is corrected every iteration by the encoder in
        updateSteeringStateForController(), so the PT2 angle cannot drift
        away from the physical steering system.
    */
    if (lateral_controller_result_.valid &&
        std::isfinite(lateral_controller_result_.delta_dot_next))
    {
        delta_dot_model_ =
            clampLocal(
                lateral_controller_result_.delta_dot_next,
                -max_steer_rate,
                max_steer_rate
            );
    }
    else
    {
        delta_dot_model_ =
            0.0;
    }

    /*
        Prevent the model rate from pushing farther into a physical steering
        limit when the measured angle is already at that limit.
    */
    if (has_delta_encoder_)
    {
        const double measured_delta =
            clampLocal(
                std::isfinite(delta_encoder_)
                    ? delta_encoder_
                    : 0.0,
                -max_steer,
                max_steer
            );

        constexpr double kSteeringLimitEpsilonRad =
            1.0e-4;

        if ((measured_delta >= max_steer - kSteeringLimitEpsilonRad &&
             delta_dot_model_ > 0.0) ||
            (measured_delta <= -max_steer + kSteeringLimitEpsilonRad &&
             delta_dot_model_ < 0.0))
        {
            delta_dot_model_ =
                0.0;
        }
    }

    updateSteeringStateForController();

    return
        delta_cmd_;
}




void Controller::updateReadyFlag()
{
    global_handling_info_.ready_to_start_drive =
        global_handling_info_.cube_mars_initialization_finished &&
        global_handling_info_.has_received_first_dv_board_message &&
        global_handling_info_.has_valid_path_from_pp &&
        global_handling_info_.has_odometry_message &&
        global_handling_info_.has_received_imu_message &&
        has_delta_encoder_;
}


void Controller::resetSpeedPid()
{
    pid_speed_hold_.reset();
}



void Controller::setPhase(DrivePhase new_phase)
{
    if (phase_info_.phase == new_phase)
    {
        return;
    }

    if (new_phase == DrivePhase::SPEED_HOLD ||
        new_phase == DrivePhase::BRAKING ||
        new_phase == DrivePhase::EMERGENCY_BRAKE ||
        new_phase == DrivePhase::FINISHED)
    {
        resetSpeedPid();
    }

    phase_info_.phase =
        new_phase;

    phase_info_.phase_start_time_s =
        ros::Time::now();
}


void Controller::computePathErrorsAndUpdatState()
{
    if (!(has_valid_path_axis_&&has_valid_path_from_pp_))
    {
        current_state_.ey = 0.0;
        current_state_.epsi = 0.0;
        current_state_.s = 0.0;
        return;
    }
    const double dx = current_x_m_ - path_start_x_m_;
    const double dy = current_y_m_ - path_start_y_m_;
    current_state_.s = dx * path_dir_x_ + dy * path_dir_y_;
    current_state_.epsi = current_yaw_rad_ -std::atan2(path_dir_y_ , path_dir_x_) ;
    current_state_.epsi = wrapAngle(current_state_.epsi);
    current_state_.ey = -dx * path_dir_y_ + dy * path_dir_x_;

}

// =============================================================================
//                              PATH / PROJECTION
// =============================================================================

void Controller::pathCallback(const geometry_msgs::PoseArray& msg)
{
    const auto& path = msg.poses;

    //std::cout << "[acc_launch_control] Received path from path planner with " << path.size() << " points." << std::endl;

    
        if (path.size() < 2)
        {
            if (printConsoleDebugInfo())
            {
                ROS_WARN_STREAM("[Path Selection] Received path with only one point: " << path.size());
            }
            return;
        }
       
        
        X_last_from_pp_.resize(path.size());
        Y_last_from_pp_.resize(path.size());
        for (size_t i = 0; i < path.size(); ++i)
        {   

            X_last_from_pp_(i) = path[i].position.x ;
            Y_last_from_pp_(i) = path[i].position.y;
        }

        emergency_check_input_.path_xy.clear();
        emergency_check_input_.path_xy.reserve(path.size());

        for (const auto& pose : path)
        {
            emergency_check_input_.path_xy.emplace_back(
                pose.position.x,
                pose.position.y
            );
        }

        emergency_check_input_.spline_is_valid =
            path.size() >= 2;

        /*
            geometry_msgs::PoseArray does not carry is_track_closed.
            For this ACC launch pipeline the path-planner check can still
            validate timeout/size/before/after using the received points.
        */
        emergency_check_input_.path_is_track_closed =
            false;

        emergency_check_input_.new_path_planner_message_get =
            true;

        // set flags of having valid path and update global handling info
        has_valid_path_from_pp_ = true;

        // set path axis and starting point for error calcuaiton and distance along path calculation
        path_start_x_m_ = X_last_from_pp_(0);
        path_start_y_m_ = Y_last_from_pp_(0);

        path_dir_x_ = X_last_from_pp_(1) - X_last_from_pp_(0);
        path_dir_y_ = Y_last_from_pp_(1) - Y_last_from_pp_(0);
        const double path_dir_norm = std::hypot(path_dir_x_, path_dir_y_);
        if (path_dir_norm > 1.0e-6)
        {
            path_dir_x_ /= path_dir_norm;
            path_dir_y_ /= path_dir_norm;
            has_valid_path_axis_ = true;    
        }
        else
        {
            has_valid_path_axis_ = false;
            if (printConsoleDebugInfo())
            {
                ROS_WARN("[acc_launch_control] The first two points of the path from path planner are too close. Path axis is invalid. Path following controllers may not work properly.");
            }
        }

        global_handling_info_.has_valid_path_from_pp = has_valid_path_axis_ && has_valid_path_from_pp_;

    publishReferencePath(X_last_from_pp_, Y_last_from_pp_);
}




// =============================================================================
//                              EMERGENCY CHECK TIMER
// =============================================================================

void Controller::emergencyCheckTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    const double current_time_s =
        ros::Time::now().toSec();

    const DrivePhase phase_now =
        phase_info_.phase;

    /*
        For ACC launch this is the driving window.
        WAITING and FINISHED are disarmed.
        EMERGENCY_BRAKE is already a latched terminal state.
    */
    const bool checker_active_now =
        phase_now == DrivePhase::LAUNCH ||
        phase_now == DrivePhase::ACCELERATION ||
        phase_now == DrivePhase::SPEED_HOLD ||
        phase_now == DrivePhase::BRAKING;

    const bool new_encoder_message_get =
        emergency_check_input_.new_encoder_message_get;

    const bool new_ins_message_get =
        emergency_check_input_.new_ins_message_get;

    const bool new_path_planner_message_get =
        emergency_check_input_.new_path_planner_message_get;

    emergency_check_input_.new_encoder_message_get = false;
    emergency_check_input_.new_ins_message_get = false;
    emergency_check_input_.new_path_planner_message_get = false;

    emergency_check_input_.encoder_reference_position_rad =
        delta_cmd_;

    const bool emergency_now =
        emergency_checker_.UpdateEmergencyCheck(
            current_state_,
            checker_active_now,
            current_time_s,

            new_encoder_message_get,
            emergency_check_input_.encoder_position_rad,
            emergency_check_input_.encoder_reference_position_rad,

            new_ins_message_get,
            emergency_check_input_.ins_xy,
            emergency_check_input_.ins_velocity_body_xy_mps,
            current_state_.vx_enc,

            emergency_check_input_.bolide_xy,
            emergency_check_input_.path_xy,
            emergency_check_input_.spline_is_valid,
            new_path_planner_message_get,
            emergency_check_input_.path_is_track_closed
        );

    const bool hard_emergency_now =
        emergency_checker_.is_hard_emergency;

    const std::string hard_reason =
        emergency_checker_.hard_emergency_reason.empty()
            ? emergency_checker_.emergency_reason
            : emergency_checker_.hard_emergency_reason;

    emergency_check_current_ =
        checker_active_now ? emergency_now : false;

    hard_emergency_check_current_ =
        checker_active_now ? hard_emergency_now : false;

    emergency_check_current_reason_ =
        emergency_check_current_
            ? emergency_checker_.emergency_reason
            : "OK";

    hard_emergency_check_current_reason_ =
        hard_emergency_check_current_
            ? hard_reason
            : "OK";

    /*
        Project rule for this ACC controller:
        ANY emergency reported by the checker becomes a permanent hard latch.
        There is no recoverable emergency here.
    */
    if (checker_active_now &&
        (emergency_now || hard_emergency_now) &&
        !emergency_latched_)
    {
        emergency_latched_ = true;
        emergency_reason_ = hard_emergency_now ? hard_reason : emergency_checker_.emergency_reason;

        if (emergency_reason_.empty())
        {
            emergency_reason_ = "EmergencyCheck triggered";
        }

        emergency_reason_ =
            "Hard emergency latch from EmergencyCheck: " + emergency_reason_;

        phase_info_.emergency_brake_start_time_s =
            ros::Time::now();

        long_debug_.if_safe = false;
        long_debug_.safety_reason = emergency_reason_;
        long_debug_.emergency_brake_requested = true;

        lat_debug_.if_safe = false;

        setPhase(DrivePhase::EMERGENCY_BRAKE);

        if (printConsoleDebugInfo())
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                "[acc_launch_control] " << emergency_reason_
            );
        }
    }

    publishEmergencyCheckDebug();
}

// =============================================================================
//                              MAIN PHASE LOGIC HANDLER
// =============================================================================
void Controller::StateMachinePhaseTransitionLogic()
{
    /*
        Emergency is triggered only by EmergencyCheck.
        In this ACC wrapper every checker emergency is hard-latched.

        Important:
            - emergency publishes finished=true in dv_interfaces::Control,
            - but it does NOT set global_handling_info_.as_finished,
              because this is not a normal completed run.
    */
    if (emergency_latched_)
    {
        setPhase(DrivePhase::EMERGENCY_BRAKE);
        return;
    }

    updateReadyFlag();

    if (global_handling_info_.as_finished)
    {
        setPhase(DrivePhase::FINISHED);
        return;
    }

    if (!global_handling_info_.ready_to_start_drive)
    {
      
            setPhase(DrivePhase::WAITING);
        
        return;
    }

    if (phase_info_.phase == DrivePhase::WAITING)
    {
        setPhase(DrivePhase::LAUNCH);
        phase_info_.launch_start_time_s = ros::Time::now();
        resetSpeedPid();
        return;
    }

    const double vx_odom_mps =
        current_state_.vx;

    const double general_vx_max =
        param_.get("general.vx_max");

    double launch_end_vx_mps =
        runtime_map_handler.getLaunchEndSpeed();

    if (launch_end_info_.valid &&
        std::isfinite(launch_end_info_.vx_mps) &&
        launch_end_info_.vx_mps > 0.1)
    {
        launch_end_vx_mps =
            launch_end_info_.vx_mps;
    }

    const bool has_launch_end_speed =
        std::isfinite(launch_end_vx_mps) &&
        launch_end_vx_mps > 0.1;

    const bool reached_launch_end_speed =
        has_launch_end_speed &&
        vx_odom_mps >= launch_end_vx_mps;

    const bool above_general_vmax =
        std::isfinite(general_vx_max) &&
        vx_odom_mps >= general_vx_max;

    const bool should_brake_now =
        shouldStartBrake(current_state_);

    /*
        Normal phase flow:

            WAITING
              -> LAUNCH
              -> ACCELERATION
              -> BRAKING
              -> COASTING
              -> FINISHED

        SPEED_HOLD is only a speed cap when vx exceeds general.vx_max.
        Emergency is terminal and does not go through COASTING / FINISHED.
    */

    if (phase_info_.phase == DrivePhase::LAUNCH)
    {
        const double launch_phase_elapsed_s =
            (ros::Time::now() - phase_info_.launch_start_time_s).toSec();

        /*
            Regulation requirement:
                after entering AS DRIVE / ready-to-drive, wait 3 seconds
                before applying the launch map.

            During this hold the phase is already LAUNCH, but the map is not
            allowed to advance and phase transitions based on launch-map end
            are disabled.  Map time t=0 is handled in computeLaunchThrottleNm().
        */
        if (launch_phase_elapsed_s < kLaunchMapStartDelayAfterDriveS)
        {
            return;
        }

        if (above_general_vmax)
        {
            setPhase(DrivePhase::SPEED_HOLD);
            resetSpeedPid();
            return;
        }

        if (reached_launch_end_speed)
        {
            setPhase(DrivePhase::ACCELERATION);
            return;
        }

        return;
    }

    if (phase_info_.phase == DrivePhase::ACCELERATION)
    {
        if (should_brake_now)
        {
            setPhase(DrivePhase::BRAKING);
            phase_info_.brake_start_time_s = ros::Time::now();
            resetSpeedPid();
            return;
        }

        if (above_general_vmax)
        {
            setPhase(DrivePhase::SPEED_HOLD);
            resetSpeedPid();
            return;
        }

        return;
    }

    if (phase_info_.phase == DrivePhase::SPEED_HOLD)
    {
        if (should_brake_now)
        {
            setPhase(DrivePhase::BRAKING);
            phase_info_.brake_start_time_s = ros::Time::now();
            resetSpeedPid();
            return;
        }

     

        return;
    }

    const double speed_abs_encoder =
        std::abs(current_state_.vx_enc);

    if (phase_info_.phase == DrivePhase::BRAKING)
    {
        const double coast_below_speed =
            param_.get("general.coast_below_speed_mps");

        if (speed_abs_encoder <= coast_below_speed)
        {
            setPhase(DrivePhase::COASTING);
        }

        return;
    }

    if (phase_info_.phase == DrivePhase::COASTING)
    {
        const double finished_encoder_speed =
            param_.get("general.finished_encoder_speed_mps");

        if (speed_abs_encoder <= finished_encoder_speed)
        {
            global_handling_info_.as_finished = true;
            setPhase(DrivePhase::FINISHED);
        }

        return;
    }

    if (phase_info_.phase == DrivePhase::FINISHED)
    {
        global_handling_info_.as_finished = true;
        return;
    }
}



void Controller::poseCallback(const nav_msgs::Odometry& msg)
{
    current_x_m_ =
        msg.pose.pose.position.x;

    current_y_m_ =
        msg.pose.pose.position.y;

    tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_yaw_rad_ =
        yaw;

    current_state_.x = current_x_m_;
    current_state_.y = current_y_m_;
    current_state_.yaw = current_yaw_rad_;

    /*
        Velocity is read here from the Odometry message carrying the pose.
        The optional /dv_odometry/odometry callback is intentionally not used.
    */
    current_state_.vx =
        msg.twist.twist.linear.x;

    current_state_.vy =
        msg.twist.twist.linear.y;

    current_state_.r =
        msg.twist.twist.angular.z;

    global_handling_info_.has_odometry_message =
        true;

    emergency_check_input_.ins_xy =
        Eigen::Vector2d(
            current_state_.x,
            current_state_.y
        );

    emergency_check_input_.bolide_xy =
        emergency_check_input_.ins_xy;

    emergency_check_input_.ins_velocity_body_xy_mps =
        Eigen::Vector2d(
            current_state_.vx,
            current_state_.vy
        );

    emergency_check_input_.new_ins_message_get =
        true;

    computePathErrorsAndUpdatState();
    updateReadyFlag();
    StateMachinePhaseTransitionLogic();

    updateSteeringStateForController();

    double total_torque_request_Nm =
        0.0;

    if (phase_info_.phase == DrivePhase::WAITING)
    {
        control_output_.steer_cmd_rad = 0.0;
        control_output_.finished = false;
        total_torque_request_Nm = 0.0;
    }
    else if (phase_info_.phase == DrivePhase::FINISHED)
    {
        control_output_.finished = true;
        total_torque_request_Nm = 0.0;
    }
    else if (phase_info_.phase == DrivePhase::EMERGENCY_BRAKE ||
             emergency_latched_)
    {
        /*
            ACC has only hard emergency semantics.

            Output order:
                1) publish zero control with finished=false,
                2) then request the DV-board emergency service.

            Do not set Control.finished here. The hard stop is delegated to
            /dv_board/emergency after the zero command is already on the topic.
        */
        long_debug_.torque_cmd_Nm = 0.0;
        long_debug_.if_safe = false;
        long_debug_.safety_reason = emergency_reason_;
        control_output_.finished = false;
        control_output_.steer_cmd_rad = 0.0;

        publishZeroControl(false);

        callDvBoardEmergencyServiceSafe(
            "[acc_launch_control]"
        );

        publishAllDebug();
        publishReferencePoint();
        return;
    }
    else if (phase_info_.phase == DrivePhase::SPEED_HOLD)
    {
        control_output_.steer_cmd_rad =
            computeLateralControlCommand();

        total_torque_request_Nm =
            computeSpeedHoldThrottleNm(
                current_state_,
                (ros::Time::now() - phase_info_.phase_start_time_s).toSec()
            );

        control_output_.finished = false;
    }
    else if (phase_info_.phase == DrivePhase::ACCELERATION)
    {
        control_output_.steer_cmd_rad =
            computeLateralControlCommand();

        total_torque_request_Nm =
            computeRuntimeAccelerationThrottleNm(
                current_state_,
                (ros::Time::now() - phase_info_.phase_start_time_s).toSec()
            );

        control_output_.finished = false;
    }
    else if (phase_info_.phase == DrivePhase::LAUNCH)
    {
        control_output_.steer_cmd_rad =
           computeLateralControlCommand();

        total_torque_request_Nm =
            computeLaunchThrottleNm(
                current_state_,
                (ros::Time::now() - phase_info_.launch_start_time_s).toSec()
            );

        control_output_.finished = false;
    }
    else if (phase_info_.phase == DrivePhase::COASTING)
    {
        control_output_.steer_cmd_rad =
            computeLateralControlCommand();

        total_torque_request_Nm =
            0.0;

        control_output_.finished = false;
    }
    else if (phase_info_.phase == DrivePhase::BRAKING)
    {
        control_output_.steer_cmd_rad =
            computeLateralControlCommand();

        total_torque_request_Nm =
            computeRuntimeBrakeThrottleNm(
                current_state_,
                (ros::Time::now() - phase_info_.brake_start_time_s).toSec()
            );

        control_output_.finished = false;
    }

     double total_torque_cmd_Nm =
        clampTorqueByDrivetrainLimits(
            param_,
            total_torque_request_Nm,
            current_state_.vx
        );

    if (std::isfinite(current_state_.vx_enc) &&
        current_state_.vx_enc < -0.01)
    {
        total_torque_cmd_Nm =
            std::max(
                total_torque_cmd_Nm,
                0.0
            );
    }

    const double speed_abs_for_vmax_check_mps =
        std::max(
            std::isfinite(current_state_.vx_enc)
                ? std::abs(current_state_.vx_enc)
                : 0.0,
            std::isfinite(current_state_.vx)
                ? std::abs(current_state_.vx)
                : 0.0
        );

    const double vx_max_mps =
        param_.get("general.vx_max");

    if (std::isfinite(vx_max_mps) &&
        vx_max_mps > 0.0 &&
        speed_abs_for_vmax_check_mps > vx_max_mps)
    {
        total_torque_cmd_Nm =
            std::min(
                total_torque_cmd_Nm,
                0.0
            );
    }

    if (phase_info_.phase == DrivePhase::WAITING ||
        phase_info_.phase == DrivePhase::FINISHED)
    {
        total_torque_cmd_Nm =
            std::max(
                total_torque_cmd_Nm,
                0.0
            );
    }

    total_torque_cmd_Nm =
        clampTorqueByDrivetrainLimits(
            param_,
            total_torque_cmd_Nm,
            current_state_.vx
        );

    const dv_control_common::RelaxedAcceleration relaxed_acceleration =
        load_transfer_relaxation_.update(
            current_state_.ax_mps2,
            current_state_.ay_mps2,
            getControlDt(),
            param_.get("model.mass_transfer.tau_load_s")
        );

    WheelTorqueAllocation allocation;

    const bool bypass_allocator_rate_limiter =
        control_output_.finished ||
        phase_info_.phase == DrivePhase::WAITING ||
        phase_info_.phase == DrivePhase::FINISHED ||
        phase_info_.phase == DrivePhase::EMERGENCY_BRAKE ||
        emergency_latched_;

    if (phase_info_.phase == DrivePhase::LAUNCH &&
        current_runtime_command_.valid)
    {
        /*
            LAUNCH allocation is taken directly from the launch map.

            The map stores axle total wheel-side torques [Nm]:
                M_front_total_cmd_Nm
                M_rear_total_cmd_Nm

            Therefore:
                FL = front_total / 2
                FR = front_total / 2
                RL = rear_total  / 2
                RR = rear_total  / 2

            Do not run the generic slip/ellipse allocator in LAUNCH.
            At vx ~= 0 it can scale the whole vector to zero.
        */
        const double front_total_map_Nm =
            std::isfinite(current_runtime_command_.M_front_total_cmd_Nm)
                ? current_runtime_command_.M_front_total_cmd_Nm
                : 0.0;

        const double rear_total_map_Nm =
            std::isfinite(current_runtime_command_.M_rear_total_cmd_Nm)
                ? current_runtime_command_.M_rear_total_cmd_Nm
                : 0.0;

        const double map_total_Nm =
            front_total_map_Nm
            + rear_total_map_Nm;

        const double scale =
            std::abs(map_total_Nm) > 1.0e-9
                ? total_torque_cmd_Nm / map_total_Nm
                : 0.0;

        allocation.requested_total_torque_Nm =
            total_torque_request_Nm;

        allocation.drivetrain_limited_total_torque_Nm =
            total_torque_cmd_Nm;

        allocation.limited_by_drivetrain =
            std::abs(total_torque_cmd_Nm - total_torque_request_Nm) > 1.0e-9;

        allocation.torque_Nm.FL =
            0.5 * front_total_map_Nm * scale;

        allocation.torque_Nm.FR =
            0.5 * front_total_map_Nm * scale;

        allocation.torque_Nm.RL =
            0.5 * rear_total_map_Nm * scale;

        allocation.torque_Nm.RR =
            0.5 * rear_total_map_Nm * scale;

        allocation.abs_torque_Nm.FL =
            std::abs(allocation.torque_Nm.FL);

        allocation.abs_torque_Nm.FR =
            std::abs(allocation.torque_Nm.FR);

        allocation.abs_torque_Nm.RL =
            std::abs(allocation.torque_Nm.RL);

        allocation.abs_torque_Nm.RR =
            std::abs(allocation.torque_Nm.RR);

        allocation.allocated_total_torque_Nm =
            sumWheelTorquesNm(
                allocation.torque_Nm
            );

        allocation.global_scale =
            1.0;

        allocation.hard_limit_scale =
            1.0;

        allocation.previous_total_limited_torque_Nm =
            last_allocator_total_limited_torque_valid
                ? last_allocator_total_limited_torque_Nm
                : 0.0;

        allocation.torque_rate_limiter_bypassed =
            bypass_allocator_rate_limiter;

        allocation.limited_by_friction_or_mech =
            false;

        /*
            LAUNCH uses map front/rear torque split directly, bypassing the
            generic normal-load allocator.  Therefore the same final total
            torque rate scale must be applied here in the wrapper.
        */
        if (!allocation.torque_rate_limiter_bypassed)
        {
            const double target_total_Nm =
                allocation.allocated_total_torque_Nm;

            const double rate_limited_total_Nm =
                rateLimitTorqueCommandNm(
                    param_,
                    allocation.previous_total_limited_torque_Nm,
                    target_total_Nm,
                    getControlDt()
                );

            double rate_scale =
                1.0;

            if (std::abs(target_total_Nm) > 1.0e-9)
            {
                rate_scale =
                    rate_limited_total_Nm / target_total_Nm;
            }

            allocation.total_torque_rate_limited_scale =
                rate_scale;

            allocation.total_torque_rate_scale_min =
                std::min(
                    1.0,
                    rate_scale
                );

            allocation.total_torque_rate_scale_max =
                std::max(
                    1.0,
                    rate_scale
                );

            allocation.limited_by_rate =
                std::abs(rate_scale - 1.0) > 1.0e-6;

            allocation.global_scale =
                rate_scale;

            allocation.torque_Nm =
                scaleWheelValuesLocal(
                    allocation.torque_Nm,
                    allocation.global_scale
                );

            allocation.abs_torque_Nm =
                absWheelValuesLocal(
                    allocation.torque_Nm
                );

            allocation.allocated_total_torque_Nm =
                sumWheelTorquesNm(
                    allocation.torque_Nm
                );
        }

        const double final_engine_per_wheel_scale =
            computeFinalPerWheelEngineLimitScale(
                param_,
                allocation.torque_Nm
            );

        if (final_engine_per_wheel_scale < 0.999999)
        {
            allocation.torque_Nm =
                scaleWheelValuesLocal(
                    allocation.torque_Nm,
                    final_engine_per_wheel_scale
                );

            allocation.abs_torque_Nm =
                absWheelValuesLocal(
                    allocation.torque_Nm
                );

            allocation.allocated_total_torque_Nm =
                sumWheelTorquesNm(
                    allocation.torque_Nm
                );

            allocation.global_scale *=
                final_engine_per_wheel_scale;

            allocation.hard_limit_scale *=
                final_engine_per_wheel_scale;

            allocation.limited_by_friction_or_mech =
                true;
        }

        if (printConsoleDebugInfo())
        {
            ROS_WARN_STREAM_THROTTLE(
                0.25,
                "[ACC LAUNCH MAP ALLOC RATE] "
                << "prev_total=" << allocation.previous_total_limited_torque_Nm
                << " target_total=" << map_total_Nm
                << " allocated_total=" << allocation.allocated_total_torque_Nm
                << " global_scale=" << allocation.global_scale
                << " rate_scale=" << allocation.total_torque_rate_limited_scale
                << " limited_by_rate=" << allocation.limited_by_rate
                << " rate_bypass=" << allocation.torque_rate_limiter_bypassed
                << " final_engine_per_wheel_scale=" << final_engine_per_wheel_scale
            );
        }
    }
    else
    {
        allocation =
            allocateWheelTorqueByNormalLoadNm(
                param_,
                total_torque_cmd_Nm,
                current_state_.vx,
                current_state_.vy,
                current_state_.r,
                relaxed_acceleration.ax_mps2,
                relaxed_acceleration.ay_mps2,
                current_state_.delta,
                last_allocator_total_limited_torque_Nm,
                last_allocator_total_limited_torque_valid,
                bypass_allocator_rate_limiter
            );
    }

control_output_.wheel_torque_cmd_Nm =
        allocation.torque_Nm;

    control_output_.total_torque_cmd_Nm =
        allocation.allocated_total_torque_Nm;

    control_output_.movement_cmd_interface =
        wheelTorqueAllocationToVehicleInterfaceMovement(
            param_,
            allocation
        );

    long_debug_.torque_cmd_Nm =
        control_output_.total_torque_cmd_Nm;


    publishControl(
        control_output_.steer_cmd_rad,
        control_output_.wheel_torque_cmd_Nm,
        control_output_.finished
    );

    publishAllDebug();
    publishReferencePoint();
}


double Controller::computeLaunchThrottleNm(const State& state,
                                           double dt_since_phase_start)
{
    (void)state;

    /*
        Launch from standstill is sampled by phase time.

        The map command is stored in current_runtime_command_.  In LAUNCH the
        front/rear split from that map is used directly for wheel allocation.
    */
    const double launch_phase_time_s =
        std::max(
            0.0,
            std::isfinite(dt_since_phase_start) ? dt_since_phase_start : 0.0
        );

    /*
        The launch phase starts when the controller becomes ready / AS DRIVE.

        The launch map itself starts only after the regulation delay:
            map_time = launch_phase_time - 3.0 s

        Before that, command zero torque and do not advance/sample the map.
    */
    if (launch_phase_time_s < kLaunchMapStartDelayAfterDriveS)
    {
        if (printConsoleDebugInfo())
        {
            ROS_INFO_STREAM_THROTTLE(
                0.5,
                "[acc_launch_control] Regulation launch hold active: "
                << launch_phase_time_s
                << " / "
                << kLaunchMapStartDelayAfterDriveS
                << " s. Commanding 0 Nm."
            );
        }

        return 0.0;
    }

    const double t_lookup_s =
        launch_phase_time_s
        - kLaunchMapStartDelayAfterDriveS;

    current_runtime_command_ =
        runtime_map_handler.getLaunchCommandByTime(
            t_lookup_s
        );

    if (!current_runtime_command_.valid)
    {
        if (printConsoleDebugInfo())
        {
            ROS_WARN_THROTTLE(
                0.5,
                "[acc_launch_control] Invalid launch runtime command by time. Returning 0 Nm."
            );
        }

        return 0.0;
    }

    if (!std::isfinite(current_runtime_command_.M_total_cmd_Nm))
    {
        if (printConsoleDebugInfo())
        {
            ROS_WARN_THROTTLE(
                0.5,
                "[acc_launch_control] Launch runtime command torque is NaN/Inf. Returning 0 Nm."
            );
        }

        return 0.0;
    }

    return current_runtime_command_.M_total_cmd_Nm;
}


double Controller::computeRuntimeAccelerationThrottleNm(
    const State& state,
    double dt_since_phase_start
)
{
    (void)dt_since_phase_start;

    return computeMaxAvailableDriveTorqueNm(
        param_,
        state.vx,
        state.vy,
        state.r,
        state.ax_mps2,
        state.ay_mps2,
        state.delta
    );
}


double Controller::computeRuntimeBrakeThrottleNm(
    const State& state,
    double dt_since_phase_start
)
{
    (void)dt_since_phase_start;

    return computeMaxAvailableBrakeTorqueNm(
        param_,
        state.vx,
        state.vy,
        state.r,
        state.ax_mps2,
        state.ay_mps2,
        state.delta
    );
}


bool Controller::shouldStartBrake(const State& state) const
{
    const double s_total_m =
        param_.get("general.s_total");

    /*
        Hard midpoint brake guard.

        Independent of predicted brake distance:
        after half of the configured straight distance, force transition
        to BRAKING. This prevents the car from staying in ACCELERATION /
        SPEED_HOLD too long if the brake-distance estimator is optimistic.
    */
    if (std::isfinite(s_total_m) &&
        s_total_m > 0.0 &&
        state.s > 0.5 * s_total_m)
    {
        return true;
    }

    return shouldStartBrakeByDistance(
        param_,
        state.s,
        state.vx,
        state.ax_mps2,
        control_output_.total_torque_cmd_Nm,
        nullptr,
        nullptr
    );
}


double Controller::computeSpeedHoldThrottleNm(const State& state,
                                              double dt_since_phase_start)
{
    (void)dt_since_phase_start;

    const double dt =
        getControlDt();

    double target_speed_mps =
        param_.get("general.vx_max");

    
    const double speed_error_mps =
        target_speed_mps - state.vx;

    pid_speed_hold_.update(
        speed_error_mps,
        dt,
        true,
        true
    );

    const double torque_ff_Nm =
        computeResistanceFeedforwardTorqueNm(
            param_,
            state.vx
        );

    const double torque_pid_Nm =
        pid_speed_hold_.get_output();

    return
        torque_ff_Nm
        + torque_pid_Nm;
}


double Controller::computeEmergencyBrakeThrottleNm(const State& state,
                                                   double dt_since_phase_start)
{
    (void)state;
    (void)dt_since_phase_start;

    return 0.0;
}


double Controller::computeLateralControlCommand()
{
    const double activation_speed_mps =
        param_.get("lateral_control.activation_speed_mps");

    if (!std::isfinite(current_state_.vx) ||
        current_state_.vx <= activation_speed_mps)
    {
        ltv_mpc_unbounded_.reset();
        lateral_controller_result_ = Result{};
        delta_cmd_ = 0.0;
        delta_dot_model_ = 0.0;
        current_state_.delta_cmd = 0.0;

        // The published steeringAngle_rad must be exactly zero below the gate.
        return 0.0;
    }

    const std::size_t horizon_size =
        static_cast<std::size_t>(
            std::llround(
                param_.get("model.ltv_mpc_unbounded.N")
            )
        );

    const double v_model_mps =
        std::max(
            current_state_.vx,
            param_.get("model.ltv_mpc_unbounded.v_min")
        );

    const std::vector<double> kappa_horizon(
        horizon_size,
        0.0
    );

    const std::vector<double> velocity_horizon(
        horizon_size,
        v_model_mps
    );

    lateral_controller_result_ =
        ltv_mpc_unbounded_.solve(
            current_state_,
            kappa_horizon,
            velocity_horizon
        );

    return integrateSteeringCommand(
        lateral_controller_result_.u_delta_cmd
    );
}



// =============================================================================
//                              OPTIONAL ODOMETRY CALLBACK
// =============================================================================

void Controller::odometryCallback(const nav_msgs::Odometry& msg)
{
    (void)msg;

    /*
        Intentionally unused in this ACC wrapper.
        Velocity is read from the Odometry pose message in poseCallback().
    */
}

// =============================================================================
//                              DV BOARD CALLBACK
// =============================================================================

void Controller::dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg)
{
    global_handling_info_.has_received_first_dv_board_message = true;
    current_state_.vx_enc =  0.25*(msg->velocity_FL
        + msg->velocity_FR
        + msg->velocity_RL
        + msg->velocity_RR);
}

// =============================================================================`  
//                              ANGLE SENSOR CALLBACK
// =============================================================================

void Controller::angleSensorCallback(const std_msgs::Float64& msg)
{
    const double max_steer =
        param_.get("steering_limit.max_steer");

    delta_encoder_ =
        clampLocal(
            std::isfinite(msg.data)
                ? msg.data
                : 0.0,
            -max_steer,
            max_steer
        );

    has_delta_encoder_ =
        true;

    current_state_.delta_enc =
        delta_encoder_;

    current_state_.delta =
        delta_encoder_;

    current_state_.delta_vehicle_used =
        delta_encoder_;

    emergency_check_input_.encoder_position_rad =
        delta_encoder_;

    emergency_check_input_.new_encoder_message_get =
        true;
}


// =============================================================================
//                              CUBE MARS STATUS CALLBACK
// =============================================================================
void Controller::cubeMarsStatusCallback(const std_msgs::Bool& msg)
{
    if(!global_handling_info_.cube_mars_initialization_finished) global_handling_info_.cube_mars_initialization_finished = msg.data;
}

// =============================================================================
//                             ODOM DEBUG CALLBACK
// =============================================================================
void Controller::odometryDebugCallback(const dv_interfaces::OdomDebug::ConstPtr& msg)
{
    current_state_.ax_mps2 = msg->accel_x;
    current_state_.ay_mps2 = msg->accel_y;

    global_handling_info_.has_received_imu_message = true;

   // std::cout << "Received odometry debug message: ax_mps2 = " << ax_mps2_ << std::endl;
}

void Controller::imuCallback(const dv_interfaces::Imu::ConstPtr& msg)
{
    current_state_.ax_mps2 =
        static_cast<double>(msg->acc.x);

    current_state_.ay_mps2 =
        static_cast<double>(msg->acc.y);

    global_handling_info_.has_received_imu_message =
        true;
}

// =============================================================================
//                             CONTROL PUBLISHING
// =============================================================================
void Controller::publishControl(double steering_rad,
                                const WheelValues& wheel_torque_cmd_Nm,
                                bool finished)
{
    control_output_.steer_cmd_rad =
        steering_rad;

    control_output_.wheel_torque_cmd_Nm =
        wheel_torque_cmd_Nm;

    control_output_.total_torque_cmd_Nm =
        sumWheelTorquesNm(
            wheel_torque_cmd_Nm
        );

    last_allocator_total_limited_torque_Nm =
        control_output_.total_torque_cmd_Nm;

    last_allocator_total_limited_torque_valid =
        true;

    control_output_.movement_cmd_interface =
        totalWheelTorqueNmToVehicleInterfaceMovement(
            param_,
            control_output_.total_torque_cmd_Nm
        );

    control_output_.finished =
        finished;

    const double gear_ratio =
        param_.get("model.drivetrain.gear_ratio");

    if (std::abs(gear_ratio) <= 1.0e-9)
    {
        throw std::runtime_error(
            "[acc_launch_control] model.drivetrain.gear_ratio must be non-zero"
        );
    }

    dv_interfaces::Control msg;

    msg.move_type =
        dv_interfaces::Control::FOUR_WHEEL;

    /*
        Allocator returns wheel-side torques [Nm], while the car interface
        expects motor-side values, so every wheel is divided by gear_ratio.
    */
    msg.torque_FL =
        static_cast<float>(wheel_torque_cmd_Nm.FL / gear_ratio);

    msg.torque_FR =
        static_cast<float>(wheel_torque_cmd_Nm.FR / gear_ratio);

    msg.torque_RL =
        static_cast<float>(wheel_torque_cmd_Nm.RL / gear_ratio);

    msg.torque_RR =
        static_cast<float>(wheel_torque_cmd_Nm.RR / gear_ratio);

    msg.steeringAngle_rad =
        static_cast<float>(steering_rad);

 

    msg.finished =
        finished;

    pub_control_.publish(msg);
}


void Controller::publishZeroControl(bool finished)
{
    publishControl(
        0.0,
        WheelValues{},
        finished
    );
}


void Controller::publishZeroMovement(double steering_rad,
                                     bool finished)
{
    publishControl(
        steering_rad,
        WheelValues{},
        finished
    );
}


void Controller::publishZeroSteering(const WheelValues& wheel_torque_cmd_Nm,
                                     bool finished)
{
    publishControl(
        0.0,
        wheel_torque_cmd_Nm,
        finished
    );
}




void Controller::publishEmergencyCheckDebug()
{
    dv_interfaces::EmergencyInfo msg;

    msg.header.stamp =
        ros::Time::now();

    msg.header.frame_id =
        "base_link";

    msg.competition_type =
        dv_interfaces::EmergencyInfo::COMPETITION_ACCELERATION;

    msg.emergency_current =
        emergency_check_current_ ||
        hard_emergency_check_current_;

    msg.emergency_latched =
        emergency_latched_;

    /*
        ACCELERATION launch convention:
            any emergency from EmergencyCheck is treated as HARD and latched.
        Therefore the debug message must report is_hard=true both while the
        emergency is current and after it has latched.
    */
    msg.is_hard =
        msg.emergency_current ||
        msg.emergency_latched;

    msg.emergency_reason_current =
        hard_emergency_check_current_
            ? hard_emergency_check_current_reason_
            : emergency_check_current_reason_;

    msg.emergency_reason_latched =
        emergency_reason_;

    msg.is_car_driving =
        phase_info_.phase == DrivePhase::LAUNCH ||
        phase_info_.phase == DrivePhase::ACCELERATION ||
        phase_info_.phase == DrivePhase::SPEED_HOLD ||
        phase_info_.phase == DrivePhase::BRAKING;

    msg.emergency_check_enabled =
        param_.getBool("general.use_emergency_check");

    msg.error_mask =
        dv_interfaces::EmergencyInfo::ERROR_NONE;

    const auto& cm =
        emergency_checker_.cube_mars_check;

    msg.use_cube_mars_encoder_check =
        param_.getBool("general.use_cube_mars_encoder_check");

    msg.use_cube_mars_following_check =
        param_.getBool("general.use_cube_mars_following_check");

    msg.cube_mars_emergency =
        cm.is_emergency;

    msg.cube_mars_initialized =
        cm.is_intialized;

    msg.cube_mars_encoder_stopped_sending =
        cm.encoder_stopped_sending;

    msg.cube_mars_reference_tracking_error_too_high =
        cm.encoder_reference_tracking_error_too_high;

    msg.cube_mars_time_since_last_encoder_update_s =
        cm.time_since_last_encoder_update;

    msg.cube_mars_encoder_position_rad =
        cm.curr_encoder_position_rad;

    msg.cube_mars_encoder_reference_position_rad =
        emergency_check_input_.encoder_reference_position_rad;

    msg.cube_mars_encoder_position_change_rad =
        cm.encoder_position_change;

    msg.cube_mars_reference_tracking_error_abs_rad =
        cm.encoder_reference_tracking_error_abs;

    msg.cube_mars_reference_tracking_error_avg_rad =
        cm.encoder_reference_tracking_error_avg;

    msg.cube_mars_no_message_threshold_s =
        cm.CUBE_MARS_TIME_WINDOW;

    msg.cube_mars_tracking_error_threshold_rad =
        cm.CUBE_MARS_REFRENCE_TRACKING_ERROR_THRESHOLD;

    if (msg.use_cube_mars_encoder_check && cm.encoder_stopped_sending)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_CUBEMARS_ENCODER_TIMEOUT;
    }

    if (msg.use_cube_mars_following_check &&
        cm.encoder_reference_tracking_error_too_high)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_CUBEMARS_TRACKING;
    }

    const auto& ds =
        emergency_checker_.dynamic_state_check;

    msg.use_dynamic_state_check =
        param_.getBool("general.use_dynamic_state_check");

    msg.dynamic_state_emergency =
        ds.is_emergency;

    msg.dynamic_state_initialized =
        ds.is_intialized;

    msg.dynamic_ey_over_limit =
        ds.ey_over_3m;

    msg.dynamic_epsi_over_limit =
        ds.epsi_over_90_deg;

    msg.dynamic_beta_over_limit =
        ds.beta_angle_over_20_deg;

    msg.dynamic_yaw_rate_over_limit =
        ds.yaw_rate_over_2_5_rad_per_sec;

    msg.dynamic_vx_mps =
        ds.curr_vx_mps;

    msg.dynamic_vy_mps =
        ds.curr_vy_mps;

    msg.dynamic_ey_m =
        ds.curr_ey_m;

    msg.dynamic_epsi_rad =
        ds.curr_epsi_rad;

    msg.dynamic_beta_rad =
        ds.curr_beta_rad;

    msg.dynamic_yaw_rate_radps =
        ds.curr_yaw_rate_rad_per_sec;

    msg.dynamic_ey_limit_m =
        ds.EY_THRESHOLD_M;

    msg.dynamic_epsi_limit_rad =
        ds.EPSI_THRESHOLD_RAD;

    msg.dynamic_beta_limit_rad =
        ds.BETA_THRESHOLD_RAD;

    msg.dynamic_yaw_rate_limit_radps =
        ds.YAW_RATE_THRESHOLD_RAD_PER_SEC;

    if (msg.use_dynamic_state_check && ds.ey_over_3m)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_EY;
    }

    if (msg.use_dynamic_state_check && ds.epsi_over_90_deg)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_EPSI;
    }

    if (msg.use_dynamic_state_check && ds.beta_angle_over_20_deg)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_BETA;
    }

    if (msg.use_dynamic_state_check && ds.yaw_rate_over_2_5_rad_per_sec)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_YAW_RATE;
    }

    const auto& ins =
        emergency_checker_.ins_pose_check;

    msg.use_ins_pose_check =
        param_.getBool("general.use_ins_pose_check");

    msg.use_ins_stability_check =
        param_.getBool("general.use_ins_stability_check");

    msg.use_ins_sliding_velocity_check =
        param_.getBool("general.use_ins_sliding_velocity_check");

    msg.ins_emergency =
        ins.is_emergency;

    msg.ins_initialized =
        ins.is_intialized;

    msg.ins_stopped_sending =
        ins.ins_stopped_sending;

    msg.ins_pose_is_changing =
        ins.ins_pose_is_changing;

    msg.ins_pose_is_stable =
        ins.ins_pose_is_stable;

    msg.ins_velocity_is_stable =
        ins.ins_velocity_is_stable;

    msg.ins_velocity_sliding =
        ins.ins_velocity_sliding;

    msg.ins_speed_over_1mps =
        ins.speed_over_1mps;

    msg.ins_speed_over_2_5mps =
        ins.speed_over_2_5mps;

    msg.ins_time_since_last_update_s =
        ins.time_since_last_ins_update;

    msg.ins_curr_x_m =
        ins.curr_x_m;

    msg.ins_curr_y_m =
        ins.curr_y_m;

    msg.ins_prev_x_m =
        ins.prev_x_m;

    msg.ins_prev_y_m =
        ins.prev_y_m;

    msg.ins_position_change_m =
        ins.ins_position_change;

    msg.ins_velocity_change_mps =
        ins.ins_velocity_change;

    msg.ins_vx_body_mps =
        emergency_check_input_.ins_velocity_body_xy_mps.x();

    msg.ins_vy_body_mps =
        emergency_check_input_.ins_velocity_body_xy_mps.y();

    msg.ins_vx_other_mps =
        current_state_.vx_enc;

    msg.ins_yaw_rate_radps =
        current_state_.r;

    msg.ins_no_message_threshold_s =
        ins.INS_NO_NEW_MESSAGE_THRESHOLD;

    msg.ins_pose_changing_threshold_m =
        ins.INS_CHANGING_THRESHOLD;

    msg.ins_pose_stability_threshold_m =
        ins.INS_STABLE_THRESHOLD;

    msg.ins_velocity_stability_threshold_mps =
        ins.INS_VELOCITY_STABLE_THRESHOLD;

    msg.ins_sliding_min_speed_mps =
        ins.INS_SLIDING_MIN_SPEED_MPS;

    msg.ins_sliding_lateral_to_longitudinal_ratio =
        ins.INS_SLIDING_LATERAL_TO_LONGITUDINAL_RATIO;

    msg.ins_sliding_max_yaw_rate_radps =
        ins.INS_SLIDING_MAX_YAW_RATE_RAD_PER_SEC;

    if (msg.use_ins_pose_check && ins.ins_stopped_sending)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_TIMEOUT;
    }

    if (msg.use_ins_pose_check && !ins.ins_pose_is_changing)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_POSE_NOT_CHANGING;
    }

    if (msg.use_ins_stability_check && !ins.ins_pose_is_stable)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_POSE_UNSTABLE;
    }

    if (msg.use_ins_stability_check && !ins.ins_velocity_is_stable)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_VELOCITY_UNSTABLE;
    }

    if (msg.use_ins_sliding_velocity_check && ins.ins_velocity_sliding)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_SLIDING;
    }

    const auto& pp =
        emergency_checker_.path_planner_check;

    (void)pp;

    /*
        ACC launch does not use PathPlannerCheck as an emergency source here.

        Keep the EmergencyInfo fields explicitly safe, regardless of the config
        flag, so the debug topic does not report stale / false path-planner
        emergency state.
    */
    msg.use_path_planner_check =
        false;

    msg.path_planner_emergency =
        false;

    msg.path_planner_initialized =
        true;

    msg.path_is_track_closed =
        true;

    msg.path_spline_is_valid =
        true;

    msg.path_planner_is_valid =
        true;

    msg.path_planner_stopped_sending =
        false;

    msg.path_bolide_before_first_point =
        false;

    msg.path_bolide_after_last_point =
        false;

    msg.path_bolide_in_break_between_last_and_first =
        false;

    msg.path_safe_because_track_closed =
        true;

    msg.path_time_since_last_update_s =
        0.0;

    msg.path_size =
        static_cast<uint32_t>(emergency_check_input_.path_xy.size());

    msg.path_bolide_x_m =
        emergency_check_input_.bolide_xy.x();

    msg.path_bolide_y_m =
        emergency_check_input_.bolide_xy.y();

    msg.path_no_message_threshold_s =
        0.0;

    msg.path_max_dist_to_break_m =
        0.0;

    pub_emergency_check_.publish(msg);
}


void Controller::publishAllDebug()
{
    publishLogInfoLongitudal();
    publishLogInfoLateral();
    publishGlobalHandlingInfo();
    publishEmergencyCheckDebug();
}



void Controller::publishLogInfoLateral()
{
    dv_interfaces::AccDebug_lat msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.if_safe =
        emergency_latched_ ? 0 : 1;

    msg.ey_m =
        current_state_.ey;

    msg.epsi_rad =
        current_state_.epsi;

    msg.curr_steer_rad =
        current_state_.delta_enc;

    msg.ref_steer_rad =
        lateral_controller_result_.delta_act_next;

    msg.steer_error_rad =
        msg.ref_steer_rad - msg.curr_steer_rad;

    msg.curr_yaw_rate_radps =
        current_state_.r;

    msg.ref_yaw_rate_radps =
        lateral_controller_result_.valid ?
        lateral_controller_result_.r_next :
        0.0;

    msg.yaw_rate_error_radps =
        msg.ref_yaw_rate_radps - msg.curr_yaw_rate_radps;

    msg.curr_vy_mps =
        current_state_.vy;

    msg.ref_vy_mps =
        lateral_controller_result_.valid ?
        lateral_controller_result_.vy_next :
        0.0;

    msg.vy_error_mps =
        msg.ref_vy_mps - msg.curr_vy_mps;

    pub_acc_debug_lat_.publish(msg);
}

void Controller::publishLogInfoLongitudal()
{
    dv_interfaces::AccDebug_long msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.if_safe =
        emergency_latched_ ? 0 : 1;

    const double R_tire =
        std::max(param_.get("model.tire.R_tire"), 1.0e-9);

    double v_ref_mps =
        0.0;

    double omega_ref_radps =
        0.0;

    double ref_slip_ratio =
        0.0;

    double s_ref_m =
        current_state_.s;

    double ref_ax_mps2 =
        0.0;

    double required_brake_distance_m = 0.0;
    double distance_available_for_brake_m = 0.0;

    (void)shouldStartBrakeByDistance(
        param_,
        current_state_.s,
        current_state_.vx,
        current_state_.ax_mps2,
        control_output_.total_torque_cmd_Nm,
        &required_brake_distance_m,
        &distance_available_for_brake_m
    );

    double optimistic_brake_distance_m =
        distance_available_for_brake_m
        - required_brake_distance_m;

    if (phase_info_.phase == DrivePhase::LAUNCH)
    {
        if (current_runtime_command_.valid)
        {
            v_ref_mps =
                current_runtime_command_.vx_mps;

            omega_ref_radps =
                0.5 *
                (
                    current_runtime_command_.omega_front_radps
                    + current_runtime_command_.omega_rear_radps
                );

            ref_slip_ratio =
                0.5 *
                (
                    current_runtime_command_.kappa_front
                    + current_runtime_command_.kappa_rear
                );

            s_ref_m =
                current_runtime_command_.s_global_m;

            ref_ax_mps2 =
                current_runtime_command_.ax_model_mps2;
        }
        else
        {
            v_ref_mps =
                current_state_.vx;

            omega_ref_radps =
                v_ref_mps / R_tire;
        }
    }
    else if (phase_info_.phase == DrivePhase::ACCELERATION)
    {
        v_ref_mps =
            param_.get("general.vx_max");

        omega_ref_radps =
            v_ref_mps / R_tire;
    }
    else if (phase_info_.phase == DrivePhase::SPEED_HOLD)
    {
        v_ref_mps =
            param_.get("general.vx_max");

        omega_ref_radps =
            v_ref_mps / R_tire;
    }
    else if (phase_info_.phase == DrivePhase::BRAKING)
    {
        v_ref_mps =
            0.0;

        omega_ref_radps =
            0.0;
    }
    else
    {
        v_ref_mps =
            0.0;

        omega_ref_radps =
            0.0;
    }

    msg.v_curr_mps =
        current_state_.vx;

    msg.v_ref_mps =
        v_ref_mps;

    msg.v_error_mps =
        msg.v_ref_mps - msg.v_curr_mps;

    const double wheel_linear_speed_mps =
        current_state_.vx_enc;

    msg.omega_curr_radps =
        wheel_linear_speed_mps / R_tire;

    msg.omega_ref_radps =
        omega_ref_radps;

    msg.omega_error_radps =
        msg.omega_ref_radps - msg.omega_curr_radps;

    msg.curr_slip_ratio =
        regularizedKappaFromWheelLinearSpeed(
            current_state_.vx,
            wheel_linear_speed_mps,
            param_
        );

    msg.ref_slip_ratio =
        ref_slip_ratio;

    msg.slip_ratio_error =
        msg.ref_slip_ratio - msg.curr_slip_ratio;

    msg.toruque_cmd_Nm =
        long_debug_.torque_cmd_Nm;

    msg.throttle_cmd_percent =
        control_output_.movement_cmd_interface;

    msg.distance_left_to_end =
        param_.get("general.s_total") - current_state_.s;

    msg.distance_optimistic_left_to_brake =
        optimistic_brake_distance_m;

    msg.s_ref_m =
        s_ref_m;

    msg.s_curr_m =
        current_state_.s;

    msg.phase =
        phaseToString(phase_info_.phase);

    msg.safety_reason =
        emergency_latched_ ? emergency_reason_ : std::string("OK");

    msg.emergency_brake_requested =
        emergency_latched_ ? 1 : 0;

    msg.max_speed_global_mps =
        param_.get("general.vx_max");

    msg.target_slip_ratio =
        msg.ref_slip_ratio;

    msg.curr_ax_mps2 =
        current_state_.ax_mps2;

    msg.ref_ax_mps2 =
        static_cast<float>(ref_ax_mps2);

    msg.ax_error_mps2 =
        msg.ref_ax_mps2 - msg.curr_ax_mps2;

    pub_acc_debug_long_.publish(msg);
}


void Controller::publishGlobalHandlingInfo()
{
    dv_interfaces::Acc_handling_info msg;

    msg.cube_mars_initialization_finished =
        global_handling_info_.cube_mars_initialization_finished;

    msg.has_received_first_dv_board_message =
        global_handling_info_.has_received_first_dv_board_message;

    msg.has_valid_path_from_pp =
        global_handling_info_.has_valid_path_from_pp;

    msg.ready_to_start_launch =
        global_handling_info_.ready_to_start_drive;
    
    msg.has_odometry_message =
        global_handling_info_.has_odometry_message;
    
    msg.has_received_imu_message =
        global_handling_info_.has_received_imu_message;
    
    msg.as_finished =
        global_handling_info_.as_finished;

    pub_global_handling_info_.publish(msg);
}
// =============================================================================
//                              VISUALIZATION
// =============================================================================

void Controller::publishReferencePath(const Eigen::VectorXd& X,
                                      const Eigen::VectorXd& Y)
{

    if(!global_handling_info_.has_valid_path_from_pp)
    {
        return;
    }
    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "acc_ref_path";

    marker.id =
        0;

    marker.type =
        visualization_msgs::Marker::LINE_STRIP;

    marker.action =
        visualization_msgs::Marker::ADD;

    marker.pose.orientation.w =
        1.0;

    marker.scale.x =
        0.05;

    marker.color.r =
        0.0;

    marker.color.g =
        1.0;

    marker.color.b =
        0.0;

    marker.color.a =
        1.0;

    marker.points.clear();
    marker.points.reserve(static_cast<std::size_t>(X.size()));

    for (int i = 0; i < X.size(); ++i)
    {
        geometry_msgs::Point p;

        p.x = X(i);
        p.y = Y(i);
        p.z = 0.0;

        marker.points.push_back(p);
    }

    pub_ref_path_.publish(marker);
}


void Controller::publishReferencePoint()
{
    if (!global_handling_info_.has_valid_path_from_pp)
    {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "acc_ref_point";

    marker.id =
        0;

    marker.type =
        visualization_msgs::Marker::SPHERE;

    marker.action =
        visualization_msgs::Marker::ADD;

    marker.pose.position.x = current_state_.s * path_dir_x_ + path_start_x_m_;
    marker.pose.position.y = current_state_.s * path_dir_y_ + path_start_y_m_;

    marker.pose.position.z =
        0.15;

    marker.pose.orientation.w =
        1.0;

    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    pub_ref_point_.publish(marker);
}



} // namespace acc_launch_control
