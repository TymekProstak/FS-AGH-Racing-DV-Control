 #include "wrapper.hpp"
#include <ros/service.h>
#include <std_srvs/Trigger.h>
#include "4wheel_utilites.hpp"
#include "segmented_skidpad_velocity_profile_helper.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace skidpad_control
{


namespace
{


double last_torque_allocator_global_scale_debug =
    1.0;

double last_torque_allocator_hard_limit_scale_debug =
    1.0;

double last_torque_allocator_rate_scale_debug =
    1.0;

double last_torque_allocator_rate_scale_min_debug =
    0.0;

double last_torque_allocator_rate_scale_max_debug =
    1.0;

double last_torque_allocator_total_after_tv_Nm_debug =
    0.0;

double last_torque_allocator_total_limited_Nm_debug =
    0.0;

double last_torque_allocator_previous_total_limited_Nm_debug =
    0.0;

bool last_torque_allocator_rate_limited_debug =
    false;

bool last_torque_allocator_rate_forced_over_hard_limit_debug =
    false;

bool last_torque_allocator_rate_bypass_debug =
    false;

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


inline double getSafeGearRatioForInterface(
    const ParamBank& param
)
{
    const double gear_ratio =
        param.get("model.drivetrain.gear_ratio");

    if (!std::isfinite(gear_ratio) ||
        std::abs(gear_ratio) <= 1.0e-9)
    {
        throw std::runtime_error(
            "[skidpad_control] model.drivetrain.gear_ratio must be finite and non-zero"
        );
    }

    return gear_ratio;
}


inline float wheelTorqueNmToVehicleInterfaceCommand(
    const ParamBank& param,
    const double wheel_torque_Nm
)
{
    /*
        Final output conversion only.

        Everything in controller / JACA / allocator is wheel-side torque [Nm].
        The vehicle interface fields torqueFL/FR/RL/RR receive motor-side
        torque command:

            interface_torque = wheel_torque_Nm / gear_ratio

        Do not use this conversion anywhere inside controller or allocator.
    */
    if (!std::isfinite(wheel_torque_Nm))
    {
        return 0.0f;
    }

    return static_cast<float>(
        wheel_torque_Nm
        / getSafeGearRatioForInterface(param)
    );
}


inline float totalWheelTorqueNmToVehicleInterfaceMovement(
    const ParamBank& param,
    const double total_wheel_torque_Nm
)
{
    /*
        Aggregate interface command. Internal total torque is wheel-side [Nm];
        the interface value is divided by the gear ratio.
    */
    if (!std::isfinite(total_wheel_torque_Nm))
    {
        return 0.0f;
    }

    return static_cast<float>(
        total_wheel_torque_Nm
        / getSafeGearRatioForInterface(param)
    );
}


double estimateLongitudinalAccelerationFromTorqueAndResistance(
    const ParamBank& param,
    double torque_cmd_Nm,
    double vx_mps)
{
    if (!std::isfinite(torque_cmd_Nm))
    {
        torque_cmd_Nm = 0.0;
    }

    if (!std::isfinite(vx_mps))
    {
        vx_mps = 0.0;
    }

    const double m =
        param.get("model.body.m");

    const double R_tire =
        param.get("model.tire.R_tire");

    if (!std::isfinite(m) || m <= 1.0e-9 ||
        !std::isfinite(R_tire) || R_tire <= 1.0e-9)
    {
        return 0.0;
    }

    const double resistance_torque_Nm =
        computeResistanceFeedforwardTorqueNm(
            param,
            vx_mps
        );

    double ax_mps2 =
        (torque_cmd_Nm - resistance_torque_Nm) / (m * R_tire);

    if (!std::isfinite(ax_mps2))
    {
        ax_mps2 = 0.0;
    }

    return ax_mps2;
}

struct JacaTorqueVectoringYawTarget
{
    double target_yaw_rate_radps = 0.0;
    double d_target_yaw_rate_d_delta = 0.0;
    double p_tv_gain = 0.0;
};

inline double smoothJacaTurnaroundGain(
    const double v_mps,
    const double v_low_mps,
    const double v_high_mps,
    const double transition_gain,
    const double low_speed_gain)
{
    const double dv =
        v_high_mps - v_low_mps;

    if (!std::isfinite(dv) || std::abs(dv) < 1.0e-9)
    {
        return 1.0;
    }

    const double z =
        (2.0 * v_mps - v_low_mps - v_high_mps) / dv;

    const double blend =
        0.5 * (1.0 - std::tanh(transition_gain * z));

    return
        1.0 + low_speed_gain * blend;
}

inline double safeForwardBodySpeedForJaca(
    const double v_raw_mps,
    const double v_min_mps)
{
    if (!std::isfinite(v_raw_mps))
    {
        return v_min_mps;
    }

    return std::max(v_raw_mps, v_min_mps);
}

inline JacaTorqueVectoringYawTarget calculateJacaTorqueVectoringYawTarget(
    const ParamBank& param,
    const double vx_body_mps,
    const double delta_act_rad,
    const double wheelbase_m)
{
    JacaTorqueVectoringYawTarget out;

    if (!param.getBool("general.general_use_jaca_torque_vectoring"))
    {
        return out;
    }

    out.p_tv_gain =
        param.get("model.torque_vectoring_jaca.p_tv_gain")
        * param.get("model.drivetrain.gear_ratio")
        * param.get("model.torque_vectoring_jaca.jaca_magic_number")
        * 4.0
        * param.get("model.torque_vectoring_jaca.track_width")
        / 2.0
        / param.get("model.tire.R_tire");

    if (!std::isfinite(out.p_tv_gain) || out.p_tv_gain <= 0.0)
    {
        out.p_tv_gain = 0.0;
        return out;
    }

    if (!std::isfinite(wheelbase_m) || wheelbase_m <= 1.0e-9)
    {
        out.p_tv_gain = 0.0;
        return out;
    }

    const double v_low_mps =
        param.get("model.torque_vectoring_jaca.speed_bled_low");

    const double v_high_mps =
        param.get("model.torque_vectoring_jaca.speed_bled_high");

    const double turn_radius_speed_gain =
        param.get("model.torque_vectoring_jaca.turn_radius_speed_gain");

    const double over_steer_gain =
        param.get("model.torque_vectoring_jaca.over_steer_gain");

    const double transition_gain =
        param.get("model.torque_vectoring_jaca.transition_gain");

    const double low_speed_gain =
        param.get("model.torque_vectoring_jaca.low_speed_gain");

    const double k_turn =
        smoothJacaTurnaroundGain(
            vx_body_mps,
            v_low_mps,
            v_high_mps,
            transition_gain,
            low_speed_gain
        );

    const double effective_wheelbase_m =
        wheelbase_m
        + turn_radius_speed_gain * vx_body_mps * vx_body_mps;

    if (!std::isfinite(effective_wheelbase_m) ||
        std::abs(effective_wheelbase_m) < 1.0e-9)
    {
        out.p_tv_gain = 0.0;
        return out;
    }

    const double yaw_target_gain =
        (1.0 + over_steer_gain) * k_turn;

    out.target_yaw_rate_radps =
        vx_body_mps
        * std::sin(delta_act_rad)
        * yaw_target_gain
        / effective_wheelbase_m;

    out.d_target_yaw_rate_d_delta =
        vx_body_mps
        * std::cos(delta_act_rad)
        * yaw_target_gain
        / effective_wheelbase_m;

    return out;
}

inline double calculateJacaTorqueVectoringYawMomentTargetNm(
    const ParamBank& param,
    const double vx_body_mps,
    const double yaw_rate_radps,
    const double delta_act_rad)
{
    const double wheelbase_m =
        param.get("model.body.l_f")
        + param.get("model.body.l_r");

    const double v_min_mps =
        param.get("model.ltv_mpc_unbounded.v_min");

    const double vx_safe_mps =
        safeForwardBodySpeedForJaca(
            vx_body_mps,
            v_min_mps
        );

    const JacaTorqueVectoringYawTarget tv_yaw_target =
        calculateJacaTorqueVectoringYawTarget(
            param,
            vx_safe_mps,
            delta_act_rad,
            wheelbase_m
        );

    return
        tv_yaw_target.p_tv_gain
        * (tv_yaw_target.target_yaw_rate_radps - yaw_rate_radps);
}


double estimateSkidpadEntryStraightLengthM(
    const Eigen::VectorXd& X,
    const Eigen::VectorXd& Y,
    const Eigen::VectorXd& S
)
{
    /*
        Estimate the entry straight length from the currently received path.

        Idea:
            entry straight has approximately zero curvature,
            the first right skidpad circle has curvature close to 1 / R_ref.

        Therefore entry length is the first S where curvature stays above
        a circle-like threshold for a few consecutive path segments.

        If the published path starts already on the right circle, this returns
        approximately 0.0 m. If no reliable circle start is found, it also
        returns 0.0 m as a safe fallback.
    */
    const int n = static_cast<int>(X.size());

    if (n < 6 || Y.size() != X.size() || S.size() != X.size())
    {
        return 0.0;
    }

    constexpr double kSkidpadReferenceRadiusM = 9.125;
    constexpr double kCircleCurvatureEnterFraction = 0.45;
    constexpr int kRequiredConsecutiveSegments = 5;

    const double kappa_circle =
        1.0 / kSkidpadReferenceRadiusM;

    const double kappa_threshold =
        kCircleCurvatureEnterFraction * kappa_circle;

    int consecutive = 0;

    for (int i = 1; i < n - 2; ++i)
    {
        const double kappa_abs =
            std::abs(
                estimateCurvatureAtSegment(
                    X,
                    Y,
                    i
                )
            );

        if (std::isfinite(kappa_abs) &&
            kappa_abs >= kappa_threshold)
        {
            ++consecutive;

            if (consecutive >= kRequiredConsecutiveSegments)
            {
                const int first_circle_segment =
                    std::max(
                        0,
                        i - kRequiredConsecutiveSegments + 1
                    );

                return clampLocal(
                    S(first_circle_segment),
                    S(0),
                    S(n - 1)
                );
            }
        }
        else
        {
            consecutive = 0;
        }
    }

    return 0.0;
}

} // anonymous namespace


// =============================================================================
//                              CONTROLLER
// =============================================================================

Controller::Controller(ros::NodeHandle& nh, const ParamBank& param)
    : param_(param),
      ltv_mpc_unbounded_(param),
      pid_speed_hold_(makeSpeedPidParams(param))
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

   // Comment out if the topic with velocity is the same as the one with pose.
    odom_sub_ = nh.subscribe(
        "/dv_odometry/odometry",
        1,
        &Controller::odometryCallback,
        this
    );

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

    pub_acc_debug_long_ = nh.advertise<dv_interfaces::SkidpadDebug_long>(
        "/skidpad_debug_long",
        1
    );

    pub_acc_debug_lat_ = nh.advertise<dv_interfaces::SkidpadDebug_lat>(
        "/skidpad_debug_lat",
        1
    );

    pub_global_handling_info_ =
        nh.advertise<dv_interfaces::Skidpad_handling_info>(
            "/skidpad_handling_info",
            1
        );

    pub_ref_path_ = nh.advertise<visualization_msgs::Marker>(
        "/skidpad/ref_path",
        1,
        true
    );

    pub_ref_point_ = nh.advertise<visualization_msgs::Marker>(
        "/skidpad/ref_point",
        1,
        true
    );

    pub_emergency_check_ =
        nh.advertise<dv_interfaces::EmergencyInfo>(
            "/emergency_info",
            1
        );

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
        ROS_INFO("[skidpad_control] Emergency checker timer started at 100 Hz.");
    }

    lateral_controller_type_ =
        LateralControllerType::LTV_MPC_UNBOUNDED;

    if (printConsoleDebugInfo())
    {
        ROS_INFO("[skidpad_control] Lateral: unbounded LTV MPC.");
        ROS_INFO("[skidpad_control] Controller initialized.");
    }
}


Controller::~Controller() = default;


// =============================================================================
//                              STATE HELPERS
// =============================================================================

double Controller::getControlDt() const
{
    const double hz =
        param_.get("model.frequency.steer_cmd_loop_hz");

    if (!std::isfinite(hz) || hz <= 0.0)
    {
        throw std::runtime_error(
            "[skidpad_control] model.frequency.steer_cmd_loop_hz must be finite and > 0"
        );
    }

    return 1.0 / hz;
}



bool Controller::useJacaTorqueVectoring() const
{
    return param_.getBool("general.general_use_jaca_torque_vectoring");
}


bool Controller::useOptimizedTorqueVectoring() const
{
    /*
        JACA has priority. If JACA is enabled, optimized Mz from LTV-MPC is not
        used even if the optimized-TV flag is accidentally also true.
    */
    return param_.getBool("general.general_use_torque_vectoring") &&
           !useJacaTorqueVectoring();
}


double Controller::getTorqueVectoringTrackWidth() const
{
    return param_.get("model.torque_vectoring_jaca.track_width");
}


double Controller::torqueDiffToYawMomentNm(double torque_diff_Nm) const
{
    const double R_tire =
        param_.get("model.tire.R_tire");

    const double track_width =
        getTorqueVectoringTrackWidth();

    if (!std::isfinite(R_tire) || R_tire <= 1.0e-9 ||
        !std::isfinite(track_width) || track_width <= 1.0e-9)
    {
        return 0.0;
    }

    return 0.5 * track_width * torque_diff_Nm / R_tire;
}


double Controller::yawMomentToTorqueDiffNm(double yaw_moment_Nm) const
{
    const double R_tire =
        param_.get("model.tire.R_tire");

    const double track_width =
        getTorqueVectoringTrackWidth();

    if (!std::isfinite(R_tire) || R_tire <= 1.0e-9 ||
        !std::isfinite(track_width) || track_width <= 1.0e-9)
    {
        return 0.0;
    }

    return 2.0 * yaw_moment_Nm * R_tire / track_width;
}


double Controller::getMaxTorqueVectoringYawMomentNm() const
{
    /*
        Current config convention only:
            model.torque_vectoring.max_torque_diff_Nm_engine

        This is engine-side torque delta per one wheel [Nm].
        For 4WD TV yaw moment limit:
            Mz_max = 2 * track_width * dT_engine * gear_ratio / R_tire
    */
    const double max_torque_diff_engine_per_wheel_Nm =
        param_.get("model.torque_vectoring.max_torque_diff_Nm_engine");

    const double gear_ratio =
        param_.get("model.drivetrain.gear_ratio");

    const double track_width =
        getTorqueVectoringTrackWidth();

    const double R_tire =
        param_.get("model.tire.R_tire");

    if (!std::isfinite(max_torque_diff_engine_per_wheel_Nm) ||
        !std::isfinite(gear_ratio) ||
        !std::isfinite(track_width) ||
        !std::isfinite(R_tire) ||
        track_width <= 0.0 ||
        std::abs(R_tire) <= 1.0e-9)
    {
        return 0.0;
    }

    return
        2.0
        * track_width
        * std::abs(max_torque_diff_engine_per_wheel_Nm)
        * std::abs(gear_ratio)
        / std::abs(R_tire);
}


double Controller::clampTorqueVectoringYawMomentNm(double yaw_moment_Nm) const
{
    const double max_yaw_moment_Nm =
        getMaxTorqueVectoringYawMomentNm();

    if (!std::isfinite(max_yaw_moment_Nm) || max_yaw_moment_Nm <= 0.0)
    {
        return 0.0;
    }

    return clampLocal(
        yaw_moment_Nm,
        -max_yaw_moment_Nm,
        max_yaw_moment_Nm
    );
}


void Controller::updateTorqueVectoringCommandFromLateralResult()
{
    optimized_torque_vectoring_used_ = false;
    jaca_torque_vectoring_used_ = useJacaTorqueVectoring();

    tv_yaw_moment_Nm_ = 0.0;
    tv_yaw_moment_raw_Nm_ = 0.0;
    tv_torque_diff_Nm_ = 0.0;

    if (useJacaTorqueVectoring())
    {
        const double jaca_yaw_moment_raw_Nm =
            calculateJacaTorqueVectoringYawMomentTargetNm(
                param_,
                current_state_.vx,
                current_state_.r,
                current_state_.delta_vehicle_used
            );

        tv_yaw_moment_raw_Nm_ =
            jaca_yaw_moment_raw_Nm;

        tv_yaw_moment_Nm_ =
            clampTorqueVectoringYawMomentNm(
                jaca_yaw_moment_raw_Nm
            );

        tv_torque_diff_Nm_ =
            yawMomentToTorqueDiffNm(
                tv_yaw_moment_Nm_
            );

        optimized_torque_vectoring_used_ = false;
        jaca_torque_vectoring_used_ = true;

        return;
    }

    if (!useOptimizedTorqueVectoring())
    {
        return;
    }

    if (!lateral_controller_result_.valid)
    {
        return;
    }

    /*
        Optimized-TV path:
            LTV-MPC already outputs a direct yaw moment Mz, not a JACA yaw-rate
            tracking target.
    */
    tv_yaw_moment_raw_Nm_ =
        lateral_controller_result_.u_tv_yaw_moment_raw_Nm;

    tv_yaw_moment_Nm_ =
        clampTorqueVectoringYawMomentNm(
            lateral_controller_result_.u_tv_yaw_moment_raw_Nm
        );

    tv_torque_diff_Nm_ =
        yawMomentToTorqueDiffNm(
            tv_yaw_moment_Nm_
        );

    optimized_torque_vectoring_used_ = true;
    jaca_torque_vectoring_used_ = false;
}


void Controller::applyTorqueVectoringToBaseTorque(double total_torque_cmd_Nm)
{
    /*
        Store only drivetrain-limited base / longitudinal torque.

        The final guard is repeated in publishControl(), but clamping here keeps
        the internal state consistent with the command that will be allocated and
        sent to the DV board.
    */
    if (!std::isfinite(total_torque_cmd_Nm))
    {
        total_torque_cmd_Nm = 0.0;
    }

    base_torque_cmd_Nm_ =
        clampTorqueByDrivetrainLimits(
            param_,
            total_torque_cmd_Nm,
            current_state_.vx
        );
}


void Controller::initializeSteeringStateIfNeeded()
{
    if (steering_state_initialized_)
    {
        return;
    }

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double initial_delta =
        has_delta_encoder_
            ? clampLocal(current_state_.delta_enc, -max_steer, max_steer)
            : 0.0;

    current_state_.delta_cmd = initial_delta;
    current_state_.delta_enc = initial_delta;
    current_state_.delta = initial_delta;
    current_state_.delta_vehicle_used = initial_delta;
    current_state_.delta_dot = 0.0;

    steering_state_initialized_ = true;
}


void Controller::updateSteeringStateForController()
{
    initializeSteeringStateIfNeeded();

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double max_steer_rate =
        param_.get("model.steering_limit.max_steer_rate");

    current_state_.delta_cmd =
        clampLocal(current_state_.delta_cmd, -max_steer, max_steer);

    if (has_delta_encoder_)
    {
        current_state_.delta_enc =
            clampLocal(current_state_.delta_enc, -max_steer, max_steer);
    }
    else
    {
        current_state_.delta_enc = current_state_.delta_cmd;
    }

    current_state_.delta = current_state_.delta_enc;
    current_state_.delta_dot =
        clampLocal(
            std::isfinite(current_state_.delta_dot)
                ? current_state_.delta_dot
                : 0.0,
            -max_steer_rate,
            max_steer_rate
        );
    current_state_.delta_vehicle_used = current_state_.delta;
}


double Controller::integrateSteeringCommand(double u_delta_cmd)
{
    initializeSteeringStateIfNeeded();

    const double dt =
        std::max(getControlDt(), 1.0e-9);

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double max_steer_rate =
        param_.get("model.steering_limit.max_steer_rate");

    const double limited_rate =
        clampLocal(
            std::isfinite(u_delta_cmd) ? u_delta_cmd : 0.0,
            -max_steer_rate,
            max_steer_rate
        );

    current_state_.delta_cmd =
        clampLocal(
            current_state_.delta_cmd + limited_rate * dt,
            -max_steer,
            max_steer
        );

    if (lateral_controller_result_.valid &&
        std::isfinite(lateral_controller_result_.delta_dot_next))
    {
        current_state_.delta_dot =
            clampLocal(
                lateral_controller_result_.delta_dot_next,
                -max_steer_rate,
                max_steer_rate
            );
    }
    else
    {
        current_state_.delta_dot = 0.0;
    }

    updateSteeringStateForController();
    return current_state_.delta_cmd;
}

void Controller::updateReadyFlag()
{
    global_handling_info_.ready_to_start_drive =
        global_handling_info_.cube_mars_initialization_finished &&
        global_handling_info_.has_received_first_dv_board_message &&
        global_handling_info_.has_valid_path_from_pp &&
        global_handling_info_.has_odometry_message &&
        global_handling_info_.has_received_imu_message;
}


void Controller::resetSpeedPid()
{
    pid_speed_hold_.reset();
}


void Controller::setPhase(DrivePhase new_phase)
{
    if (phase_ == new_phase)
    {
        return;
    }

    phase_ = new_phase;

    /*
        Przy zmianie fazy kasuję PID, żeby np. dodatnia całka z DRIVE
        nie walczyła z hamowaniem w BRAKING.
    */
    resetSpeedPid();
}


void Controller::updatePhase()
{
    updateReadyFlag();

    const double speed_abs =
        std::abs(current_state_.vx_enc);

    const double finished_encoder_speed =
        param_.get("general.finished_encoder_speed_mps");

    const double coast_below_speed =
        param_.get("general.coast_below_speed_mps");

    const bool speed_under_finished_threshold =
        speed_abs < finished_encoder_speed;

    const bool speed_under_coast_threshold =
        speed_abs < coast_below_speed;

    const bool hard_emergency_active =
        hard_emergency_check_current_ ||
        hard_emergency_check_latched_;

    const bool normal_emergency_active =
        emergency_check_current_;

    // -------------------------------------------------------------------------
    // HARD emergency is permanent once latched.
    //
    // It does NOT go through COASTING -> FINISHED. It also does not depend on
    // ready_to_start_drive after it has been latched. The only reset is a node /
    // controller restart or explicit future reset logic.
    // -------------------------------------------------------------------------
    if (phase_ == DrivePhase::EMERGENCY_STOP_HARD ||
        hard_emergency_active)
    {
        setPhase(DrivePhase::EMERGENCY_STOP_HARD);
        return;
    }

    if (global_handling_info_.as_finished)
    {
        setPhase(DrivePhase::FINISHED);
        return;
    }

    if (!global_handling_info_.ready_to_start_drive)
    {
        final_brake_requested_ = false;
        setPhase(DrivePhase::WAITING);
        return;
    }

    // -------------------------------------------------------------------------
    // BRAKING is the final-brake state in skidpad_control.
    // It is the only braking state that is allowed to finish the run:
    // BRAKING -> COASTING -> FINISHED.
    // -------------------------------------------------------------------------
    if (phase_ == DrivePhase::BRAKING)
    {
        final_brake_requested_ = true;

        if (speed_under_coast_threshold)
        {
            setPhase(DrivePhase::COASTING);
        }

        return;
    }

    if (phase_ == DrivePhase::COASTING)
    {
        if (global_handling_info_.has_received_first_dv_board_message &&
            speed_under_finished_threshold)
        {
            global_handling_info_.as_finished = true;
            setPhase(DrivePhase::FINISHED);
        }

        return;
    }

    if (phase_ == DrivePhase::FINISHED)
    {
        global_handling_info_.as_finished = true;
        return;
    }

    // -------------------------------------------------------------------------
    // Normal emergency is recoverable.
    //
    // It can only be entered from DRIVE. While already in EMERGENCY_STOP, the
    // emergency checker is still allowed to run. If the normal emergency clears
    // and there is no hard latch, the controller returns to DRIVE.
    //
    // It does NOT go through COASTING -> FINISHED.
    // -------------------------------------------------------------------------
    if (phase_ == DrivePhase::EMERGENCY_STOP)
    {
        if (normal_emergency_active)
        {
            return;
        }

        setPhase(DrivePhase::DRIVE);
        return;
    }

    // -------------------------------------------------------------------------
    // New emergency transitions are accepted only from DRIVE.
    // This prevents BRAKING / COASTING / FINISHED from being reclassified into
    // emergency states by checks that are no longer meaningful there.
    // -------------------------------------------------------------------------
    if (phase_ == DrivePhase::DRIVE)
    {
        if (hard_emergency_active)
        {
            setPhase(DrivePhase::EMERGENCY_STOP_HARD);
            return;
        }

        if (normal_emergency_active)
        {
            setPhase(DrivePhase::EMERGENCY_STOP);
            return;
        }

        if (has_valid_path_from_pp_ && S_last_from_pp_.size() > 1)
        {
            const double total_s =
                S_last_from_pp_(S_last_from_pp_.size() - 1);

            const double brake_margin =
                param_.get("general.brake_start_distance_before_end_m");

            const bool on_final_straight =
                along_skidpad_ref_path_m_ >= total_s - brake_margin;

            if (on_final_straight)
            {
                final_brake_requested_ = true;
            }
        }

        if (final_brake_requested_)
        {
            setPhase(DrivePhase::BRAKING);
            return;
        }
    }

    if (phase_ == DrivePhase::WAITING)
    {
        setPhase(DrivePhase::DRIVE);
    }
}




// =============================================================================
//                              EMERGENCY CHECK TIMER
// =============================================================================

void Controller::emergencyCheckTimerCallback(
    const ros::TimerEvent& event
)
{
    (void)event;

    const double current_time_s =
        ros::Time::now().toSec();

    const DrivePhase phase_now =
        phase_;

    /*
        The checker runs in DRIVE and in recoverable normal emergency stop.

        DRIVE:
            - normal emergency may enter EMERGENCY_STOP,
            - hard emergency may latch and enter EMERGENCY_STOP_HARD.

        EMERGENCY_STOP:
            - only normal emergency current is tracked so it can clear and allow
              the state machine to return to DRIVE,
            - hard emergency is deliberately not latched here because new
              emergency transitions are allowed only from DRIVE.

        Other phases:
            - checker input is disarmed / treated as not driving.
    */
    const bool checker_active_now =
        phase_now == DrivePhase::DRIVE ||
        phase_now == DrivePhase::EMERGENCY_STOP;

    const bool hard_latch_allowed_now =
        phase_now == DrivePhase::DRIVE;

    const bool new_encoder_message_get =
        emergency_check_input_.new_encoder_message_get;

    const bool new_ins_message_get =
        emergency_check_input_.new_ins_message_get;

    const bool new_path_planner_message_get =
        emergency_check_input_.new_path_planner_message_get;

    /*
        These flags mean "new message since the previous emergency timer tick".
        I copy them first and clear them immediately, exactly like in dv_control.
        Then the checker receives edge-like information at 100 Hz instead of
        a permanently latched "new message" flag.
    */
    emergency_check_input_.new_encoder_message_get =
        false;

    emergency_check_input_.new_ins_message_get =
        false;

    emergency_check_input_.new_path_planner_message_get =
        false;

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

    const bool hard_emergency_now_raw =
        emergency_checker_.is_hard_emergency;

    const std::string hard_emergency_reason =
        emergency_checker_.hard_emergency_reason.empty()
            ? emergency_checker_.emergency_reason
            : emergency_checker_.hard_emergency_reason;

    emergency_check_current_ =
        checker_active_now
            ? emergency_now
            : false;

    hard_emergency_check_current_ =
        hard_latch_allowed_now
            ? hard_emergency_now_raw
            : false;

    emergency_check_current_reason_ =
        emergency_check_current_
            ? emergency_checker_.emergency_reason
            : "OK";

    hard_emergency_check_current_reason_ =
        hard_emergency_check_current_
            ? hard_emergency_reason
            : "OK";

    /*
        Only HARD emergency is latched, and only when detected in DRIVE.
        This latch is permanent for the lifetime of this controller instance;
        it is intentionally not cleared when the phase leaves DRIVE.
    */
    if (hard_latch_allowed_now &&
        hard_emergency_now_raw &&
        !hard_emergency_check_latched_)
    {
        hard_emergency_check_latched_ =
            true;

        hard_emergency_check_latched_reason_ =
            hard_emergency_reason;
    }

    publishEmergencyCheckDebug();

    if (hard_emergency_check_current_ || hard_emergency_check_latched_)
    {
        if (printConsoleDebugInfo())
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                "[skidpad_control][HardEmergencyCheck] "
                << hard_emergency_check_latched_reason_
            );
        }
    }
    else if (emergency_check_current_)
    {
        if (printConsoleDebugInfo())
        {
            ROS_WARN_STREAM_THROTTLE(
                0.2,
                "[skidpad_control][EmergencyCheck] "
                << emergency_check_current_reason_
            );
        }
    }
}



// =============================================================================
//                              PATH / PROJECTION
// =============================================================================

void Controller::pathCallback(const geometry_msgs::PoseArray& msg)
{

    // //static bool already_loaded_path = false;

    // if (already_loaded_path)
    // {
    //     return;
    // }

    // already_loaded_path = true;
    copyPathMessageToEigen(msg, X_last_from_pp_, Y_last_from_pp_);

    emergency_check_input_.path_xy.clear();
    emergency_check_input_.path_xy.reserve(msg.poses.size());

    for (const auto& pose : msg.poses)
    {
        emergency_check_input_.path_xy.emplace_back(
            pose.position.x,
            pose.position.y
        );
    }

    emergency_check_input_.path_is_track_closed = false;
    emergency_check_input_.new_path_planner_message_get = true;

    if (X_last_from_pp_.size() < 2 ||
        Y_last_from_pp_.size() != X_last_from_pp_.size())
    {
        has_valid_path_from_pp_ = false;
        has_valid_path_spline_ = false;

        global_handling_info_.has_valid_path_from_pp = false;

        emergency_check_input_.spline_is_valid = false;

        if (printConsoleDebugInfo())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[skidpad_control] Invalid path from PP."
            );
        }

        return;
    }
     const bool new_path_start =
        isNewPathStart(X_last_from_pp_(0), Y_last_from_pp_(0));

    if (!new_path_start)
    {
        if (printConsoleDebugInfo())
        {
            ROS_DEBUG_THROTTLE(
                1.0,
                "[skidpad_control] Received PP path, but start is the same as previous. Ignoring."
            );
        }

        return;
    }
     if (printConsoleDebugInfo())
     {
         ROS_INFO(
            "[skidpad_control] Received new path from path planning. Resetting path projection and path spline."
        );
     }
    
    S_last_from_pp_ =
        buildArcLength(X_last_from_pp_, Y_last_from_pp_);

    skidpad_entry_straight_length_m_ =
        estimateSkidpadEntryStraightLengthM(
            X_last_from_pp_,
            Y_last_from_pp_,
            S_last_from_pp_
        );

    if (printConsoleDebugInfo())
    {
        ROS_INFO(
            "[skidpad_control] Estimated skidpad entry straight length: %.3f m",
            skidpad_entry_straight_length_m_
        );
    }

    has_valid_path_spline_ =
        path_spline_.fit(
            S_last_from_pp_,
            X_last_from_pp_,
            Y_last_from_pp_
        );

    emergency_check_input_.spline_is_valid =
        has_valid_path_spline_;

    if (!has_valid_path_spline_)
    {
        if (printConsoleDebugInfo())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[skidpad_control] Failed to build path spline. Falling back to polyline curvature."
            );
        }
    }

    has_valid_path_from_pp_ = true;
    global_handling_info_.has_valid_path_from_pp = true;

    /*
        WAŻNE:
        Nie resetuję projekcji przy każdym przyjściu /path_planning/path.

        Ten sam path przychodzi cyklicznie, więc resetowanie:
            has_path_projection_ = false;
            last_projection_segment_index_ = 0;
            along_skidpad_ref_path_m_ = 0.0;

        powoduje, że kontroler co chwilę robi globalną inicjalizację.
        Na skidpadzie to jest złe, bo kolejne okrążenia mają te same XY.
    */
    if (!has_path_projection_)
    {
        last_projection_segment_index_ = 0;
        along_skidpad_ref_path_m_ = 0.0;
    }
    else
    {
        last_projection_segment_index_ =
            std::max(
                0,
                std::min(
                    last_projection_segment_index_,
                    static_cast<int>(X_last_from_pp_.size()) - 2
                )
            );

        along_skidpad_ref_path_m_ =
            clampLocal(
                along_skidpad_ref_path_m_,
                S_last_from_pp_(0),
                S_last_from_pp_(S_last_from_pp_.size() - 1)
            );
    }

    publishReferencePath(X_last_from_pp_, Y_last_from_pp_);
}


bool Controller::updatePathProjectionForCurrentController()
{
    if (!has_valid_path_from_pp_)
    {
        return false;
    }

    double qx =
        current_state_.x;

    double qy =
        current_state_.y;

    //// ------------------------------------------------------------------------
    //// Stage 1: RAW monotonic segment search.
    ////
    //// This is the topology / anti-jump stage. I search only on the RAW path
    //// from path planning. No spline nearest point, no yaw score, no branch
    //// switching. The result gives a safe monotonic anchor:
    ////     coarse_projection.segment_index
    ////     coarse_projection.s_m
    //// ------------------------------------------------------------------------

    const double previous_s_m =
        along_skidpad_ref_path_m_;

    const bool had_previous_projection =
        has_path_projection_;

    const PathProjection coarse_projection =
        projectToPathMonotonic(
            X_last_from_pp_,
            Y_last_from_pp_,
            S_last_from_pp_,
            qx,
            qy,
            current_state_.yaw,
            last_projection_segment_index_,
            previous_s_m,
            had_previous_projection
        );

    if (!coarse_projection.valid)
    {
        return false;
    }

    //// ------------------------------------------------------------------------
    //// Stage 2: local spline refinement.
    ////
    //// The RAW projection decides where I am on the track. The spline is only
    //// allowed to refine locally around that RAW s. This gives smooth:
    ////     s, x_ref, y_ref, yaw_ref, ey, epsi, kappa_ref
    //// without losing the monotonic RAW segment anchor.
    //// ------------------------------------------------------------------------

    constexpr double kSplineLocalProjectionHalfWindowM = 0.20;

    PathProjection final_projection =
        coarse_projection;

    if (has_valid_path_spline_ && path_spline_.isValid())
    {
        const double min_allowed_s_m =
            had_previous_projection
                ? previous_s_m
                : path_spline_.sMin();

        const PathProjection refined_projection =
            refineProjectionOnSplineLocal(
                path_spline_,
                coarse_projection,
                qx,
                qy,
                current_state_.yaw,
                min_allowed_s_m,
                kSplineLocalProjectionHalfWindowM
            );

        if (refined_projection.valid)
        {
            final_projection =
                refined_projection;
        }
    }

    proj_ =
        final_projection;

    current_state_.ey =
        proj_.ey_m;

    current_state_.epsi =
        proj_.epsi_rad;

    current_state_.s =
        proj_.s_m;

    // The next monotonic search must stay anchored to the RAW segment search,
    // not to a spline-global nearest point.
    last_projection_segment_index_ =
        coarse_projection.segment_index;

    along_skidpad_ref_path_m_ =
        proj_.s_m;

    has_path_projection_ =
        true;

    publishReferencePoint();

    return true;
}


// =============================================================================
//                             CONTROL CALLBACK
// =============================================================================

void Controller::poseCallback(const nav_msgs::Odometry& msg)
{
    // -------------------------------------------------------------------------
    // Diagnostic rate counter
    // -------------------------------------------------------------------------
    /*
        This counter checks how often this node really enters poseCallback().
        It is intentionally not throttled with ROS_INFO_THROTTLE, because here
        I want a true 1-second counted rate, not occasional sampled logs.
    */
    static int pose_callback_count = 0;
    static ros::WallTime last_pose_rate_print_time = ros::WallTime::now();

    ++pose_callback_count;

    const ros::WallTime now_pose_rate = ros::WallTime::now();
    const double pose_rate_window_s =
        (now_pose_rate - last_pose_rate_print_time).toSec();

    if (pose_rate_window_s >= 1.0)
    {
        // ROS_INFO(
        //     "[rate_check] poseCallback real rate = %.1f Hz",
        //     static_cast<double>(pose_callback_count) / pose_rate_window_s
        // );

        pose_callback_count = 0;
        last_pose_rate_print_time = now_pose_rate;
    }

    // -------------------------------------------------------------------------
    // Pose
    // -------------------------------------------------------------------------

    current_state_.x =
        msg.pose.pose.position.x;

    current_state_.y =
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

    current_state_.yaw =
        yaw;

    // -------------------------------------------------------------------------
    // Velocity in body frame
    // -------------------------------------------------------------------------
    // Use this only when the pose topic also contains valid velocity.

    // const double vx_world =
    //     msg.twist.twist.linear.x;

    // const double vy_world =
    //     msg.twist.twist.linear.y;

    // const double c =
    //     std::cos(current_state_.yaw);

    // const double s =
    //     std::sin(current_state_.yaw);

    // const double vx_body =
    //     c * vx_world + s * vy_world;

    // const double vy_body =
    //    -s * vx_world + c * vy_world;

    current_state_.vx =  msg.twist.twist.linear.x;

    current_state_.vy =  msg.twist.twist.linear.y;

    
    current_state_.r =
        msg.twist.twist.angular.z;

    global_handling_info_.has_odometry_message = true;

    emergency_check_input_.ins_xy =
        Eigen::Vector2d(
            current_state_.x,
            current_state_.y
        );

    emergency_check_input_.ins_velocity_body_xy_mps =
        Eigen::Vector2d(
            current_state_.vx,
            current_state_.vy
        );

    emergency_check_input_.bolide_xy =
        Eigen::Vector2d(
            current_state_.x,
            current_state_.y
        );

    emergency_check_input_.new_ins_message_get =
        true;

    // -------------------------------------------------------------------------
    // Path projection
    // -------------------------------------------------------------------------

    if (!updatePathProjectionForCurrentController())
    {
        has_path_projection_ = false;

        if (printConsoleDebugInfo())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[skidpad_control] Path projection is invalid. Publishing last control command instead."
            );
        }

        publishControl(
            control_output_.steer_rad,
            base_torque_cmd_Nm_,
            control_output_.finished
        );

        publishAllDebug();
        return;
    }

    // -------------------------------------------------------------------------
    // Phase
    // -------------------------------------------------------------------------

    updatePhase();
    updateReadyFlag();

    const double dt =
        getControlDt();

    if (global_handling_info_.ready_to_start_drive)
    {
        // ---------------------------------------------------------------------
        // Longitudinal control
        // ---------------------------------------------------------------------

        constexpr double SAFE_EMERGENCY_SPEED_MPS =
            3.0;

        const bool phase_is_braking =
            phase_ == DrivePhase::BRAKING;

        const bool phase_is_normal_emergency =
            phase_ == DrivePhase::EMERGENCY_STOP;

        const bool phase_is_hard_emergency =
            phase_ == DrivePhase::EMERGENCY_STOP_HARD;

        double target_speed_mps =
            segmentedSkidpadVelocityRefAtS(
                param_,
                along_skidpad_ref_path_m_,
                skidpad_entry_straight_length_m_
            );

        bool run_longitudinal_pid =
            true;

        if (phase_is_braking)
        {
            /*
                Final braking:
                    brake with speed PID to 0 m/s.
            */
            target_speed_mps =
                0.0;
        }
        else if (phase_is_normal_emergency)
        {
            /*
                Recoverable emergency stop:
                    keep normal steering,
                    slow down / hold the car at safe rolling speed.

                This state must not finish the AS mission.
            */
            target_speed_mps =
                SAFE_EMERGENCY_SPEED_MPS;
        }
        else if (phase_is_hard_emergency)
        {
            /*
                HARD emergency stop:
                    do not run speed PID.

                The final output is handled after lateral control as:
                    publishZeroControl(false)
                    then /dv_board/emergency service call.

                That publishes:
                    steering = 0
                    movement = 0
                    per-wheel torques = 0
                    Control.finished = false

                It deliberately does not set global_handling_info_.as_finished.
            */
            target_speed_mps =
                0.0;

            run_longitudinal_pid =
                false;
        }

        current_state_.v_ref =
            target_speed_mps;

        current_state_.a_ref =
            0.0;

        if (!run_longitudinal_pid)
        {
            applyTorqueVectoringToBaseTorque(
                0.0
            );
        }
        else
        {
            /*
                Speed feedback selection.

                Normal DRIVE / BRAKING:
                    use INS / odometry body vx.

                Recoverable EMERGENCY_STOP:
                    use encoder speed, because wheel encoder is treated here
                    as the more reliable signal for emergency speed-hold
                    to 3 m/s.

                This affects only the speed PID feedback and drivetrain
                power-limit speed in this branch. The rest of the vehicle state
                still uses current_state_.vx.
            */
            double speed_feedback_mps =
                current_state_.vx;

            if (phase_is_normal_emergency)
            {
                speed_feedback_mps =
                    std::isfinite(current_state_.vx_enc)
                        ? std::max(0.0, current_state_.vx_enc)
                        : current_state_.vx;
            }

            const double speed_error_mps =
                current_state_.v_ref - speed_feedback_mps;

            /*
                Feedforward only in normal DRIVE.

                BRAKING:
                    pure PID to 0 m/s, optionally multiplied by brake_gain.

                EMERGENCY_STOP:
                    pure PID to 3 m/s, but feedback is vx_enc.
                    No brake_gain here, because this is recoverable safe
                    rolling mode, not final braking.
            */

            double torque_ff_Nm =
                (phase_ == DrivePhase::DRIVE)
                    ? computeResistanceFeedforwardTorqueNm(
                          param_,
                          current_state_.vx
                      )
                    : 0.0;

            pid_speed_hold_.update(
                speed_error_mps,
                dt,
                true,
                true
            );

            const double torque_pid_Nm =
                pid_speed_hold_.get_output();

            double torque_cmd_Nm =
                torque_ff_Nm
                + torque_pid_Nm;

            if (phase_is_braking)
            {
                torque_cmd_Nm =
                    param_.get("general.brake_gain")
                    * torque_cmd_Nm;
            }

            /*
                Keep the old low-speed boost behavior, but base the threshold
                on the selected speed feedback. In EMERGENCY_STOP this means
                vx_enc decides whether the low-speed boost branch is active.
            */
            if (phase_is_braking || speed_feedback_mps < 5.0)
            {
                torque_cmd_Nm =
                    param_.get("general.brake_gain") * 2.00
                    * torque_cmd_Nm;
            }

            torque_cmd_Nm =
                clampTorqueByDrivetrainLimits(
                    param_,
                    torque_cmd_Nm,
                    speed_feedback_mps
                );

            applyTorqueVectoringToBaseTorque(
                torque_cmd_Nm
            );
        }

        // ---------------------------------------------------------------------
        // Lateral control
        // ---------------------------------------------------------------------

        std::vector<double> kappa_horizon;

        if (has_valid_path_spline_)
        {
            kappa_horizon =
                buildSplineCurvatureHorizon(
                    param_,
                    path_spline_,
                    along_skidpad_ref_path_m_,
                    current_state_.vx
                );
        }
        else
        {
            kappa_horizon =
                buildCurvatureHorizon(
                    param_,
                    X_last_from_pp_,
                    Y_last_from_pp_,
                    S_last_from_pp_,
                    along_skidpad_ref_path_m_,
                    current_state_.vx
                );
        }

        const std::vector<double> v_body_ref_horizon =
            buildSegmentedSkidpadVelocityRefHorizon(
                param_,
                along_skidpad_ref_path_m_,
                current_state_.vx,
                skidpad_entry_straight_length_m_
            );

        updateSteeringStateForController();

        lateral_controller_result_ =
            ltv_mpc_unbounded_.solve(
                current_state_,
                kappa_horizon,
                v_body_ref_horizon
            );

        control_output_.steer_rad =
            integrateSteeringCommand(
                lateral_controller_result_.u_delta_cmd
            );

        if (!lateral_controller_result_.valid)
        {
            if (printConsoleDebugInfo())
            {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[skidpad_control] Lateral controller returned invalid result. Publishing last control command instead."
                );
            }

            publishControl(
                control_output_.steer_rad,
                base_torque_cmd_Nm_,
                control_output_.finished
            );

            publishAllDebug();
            return;
        }

        updateTorqueVectoringCommandFromLateralResult();

        emergency_check_input_.encoder_reference_position_rad =
            control_output_.steer_rad;

    }

    // -------------------------------------------------------------------------
    // Publish control and debug info
    // -------------------------------------------------------------------------

    if (phase_ == DrivePhase::WAITING)
    {
        publishZeroControl(false);
        publishAllDebug();
        return;
    }

    if (phase_ == DrivePhase::DRIVE)
    {
        publishControl(
            control_output_.steer_rad,
            base_torque_cmd_Nm_
        );

        publishAllDebug();
        return;
    }

    if (phase_ == DrivePhase::BRAKING)
    {
        publishControl(
            control_output_.steer_rad,
            base_torque_cmd_Nm_
        );

        publishAllDebug();
        return;
    }

    if (phase_ == DrivePhase::EMERGENCY_STOP)
    {
        publishControl(
            control_output_.steer_rad,
            base_torque_cmd_Nm_
        );

        publishAllDebug();
        return;
    }

    if (phase_ == DrivePhase::EMERGENCY_STOP_HARD)
    {
        /*
            HARD emergency output order:
                1) zero steering / zero torque on /dv_board/control,
                   with Control.finished=false,
                2) then request the DV-board emergency service.

            Do not use Control.finished=true here. The hard stop is delegated to
            /dv_board/emergency after the zero command is already on the topic.
        */
        publishZeroControl(false);

        callDvBoardEmergencyServiceSafe(
            "[skidpad_control]"
        );

        publishAllDebug();
        return;
    }

    if (phase_ == DrivePhase::COASTING)
    {
        publishZeroMovement(control_output_.steer_rad);
        publishAllDebug();
        return;
    }

    if (phase_ == DrivePhase::FINISHED)
    {
        publishZeroMovement(control_output_.steer_rad, true);
        publishAllDebug();
        return;
    }
}


// =============================================================================
//                              OPTIONAL ODOMETRY CALLBACK
// =============================================================================

void Controller::odometryCallback(const nav_msgs::Odometry& msg)
{
    /*
        Optional velocity callback, matching dv_control.

        Use it when /dv_odometry/odometry carries body-frame velocity / yaw rate
        separately from /ins/pose. If /ins/pose already carries valid twist,
        these assignments are simply refreshed from the odometry topic.
    */
    current_state_.vx =
        msg.twist.twist.linear.x;

    current_state_.vy =
        msg.twist.twist.linear.y;

    current_state_.r =
        msg.twist.twist.angular.z;

    global_handling_info_.has_odometry_message =
        true;
}

// =============================================================================
//                              ODOM DEBUG CALLBACK
// =============================================================================

void Controller::odometryDebugCallback(const dv_interfaces::OdomDebug::ConstPtr& msg)
{
    current_state_.acc_x =
        msg->accel_x;

    current_state_.acc_y =
        msg->accel_y;

    global_handling_info_.has_received_imu_message =
        true;
}

void Controller::imuCallback(const dv_interfaces::Imu::ConstPtr& msg)
{
    current_state_.acc_x =
        static_cast<double>(msg->acc.x);

    current_state_.acc_y =
        static_cast<double>(msg->acc.y);

    global_handling_info_.has_received_imu_message =
        true;
}

void Controller::dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg)
{
    global_handling_info_.has_received_first_dv_board_message = true;
    current_state_.vx_enc = 0.25*(msg->velocity_FL + msg->velocity_FR + msg->velocity_RL + msg->velocity_RR);
}

void Controller::angleSensorCallback(const std_msgs::Float64& msg)
{
    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double measured_delta =
        clampLocal(
            msg.data,
            -max_steer,
            max_steer
        );

    current_state_.delta_enc =
        measured_delta;

    has_delta_encoder_ =
        true;

    current_state_.delta =
        current_state_.delta_enc;

    current_state_.delta_vehicle_used =
        current_state_.delta;

    emergency_check_input_.encoder_position_rad =
        current_state_.delta_enc;

    emergency_check_input_.new_encoder_message_get =
        true;
}

void Controller::cubeMarsStatusCallback(const std_msgs::Bool& msg)
{
    global_handling_info_.cube_mars_initialization_finished = msg.data;
}


// =============================================================================
//                              CONTROL PUBLISHING
// =============================================================================

void Controller::publishControl(double steering_rad,
                                double total_torque_cmd_Nm,
                                bool finished)
{
    control_output_.steer_rad =
        steering_rad;

    control_output_.finished =
        finished;

    dv_interfaces::Control msg;

    /*
        All controller, TV, JACA and allocator values are wheel-side Nm.
    */
    msg.move_type =
        dv_interfaces::Control::FOUR_WHEEL;

    msg.steeringAngle_rad =
        static_cast<float>(steering_rad);


    msg.finished =
        finished;

    if (!std::isfinite(total_torque_cmd_Nm))
    {
        total_torque_cmd_Nm = 0.0;
    }

    /*
        Do not command negative torque while the encoder says the car is
        rolling backwards. This is only a final safety guard.
    */
    if (std::isfinite(current_state_.vx_enc) &&
        current_state_.vx_enc < 0.0)
    {
        total_torque_cmd_Nm =
            std::max(
                total_torque_cmd_Nm,
                0.0
            );
    }

    /*
        Final pre-allocation clamp.

        The base / longitudinal torque must be drivetrain-limited before the
        allocator sees it and before anything is converted to DV-board units.
        Torque vectoring is handled separately through desired_yaw_moment_Nm.
    */
    total_torque_cmd_Nm =
        clampTorqueByDrivetrainLimits(
            param_,
            total_torque_cmd_Nm,
            current_state_.vx
        );

    base_torque_cmd_Nm_ =
        total_torque_cmd_Nm;

    const double ax_for_allocation_mps2 =
        std::isfinite(current_state_.acc_x)
            ? current_state_.acc_x
            : estimateLongitudinalAccelerationFromTorqueAndResistance(
                  param_,
                  total_torque_cmd_Nm,
                  current_state_.vx
              );

    const double ay_for_allocation_mps2 =
        std::isfinite(current_state_.acc_y)
            ? current_state_.acc_y
            : 0.0;

    const dv_control_common::RelaxedAcceleration relaxed_acceleration =
        load_transfer_relaxation_.update(
            ax_for_allocation_mps2,
            ay_for_allocation_mps2,
            getControlDt(),
            param_.get("model.mass_transfer.tau_load_s")
        );

    TorqueAllocationInput allocation_input;

    allocation_input.total_torque_cmd_Nm =
        total_torque_cmd_Nm;

    allocation_input.desired_yaw_moment_Nm =
        tv_yaw_moment_Nm_;

    allocation_input.vx_mps =
        current_state_.vx;

    allocation_input.vy_mps =
        current_state_.vy;

    allocation_input.yaw_rate_radps =
        current_state_.r;

    allocation_input.ax_mps2 =
        relaxed_acceleration.ax_mps2;

    allocation_input.ay_mps2 =
        relaxed_acceleration.ay_mps2;

    allocation_input.delta_bicycle_rad =
        current_state_.delta_vehicle_used;

    allocation_input.previous_total_limited_torque_Nm =
        last_allocator_total_limited_torque_Nm;

    allocation_input.previous_total_limited_torque_valid =
        last_allocator_total_limited_torque_valid;

    allocation_input.bypass_torque_rate_limiter =
        finished ||
        phase_ == DrivePhase::WAITING ||
        phase_ == DrivePhase::EMERGENCY_STOP_HARD ||
        phase_ == DrivePhase::FINISHED;

    const TorqueAllocationResult allocation_result =
        allocateWheelTorquesForDvControl(
            param_,
            allocation_input
        );

    last_torque_allocator_global_scale_debug =
        allocation_result.global_scale;

    last_torque_allocator_hard_limit_scale_debug =
        allocation_result.hard_limit_scale;

    last_torque_allocator_rate_scale_debug =
        allocation_result.total_torque_rate_limited_scale;

    last_torque_allocator_rate_scale_min_debug =
        allocation_result.total_torque_rate_scale_min;

    last_torque_allocator_rate_scale_max_debug =
        allocation_result.total_torque_rate_scale_max;

    last_torque_allocator_total_after_tv_Nm_debug =
        allocation_result.total_torque_after_tv_Nm;

    last_torque_allocator_total_limited_Nm_debug =
        allocation_result.total_torque_limited_Nm;

    last_torque_allocator_previous_total_limited_Nm_debug =
        allocation_result.previous_total_limited_torque_Nm;

    last_torque_allocator_rate_limited_debug =
        allocation_result.torque_rate_limited;

    last_torque_allocator_rate_forced_over_hard_limit_debug =
        allocation_result.torque_rate_forced_over_hard_limit;

    last_torque_allocator_rate_bypass_debug =
        allocation_result.torque_rate_limiter_bypassed;

    last_allocator_total_limited_torque_Nm =
        allocation_result.total_torque_limited_Nm;

    last_allocator_total_limited_torque_valid =
        true;

    msg.torque_FL =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation_result.torque_limited_Nm.FL
        );

    msg.torque_FR =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation_result.torque_limited_Nm.FR
        );

    msg.torque_RL =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation_result.torque_limited_Nm.RL
        );

    msg.torque_RR =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation_result.torque_limited_Nm.RR
        );

    if (printConsoleDebugInfo())
    {
        ROS_INFO_STREAM_THROTTLE(
            0.2,
            "[SKIDPAD_TORQUE_ALLOC_DEBUG] "
            << "mode=" << torqueAllocationModeToString(allocation_result.mode)
    
            << " | total_req_Nm=" << total_torque_cmd_Nm
            << " | total_after_tv_Nm=" << allocation_result.total_torque_after_tv_Nm
            << " | total_limited_Nm=" << allocation_result.total_torque_limited_Nm
            << " | prev_total_limited_Nm=" << allocation_result.previous_total_limited_torque_Nm
    
            << " | global_scale=" << allocation_result.global_scale
            << " | hard_scale=" << allocation_result.hard_limit_scale
            << " | rate_scale=" << allocation_result.total_torque_rate_limited_scale
            << " | rate_scale_min=" << allocation_result.total_torque_rate_scale_min
            << " | rate_scale_max=" << allocation_result.total_torque_rate_scale_max
    
            << " | rate_limited=" << allocation_result.torque_rate_limited
            << " | rate_forced_over_hard=" << allocation_result.torque_rate_forced_over_hard_limit
            << " | rate_bypass=" << allocation_result.torque_rate_limiter_bypassed
            << " | Mdot_up=" << allocation_result.normalized_M_dot_up
            << " | Mdot_down=" << allocation_result.normalized_M_dot_down
    
            << " | wheel_Nm=["
            << allocation_result.torque_limited_Nm.FL << ", "
            << allocation_result.torque_limited_Nm.FR << ", "
            << allocation_result.torque_limited_Nm.RL << ", "
            << allocation_result.torque_limited_Nm.RR << "]"
    
            << " | interface=["
            << msg.torque_FL << ", "
            << msg.torque_FR << ", "
            << msg.torque_RL << ", "
            << msg.torque_RR << "]"
        );
    }

    pub_control_.publish(msg);
}


void Controller::publishZeroControl(bool finished)
{
    base_torque_cmd_Nm_ = 0.0;
    tv_yaw_moment_Nm_ = 0.0;
    tv_yaw_moment_raw_Nm_ = 0.0;
    tv_torque_diff_Nm_ = 0.0;

    publishControl(0.0, 0.0, finished);
}


void Controller::publishZeroMovement(double steering_rad, bool finished)
{
    base_torque_cmd_Nm_ = 0.0;
    tv_yaw_moment_Nm_ = 0.0;
    tv_yaw_moment_raw_Nm_ = 0.0;
    tv_torque_diff_Nm_ = 0.0;

    publishControl(steering_rad, 0.0, finished);
}


void Controller::publishZeroSteering(double total_torque_cmd_Nm, bool finished)
{
    publishControl(0.0, total_torque_cmd_Nm, finished);
}


void Controller::publishAllDebug()
{
    publishLogInfoLongitudal();
    publishLogInfoLateral();
    publishGlobalHandlingInfo();
}


void Controller::publishEmergencyCheckDebug()
{
    dv_interfaces::EmergencyInfo msg;

    msg.header.stamp =
        ros::Time::now();

    msg.header.frame_id =
        "base_link";

    msg.competition_type =
        dv_interfaces::EmergencyInfo::COMPETITION_SKIDPAD;

    msg.emergency_current =
        emergency_check_current_ ||
        hard_emergency_check_current_;

    /*
        The existing EmergencyInfo message has one latched field.
        In the hard/non-hard split, this latched field represents the permanent
        HARD emergency latch. Normal emergency is published only as current.
    */
    msg.emergency_latched =
        hard_emergency_check_latched_;

    /*
        Skidpad has a recoverable normal emergency and a permanent hard
        emergency.  Report is_hard only for the hard path.
    */
    msg.is_hard =
        hard_emergency_check_current_ ||
        hard_emergency_check_latched_ ||
        phase_ == DrivePhase::EMERGENCY_STOP_HARD;

    msg.emergency_reason_current =
        hard_emergency_check_current_
            ? hard_emergency_check_current_reason_
            : emergency_check_current_reason_;

    msg.emergency_reason_latched =
        hard_emergency_check_latched_reason_;

    msg.is_car_driving =
        phase_ == DrivePhase::DRIVE ||
        phase_ == DrivePhase::EMERGENCY_STOP;

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

    if (cm.encoder_stopped_sending)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_CUBEMARS_ENCODER_TIMEOUT;
    }

    if (cm.encoder_reference_tracking_error_too_high)
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
        ds.ey_over_1_5m;

    msg.dynamic_epsi_over_limit =
        ds.epsi_over_75_deg;

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

    if (ds.ey_over_1_5m)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_EY;
    }

    if (ds.epsi_over_75_deg)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_EPSI;
    }

    if (ds.beta_angle_over_20_deg)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_DYNAMIC_BETA;
    }

    if (ds.yaw_rate_over_2_5_rad_per_sec)
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

    if (ins.ins_stopped_sending)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_TIMEOUT;
    }

    if (!ins.ins_pose_is_changing)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_POSE_NOT_CHANGING;
    }

    if (!ins.ins_pose_is_stable)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_POSE_UNSTABLE;
    }

    if (!ins.ins_velocity_is_stable)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_VELOCITY_UNSTABLE;
    }

    if (ins.ins_velocity_sliding)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_INS_SLIDING;
    }

    const auto& pp =
        emergency_checker_.path_planner_check;

    msg.use_path_planner_check =
        param_.getBool("general.use_path_planner_check");

    msg.path_planner_emergency =
        pp.is_emergency;

    msg.path_planner_initialized =
        pp.is_intialized;

    msg.path_is_track_closed =
        pp.is_track_closed;

    msg.path_spline_is_valid =
        pp.is_spline_valid;

    msg.path_planner_is_valid =
        pp.path_planner_is_valid;

    msg.path_planner_stopped_sending =
        pp.path_planner_stopped_sending;

    msg.path_bolide_before_first_point =
        pp.bolide_before_first_point;

    msg.path_bolide_after_last_point =
        pp.bolide_after_last_point;

    msg.path_bolide_in_break_between_last_and_first =
        pp.bolide_in_break_between_first_and_last;

    msg.path_safe_because_track_closed =
        pp.PathIsAlwaysSafeBecauseTrackClosed();

    msg.path_time_since_last_update_s =
        pp.time_since_last_path_planner_update;

    msg.path_size =
        static_cast<uint32_t>(emergency_check_input_.path_xy.size());

    msg.path_bolide_x_m =
        emergency_check_input_.bolide_xy.x();

    msg.path_bolide_y_m =
        emergency_check_input_.bolide_xy.y();

    msg.path_no_message_threshold_s =
        pp.PATH_PLANNER_NO_NEW_MESSAGE_THRESHOLD;

    msg.path_max_dist_to_break_m =
        pp.PATH_PLANNER_MAX_DIST_TO_BREAK_M;

    if (!pp.is_spline_valid)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_PATH_SPLINE_INVALID;
    }

    if (pp.path_planner_stopped_sending)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_PATH_TIMEOUT;
    }

    if (!pp.path_planner_is_valid)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_PATH_TOO_SHORT;
    }

    if (pp.bolide_before_first_point)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_PATH_BEFORE_FIRST;
    }

    if (pp.bolide_after_last_point)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_PATH_AFTER_LAST;
    }

    if (pp.bolide_in_break_between_first_and_last)
    {
        msg.error_mask |=
            dv_interfaces::EmergencyInfo::ERROR_PATH_IN_BREAK;
    }

    pub_emergency_check_.publish(msg);
}


// =============================================================================
//                              DEBUG PUBLISHERS
// =============================================================================

void Controller::publishLogInfoLongitudal()
{
    dv_interfaces::SkidpadDebug_long msg;

    msg.v_curr_mps =
        current_state_.vx;

    msg.v_ref_mps =
        current_state_.v_ref;

    msg.v_error_mps =
        msg.v_ref_mps - msg.v_curr_mps;

    /*
        Internal torque is wheel-side Nm. This is the base longitudinal request
        before allocation / TV / final limiter.
    */
    msg.torque_cmd_Nm =
        base_torque_cmd_Nm_;

    /*
        Aggregate vehicle-interface command after final conversion.
    */
    msg.throttle_cmd_percent =
        control_output_.movement_percent;

    msg.phase =
        phaseToString(phase_);

    pub_acc_debug_long_.publish(msg);
}


void Controller::publishLogInfoLateral()
{
    dv_interfaces::SkidpadDebug_lat msg;

    msg.ey_m =
        current_state_.ey;

    msg.epsi_rad =
        current_state_.epsi;

    msg.curr_steer_rad =
        current_state_.delta_vehicle_used;

    msg.ref_steer_rad =
        lateral_controller_result_.delta_act_next;

    msg.steer_error_rad =
        msg.ref_steer_rad - msg.curr_steer_rad;

    msg.curr_yaw_rate_radps =
        current_state_.r;

    msg.ref_yaw_rate_radps =
        lateral_controller_result_.r_next;

    msg.yaw_rate_error_radps =
        msg.ref_yaw_rate_radps - msg.curr_yaw_rate_radps;

    msg.curr_vy_mps =
        current_state_.vy;

    msg.ref_vy_mps =
        lateral_controller_result_.vy_next;

    msg.vy_error_mps =
        msg.ref_vy_mps - msg.curr_vy_mps;

    /*
        4-wheel allocator / rate-limiter debug.
        These fields require the matching additions in SkidpadDebug_lat.msg.
    */
    msg.torque_allocator_global_scale =
        last_torque_allocator_global_scale_debug;

    msg.torque_allocator_hard_limit_scale =
        last_torque_allocator_hard_limit_scale_debug;

    msg.torque_allocator_rate_scale =
        last_torque_allocator_rate_scale_debug;

    msg.torque_allocator_rate_scale_min =
        last_torque_allocator_rate_scale_min_debug;

    msg.torque_allocator_rate_scale_max =
        last_torque_allocator_rate_scale_max_debug;

    msg.torque_allocator_total_after_tv_Nm =
        last_torque_allocator_total_after_tv_Nm_debug;

    msg.torque_allocator_total_limited_Nm =
        last_torque_allocator_total_limited_Nm_debug;

    msg.torque_allocator_previous_total_limited_Nm =
        last_torque_allocator_previous_total_limited_Nm_debug;

    msg.torque_allocator_rate_limited =
        last_torque_allocator_rate_limited_debug;

    msg.torque_allocator_rate_forced_over_hard_limit =
        last_torque_allocator_rate_forced_over_hard_limit_debug;

    msg.torque_allocator_rate_bypass =
        last_torque_allocator_rate_bypass_debug;

    /*
        Torque-vectoring debug.
    */
    msg.optimized_torque_vectoring_used =
        optimized_torque_vectoring_used_;

    msg.jaca_torque_vectoring_used =
        jaca_torque_vectoring_used_;

    msg.tv_yaw_moment_Nm =
        tv_yaw_moment_Nm_;

    msg.tv_yaw_moment_raw_Nm =
        tv_yaw_moment_raw_Nm_;

    msg.tv_torque_diff_Nm =
        tv_torque_diff_Nm_;

    pub_acc_debug_lat_.publish(msg);
}


void Controller::publishGlobalHandlingInfo()
{
    dv_interfaces::Skidpad_handling_info msg;

    msg.cube_mars_initialization_finished =
        global_handling_info_.cube_mars_initialization_finished;

    msg.has_received_first_dv_board_message =
        global_handling_info_.has_received_first_dv_board_message;

    msg.has_valid_path_from_pp =
        global_handling_info_.has_valid_path_from_pp;

    msg.ready_to_start_drive =
        global_handling_info_.ready_to_start_drive;

    msg.has_odometry_message =
        global_handling_info_.has_odometry_message;

    msg.has_received_imu_message =
        global_handling_info_.has_received_imu_message;

    /*
        Current drive phase, exported directly from the state machine.

        phase:
            human-readable name, same convention as SkidpadDebug_long.

        phase_id:
            numeric value of DrivePhase enum:
                WAITING             = 0
                DRIVE               = 1
                BRAKING             = 2
                COASTING            = 3
                FINISHED            = 4
                EMERGENCY_STOP      = 5
                EMERGENCY_STOP_HARD = 6
    */
    msg.phase =
        phaseToString(phase_);

    // msg.phase =
    //     static_cast<int>(phase_);

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
    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "skidpad_ref_path";

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
    if (!has_path_projection_)
    {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "skidpad_ref_point";

    marker.id =
        0;

    marker.type =
        visualization_msgs::Marker::SPHERE;

    marker.action =
        visualization_msgs::Marker::ADD;

    /*
        Ref point is the final projection point.
        With valid spline it is the locally refined spline point; otherwise it
        falls back to the RAW monotonic polyline projection point.
    */
    marker.pose.position.x =
        proj_.x_ref_m;

    marker.pose.position.y =
        proj_.y_ref_m;

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




} // namespace skidpad_control
