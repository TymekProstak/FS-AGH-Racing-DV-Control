#include "wrapper.hpp"
#include "4wheel_utilites.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace dv_control
{

namespace
{


bool g_print_console_debug_info =
    true;

inline bool printConsoleDebugInfo()
{
    return g_print_console_debug_info;
}

constexpr double kPathPointChangeToleranceM =
    1.0e-6;

constexpr double kFinalBrakeTargetAccelerationMps2 =
    -4.5;

constexpr double kSafeRollingTargetSpeedMps =
    3.0;

bool incomingPathMatchesCachedGeometry(
    const std::vector<geometry_msgs::Pose>& path,
    bool is_closed,
    const Eigen::VectorXd& X_cached,
    const Eigen::VectorXd& Y_cached,
    bool cached_is_closed,
    bool cached_spline_valid)
{
    if (!cached_spline_valid ||
        is_closed != cached_is_closed ||
        X_cached.size() != Y_cached.size() ||
        static_cast<std::size_t>(X_cached.size()) != path.size())
    {
        return false;
    }

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        const double x =
            path[i].position.x;

        const double y =
            path[i].position.y;

        if (!std::isfinite(x) ||
            !std::isfinite(y))
        {
            return false;
        }

        const int index =
            static_cast<int>(i);

        if (std::abs(x - X_cached(index)) > kPathPointChangeToleranceM ||
            std::abs(y - Y_cached(index)) > kPathPointChangeToleranceM)
        {
            return false;
        }
    }

    return true;
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
        getTireRadiusM(param);

    const double I_wheel =
        param.get("model.body.wheel_rot_inertia_kgm2");

    const double num_wheels =
        param.get("model.body.num_rotating_wheels");

    if (!std::isfinite(m) || m <= 1.0e-9)
    {
        throw std::runtime_error(
            "[dv_control] model.body.m must be finite and > 0"
        );
    }

    if (!std::isfinite(R_tire) || R_tire <= 1.0e-9)
    {
        throw std::runtime_error(
            "[dv_control] model.tire.R_tire must be finite and > 0"
        );
    }

    if (!std::isfinite(I_wheel) || I_wheel < 0.0)
    {
        throw std::runtime_error(
            "[dv_control] model.body.wheel_rot_inertia_kgm2 "
            "must be finite and >= 0"
        );
    }

    if (!std::isfinite(num_wheels) || num_wheels < 0.0)
    {
        throw std::runtime_error(
            "[dv_control] model.body.num_rotating_wheels "
            "must be finite and >= 0"
        );
    }

    /*
        Equivalent translational mass including wheel rotational inertia:

            m_eq = m + n * I_wheel / R_tire^2

        Feedforward relation:

            T_total =
                m_eq * ax * R_tire
                + T_resistance

        Therefore:

            ax =
                (T_total - T_resistance)
                / (m_eq * R_tire)
    */
    const double equivalent_mass =
        m
        + num_wheels * I_wheel
            / (R_tire * R_tire);

    const double resistance_torque_Nm =
        computeResistanceFeedforwardTorqueNm(
            param,
            vx_mps
        );

    const double acceleration_torque_Nm =
        torque_cmd_Nm - resistance_torque_Nm;

    double ax_mps2 =
        acceleration_torque_Nm
        / (equivalent_mass * R_tire);

    if (!std::isfinite(ax_mps2))
    {
        ax_mps2 = 0.0;
    }

    const double ax_limit =
        std::abs(param.get("general.ax_max"));

    if (std::isfinite(ax_limit) && ax_limit > 0.0)
    {
        ax_mps2 =
            clampLocal(
                ax_mps2,
                -ax_limit,
                ax_limit
            );
    }

    return ax_mps2;
}


// =============================================================================
//                  LATERAL-ERROR SLOWING-DOWN HEURISTIC
// =============================================================================

struct SlowingDownHeuristicResult
{
    bool enabled = false;
    bool applied = false;

    double input_total_torque_Nm = 0.0;
    double output_total_torque_Nm = 0.0;

    double requested_ax_mps2 = 0.0;
    double output_ax_mps2 = 0.0;

    double drive_scale = 1.0;
    double brake_activation = 0.0;
    double recovery_ax_mps2 = 0.0;
};


inline double slowingDownSmoothstep01(
    const double value)
{
    const double u =
        std::max(
            0.0,
            std::min(
                1.0,
                value
            )
        );

    return
        u * u * (3.0 - 2.0 * u);
}


SlowingDownHeuristicResult applySlowingDownHeuristic(
    const ParamBank& param,
    const double requested_total_torque_Nm,
    const double ey_m,
    const double vx_mps)
{
    SlowingDownHeuristicResult result;

    result.input_total_torque_Nm =
        std::isfinite(requested_total_torque_Nm)
            ? requested_total_torque_Nm
            : 0.0;

    result.output_total_torque_Nm =
        result.input_total_torque_Nm;

    result.enabled =
        param.getBool(
            "general.use_slowing_down_heuristic"
        );

    if (!result.enabled)
    {
        return result;
    }

    if (!std::isfinite(ey_m) ||
        !std::isfinite(vx_mps))
    {
        return result;
    }

    const double ey_drive_start_m =
        param.get(
            "slowing_down_heuristic.ey_drive_start_m"
        );

    const double ey_drive_zero_m =
        param.get(
            "slowing_down_heuristic.ey_drive_zero_m"
        );

    const double ey_brake_start_m =
        param.get(
            "slowing_down_heuristic.ey_brake_start_m"
        );

    const double ey_brake_full_m =
        param.get(
            "slowing_down_heuristic.ey_brake_full_m"
        );

    const double max_recovery_deceleration_mps2 =
        param.get(
            "slowing_down_heuristic."
            "max_recovery_deceleration_mps2"
        );

    const bool parameters_valid =
        std::isfinite(ey_drive_start_m) &&
        std::isfinite(ey_drive_zero_m) &&
        std::isfinite(ey_brake_start_m) &&
        std::isfinite(ey_brake_full_m) &&
        std::isfinite(max_recovery_deceleration_mps2) &&

        ey_drive_start_m >= 0.0 &&
        ey_drive_zero_m > ey_drive_start_m &&

        ey_brake_start_m >= 0.0 &&
        ey_brake_full_m > ey_brake_start_m &&

        max_recovery_deceleration_mps2 >= 0.0;

    if (!parameters_valid)
    {
        throw std::runtime_error(
            "[dv_control] Invalid slowing_down_heuristic parameters"
        );
    }

    const double abs_ey_m =
        std::abs(ey_m);

    /*
        Positive acceleration reduction:

            |ey| <= ey_drive_start_m:
                preserve 100% of requested positive ax

            |ey| >= ey_drive_zero_m:
                preserve 0% of requested positive ax

        Between the thresholds the scale follows a C1-continuous smoothstep.
    */
    const double drive_reduction =
        slowingDownSmoothstep01(
            (
                abs_ey_m
                - ey_drive_start_m
            )
            / (
                ey_drive_zero_m
                - ey_drive_start_m
            )
        );

    result.drive_scale =
        1.0 - drive_reduction;

    /*
        Recovery deceleration:

            |ey| <= ey_brake_start_m:
                0 m/s^2

            |ey| >= ey_brake_full_m:
                -max_recovery_deceleration_mps2
    */
    result.brake_activation =
        slowingDownSmoothstep01(
            (
                abs_ey_m
                - ey_brake_start_m
            )
            / (
                ey_brake_full_m
                - ey_brake_start_m
            )
        );

    result.recovery_ax_mps2 =
        -max_recovery_deceleration_mps2
        * result.brake_activation;

    result.requested_ax_mps2 =
        estimateLongitudinalAccelerationFromTorqueAndResistance(
            param,
            result.input_total_torque_Nm,
            vx_mps
        );

    if (result.requested_ax_mps2 >= 0.0)
    {
        /*
            For a positive request:
                1) fade the positive acceleration with |ey|,
                2) add the smooth negative recovery target.

            The result cannot become more negative than the configured
            recovery deceleration because drive_scale is non-negative.
        */
        result.output_ax_mps2 =
            result.requested_ax_mps2
            * result.drive_scale
            + result.recovery_ax_mps2;
    }
    else
    {
        /*
            The longitudinal controller already requests braking.

            Do not sum two negative accelerations. Preserve whichever target
            is more negative:
                - the controller request,
                - the recovery heuristic.
        */
        result.output_ax_mps2 =
            std::min(
                result.requested_ax_mps2,
                result.recovery_ax_mps2
            );
    }

    result.output_total_torque_Nm =
        computeAccelerationFeedforwardTorqueNm(
            param,
            result.output_ax_mps2,
            vx_mps
        );

    if (!std::isfinite(result.output_total_torque_Nm))
    {
        result.output_total_torque_Nm =
            result.input_total_torque_Nm;

        result.output_ax_mps2 =
            result.requested_ax_mps2;

        return result;
    }

    result.applied =
        std::abs(
            result.output_total_torque_Nm
            - result.input_total_torque_Nm
        ) > 1.0e-9;

    return result;
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
            "[dv_control] model.drivetrain.gear_ratio must be finite and non-zero"
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
        The vehicle interface fields torqueFL/FR/RL/RR must receive the
        corresponding motor-side command:

            interface_torque = wheel_torque_Nm / gear_ratio

        Do not use this conversion anywhere inside the controller logic.
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





std::vector<Vec2> makeSplinePoints(
    const Eigen::VectorXd& X,
    const Eigen::VectorXd& Y)
{
    std::vector<Vec2> points;

    if (X.size() != Y.size())
    {
        return points;
    }

    points.reserve(static_cast<std::size_t>(X.size()));

    for (int i = 0; i < X.size(); ++i)
    {
        points.emplace_back(
            static_cast<float>(X(i)),
            static_cast<float>(Y(i))
        );
    }

    return points;
}

bool finiteStdVector(const std::vector<double>& values)
{
    return std::all_of(
        values.begin(),
        values.end(),
        [](double value)
        {
            return std::isfinite(value);
        }
    );
}

bool localPlannerReferenceUsable(
    const LocalPlannerResult& ref,
    int required_horizon)
{
    if (!ref.valid || required_horizon <= 0)
    {
        return false;
    }

    if (!std::isfinite(ref.ey) ||
        !std::isfinite(ref.epsi) ||
        !std::isfinite(ref.s) ||
        !std::isfinite(ref.x_ref_point) ||
        !std::isfinite(ref.y_ref_point))
    {
        return false;
    }

    if (ref.X_ref.size() < required_horizon ||
        ref.Y_ref.size() < required_horizon ||
        static_cast<int>(ref.curvature_ref.size()) < required_horizon ||
        static_cast<int>(ref.velocity_ref.size()) < required_horizon ||
        static_cast<int>(ref.acceleration_ref.size()) < required_horizon)
    {
        return false;
    }

    return
        ref.X_ref.allFinite() &&
        ref.Y_ref.allFinite() &&
        finiteStdVector(ref.curvature_ref) &&
        finiteStdVector(ref.velocity_ref) &&
        finiteStdVector(ref.acceleration_ref);
}


/*
    Emergency fallback used only when generation of a fresh reference fails.

    The old reference horizon remains geometrically usable, but its stored
    ey/epsi correspond to an older vehicle pose. Recompute those two errors by
    projecting the current control point onto the closest FINITE segment of
    that same old reference horizon used by the lateral controller.

    This is intentionally a simple polyline projection. It does not extrapolate
    any segment to infinity and it does not modify the old velocity/curvature
    profiles. It only refreshes:
        - ey,
        - epsi,
        - x_ref_point / y_ref_point.
*/
bool refreshFallbackErrorsFromReferenceSegments(
    LocalPlannerResult& ref,
    const State& projection_state,
    State& control_state)
{
    if (!ref.valid ||
        ref.X_ref.size() != ref.Y_ref.size() ||
        ref.X_ref.size() < 2 ||
        !ref.X_ref.allFinite() ||
        !ref.Y_ref.allFinite() ||
        !std::isfinite(projection_state.x) ||
        !std::isfinite(projection_state.y) ||
        !std::isfinite(projection_state.yaw))
    {
        return false;
    }

    double best_distance_squared =
        std::numeric_limits<double>::infinity();

    double best_projection_x = 0.0;
    double best_projection_y = 0.0;
    double best_yaw_ref = 0.0;
    double best_ey = 0.0;

    bool found_valid_segment =
        false;

    for (Eigen::Index i = 0;
         i + 1 < ref.X_ref.size();
         ++i)
    {
        const double ax =
            ref.X_ref(i);

        const double ay =
            ref.Y_ref(i);

        const double bx =
            ref.X_ref(i + 1);

        const double by =
            ref.Y_ref(i + 1);

        const double abx =
            bx - ax;

        const double aby =
            by - ay;

        const double length_squared =
            abx * abx + aby * aby;

        if (!std::isfinite(length_squared) ||
            length_squared <= 1.0e-12)
        {
            continue;
        }

        const double apx =
            projection_state.x - ax;

        const double apy =
            projection_state.y - ay;

        const double segment_alpha =
            std::clamp(
                (apx * abx + apy * aby) /
                    length_squared,
                0.0,
                1.0
            );

        const double projection_x =
            ax + segment_alpha * abx;

        const double projection_y =
            ay + segment_alpha * aby;

        const double error_x =
            projection_state.x - projection_x;

        const double error_y =
            projection_state.y - projection_y;

        const double distance_squared =
            error_x * error_x + error_y * error_y;

        if (!std::isfinite(distance_squared) ||
            distance_squared >= best_distance_squared)
        {
            continue;
        }

        const double segment_length =
            std::sqrt(length_squared);

        const double tangent_x =
            abx / segment_length;

        const double tangent_y =
            aby / segment_length;

        const double yaw_ref =
            std::atan2(tangent_y, tangent_x);

        const double ey =
            -tangent_y * error_x +
             tangent_x * error_y;

        if (!std::isfinite(yaw_ref) ||
            !std::isfinite(ey))
        {
            continue;
        }

        best_distance_squared =
            distance_squared;

        best_projection_x =
            projection_x;

        best_projection_y =
            projection_y;

        best_yaw_ref =
            yaw_ref;

        best_ey =
            ey;

        found_valid_segment =
            true;
    }

    if (!found_valid_segment)
    {
        return false;
    }

    constexpr double kTwoPi =
        6.283185307179586476925286766559;

    const double refreshed_epsi =
        std::remainder(
            projection_state.yaw - best_yaw_ref,
            kTwoPi
        );

    if (!std::isfinite(refreshed_epsi))
    {
        return false;
    }

    ref.x_ref_point =
        best_projection_x;

    ref.y_ref_point =
        best_projection_y;

    ref.ey =
        best_ey;

    ref.epsi =
        refreshed_epsi;

    control_state.ey =
        best_ey;

    control_state.epsi =
        refreshed_epsi;

    return true;
}

double getAccelerationReferenceForControl(
    LongitudinalControllerMode /*mode*/,
    const LocalPlannerResult& ref_path)
{
    return ref_path.acceleration_ref.size() > 1
        ? ref_path.acceleration_ref[1]
        : 0.0;
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

    /*
        JACA formula convention.

        Keep the controller/allocation pipeline in wheel-side Nm, but compute
        the JACA proportional gain exactly from the configured JACA factors:

            p_tv = p_tv_gain * gear_ratio * jaca_magic_number

        This changes only the JACA yaw-rate-control formula. It does not change
        the selected torque-allocation mode.
    */
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

    /*
        Same low-speed boost as in the JACA MPC model.

        The MPC helper reads model.torque_vectoring_jaca.low_speed_gain.
        Your current JSON shown in chat does not contain that key, so I keep
        the old intended value directly here: below v_low the yaw-rate target
        gain goes towards 1.5 instead of 1.0.
    */
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

} // anonymous namespace


// =============================================================================
//                              CONTROLLER
// =============================================================================

Controller::Controller(ros::NodeHandle& nh, const ParamBank& param)
    : param_(param),
      mpc_unbounded_(param),
      pid_speed_hold_(makeSpeedPidParams(param))
{
    g_print_console_debug_info =
        param_.getBool("general.print_console_debug_info");

    lap_count_sub_ = nh.subscribe(
        "/dv_logger/lap_count",
        1,
        &Controller::lapCountCallback,
        this
    );

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

    odom_debug_sub_ = nh.subscribe(
        "/dv_odometry/odometry_debug",
        1,
        &Controller::odometryDebugCallback,
        this
    );

    dv_board_sub_ = nh.subscribe(
        "/dv_board/data",
        1,
        &Controller::dvBoardCallback,
        this
    );

    imu_sub_ = nh.subscribe(
        "/dv_board/imu",
        1,
        &Controller::imuCallback,
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

    pub_debug_long_ = nh.advertise<dv_interfaces::ControlDebug_long>(
        "/dv_control/control_debug_long",
        1
    );

    pub_debug_lat_ = nh.advertise<dv_interfaces::ControlDebug_lat>(
        "/dv_control/control_debug_lat",
        1
    );

    pub_global_handling_info_ =
        nh.advertise<dv_interfaces::Controlhandling_info>(
            "/dv_control/control_handling_info",
            1
        );

    pub_emergency_check_ =
        nh.advertise<dv_interfaces::EmergencyAutoX>(
            "/dv_control/emergency_info",
            1
        );

    pub_ref_path_ = nh.advertise<visualization_msgs::Marker>(
        "/dv_control/ref_path",
        1,
        true
    );

    pub_raw_path_from_pp_ = nh.advertise<visualization_msgs::Marker>(
        "/dv_control/raw_path_from_pp",
        1,
        true
    );

    pub_ref_point_ = nh.advertise<visualization_msgs::Marker>(
        "/dv_control/ref_point",
        1,
        true
    );

    lateral_controller_type_ =
        LateralControllerType::MPC_UNBOUNDED;

    longitudinal_controller_mode_ =
        LongitudinalControllerMode::VELOCITY_PROFILE;

    if (printConsoleDebugInfo())
    {
        ROS_INFO("[dv_control] Lateral: unbounded LTV MPC.");
        ROS_INFO("[dv_control] Longitudinal reference: BF profile.");
    }


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
        ROS_INFO("[dv_control] Emergency checker timer started at 100 Hz.");
    }

    if (printConsoleDebugInfo())
    {
        ROS_INFO("[dv_control] Controller initialized.");
    }
    publishZeroControl(false);
}

// =============================================================================
//                              STATE HELPERS
// =============================================================================


bool Controller::useJacaTorqueVectoring() const
{
    return param_.getBool("general.general_use_jaca_torque_vectoring");
}

bool Controller::useOptimizedTorqueVectoring() const
{
    /*
        I give JACA priority. If JACA is enabled, optimized Mz is not used,
        even if the optimized-TV flag is accidentally also true.
    */
    return param_.getBool("general.general_use_torque_vectoring") &&
           !useJacaTorqueVectoring();
}







double Controller::calculateJacaTvYawMomentNmForDebug() const
{
    const double yaw_moment_Nm =
        calculateJacaTorqueVectoringYawMomentTargetNm(
            param_,
            current_state_.vx,
            current_state_.r,
            current_state_.delta_vehicle_used
        );

    return std::isfinite(yaw_moment_Nm)
        ? yaw_moment_Nm
        : 0.0;
}

void Controller::updateTorqueVectoringCommandFromLateralResult()
{
    control_output_.tv_yaw_moment_raw_Nm = 0.0;
    control_output_.tv_yaw_moment_Nm = 0.0;

    lat_debug_.optimized_torque_vectoring_used = false;
    lat_debug_.jaca_torque_vectoring_used = false;

    lat_debug_.tv_yaw_moment_raw_Nm = 0.0;
    lat_debug_.tv_yaw_moment_Nm = 0.0;

    /*
        Always calculate JACA for comparison/debug, even when optimized TV or
        no TV is selected.
    */
    lat_debug_.jaca_tv_yaw_moment_Nm =
        calculateJacaTvYawMomentNmForDebug();

    if (useJacaTorqueVectoring())
    {
        control_output_.tv_yaw_moment_raw_Nm =
            lat_debug_.jaca_tv_yaw_moment_Nm;

        control_output_.tv_yaw_moment_Nm =
            control_output_.tv_yaw_moment_raw_Nm;

        lat_debug_.jaca_torque_vectoring_used =
            true;

        lat_debug_.tv_yaw_moment_raw_Nm =
            control_output_.tv_yaw_moment_raw_Nm;

        lat_debug_.tv_yaw_moment_Nm =
            control_output_.tv_yaw_moment_Nm;

        return;
    }

    if (!useOptimizedTorqueVectoring() ||
        !lateral_controller_result_.valid)
    {
        return;
    }

    const double optimized_yaw_moment_Nm =
        lateral_controller_result_.u_tv_yaw_moment_raw_Nm;

    if (!std::isfinite(optimized_yaw_moment_Nm))
    {
        return;
    }

    control_output_.tv_yaw_moment_raw_Nm =
        optimized_yaw_moment_Nm;

    control_output_.tv_yaw_moment_Nm =
        optimized_yaw_moment_Nm;

    lat_debug_.optimized_torque_vectoring_used =
        true;

    lat_debug_.tv_yaw_moment_raw_Nm =
        control_output_.tv_yaw_moment_raw_Nm;

    lat_debug_.tv_yaw_moment_Nm =
        control_output_.tv_yaw_moment_Nm;
}

void Controller::setBaseTorqueCommandNm(double total_torque_cmd_Nm)
{
    control_output_.base_torque_cmd_Nm =
        std::isfinite(total_torque_cmd_Nm)
            ? total_torque_cmd_Nm
            : 0.0;
}


double Controller::getControlDt() const
{
    const double hz =
        param_.get("model.frequency.steer_cmd_loop_hz");

    if (!std::isfinite(hz) || hz <= 0.0)
    {
        throw std::runtime_error(
            "[dv_control] model.frequency.steer_cmd_loop_hz must be finite and > 0"
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
        param_.get("model.steering_limit.max_steer");

    const double initial_delta =
        has_delta_encoder_
            ? clampLocal(
                  current_state_.delta_enc,
                  -max_steer,
                  max_steer
              )
            : 0.0;

    /*
        Fixed steering-state convention:

            delta:
                measured physical steering angle from the encoder,

            delta_dot:
                PT2 model steering-rate state propagated by the selected
                model-based controller,

            delta_cmd:
                absolute steering command obtained by integrating
                u_delta_cmd.

        PT2 is always used.
    */
    current_state_.delta_cmd =
        initial_delta;

    current_state_.delta_enc =
        initial_delta;

    current_state_.delta =
        initial_delta;

    current_state_.delta_vehicle_used =
        initial_delta;

    delta_dot_model_ =
        0.0;

    current_state_.delta_dot =
        delta_dot_model_;

    steering_state_initialized_ =
        true;
}


void Controller::updateSteeringStateForController()
{
    initializeSteeringStateIfNeeded();

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double max_steer_rate =
        param_.get("model.steering_limit.max_steer_rate");

    current_state_.delta_cmd =
        clampLocal(
            current_state_.delta_cmd,
            -max_steer,
            max_steer
        );

    /*
        Close the steering-angle state with the physical encoder every cycle.
        The encoder is intentionally not differentiated.
    */
    current_state_.delta_enc =
        has_delta_encoder_
            ? clampLocal(
                  current_state_.delta_enc,
                  -max_steer,
                  max_steer
              )
            : current_state_.delta_cmd;

    current_state_.delta =
        current_state_.delta_enc;

    /*
        Carry only the PT2 model rate between controller iterations.
    */
    current_state_.delta_dot =
        std::isfinite(delta_dot_model_)
            ? clampLocal(
                  delta_dot_model_,
                  -max_steer_rate,
                  max_steer_rate
              )
            : 0.0;

    /*
        Do not let the model claim that steering continues moving farther
        into a physical angle limit.
    */
    constexpr double kSteeringLimitEpsilonRad =
        1.0e-6;

    if ((current_state_.delta >=
             max_steer - kSteeringLimitEpsilonRad &&
         current_state_.delta_dot > 0.0) ||
        (current_state_.delta <=
             -max_steer + kSteeringLimitEpsilonRad &&
         current_state_.delta_dot < 0.0))
    {
        current_state_.delta_dot =
            0.0;

        delta_dot_model_ =
            0.0;
    }

    current_state_.delta_vehicle_used =
        current_state_.delta;
}


double Controller::integrateSteeringCommand(double u_delta_cmd)
{
    initializeSteeringStateIfNeeded();

    if (!std::isfinite(u_delta_cmd))
    {
        u_delta_cmd =
            0.0;
    }

    const double dt =
        std::max(
            getControlDt(),
            1.0e-9
        );

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double max_steer_rate =
        param_.get("model.steering_limit.max_steer_rate");

    const double limited_u_delta_cmd =
        clampLocal(
            u_delta_cmd,
            -max_steer_rate,
            max_steer_rate
        );

    current_state_.delta_cmd =
        clampLocal(
            current_state_.delta_cmd
                + limited_u_delta_cmd * dt,
            -max_steer,
            max_steer
        );

    if (lateral_controller_result_.valid &&
        std::isfinite(
            lateral_controller_result_.delta_dot_next
        ))
    {
        /*
            Use x_next(delta_dot) directly. Never reconstruct the rate by
            differentiating the encoder or delta_act_next.
        */
        delta_dot_model_ =
            clampLocal(
                lateral_controller_result_.delta_dot_next,
                -max_steer_rate,
                max_steer_rate
            );
    }
    else
    {
        delta_dot_model_ = 0.0;
    }

    updateSteeringStateForController();

    return current_state_.delta_cmd;
}


void Controller::updateReadyFlag()
{
    /*
        The lateral model always uses the physical encoder angle as delta.
        Therefore at least one valid finite encoder sample is mandatory before
        leaving WAITING.
    */
    global_handling_info_.ready_to_start_drive =
        global_handling_info_.has_valid_path_from_pp &&
        global_handling_info_.cube_mars_initialization_finished &&
        global_handling_info_.has_received_first_dv_board_message &&
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

    if (new_phase == DrivePhase::FINAL_BRAKE ||
        new_phase == DrivePhase::EMERGENCY_BRAKE)
    {
        resetSpeedPid();
    }

    phase_info_.phase =
        new_phase;
}

// =============================================================================
//                              PHASE LOGIC
// =============================================================================

void Controller::StateMachinePhaseTransitionLogic(
    bool recoverable_control_fault)
{
    updateReadyFlag();

    const double speed_abs =
        std::abs(current_state_.vx_enc);

    const bool speed_under_coast_threshold =
        speed_abs < param_.get("general.speed_under_coast_threshold");

    const bool speed_under_finished_threshold =
        speed_abs < param_.get("general.speed_under_finished_threshold");

    const bool final_brake_requested =
        global_handling_info_.final_brake_requested ||
        phase_info_.should_make_final_brake;

    if (global_handling_info_.as_finished ||
        phase_info_.phase == DrivePhase::FINISHED)
    {
        global_handling_info_.as_finished = true;
        setPhase(DrivePhase::FINISHED);
        return;
    }

    if (phase_info_.phase == DrivePhase::FINAL_BRAKE)
    {
        global_handling_info_.final_brake_requested = true;
        phase_info_.should_make_final_brake = true;

        if (speed_under_coast_threshold)
        {
            setPhase(DrivePhase::COASTING);
        }

        return;
    }

    if (phase_info_.phase == DrivePhase::COASTING)
    {
        global_handling_info_.final_brake_requested = true;
        phase_info_.should_make_final_brake = true;

        if (speed_under_finished_threshold)
        {
            global_handling_info_.as_finished = true;
            setPhase(DrivePhase::FINISHED);
        }

        return;
    }

    if (!global_handling_info_.ready_to_start_drive)
    {
        global_handling_info_.final_brake_requested = false;
        phase_info_.should_make_final_brake = false;

        setPhase(DrivePhase::WAITING);
        return;
    }

    if (final_brake_requested)
    {
        global_handling_info_.final_brake_requested = true;
        phase_info_.should_make_final_brake = true;

        setPhase(DrivePhase::FINAL_BRAKE);
        return;
    }

    const bool emergency_requested =
        recoverable_control_fault ||
        !emergency_check_is_safe_;

    if (emergency_requested)
    {
        setPhase(DrivePhase::EMERGENCY_BRAKE);
        return;
    }

    if (phase_info_.phase == DrivePhase::EMERGENCY_BRAKE ||
        phase_info_.phase == DrivePhase::WAITING)
    {
        setPhase(DrivePhase::DRIVE);
        return;
    }
}





// =============================================================================
//                              EMERGENCY CHECK TIMER
// =============================================================================

void Controller::emergencyCheckTimerCallback(
    const ros::TimerEvent& event)
{
    (void)event;

    const bool checker_active =
        phase_info_.phase == DrivePhase::DRIVE ||
        phase_info_.phase == DrivePhase::EMERGENCY_BRAKE;

    const bool new_encoder_message =
        emergency_check_input_.new_encoder_message;

    const bool new_ins_message =
        emergency_check_input_.new_ins_message;

    emergency_check_input_.new_encoder_message = false;
    emergency_check_input_.new_ins_message = false;

    const EmergencyCheckResult result =
        emergency_checker_.UpdateEmergencyCheck(
            checker_active,
            ros::Time::now().toSec(),

            new_encoder_message,
            emergency_check_input_.encoder_position_rad,
            emergency_check_input_.encoder_reference_position_rad,

            new_ins_message,
            emergency_check_input_.ins_xy,

            current_state_.vx_enc,

            Eigen::Vector2d(
                current_state_.x,
                current_state_.y
            ),

            emergency_check_input_.path_xy
        );

    emergency_check_is_safe_ =
        result.is_safe;

    emergency_check_reason_ =
        result.is_safe
            ? "OK"
            : result.reason;

    publishEmergencyCheckDebug();

    if (!emergency_check_is_safe_)
    {
        ROS_ERROR_STREAM_THROTTLE(
            0.2,
            "[dv_control][EmergencyCheck] "
            << emergency_check_reason_
        );
    }
}


// =============================================================================
//                              PATH CALLBACK
// =============================================================================

void Controller::pathCallback(const dv_interfaces::Path& msg)
{
    const auto& path =
        msg.path.poses;

    const bool is_closed =
        msg.is_track_closed;

    /*
        Rebuild smoothing and spline only when the actual path geometry or the
        open/closed flag changes. ROS timestamps do not matter here.
    */
    if (incomingPathMatchesCachedGeometry(
            path,
            is_closed,
            X_last_from_pp_,
            Y_last_from_pp_,
            closed_path_received_,
            path_spline_valid_))
    {
        path_update_fault_active_ =
            false;

        path_update_fault_reason_ =
            "OK";

        if (printConsoleDebugInfo())
        {
            ROS_INFO_STREAM_THROTTLE(
                2.0,
                "[Path Selection] Geometry unchanged. "
                "Reusing the full smoothed path and cached spline."
            );
        }

        return;
    }

    const auto rejectPathUpdate =
        [this](const std::string& reason)
        {
            path_update_fault_active_ =
                true;

            path_update_fault_reason_ =
                reason;

            /*
                Transactional path-update failure:
                    - previous raw path remains,
                    - previous smoothed path remains,
                    - previous valid spline remains,
                    - previous ref_path remains as the final fallback.

                If an older valid path exists, poseCallback continues to build
                a FRESH reference from that cached path. A rejected incoming
                path is diagnostic only and does NOT request EMERGENCY_BRAKE.

                If no valid path has ever been accepted, readiness remains
                false and the controller stays in WAITING.
            */
            if (!path_spline_valid_)
            {
                global_handling_info_.has_valid_path_from_pp =
                    false;
            }

            ROS_WARN_STREAM_THROTTLE(
                0.5,
                "[Path Selection] Rejected changed path: "
                << reason
                << ". Previous path/spline/reference are preserved; "
                   "no emergency is requested by this path-update failure."
            );
        };

    if (path.size() < 2 ||
        (is_closed && path.size() < 3))
    {
        rejectPathUpdate(
            is_closed
                ? "CLOSED_PATH_HAS_LESS_THAN_THREE_POINTS"
                : "OPEN_PATH_HAS_LESS_THAN_TWO_POINTS"
        );

        return;
    }

    Eigen::VectorXd X_candidate(
        static_cast<int>(path.size())
    );

    Eigen::VectorXd Y_candidate(
        static_cast<int>(path.size())
    );

    std::vector<Eigen::Vector2d> emergency_path_candidate;
    emergency_path_candidate.reserve(path.size());

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        const double x =
            path[i].position.x;

        const double y =
            path[i].position.y;

        if (!std::isfinite(x) ||
            !std::isfinite(y))
        {
            rejectPathUpdate(
                "PATH_CONTAINS_NONFINITE_POINT"
            );

            return;
        }

        const int index =
            static_cast<int>(i);

        X_candidate(index) =
            x;

        Y_candidate(index) =
            y;

        emergency_path_candidate.emplace_back(
            x,
            y
        );
    }

    SmoothedXY smoothed_candidate;
    TrackSpline2D spline_candidate;

    try
    {
        /*
            Open and closed paths both use the COMPLETE path. There is no
            moving window, no closest-index extraction and no short cache.
        */
        smoothed_candidate =
            smoothXY(
                X_candidate,
                Y_candidate,
                param_.get("general.smoothing_factor_of_pp_result")
            );

        const int minimum_smoothed_points =
            is_closed ? 3 : 2;

        if (smoothed_candidate.x.size() < minimum_smoothed_points ||
            smoothed_candidate.x.size() != smoothed_candidate.y.size() ||
            !smoothed_candidate.x.allFinite() ||
            !smoothed_candidate.y.allFinite())
        {
            rejectPathUpdate(
                "SMOOTHED_FULL_PATH_INVALID"
            );

            return;
        }

        spline_candidate.build(
            makeSplinePoints(
                smoothed_candidate.x,
                smoothed_candidate.y
            ),
            is_closed
        );
    }
    catch (const std::exception& exception)
    {
        rejectPathUpdate(
            std::string("PATH_PROCESSING_EXCEPTION: ")
            + exception.what()
        );

        return;
    }
    catch (...)
    {
        rejectPathUpdate(
            "PATH_PROCESSING_UNKNOWN_EXCEPTION"
        );

        return;
    }

    if (!spline_candidate.valid())
    {
        rejectPathUpdate(
            "SPLINE_BUILD_FAILED"
        );

        return;
    }

    /*
        Transactional commit. Nothing visible to the controller is replaced
        until raw-path validation, smoothing and spline construction all pass.
    */
    X_last_from_pp_ =
        std::move(X_candidate);

    Y_last_from_pp_ =
        std::move(Y_candidate);

    X_plan_ =
        std::move(smoothed_candidate.x);

    Y_plan_ =
        std::move(smoothed_candidate.y);

    path_spline_ =
        std::move(spline_candidate);

    path_spline_valid_ =
        true;

    closed_path_received_ =
        is_closed;

    emergency_check_input_.path_xy =
        std::move(emergency_path_candidate);

    global_handling_info_.has_valid_path_from_pp =
        true;

    path_update_fault_active_ =
        false;

    path_update_fault_reason_ =
        "OK";

    first_path_received_ =
        true;

    /*
        Intentionally preserve ref_path.

        Every pose callback projects globally onto the complete cached spline.
        If reference generation from the changed spline fails, poseCallback
        keeps the previous ref_path for lateral control and enters emergency.
    */

    if (printConsoleDebugInfo())
    {
        ROS_INFO_STREAM(
            "[Path Selection] Accepted changed full path and rebuilt spline."
            << " closed=" << closed_path_received_
            << " raw_size=" << X_last_from_pp_.size()
            << " smoothed_size=" << X_plan_.size()
            << " previous_ref_preserved=" << ref_path.valid
            << " spline_valid=" << path_spline_valid_
        );
    }
}

// =============================================================================
//                              MAIN CONTROL CALLBACK
// =============================================================================

void Controller::poseCallback(const nav_msgs::Odometry& msg)
{
    const double pose_callback_t0 =
        ros::WallTime::now().toSec();

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

    tf2::Matrix3x3(q).getRPY(
        roll,
        pitch,
        yaw
    );

    current_state_.yaw =
        yaw;

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

    emergency_check_input_.new_ins_message =
        true;

    updateReadyFlag();

    /*
        No projection or controller solve is performed until every readiness
        condition is true.
    */
    if (!global_handling_info_.ready_to_start_drive)
    {
        StateMachinePhaseTransitionLogic(false);

        if (printConsoleDebugInfo())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[dv_control] WAITING for a complete ready state."
            );
        }

        publishZeroControl(false);
        publishAllDebug();
        return;
    }

    const Eigen::VectorXd& X_path_raw =
        X_last_from_pp_;

    const Eigen::VectorXd& Y_path_raw =
        Y_last_from_pp_;

    const int planner_N =
        static_cast<int>(
            std::llround(
                param_.get("model.ltv_mpc_unbounded.N")
            )
        );

    const bool cached_path_runtime_valid =
        first_path_received_ &&
        global_handling_info_.has_valid_path_from_pp &&
        path_spline_valid_ &&
        X_last_from_pp_.size() >= 2 &&
        X_last_from_pp_.size() == Y_last_from_pp_.size() &&
        X_plan_.size() >= 2 &&
        X_plan_.size() == Y_plan_.size() &&
        path_spline_.isClosed() == closed_path_received_;

    /*
        A rejected new path update is a recoverable control fault, but it does
        NOT disable the planner. pathCallback() kept the previous valid path
        and spline transactionally, so the current pose is still projected on
        that last valid spline and a fresh reference is generated from it.
    */
    const bool path_update_fault_this_cycle =
        path_update_fault_active_;

    const std::string path_update_fault_reason_this_cycle =
        path_update_fault_active_
            ? path_update_fault_reason_
            : "OK";

    bool reference_fault_this_cycle =
        false;

    std::string reference_fault_reason =
        "OK";

    if (!cached_path_runtime_valid)
    {
        reference_fault_this_cycle =
            true;

        reference_fault_reason =
            "CACHED_FULL_PATH_OR_SPLINE_INVALID";
    }
    else if (planner_N <= 0)
    {
        reference_fault_this_cycle =
            true;

        reference_fault_reason =
            "PLANNER_HORIZON_INVALID";
    }

    // =========================================================================
    //                  GLOBAL PROJECTION / REFERENCE
    // =========================================================================

    const double projection_t0 =
        ros::WallTime::now().toSec();

    if (!reference_fault_this_cycle)
    {
        LocalPlannerResult candidate_ref_path;

        try
        {
            State projection_state =
                current_state_;

            candidate_ref_path =
                buildLocalPlannerReferenceFromSpline(
                    X_plan_,
                    Y_plan_,
                    projection_state,
                    planner_N,
                    param_,
                    closed_path_received_,
                    path_spline_,
                    allow_before_start_reference_
                );
        }
        catch (const std::exception& exception)
        {
            reference_fault_this_cycle =
                true;

            reference_fault_reason =
                std::string("LOCAL_PLANNER_EXCEPTION: ")
                + exception.what();
        }
        catch (...)
        {
            reference_fault_this_cycle =
                true;

            reference_fault_reason =
                "LOCAL_PLANNER_UNKNOWN_EXCEPTION";
        }

        if (!reference_fault_this_cycle)
        {
            if (localPlannerReferenceUsable(
                    candidate_ref_path,
                    planner_N))
            {
                /*
                    BEFORE_START is allowed only during the initial approach.

                    The first fully usable result with before_bolide == false
                    permanently closes that mode. Future planner calls receive
                    allow_before_start_reference_ == false and skip the
                    artificial BEFORE_START branch inside path processing.
                */
                if (allow_before_start_reference_ &&
                    !candidate_ref_path.before_bolide)
                {
                    allow_before_start_reference_ =
                        false;

                    if (printConsoleDebugInfo())
                    {
                        ROS_INFO(
                            "[dv_control][BEFORE_START_LOCKED_OUT] "
                            "A non-BEFORE_START reference was accepted. "
                            "Future BEFORE_START handling is disabled inside "
                            "path processing."
                        );
                    }
                }

                /*
                    Commit only a fully usable reference. A failed candidate
                    does not overwrite the previous reference.
                */
                ref_path =
                    std::move(candidate_ref_path);

                current_state_.ey =
                    ref_path.ey;

                current_state_.epsi =
                    ref_path.epsi;

                current_state_.s =
                    ref_path.s;
            }
            else
            {
                reference_fault_this_cycle =
                    true;

                reference_fault_reason =
                    "LOCAL_PLANNER_REFERENCE_INVALID_OR_NONFINITE";
            }
        }
    }

    bool fallback_reference_errors_refreshed =
        false;

    bool fallback_reference_error_refresh_failed =
        false;

    if (reference_fault_this_cycle &&
        localPlannerReferenceUsable(
            ref_path,
            planner_N
        ))
    {
        State fallback_projection_state =
            current_state_;

        fallback_reference_errors_refreshed =
            refreshFallbackErrorsFromReferenceSegments(
                ref_path,
                fallback_projection_state,
                current_state_
            );

        fallback_reference_error_refresh_failed =
            !fallback_reference_errors_refreshed;

        if (fallback_reference_errors_refreshed)
        {
            ROS_WARN_STREAM_THROTTLE(
                0.5,
                "[dv_control][REFERENCE_FALLBACK] "
                "Fresh reference generation failed. "
                "Recomputed ey/epsi from the closest segment of the old "
                "reference horizon."
                << " ey=" << current_state_.ey
                << " epsi=" << current_state_.epsi
            );
        }
        else
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                "[dv_control][REFERENCE_FALLBACK_INVALID] "
                "Fresh reference generation failed and ey/epsi could not "
                "be refreshed from the old reference horizon."
            );
        }
    }

    const bool two_point_reference_mode_active =
        ref_path.valid &&
        !closed_path_received_ &&
        X_last_from_pp_.size() == 2 &&
        Y_last_from_pp_.size() == 2;

    if (two_point_reference_mode_active)
    {
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[dv_control][TWO_POINT_MODE] "
            "Straight reference, safe-speed PID target="
            << kSafeRollingTargetSpeedMps
            << " m/s"
        );
    }

    if (ref_path.valid && ref_path.before_bolide)
    {
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[dv_control][BEFORE_START] "
            "Artificial start-line reference, safe-speed PID target="
            << kSafeRollingTargetSpeedMps
            << " m/s"
            << " | allow_before_start="
            << allow_before_start_reference_
        );
    }

    if (ref_path.valid && ref_path.after_bolide)
    {
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[dv_control][AFTER_END] "
            "Artificial endpoint continuation, safe-speed PID target="
            << kSafeRollingTargetSpeedMps
            << " m/s"
        );
    }

    /*
        A rejected path update still produces a fresh reference from the
        last valid cached spline. Only an actual reference-generation failure
        leaves ref_path unchanged as the final lateral fallback.
    */
    if (ref_path.valid)
    {
        publishReferencePath(
            ref_path.X_ref,
            ref_path.Y_ref
        );

        publishReferencePoint();
    }

    publishRawPathFromPP(
        X_path_raw,
        Y_path_raw
    );

    const double projection_ms =
        1000.0 * (
            ros::WallTime::now().toSec()
            - projection_t0
        );

    // =========================================================================
    //                              LATERAL CONTROL
    // =========================================================================

    const double control_t0 =
        ros::WallTime::now().toSec();

    /*
        A lateral failure must preserve the last command that was actually
        accepted for publication. Reference generation may still fail while
        lateral control continues on the previous ref_path.
    */
    const double previous_steer_cmd_rad =
        control_output_.steer_cmd_rad;

    updateSteeringStateForController();

    bool lateral_fault_this_cycle =
        false;

    std::string lateral_fault_reason =
        "OK";

    if (fallback_reference_error_refresh_failed)
    {
        lateral_fault_this_cycle =
            true;

        lateral_fault_reason =
            "FALLBACK_REFERENCE_ERROR_REFRESH_FAILED";
    }
    else if (!localPlannerReferenceUsable(
                 ref_path,
                 planner_N))
    {
        lateral_fault_this_cycle =
            true;

        lateral_fault_reason =
            "NO_USABLE_PREVIOUS_OR_CURRENT_REFERENCE_FOR_LATERAL";
    }
    else
    {
        Result candidate_lateral_result;
        bool lateral_controller_selected =
            true;

        try
        {
            if (lateral_controller_type_ ==
                LateralControllerType::MPC_UNBOUNDED)
            {
                candidate_lateral_result =
                    mpc_unbounded_.solve(
                        current_state_,
                        ref_path.curvature_ref,
                        ref_path.velocity_ref
                    );
            }
            else
            {
                lateral_controller_selected =
                    false;
            }
        }
        catch (const std::exception& exception)
        {
            lateral_fault_this_cycle =
                true;

            lateral_fault_reason =
                std::string("LATERAL_CONTROLLER_EXCEPTION: ")
                + exception.what();
        }
        catch (...)
        {
            lateral_fault_this_cycle =
                true;

            lateral_fault_reason =
                "LATERAL_CONTROLLER_UNKNOWN_EXCEPTION";
        }

        if (!lateral_fault_this_cycle &&
            !lateral_controller_selected)
        {
            lateral_fault_this_cycle =
                true;

            lateral_fault_reason =
                "NO_LATERAL_CONTROLLER_SELECTED";
        }
        else if (!lateral_fault_this_cycle &&
                 (!candidate_lateral_result.valid ||
                  !std::isfinite(
                      candidate_lateral_result.u_delta_cmd
                  )))
        {
            lateral_fault_this_cycle =
                true;

            lateral_fault_reason =
                "LATERAL_CONTROLLER_RESULT_INVALID_OR_NONFINITE";
        }
        else if (!lateral_fault_this_cycle)
        {
            lateral_controller_result_ =
                candidate_lateral_result;

            const double new_steer_cmd_rad =
                integrateSteeringCommand(
                    lateral_controller_result_.u_delta_cmd
                );

            if (!std::isfinite(new_steer_cmd_rad))
            {
                control_output_.steer_cmd_rad =
                    previous_steer_cmd_rad;

                lateral_fault_this_cycle =
                    true;

                lateral_fault_reason =
                    "LATERAL_STEERING_COMMAND_NONFINITE";
            }
            else
            {
                control_output_.steer_cmd_rad =
                    new_steer_cmd_rad;

                updateTorqueVectoringCommandFromLateralResult();

                lat_debug_.solver_failed =
                    false;
            }
        }
    }

    if (lateral_fault_this_cycle)
    {
        /*
            Lateral failure policy:
                - freeze the previous steering command,
                - keep the previous accepted lateral result/reference intact,
                - request EMERGENCY_BRAKE, which uses the safe-speed PID.
        */
        control_output_.steer_cmd_rad =
            previous_steer_cmd_rad;

        control_output_.tv_yaw_moment_raw_Nm =
            0.0;

        control_output_.tv_yaw_moment_Nm =
            0.0;

        lat_debug_.optimized_torque_vectoring_used =
            false;

        lat_debug_.jaca_torque_vectoring_used =
            false;

        lat_debug_.tv_yaw_moment_raw_Nm =
            0.0;

        lat_debug_.tv_yaw_moment_Nm =
            0.0;

        lat_debug_.solver_failed =
            true;

        lat_debug_.jaca_tv_yaw_moment_Nm =
            calculateJacaTvYawMomentNmForDebug();
    }

    /*
        A rejected incoming path update is NOT a control emergency. The cached
        valid path remains active and reference generation continues from it.

        Only failures of:
            - the reference generator, or
            - the lateral controller
        request EMERGENCY_BRAKE / safe-speed operation.
    */
    const bool recoverable_control_fault =
        reference_fault_this_cycle ||
        lateral_fault_this_cycle;

    const std::string control_fault_reason =
        reference_fault_this_cycle
            ? reference_fault_reason
            : (
                lateral_fault_this_cycle
                    ? lateral_fault_reason
                    : "OK"
              );

    /*
        State transition happens AFTER reference and lateral processing.

        Therefore, when reference generation fails but an older ref_path is
        still usable, lateral control is still evaluated normally from that
        old reference before the longitudinal side enters safe-speed mode.
    */
    StateMachinePhaseTransitionLogic(
        recoverable_control_fault
    );

    if (path_update_fault_this_cycle)
    {
        ROS_WARN_STREAM_THROTTLE(
            0.5,
            "[dv_control][PathUpdateIgnored] "
            << path_update_fault_reason_this_cycle
            << " | cached_path_kept="
            << cached_path_runtime_valid
            << " | emergency_requested=false"
        );
    }

    if (recoverable_control_fault)
    {
        ROS_ERROR_STREAM_THROTTLE(
            0.2,
            "[dv_control][RecoverableControlFault] "
            << control_fault_reason
            << " | previous_ref_used_for_lateral="
            << ref_path.valid
            << " | fallback_ey_epsi_refreshed="
            << fallback_reference_errors_refreshed
            << " | previous_steer_preserved_on_lateral_fault="
            << lateral_fault_this_cycle
            << " | longitudinal phase="
            << static_cast<int>(phase_info_.phase)
        );
    }

    emergency_check_input_.encoder_reference_position_rad =
        control_output_.steer_cmd_rad;

    const double control_ms =
        1000.0 * (
            ros::WallTime::now().toSec()
            - control_t0
        );

    const double pose_callback_ms =
        1000.0 * (
            ros::WallTime::now().toSec()
            - pose_callback_t0
        );

    static unsigned long long debug_samples =
        0ULL;

    static double sum_ey =
        0.0;

    static double sum_abs_ey =
        0.0;

    static double sum_epsi =
        0.0;

    static double sum_abs_epsi =
        0.0;

    static double sum_vx =
        0.0;

    static double sum_abs_vx =
        0.0;

    static double sum_projection_ms =
        0.0;

    static double sum_control_ms =
        0.0;

    static double sum_pose_callback_ms =
        0.0;

    static double max_projection_ms =
        0.0;

    static double max_control_ms =
        0.0;

    static double max_pose_callback_ms =
        0.0;

    debug_samples++;

    sum_ey +=
        current_state_.ey;

    sum_abs_ey +=
        std::abs(current_state_.ey);

    sum_epsi +=
        current_state_.epsi;

    sum_abs_epsi +=
        std::abs(current_state_.epsi);

    sum_vx +=
        current_state_.vx;

    sum_abs_vx +=
        std::abs(current_state_.vx);

    sum_projection_ms +=
        projection_ms;

    sum_control_ms +=
        control_ms;

    sum_pose_callback_ms +=
        pose_callback_ms;

    max_projection_ms =
        std::max(
            max_projection_ms,
            projection_ms
        );

    max_control_ms =
        std::max(
            max_control_ms,
            control_ms
        );

    max_pose_callback_ms =
        std::max(
            max_pose_callback_ms,
            pose_callback_ms
        );

    const double inv_debug_samples =
        1.0 / static_cast<double>(debug_samples);

    constexpr double rad_to_deg =
        57.29577951308232;

    if (printConsoleDebugInfo())
    {
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "[dv_control stats]"
            << " samples=" << debug_samples

            << " | mean_ey="
            << sum_ey * inv_debug_samples
            << " m"

            << " | mean_abs_ey="
            << sum_abs_ey * inv_debug_samples
            << " m"

            << " | mean_epsi="
            << sum_epsi * inv_debug_samples
            << " rad"

            << " | mean_abs_epsi="
            << sum_abs_epsi * inv_debug_samples
            << " rad"

            << " | mean_abs_epsi_deg="
            << sum_abs_epsi
                * inv_debug_samples
                * rad_to_deg
            << " deg"

            << " | vx_now="
            << current_state_.vx
            << " mps"

            << " | mean_vx="
            << sum_vx * inv_debug_samples
            << " mps"

            << " | mean_abs_vx="
            << sum_abs_vx * inv_debug_samples
            << " mps"

            << " | projection_now="
            << projection_ms
            << " ms"

            << " | mean_projection="
            << sum_projection_ms * inv_debug_samples
            << " ms"

            << " | max_projection="
            << max_projection_ms
            << " ms"

            << " | control_now="
            << control_ms
            << " ms"

            << " | mean_control="
            << sum_control_ms * inv_debug_samples
            << " ms"

            << " | max_control="
            << max_control_ms
            << " ms"

            << " | callback_now="
            << pose_callback_ms
            << " ms"

            << " | mean_callback="
            << sum_pose_callback_ms * inv_debug_samples
            << " ms"

            << " | max_callback="
            << max_pose_callback_ms
            << " ms"
        );
    }

    // =========================================================================
    //                           LONGITUDINAL CONTROL
    // =========================================================================

    const double longitudinal_control_t0 =
        ros::WallTime::now().toSec();

    const double dt =
        getControlDt();

    const bool phase_is_final_brake =
        phase_info_.phase ==
        DrivePhase::FINAL_BRAKE;

    const bool phase_is_emergency =
        phase_info_.phase ==
        DrivePhase::EMERGENCY_BRAKE;

    const bool force_before_path_safe_speed =
        ref_path.valid &&
        ref_path.before_bolide;

    const bool force_after_path_safe_speed =
        ref_path.valid &&
        ref_path.after_bolide;

    /*
        Exactly two accepted path-planning points define a straight-line mode.
        Lateral control still runs normally from ref_path (ey/epsi and zero
        curvature), but longitudinal control is always the speed PID at 3 m/s.
    */
    const bool force_two_point_path_speed_pid =
        ref_path.valid &&
        !closed_path_received_ &&
        X_last_from_pp_.size() == 2 &&
        Y_last_from_pp_.size() == 2;

    if (phase_is_final_brake)
    {
        /*
            FINAL_BRAKE is independent of the selected longitudinal mode.

            The requested total wheel-side torque corresponds to a fixed:
                ax_target = -4.5 m/s^2
        */
        const double torque_cmd_Nm =
            computeAccelerationFeedforwardTorqueNm(
                param_,
                kFinalBrakeTargetAccelerationMps2,
                current_state_.vx
            );

        long_debug_.torque_cmd_Nm =
            torque_cmd_Nm;

        setBaseTorqueCommandNm(
            torque_cmd_Nm
        );
    }
    else
    {
        const bool use_speed_pid =
            phase_is_emergency ||
            force_before_path_safe_speed ||
            force_after_path_safe_speed ||
            force_two_point_path_speed_pid ||
            longitudinal_controller_mode_ ==
                LongitudinalControllerMode::CONSTANT_SPEED;

        if (use_speed_pid)
        {
            double target_speed_mps =
                param_.get("general.v_ref");

            if (phase_is_emergency ||
                force_before_path_safe_speed ||
                force_after_path_safe_speed ||
                force_two_point_path_speed_pid)
            {
                target_speed_mps =
                    kSafeRollingTargetSpeedMps;
            }

            double speed_feedback_mps =
                current_state_.vx;

            if (phase_is_emergency)
            {
                speed_feedback_mps =
                    std::isfinite(current_state_.vx_enc)
                        ? std::max(
                              0.0,
                              current_state_.vx_enc
                          )
                        : current_state_.vx;
            }

            const double speed_error_mps =
                target_speed_mps
                - speed_feedback_mps;

            /*
                Resistance feedforward is used for:
                    - normal CONSTANT_SPEED mode,
                    - BEFORE_START safe-speed PID,
                    - exactly-two-point safe-speed PID.

                It remains disabled for:
                    - EMERGENCY_BRAKE,
                    - AFTER_END.
            */
            const bool use_resistance_ff =
                phase_info_.phase ==
                    DrivePhase::DRIVE &&
                !force_after_path_safe_speed &&
                (
                    force_before_path_safe_speed ||
                    force_two_point_path_speed_pid ||
                    longitudinal_controller_mode_ ==
                        LongitudinalControllerMode::CONSTANT_SPEED
                );

            const double torque_ff_Nm =
                use_resistance_ff
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

            const double torque_cmd_Nm =
                torque_ff_Nm
                + pid_speed_hold_.get_output();

            long_debug_.torque_cmd_Nm =
                torque_cmd_Nm;

            setBaseTorqueCommandNm(
                torque_cmd_Nm
            );
        }
        else if (
            longitudinal_controller_mode_ ==
                LongitudinalControllerMode::VELOCITY_PROFILE)
        {
            const double a_ref =
                getAccelerationReferenceForControl(
                    longitudinal_controller_mode_,
                    ref_path
                );

            const double torque_cmd_Nm =
                computeAccelerationFeedforwardTorqueNm(
                    param_,
                    a_ref,
                    current_state_.vx
                );

            long_debug_.torque_cmd_Nm =
                torque_cmd_Nm;

            setBaseTorqueCommandNm(
                torque_cmd_Nm
            );
        }
        else
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[dv_control] No longitudinal controller mode selected."
            );

            long_debug_.torque_cmd_Nm =
                0.0;

            setBaseTorqueCommandNm(
                0.0
            );
        }
    }

    const double longitudinal_control_ms =
        1000.0 * (
            ros::WallTime::now().toSec()
            - longitudinal_control_t0
        );

    const double callback_before_publish_ms =
        1000.0 * (
            ros::WallTime::now().toSec()
            - pose_callback_t0
        );

    const double control_cycle_budget_ms =
        1000.0 * getControlDt();

    const double callback_budget_usage_percent =
        control_cycle_budget_ms > 1.0e-9
            ? 100.0
                * callback_before_publish_ms
                / control_cycle_budget_ms
            : 0.0;

    if (printConsoleDebugInfo())
    {
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "[CONTROL_TIMING] "
            << "projection_and_reference="
            << projection_ms
            << " ms"

            << " | lateral_control="
            << control_ms
            << " ms"

            << " | longitudinal_control="
            << longitudinal_control_ms
            << " ms"

            << " | callback_before_publish="
            << callback_before_publish_ms
            << " ms"

            << " | cycle_budget="
            << control_cycle_budget_ms
            << " ms"

            << " | budget_usage="
            << callback_budget_usage_percent
            << "%"
        );
    }

    // =========================================================================
    //                              PHASE OUTPUT
    // =========================================================================

    publishGlobalHandlingInfo();

    if (phase_info_.phase ==
        DrivePhase::WAITING)
    {
        publishZeroControl(false);
        publishAllDebug();
        return;
    }

    if (phase_info_.phase ==
        DrivePhase::DRIVE)
    {
        publishControl(
            control_output_.steer_cmd_rad,
            control_output_.base_torque_cmd_Nm,
            false
        );

        publishAllDebug();
        return;
    }

    if (phase_info_.phase ==
        DrivePhase::FINAL_BRAKE)
    {
        publishControl(
            control_output_.steer_cmd_rad,
            control_output_.base_torque_cmd_Nm,
            false
        );

        publishAllDebug();
        return;
    }

    if (phase_info_.phase ==
        DrivePhase::EMERGENCY_BRAKE)
    {
        publishControl(
            control_output_.steer_cmd_rad,
            control_output_.base_torque_cmd_Nm,
            false
        );

        publishAllDebug();
        return;
    }

    if (phase_info_.phase ==
        DrivePhase::COASTING)
    {
        publishZeroMovement(
            control_output_.steer_cmd_rad,
            false
        );

        publishAllDebug();
        return;
    }

    if (phase_info_.phase ==
        DrivePhase::FINISHED)
    {
        publishZeroMovement(
            control_output_.steer_cmd_rad,
            true
        );

        publishAllDebug();
        return;
    }

    publishAllDebug();
}

// =============================================================================
//                              OPTIONAL ODOMETRY CALLBACK
// =============================================================================

// =============================================================================
//                              ANGLE SENSOR CALLBACK
// =============================================================================

void Controller::angleSensorCallback(const std_msgs::Float64& msg)
{
    if (!std::isfinite(msg.data))
    {
        ROS_ERROR_THROTTLE(
            1.0,
            "[dv_control] Steering encoder returned NaN/Inf. "
            "Ignoring sample."
        );

        return;
    }

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    current_state_.delta_enc =
        clampLocal(
            msg.data,
            -max_steer,
            max_steer
        );

    /*
        Readiness is granted only after a valid finite sample.
    */
    has_delta_encoder_ =
        true;

    emergency_check_input_.encoder_position_rad =
        current_state_.delta_enc;

    emergency_check_input_.new_encoder_message =
        true;

    /*
        Initialize command and PT2 rate from the first physical angle.
        Do not differentiate the encoder.
    */
    if (!steering_state_initialized_)
    {
        initializeSteeringStateIfNeeded();
    }

    current_state_.delta =
        current_state_.delta_enc;

    current_state_.delta_vehicle_used =
        current_state_.delta_enc;
}

// =============================================================================
//                              CUBE MARS STATUS CALLBACK
// =============================================================================

void Controller::cubeMarsStatusCallback(const std_msgs::Bool& msg)
{
    if( global_handling_info_.cube_mars_initialization_finished ) return;
    global_handling_info_.cube_mars_initialization_finished =
        msg.data;

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

    updateReadyFlag();
}

void Controller::imuCallback(const dv_interfaces::Imu::ConstPtr& msg)
{
    current_state_.acc_x =
        static_cast<double>(msg->acc.x);

    current_state_.acc_y =
        static_cast<double>(msg->acc.y);

    global_handling_info_.has_received_imu_message =
        true;

    updateReadyFlag();
}

// =============================================================================
//                              DV BOARD CALLBACK
// =============================================================================

void Controller::dvBoardCallback(const dv_interfaces::DV_board::ConstPtr& msg)
{
    global_handling_info_.has_received_first_dv_board_message =
        true;

    current_state_.vx_enc =
       0.25 * (
    msg->velocity_FL +
    msg->velocity_FR +
    msg->velocity_RL +
    msg->velocity_RR
);
}

// =============================================================================
//                              CONTROL PUBLISHING
// =============================================================================

void Controller::publishControl(
    double steering_rad,
    double total_torque_cmd_Nm,
    bool finished)
{
    const double publish_control_t0 =
        ros::WallTime::now().toSec();

    if (!std::isfinite(steering_rad))
    {
        ROS_ERROR_STREAM_THROTTLE(
            0.2,
            "[dv_control][INVALID_CONTROL_OUTPUT] "
            "Non-finite steering command. Publishing 0 rad."
        );
    }

    control_output_.steer_cmd_rad =
        std::isfinite(steering_rad)
            ? steering_rad
            : 0.0;

    if (!std::isfinite(total_torque_cmd_Nm))
    {
        ROS_ERROR_STREAM_THROTTLE(
            0.2,
            "[dv_control][INVALID_CONTROL_OUTPUT] "
            "Non-finite total torque command. Publishing 0 Nm."
        );

        total_torque_cmd_Nm =
            0.0;
    }

    /*
        The slowing-down heuristic is deliberately applied only during normal
        DRIVE with a valid on-path reference.

        It is not used during:
            - FINAL_BRAKE,
            - EMERGENCY_BRAKE,
            - COASTING,
            - FINISHED,
            - before-start safe-speed control,
            - after-end safe-speed control.

        Exactly-two-point straight mode intentionally remains eligible for
        this heuristic. Its longitudinal controller is still the safe-speed
        PID at 3 m/s, but the lateral-error recovery heuristic may reduce its
        positive torque or request recovery braking when |ey| grows.

        The allocator's existing total-torque rate limiter remains the only
        rate limiter. This heuristic modifies the requested total wheel torque
        before it reaches the allocator.
    */
    SlowingDownHeuristicResult slowing_heuristic;

    const bool slowing_heuristic_allowed =
        phase_info_.phase == DrivePhase::DRIVE &&
        ref_path.valid &&
        !ref_path.before_bolide &&
        !ref_path.after_bolide;

    if (slowing_heuristic_allowed)
    {
        try
        {
            slowing_heuristic =
                applySlowingDownHeuristic(
                    param_,
                    total_torque_cmd_Nm,
                    current_state_.ey,
                    current_state_.vx
                );

            total_torque_cmd_Nm =
                slowing_heuristic.output_total_torque_Nm;
        }
        catch (const std::exception& exception)
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                "[dv_control][SLOWING_DOWN_HEURISTIC_EXCEPTION] "
                << exception.what()
                << " | heuristic bypassed for this cycle"
            );
        }
        catch (...)
        {
            ROS_ERROR_STREAM_THROTTLE(
                0.2,
                "[dv_control][SLOWING_DOWN_HEURISTIC_UNKNOWN_EXCEPTION] "
                "heuristic bypassed for this cycle"
            );
        }

        if (printConsoleDebugInfo() &&
            slowing_heuristic.enabled)
        {
            ROS_INFO_STREAM_THROTTLE(
                0.2,
                "[SLOWING_DOWN_HEURISTIC]"
                << " applied=" << slowing_heuristic.applied
                << " ey=" << current_state_.ey
                << " m"
                << " abs_ey=" << std::abs(current_state_.ey)
                << " m"
                << " ax_requested="
                << slowing_heuristic.requested_ax_mps2
                << " mps2"
                << " drive_scale="
                << slowing_heuristic.drive_scale
                << " brake_activation="
                << slowing_heuristic.brake_activation
                << " recovery_ax="
                << slowing_heuristic.recovery_ax_mps2
                << " mps2"
                << " ax_output="
                << slowing_heuristic.output_ax_mps2
                << " mps2"
                << " torque_input="
                << slowing_heuristic.input_total_torque_Nm
                << " Nm"
                << " torque_output="
                << slowing_heuristic.output_total_torque_Nm
                << " Nm"
            );
        }
    }

    /*
        Keep the two control-level guards which are not part of the allocator:
            - do not command negative torque while measured wheel speed is
              negative,
            - do not command positive torque above configured v_max.
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

    const double speed_for_vmax_check_mps =
        std::max(
            std::isfinite(current_state_.vx_enc)
                ? std::abs(current_state_.vx_enc)
                : 0.0,
            std::isfinite(current_state_.vx)
                ? std::abs(current_state_.vx)
                : 0.0
        );

    const double v_max_mps =
        param_.get("general.v_max");

    if (std::isfinite(v_max_mps) &&
        v_max_mps > 0.0 &&
        speed_for_vmax_check_mps > v_max_mps)
    {
        total_torque_cmd_Nm =
            std::min(
                total_torque_cmd_Nm,
                0.0
            );
    }

    control_output_.base_torque_cmd_Nm =
        total_torque_cmd_Nm;

    long_debug_.torque_cmd_Nm =
        total_torque_cmd_Nm;

    SimpleTorqueAllocatorInput input;

    input.total_torque_cmd_Nm =
        total_torque_cmd_Nm;

    input.desired_yaw_moment_Nm =
        control_output_.tv_yaw_moment_Nm;

    input.vx_mps =
        current_state_.vx;

    const double raw_ax_mps2 =
        std::isfinite(current_state_.acc_x)
            ? current_state_.acc_x
            : estimateLongitudinalAccelerationFromTorqueAndResistance(
                  param_,
                  total_torque_cmd_Nm,
                  current_state_.vx
              );

    const double raw_ay_mps2 =
        std::isfinite(current_state_.acc_y)
            ? current_state_.acc_y
            : 0.0;

    const dv_control_common::RelaxedAcceleration relaxed_acceleration =
        load_transfer_relaxation_.update(
            raw_ax_mps2,
            raw_ay_mps2,
            getControlDt(),
            param_.get("model.mass_transfer.tau_load_s")
        );

    input.ax_mps2 =
        relaxed_acceleration.ax_mps2;

    input.ay_mps2 =
        relaxed_acceleration.ay_mps2;

    input.delta_bicycle_rad =
        current_state_.delta_vehicle_used;

    input.previous_total_output_torque_Nm =
        control_output_.torque_allocation.output_total_torque_Nm;

    input.previous_total_output_valid =
        control_output_.torque_allocation_valid;

    input.bypass_total_rate_limiter =
        finished ||
        phase_info_.phase == DrivePhase::WAITING ||
        phase_info_.phase == DrivePhase::COASTING ||
        phase_info_.phase == DrivePhase::FINISHED;

    const double allocator_t0 =
        ros::WallTime::now().toSec();

    control_output_.torque_allocation =
        allocateSimpleWheelTorques(
            param_,
            input
        );

    control_output_.torque_allocation_valid =
        true;

    const double allocator_ms =
        1000.0 * (
            ros::WallTime::now().toSec()
            - allocator_t0
        );

    const SimpleTorqueAllocatorResult& allocation =
        control_output_.torque_allocation;

    dv_interfaces::Control msg;

    if(useJacaTorqueVectoring())
    {
        msg.move_type =
            dv_interfaces::Control::ONE_WHEEL;
    }
    else
    {
        msg.move_type =
            dv_interfaces::Control::FOUR_WHEEL;
    }
   

    msg.steeringAngle_rad =
        static_cast<float>(
            control_output_.steer_cmd_rad
        );

    msg.finished =
        finished;

    msg.torque_FL =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation.torque_output_Nm.FL
        );

    msg.torque_FR =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation.torque_output_Nm.FR
        );

    msg.torque_RL =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation.torque_output_Nm.RL
        );

    msg.torque_RR =
        wheelTorqueNmToVehicleInterfaceCommand(
            param_,
            allocation.torque_output_Nm.RR
        );

    pub_control_.publish(msg);

    if (printConsoleDebugInfo())
    {
        ROS_INFO_STREAM_THROTTLE(
            0.2,
            "[TORQUE_ALLOC_DEBUG]"
            << " req_total=" << allocation.requested_total_torque_Nm
            << " direction_total=" << allocation.total_after_direction_limit_Nm
            << " power_total=" << allocation.total_after_power_limit_Nm
            << " rate_total=" << allocation.rate_limited_total_torque_Nm
            << " output_total=" << allocation.output_total_torque_Nm
            << " Mz_req=" << allocation.requested_tv_yaw_moment_Nm
            << " Mz_scaled=" << allocation.scaled_tv_yaw_moment_Nm
            << " tv_speed_scale=" << allocation.tv_speed_scale
            << " wheel_scale=" << allocation.per_wheel_limit_scale
            << " wheel_limited=" << allocation.per_wheel_limited
            << " torque=["
            << allocation.torque_output_Nm.FL << ","
            << allocation.torque_output_Nm.FR << ","
            << allocation.torque_output_Nm.RL << ","
            << allocation.torque_output_Nm.RR << "]"
            << " allocator_ms=" << allocator_ms
        );
    }

    if (printConsoleDebugInfo())
    {
        const double publish_control_total_ms =
            1000.0 * (
                ros::WallTime::now().toSec()
                - publish_control_t0
            );

        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "[PUBLISH_CONTROL_TIMING]"
            << " allocator=" << allocator_ms << " ms"
            << " total=" << publish_control_total_ms << " ms"
        );
    }
}

void Controller::publishZeroControl(bool finished)
{
    control_output_.base_torque_cmd_Nm = 0.0;
    control_output_.tv_yaw_moment_Nm = 0.0;
    control_output_.tv_yaw_moment_raw_Nm = 0.0;

    /*
        Invalidating the previous allocator output makes this zero command
        immediate instead of rate-limiting it from the previous drive torque.
    */
    control_output_.torque_allocation =
        SimpleTorqueAllocatorResult{};

    control_output_.torque_allocation_valid =
        false;

    publishControl(
        0.0,
        0.0,
        finished
    );
}

void Controller::publishZeroMovement(
    double steering_rad,
    bool finished)
{
    control_output_.base_torque_cmd_Nm = 0.0;
    control_output_.tv_yaw_moment_Nm = 0.0;
    control_output_.tv_yaw_moment_raw_Nm = 0.0;

    control_output_.torque_allocation =
        SimpleTorqueAllocatorResult{};

    control_output_.torque_allocation_valid =
        false;

    publishControl(
        steering_rad,
        0.0,
        finished
    );
}


// =============================================================================
//                              DEBUG PUBLISHING
// =============================================================================

void Controller::publishGlobalHandlingInfo()
{
    dv_interfaces::Controlhandling_info msg;

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

    msg.as_finished =
        global_handling_info_.as_finished;

    msg.phase =
        phaseToString(phase_info_.phase);

    msg.lap_count =
        global_handling_info_.lap_count;

    msg.final_brake_requested =
        global_handling_info_.final_brake_requested;

    pub_global_handling_info_.publish(msg);
}

void Controller::publishControlDebugLongitudinal()
{
    dv_interfaces::ControlDebug_long msg;

    double v_ref =
        0.0;

    double ax_ref =
        0.0;

    if (phase_info_.phase ==
        DrivePhase::FINAL_BRAKE)
    {
        v_ref =
            0.0;

        ax_ref =
            kFinalBrakeTargetAccelerationMps2;
    }
    else if (phase_info_.phase ==
             DrivePhase::EMERGENCY_BRAKE)
    {
        v_ref =
            kSafeRollingTargetSpeedMps;

        ax_ref =
            0.0;
    }
    else if (ref_path.valid &&
             ref_path.after_bolide)
    {
        v_ref =
            kSafeRollingTargetSpeedMps;

        ax_ref =
            0.0;
    }
    else if (longitudinal_controller_mode_ ==
             LongitudinalControllerMode::CONSTANT_SPEED)
    {
        v_ref =
            param_.get("general.v_ref");

        ax_ref =
            0.0;
    }
    else
    {
        v_ref =
            ref_path.velocity_ref.size() > 1
                ? ref_path.velocity_ref[1]
                : 0.0;

        ax_ref =
            getAccelerationReferenceForControl(
                longitudinal_controller_mode_,
                ref_path
            );
    }

    const double ax_curr_debug_mps2 =
        std::isfinite(current_state_.acc_x)
            ? current_state_.acc_x
            : 0.0;

    msg.v_curr_mps =
        current_state_.vx;

    msg.v_ref_mps =
        v_ref;

    msg.v_error_mps =
        v_ref
        - current_state_.vx;

    msg.torque_cmd_Nm =
        long_debug_.torque_cmd_Nm;

    msg.ax_mps2 =
        ax_curr_debug_mps2;

    msg.ax_ref_mps2 =
        ax_ref;

    msg.ax_error_mps2 =
        ax_ref
        - ax_curr_debug_mps2;

    msg.s_curr_m =
        ref_path.s;

    msg.phase =
        phaseToString(
            phase_info_.phase
        );

    pub_debug_long_.publish(msg);
}

void Controller::publishControlDebugLateral()
{
    dv_interfaces::ControlDebug_lat msg;

    const double curr_steer =
        has_delta_encoder_
            ? current_state_.delta_enc
            : current_state_.delta_vehicle_used;

    const double ref_steer =
        control_output_.steer_cmd_rad;

    msg.ey_m =
        current_state_.ey;

    msg.epsi_rad =
        current_state_.epsi;

    msg.curr_steer_rad =
        curr_steer;

    msg.ref_steer_rad =
        lateral_controller_result_.delta_act_next;

    msg.steer_error_rad =
        ref_steer - curr_steer;

    msg.curr_yaw_rate_radps =
        current_state_.r;

    msg.ref_yaw_rate_radps =
        lateral_controller_result_.r_next;

    msg.yaw_rate_error_radps =
        msg.ref_yaw_rate_radps - current_state_.r;

    msg.curr_vy_mps =
        current_state_.vy;

    msg.ref_vy_mps =
        lateral_controller_result_.vy_next;

    msg.vy_error_mps =
        msg.ref_vy_mps - current_state_.vy;

    msg.ay_mps2 =
        current_state_.acc_y;

    msg.optimized_torque_vectoring_used =
        lat_debug_.optimized_torque_vectoring_used;

    msg.jaca_torque_vectoring_used =
        lat_debug_.jaca_torque_vectoring_used;

    msg.tv_yaw_moment_raw_Nm =
        lat_debug_.tv_yaw_moment_raw_Nm;

    msg.tv_yaw_moment_Nm =
        lat_debug_.tv_yaw_moment_Nm;

    msg.jaca_tv_yaw_moment_Nm =
        lat_debug_.jaca_tv_yaw_moment_Nm;

    msg.torque_allocator_global_scale =
        control_output_.torque_allocation_valid
            ? control_output_.torque_allocation.per_wheel_limit_scale
            : 1.0;

    pub_debug_lat_.publish(msg);
}


void Controller::publishEmergencyCheckDebug()
{
    dv_interfaces::EmergencyAutoX msg;

    msg.header.stamp =
        ros::Time::now();

    msg.header.frame_id =
        "base_link";

    msg.is_safe =
        emergency_check_is_safe_;

    msg.reason =
        emergency_check_reason_;

    pub_emergency_check_.publish(msg);
}

void Controller::publishAllDebug()
{
    publishGlobalHandlingInfo();
    publishControlDebugLongitudinal();
    publishControlDebugLateral();
}

// =============================================================================
//                              REFERENCE POINT MARKER
// =============================================================================

void Controller::publishReferencePoint()
{
    if (!ref_path.valid)
    {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "dv_control_ref_point";

    marker.id =
        1;

    marker.type =
        visualization_msgs::Marker::SPHERE;

    marker.action =
        visualization_msgs::Marker::ADD;

    marker.pose.position.x =
        ref_path.x_ref_point;

    marker.pose.position.y =
        ref_path.y_ref_point;

    marker.pose.position.z =
        0.05;

    marker.pose.orientation.w =
        1.0;

    marker.scale.x =
        1.05;

    marker.scale.y =
        1.05;

    marker.scale.z =
        1.05;

    marker.color.a =
        1.0;

    marker.color.r =
        1.0;

    marker.color.g =
        0.0;

    marker.color.b =
        0.0;

    pub_ref_point_.publish(marker);
}

// =============================================================================
//                              PATH MARKER
// =============================================================================

void Controller::publishReferencePath(const Eigen::VectorXd& X,
                                      const Eigen::VectorXd& Y)
{
    if (X.size() != Y.size() || X.size() <= 0)
    {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "dv_control_ref_path";

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

        p.x =
            X(i);

        p.y =
            Y(i);

        p.z =
            0.0;

        marker.points.push_back(p);
    }

    pub_ref_path_.publish(marker);
}

void Controller::publishRawPathFromPP(const Eigen::VectorXd& X,
                                      const Eigen::VectorXd& Y)
{
    if (X.size() != Y.size() || X.size() <= 0)
    {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id =
        "map";

    marker.header.stamp =
        ros::Time::now();

    marker.ns =
        "dv_control_raw_path_from_pp";

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
        1.0;

    marker.color.g =
        0.0;

    marker.color.b =
        0.0;

    marker.color.a =
        1.0;

    marker.points.clear();
    marker.points.reserve(static_cast<std::size_t>(X.size()));

    for (int i = 0; i < X.size(); ++i)
    {
        geometry_msgs::Point p;

        p.x =
            X(i);

        p.y =
            Y(i);

        p.z =
            0.0;

        marker.points.push_back(p);
    }

    pub_raw_path_from_pp_.publish(marker);
}

//=============================================================================
//                             LAP COUNTER SUBSCRIBER
//=============================================================================
void Controller::lapCountCallback(const std_msgs::Int32& msg)
{
    global_handling_info_.lap_count =
        msg.data;


    if(printConsoleDebugInfo())
    {
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "[dv_control] Lap counter updated: "
            << global_handling_info_.lap_count
        );
    }

    if(global_handling_info_.lap_count >= param_.get("general.stop_after_lap_count"))
    {
        
        if(printConsoleDebugInfo())
        {
            ROS_INFO_STREAM_THROTTLE(
                1.0,
                "[dv_control] Final brake request."
            );
        }
        global_handling_info_.final_brake_requested = true;
    }

}

} // namespace dv_control
