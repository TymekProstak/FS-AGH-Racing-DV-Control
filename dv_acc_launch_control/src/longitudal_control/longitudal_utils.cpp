#include "longitudal_utils.hpp"

#include "math_utils.hpp"
#include "dv_control_common/load_transfer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <iomanip>
#include <sstream>

#include <ros/ros.h>

namespace acc_launch_control
{

namespace
{

double clampLocalLong(const double x,
                      const double lo,
                      const double hi)
{
    return std::max(lo, std::min(x, hi));
}

double getTireRadiusM(const ParamBank& P)
{
    const double R_tire =
        P.get("model.tire.R_tire");

    if (R_tire <= 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.tire.R_tire must be > 0"
        );
    }

    return R_tire;
}

double getTrackWidthM(const ParamBank& P)
{
    const double track_width =
        P.get("model.body.track_width");

    if (track_width <= 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.body.track_width must be > 0"
        );
    }

    return track_width;
}


bool useAeroDownforce(const ParamBank& P)
{
    /*
        Optional aero switch.

        If the config contains:
            model.body.use_aero
        then aero downforce is used only when this flag is true.

        If the key is missing, aerodynamic downforce remains enabled.
    */
    const auto it = P.idx.find("model.body.use_aero");

    if (it == P.idx.end())
    {
        return true;
    }

    return P.values.at(static_cast<std::size_t>(it->second)) != 0.0;
}

void computeAeroDownforceN(const ParamBank& P,
                           const double vx_mps,
                           double& front_aero_N,
                           double& rear_aero_N)
{
    front_aero_N = 0.0;
    rear_aero_N = 0.0;

    if (!useAeroDownforce(P))
    {
        return;
    }

    const double Cl1 =
        P.get("model.body.Cl1");

    const double Cl2 =
        P.get("model.body.Cl2");

    const double v =
        std::abs(vx_mps);

    front_aero_N =
        std::max(0.0, Cl1 * v * v);

    rear_aero_N =
        std::max(0.0, Cl2 * v * v);
}

double computeTotalAeroDownforceN(const ParamBank& P,
                                  const double vx_mps)
{
    double front_aero_N = 0.0;
    double rear_aero_N = 0.0;

    computeAeroDownforceN(
        P,
        vx_mps,
        front_aero_N,
        rear_aero_N
    );

    return
        front_aero_N
        + rear_aero_N;
}

double getGearRatio(const ParamBank& P)
{
    const double gear_ratio =
        P.get("model.drivetrain.gear_ratio");

    if (!std::isfinite(gear_ratio) || std::abs(gear_ratio) <= 1.0e-9)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.gear_ratio must be finite and non-zero"
        );
    }

    return gear_ratio;
}

double getConfiguredTotalWheelDriveEnvelopeNm(const ParamBank& P)
{
    const double limit =
        P.get("model.drivetrain.M_wheel_drive_max_Nm");

    if (!std::isfinite(limit) || limit < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.M_wheel_drive_max_Nm must be finite and >= 0"
        );
    }

    return limit;
}

double getConfiguredTotalWheelBrakeEnvelopeNm(const ParamBank& P)
{
    const double limit =
        P.get("model.drivetrain.M_wheel_brake_max_Nm");

    if (!std::isfinite(limit) || limit < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.M_wheel_brake_max_Nm must be finite and >= 0"
        );
    }

    return limit;
}

double getTotalWheelDriveLimitFromEngineUsageNm(const ParamBank& P)
{
    const double engine_total_Nm =
        P.get("model.drivetrain.M_engine_max_Nm_usage_drive_total");

    if (!std::isfinite(engine_total_Nm) || engine_total_Nm < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.M_engine_max_Nm_usage_drive_total must be finite and >= 0"
        );
    }

    return
        engine_total_Nm
        * std::abs(getGearRatio(P));
}

double getTotalWheelBrakeLimitFromEngineUsageNm(const ParamBank& P)
{
    const double engine_total_Nm =
        P.get("model.drivetrain.M_engine_max_Nm_usage_brake_total");

    if (!std::isfinite(engine_total_Nm) || engine_total_Nm < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.M_engine_max_Nm_usage_brake_total must be finite and >= 0"
        );
    }

    return
        engine_total_Nm
        * std::abs(getGearRatio(P));
}

double getStaticTotalWheelDriveLimitNm(const ParamBank& P)
{
    return std::min(
        getConfiguredTotalWheelDriveEnvelopeNm(P),
        getTotalWheelDriveLimitFromEngineUsageNm(P)
    );
}

double getStaticTotalWheelBrakeLimitNm(const ParamBank& P)
{
    return std::min(
        getConfiguredTotalWheelBrakeEnvelopeNm(P),
        getTotalWheelBrakeLimitFromEngineUsageNm(P)
    );
}

double getWheelDriveLimitFromEngineNm(const ParamBank& P)
{
    const double engine_per_wheel_Nm =
        P.get("model.drivetrain.wheel_max_torque_engine_drive_Nm");

    if (!std::isfinite(engine_per_wheel_Nm) || engine_per_wheel_Nm < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.wheel_max_torque_engine_drive_Nm must be finite and >= 0"
        );
    }

    return
        engine_per_wheel_Nm
        * std::abs(getGearRatio(P));
}

double getWheelBrakeLimitFromEngineNm(const ParamBank& P)
{
    const double engine_per_wheel_Nm =
        P.get("model.drivetrain.wheel_max_torque_engine_brake_Nm");

    if (!std::isfinite(engine_per_wheel_Nm) || engine_per_wheel_Nm < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.wheel_max_torque_engine_brake_Nm must be finite and >= 0"
        );
    }

    return
        engine_per_wheel_Nm
        * std::abs(getGearRatio(P));
}

double getMaxTotalWheelTorqueNm(const ParamBank& P)
{
    return std::max(
        getStaticTotalWheelDriveLimitNm(P),
        getStaticTotalWheelBrakeLimitNm(P)
    );
}

double getMaxWheelTorqueNm(const ParamBank& P)
{
    return std::max(
        getWheelDriveLimitFromEngineNm(P),
        getWheelBrakeLimitFromEngineNm(P)
    );
}


PIDParams makePidParamsFromPrefix(const ParamBank& P,
                                  const std::string& prefix)
{
    PIDParams params;

    params.Kp =
        P.get(prefix + ".Kp");

    params.Ki =
        P.get(prefix + ".Ki");

    params.Kd =
        P.get(prefix + ".Kd");

    params.saturation_lower =
        P.get(prefix + ".saturation_lower_Nm");

    params.saturation_upper =
        P.get(prefix + ".saturation_upper_Nm");

    params.anti_windup_gain =
        P.get(prefix + ".anti_windup_gain");

    params.leak_time_scale =
        P.get(prefix + ".leak_time_scale");

    params.use_output_rate_limit =
        P.getBool(prefix + ".use_output_rate_limit");

    params.output_rate_up =
        P.get(prefix + ".output_rate_up");

    params.output_rate_down =
        P.get(prefix + ".output_rate_down");

    return params;
}

WheelValues scaleWheelValues(const WheelValues& in,
                             const double scale)
{
    WheelValues out;

    out.FL = in.FL * scale;
    out.FR = in.FR * scale;
    out.RL = in.RL * scale;
    out.RR = in.RR * scale;

    return out;
}

WheelValues minWheelValues(const WheelValues& a,
                           const WheelValues& b)
{
    WheelValues out;

    out.FL = std::min(a.FL, b.FL);
    out.FR = std::min(a.FR, b.FR);
    out.RL = std::min(a.RL, b.RL);
    out.RR = std::min(a.RR, b.RR);

    return out;
}

double computeGlobalTorqueScaleFromWheelLimits(
    const WheelValues& requested_abs_torque_Nm,
    const WheelValues& max_abs_torque_Nm
)
{
    double scale =
        1.0;

    if (requested_abs_torque_Nm.FL > 1.0e-9)
    {
        scale =
            std::min(
                scale,
                max_abs_torque_Nm.FL / requested_abs_torque_Nm.FL
            );
    }

    if (requested_abs_torque_Nm.FR > 1.0e-9)
    {
        scale =
            std::min(
                scale,
                max_abs_torque_Nm.FR / requested_abs_torque_Nm.FR
            );
    }

    if (requested_abs_torque_Nm.RL > 1.0e-9)
    {
        scale =
            std::min(
                scale,
                max_abs_torque_Nm.RL / requested_abs_torque_Nm.RL
            );
    }

    if (requested_abs_torque_Nm.RR > 1.0e-9)
    {
        scale =
            std::min(
                scale,
                max_abs_torque_Nm.RR / requested_abs_torque_Nm.RR
            );
    }

    return clampLocalLong(
        scale,
        0.0,
        1.0
    );
}

double getControlDtForAllocator(const ParamBank& P)
{
    const double hz =
        P.get("frequency.steer_cmd_loop_hz");

    if (!std::isfinite(hz) || hz <= 1.0e-9)
    {
        throw std::runtime_error(
            "[longitudal_utils] frequency.steer_cmd_loop_hz must be finite and > 0"
        );
    }

    return 1.0 / hz;
}


double getNormalizedAllocatorTorqueRateUp(const ParamBank& P)
{
    const double rate =
        P.get("model.drivetrain.M_dot_up");

    if (!std::isfinite(rate) || rate < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.M_dot_up "
            "must be finite and >= 0, interpreted as normalized total-torque rate [1/s]"
        );
    }

    return rate;
}


double getNormalizedAllocatorTorqueRateDown(const ParamBank& P)
{
    const double rate =
        P.get("model.drivetrain.M_dot_down");

    if (!std::isfinite(rate) || rate < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.M_dot_down "
            "must be finite and >= 0, interpreted as normalized total-torque rate [1/s]"
        );
    }

    return rate;
}


double computeTotalTorqueRateScaleMin(const double previous_total_Nm,
                                      const double target_total_Nm,
                                      const double max_step_up_Nm,
                                      const double max_step_down_Nm)
{
    constexpr double EPS =
        1.0e-9;

    if (std::abs(target_total_Nm) <= EPS)
    {
        return 0.0;
    }

    const double allowed_low =
        previous_total_Nm - max_step_down_Nm;

    const double allowed_high =
        previous_total_Nm + max_step_up_Nm;

    const double s1 =
        allowed_low / target_total_Nm;

    const double s2 =
        allowed_high / target_total_Nm;

    return std::max(
        0.0,
        std::min(s1, s2)
    );
}


double computeTotalTorqueRateScaleMax(const double previous_total_Nm,
                                      const double target_total_Nm,
                                      const double max_step_up_Nm,
                                      const double max_step_down_Nm)
{
    constexpr double EPS =
        1.0e-9;

    if (std::abs(target_total_Nm) <= EPS)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double allowed_low =
        previous_total_Nm - max_step_down_Nm;

    const double allowed_high =
        previous_total_Nm + max_step_up_Nm;

    const double s1 =
        allowed_low / target_total_Nm;

    const double s2 =
        allowed_high / target_total_Nm;

    return std::max(
        0.0,
        std::max(s1, s2)
    );
}


double safeVxSignedForSlip(const double vx_mps)
{
    constexpr double EPS_VX =
        1.0e-4;

    /*
        There is no reverse-driving mode in this controller.

        This helper is used only as atan2 denominator in the slip estimator.
        Therefore it must never return a negative value.  A tiny negative vx
        near standstill must not flip the slip angle by pi and saturate the
        friction ellipse.
    */
    if (!std::isfinite(vx_mps))
    {
        return EPS_VX;
    }

    return std::max(
        vx_mps,
        EPS_VX
    );
}


double vxForSlipAngleCalculationMps(const double vx_mps)
{
    /*
        There is no reverse-driving mode here.

        Slip/Fy/friction-ellipse calculations must not use near-zero or
        negative longitudinal speed.  A tiny negative vx near standstill can
        flip atan2 by pi and make the limiter remove all available Fx.

        This does not change drivetrain power limiting or published velocity.
    */
    constexpr double MIN_VX_FOR_SLIP_ANGLE_MPS =
        2.0;

    if (!std::isfinite(vx_mps))
    {
        return MIN_VX_FOR_SLIP_ANGLE_MPS;
    }

    return std::max(
        vx_mps,
        MIN_VX_FOR_SLIP_ANGLE_MPS
    );
}

void rotateVelocityToWheelFrame(const double wheel_angle_rad,
                                double& vx_mps,
                                double& vy_mps)
{
    const double c =
        std::cos(wheel_angle_rad);

    const double s =
        std::sin(wheel_angle_rad);

    const double vx0 =
        vx_mps;

    const double vy0 =
        vy_mps;

    /*
        Same transform as reference allocator:

            vx_local =  c * vx + s * vy
            vy_local = -s * vx + c * vy
    */

    vx_mps =
        c * vx0
        + s * vy0;

    vy_mps =
        -s * vx0
        + c * vy0;
}


std::string wheelValuesToString(const WheelValues& v,
                                const int precision = 3)
{
    std::ostringstream ss;

    ss << std::fixed << std::setprecision(precision)
       << "{FL=" << v.FL
       << ", FR=" << v.FR
       << ", RL=" << v.RL
       << ", RR=" << v.RR
       << "}";

    return ss.str();
}


std::string wheelValuesDegToString(const WheelValues& v,
                                   const int precision = 2)
{
    constexpr double RAD_TO_DEG =
        57.2957795130823208768;

    std::ostringstream ss;

    ss << std::fixed << std::setprecision(precision)
       << "{FL=" << v.FL * RAD_TO_DEG
       << ", FR=" << v.FR * RAD_TO_DEG
       << ", RL=" << v.RL * RAD_TO_DEG
       << ", RR=" << v.RR * RAD_TO_DEG
       << "}";

    return ss.str();
}


WheelValues computeLateralUsageFromFyAndFz(const ParamBank& P,
                                           const WheelValues& Fy_N,
                                           const WheelValues& Fz_N)
{
    const double mu_y =
        std::max(
            1.0e-9,
            P.get("general.mu_y")
        );

    WheelValues out;

    out.FL =
        std::abs(Fy_N.FL) /
        std::max(1.0e-6, mu_y * std::max(1.0, Fz_N.FL));

    out.FR =
        std::abs(Fy_N.FR) /
        std::max(1.0e-6, mu_y * std::max(1.0, Fz_N.FR));

    out.RL =
        std::abs(Fy_N.RL) /
        std::max(1.0e-6, mu_y * std::max(1.0, Fz_N.RL));

    out.RR =
        std::abs(Fy_N.RR) /
        std::max(1.0e-6, mu_y * std::max(1.0, Fz_N.RR));

    return out;
}


WheelValues computeWheelScaleCandidates(const WheelValues& requested_abs_torque_Nm,
                                        const WheelValues& max_abs_torque_Nm)
{
    WheelValues out;

    out.FL =
        requested_abs_torque_Nm.FL > 1.0e-9
            ? max_abs_torque_Nm.FL / requested_abs_torque_Nm.FL
            : 999.0;

    out.FR =
        requested_abs_torque_Nm.FR > 1.0e-9
            ? max_abs_torque_Nm.FR / requested_abs_torque_Nm.FR
            : 999.0;

    out.RL =
        requested_abs_torque_Nm.RL > 1.0e-9
            ? max_abs_torque_Nm.RL / requested_abs_torque_Nm.RL
            : 999.0;

    out.RR =
        requested_abs_torque_Nm.RR > 1.0e-9
            ? max_abs_torque_Nm.RR / requested_abs_torque_Nm.RR
            : 999.0;

    return out;
}


double sumPositiveWheelLoadsN(const WheelValues& Fz_N)
{
    return
        std::max(0.0, Fz_N.FL)
        + std::max(0.0, Fz_N.FR)
        + std::max(0.0, Fz_N.RL)
        + std::max(0.0, Fz_N.RR);
}


/*
    Global longitudinal traction cap.

    This cap is intentionally independent of the slip-angle / friction-ellipse
    limiter.

    The friction ellipse is disabled below 3 m/s because the lateral slip-angle
    estimate can be garbage at low speed.  But disabling the ellipse must NOT
    disable the basic longitudinal physics limit:

        |sum(T_wheel)| <= mu_x * sum(Fz) * R_tire

    Without this cap, right after LAUNCH, at vx < 3 m/s, ACCELERATION can jump
    from the launch-map torque, which is mu-limited, to the engine/drivetrain
    limit.
*/
double computeTotalLongitudinalFrictionCapNm(
    const ParamBank& P,
    const double vx_mps,
    const double ax_mps2,
    const double ay_mps2
)
{
    const WheelValues Fz_N =
        estimateWheelNormalLoadsN(
            P,
            vx_mps,
            ax_mps2,
            ay_mps2
        );

    const double Fz_sum_N =
        std::max(
            1.0,
            sumPositiveWheelLoadsN(Fz_N)
        );

    const double mu_x =
        std::max(
            0.0,
            P.get("general.mu_x")
        );

    return
        mu_x
        * Fz_sum_N
        * getTireRadiusM(P);
}

} // namespace


// =============================================================================
//                              PID CONFIG
// =============================================================================

PIDParams makeSpeedPidParams(const ParamBank& P)
{
    return makePidParamsFromPrefix(P, "speed_pid");
}


// =============================================================================
//                          RESISTANCE / FEEDFORWARD
// =============================================================================

double computeResistanceFeedforwardTorqueNm(const ParamBank& P,
                                            double vx_mps)
{
    const double m =
        P.get("model.body.m");

    const double g =
        P.get("model.body.g");

    const double Cd =
        P.get("model.body.Cd");

    const double Crr =
        P.get("model.body.rolling_resistance_coeff");

    const double resistance_constant_N =
        P.get("model.body.resistance_constant_N");

    const double resistance_linear_N_per_mps =
        P.get("model.body.resistance_linear_N_per_mps");

    const double R_tire =
        getTireRadiusM(P);

    const double v =
        std::abs(vx_mps);

    const double Fz_for_rolling_N =
        m * g
        + computeTotalAeroDownforceN(P, vx_mps);

    const double F_roll =
        Crr * Fz_for_rolling_N;

    const double F_drag =
        Cd * v * v;

    const double F_empirical =
        resistance_constant_N
        + resistance_linear_N_per_mps * v;

    const double F_res =
        F_roll
        + F_drag
        + F_empirical;

    return F_res * R_tire;
}


double computeAccelerationFeedforwardTorqueNm(const ParamBank& P,
                                              double ax_mps2,
                                              double vx_mps)
{
    const double m =
        P.get("model.body.m");

    const double R_tire =
        getTireRadiusM(P);

    const double I_wheel =
        P.get("model.tire.I_wheel");

    const double equivalent_mass =
        m
        + 4.0 * I_wheel / (R_tire * R_tire);

    const double torque_accel_Nm =
        equivalent_mass * ax_mps2 * R_tire;

    const double torque_res_Nm =
        computeResistanceFeedforwardTorqueNm(P, vx_mps);

    return
        torque_accel_Nm
        + torque_res_Nm;
}


// =============================================================================
//                              DRIVETRAIN LIMITS
// =============================================================================

double clampTorqueByDrivetrainLimits(const ParamBank& P,
                                     double total_torque_cmd_Nm,
                                     double vx_mps)
{
    const double R_tire =
        getTireRadiusM(P);

    const double omega_wheel =
        std::abs(vx_mps) / R_tire;

    double drive_limit_Nm =
        getStaticTotalWheelDriveLimitNm(P);

    double brake_limit_Nm =
        getStaticTotalWheelBrakeLimitNm(P);

    if (P.getBool("model.drivetrain.use_drive_power_limit"))
    {
        const double P_drive_max_W =
            P.get("model.drivetrain.P_wheel_drive_max_W");

        if (omega_wheel > 1.0)
        {
            drive_limit_Nm =
                std::min(
                    drive_limit_Nm,
                    P_drive_max_W / omega_wheel
                );
        }
    }

    if (P.getBool("model.drivetrain.use_brake_power_limit"))
    {
        const double P_brake_max_W =
            P.get("model.drivetrain.P_wheel_brake_max_W");

        if (omega_wheel > 1.0)
        {
            brake_limit_Nm =
                std::min(
                    brake_limit_Nm,
                    P_brake_max_W / omega_wheel
                );
        }
    }

    drive_limit_Nm =
        std::max(0.0, drive_limit_Nm);

    brake_limit_Nm =
        std::max(0.0, brake_limit_Nm);

    return clampLocalLong(
        total_torque_cmd_Nm,
        -brake_limit_Nm,
        drive_limit_Nm
    );
}


double getRuntimeTorqueRateLimitNmps(const ParamBank& P,
                                      const bool torque_increasing)
{
    const double normalized_rate =
        torque_increasing
            ? getNormalizedAllocatorTorqueRateUp(P)
            : getNormalizedAllocatorTorqueRateDown(P);

    const double total_limit_Nm =
        torque_increasing
            ? getStaticTotalWheelDriveLimitNm(P)
            : getStaticTotalWheelBrakeLimitNm(P);

    return
        normalized_rate
        * total_limit_Nm;
}


double rateLimitTorqueCommandNm(const ParamBank& P,
                                const double current_torque_Nm,
                                const double target_torque_Nm,
                                const double dt_s)
{
    const double delta_raw =
        target_torque_Nm - current_torque_Nm;

    if (std::abs(delta_raw) <= 1.0e-12)
    {
        return current_torque_Nm;
    }

    const bool torque_increasing =
        delta_raw > 0.0;

    const double max_rate_Nmps =
        getRuntimeTorqueRateLimitNmps(
            P,
            torque_increasing
        );

    if (!std::isfinite(max_rate_Nmps) || max_rate_Nmps <= 0.0)
    {
        return target_torque_Nm;
    }

    const double max_delta =
        max_rate_Nmps
        * std::max(dt_s, 1.0e-6);

    if (torque_increasing)
    {
        return
            current_torque_Nm
            + std::min(delta_raw, max_delta);
    }

    return
        current_torque_Nm
        + std::max(delta_raw, -max_delta);
}



// =============================================================================
//                              FINAL CAR INTERFACE
// =============================================================================

double sumWheelTorquesNm(const WheelValues& wheel_torque_Nm)
{
    return
        wheel_torque_Nm.FL
        + wheel_torque_Nm.FR
        + wheel_torque_Nm.RL
        + wheel_torque_Nm.RR;
}


double totalWheelTorqueNmToVehicleInterfaceMovement(const ParamBank& P,
                                                    double total_wheel_torque_Nm)
{
    const double gear_ratio =
        P.get("model.drivetrain.gear_ratio");

    if (std::abs(gear_ratio) <= 1.0e-9)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.drivetrain.gear_ratio must be non-zero"
        );
    }

    return total_wheel_torque_Nm / gear_ratio;
}


double wheelTorqueAllocationToVehicleInterfaceMovement(
    const ParamBank& P,
    const WheelTorqueAllocation& allocation
)
{
    return totalWheelTorqueNmToVehicleInterfaceMovement(
        P,
        allocation.allocated_total_torque_Nm
    );
}


// =============================================================================
//                              SLIP HELPERS
// =============================================================================

double slipDenominatorRegularizedNumeric(double vx,
                                          const ParamBank& P)
{
    const double v_th =
        std::max(
            1.0e-9,
            P.get("model.tire.v_threshold")
        );

    const double v_abs =
        std::abs(vx);

    const double denom_low =
        0.5 * (v_th + vx * vx / v_th);

    const double denom_high =
        v_abs;

    if (v_abs > v_th)
    {
        return std::max(denom_high, 1.0e-9);
    }

    return std::max(denom_low, 1.0e-9);
}


double regularizedKappaFromWheelLinearSpeed(double vx,
                                            double v_wheel,
                                            const ParamBank& P)
{
    const double denom =
        slipDenominatorRegularizedNumeric(vx, P);

    return
        (v_wheel - vx)
        / denom;
}


// =============================================================================
//              PER-WHEEL LOAD, SLIP, FY, ELLIPSE AND ALLOCATION
// =============================================================================

SteeringAngles computeAntiAckermannSteeringAngles(double delta_bicycle_rad,
                                                  const ParamBank& P)
{
    (void)P;

    const double d =
        delta_bicycle_rad;

    constexpr double ACKERMANN_QUAD_COEFF =
        9.298947992255e-02;

    constexpr double TOE_OFFSET_RAD =
        3.242826792680e-03;

    const double d2 =
        d * d;

    SteeringAngles out;

    out.delta_left_rad =
        d
        - ACKERMANN_QUAD_COEFF * d2
        - TOE_OFFSET_RAD;

    out.delta_right_rad =
        d
        + ACKERMANN_QUAD_COEFF * d2
        + TOE_OFFSET_RAD;

    return out;
}


WheelValues estimateWheelNormalLoadsN(const ParamBank& P,
                                      double vx_mps,
                                      double ax_mps2,
                                      double ay_mps2)
{
    double front_aero_N = 0.0;
    double rear_aero_N = 0.0;

    computeAeroDownforceN(
        P,
        vx_mps,
        front_aero_N,
        rear_aero_N
    );

    dv_control_common::WheelLoadModelParameters model;
    model.mass_kg = P.get("model.body.m");
    model.gravity_mps2 = P.get("model.body.g");
    model.lf_m = P.get("model.body.l_f");
    model.lr_m = P.get("model.body.l_r");
    model.h_cg_m = P.get("model.body.h_cg");
    model.track_front_m = getTrackWidthM(P);
    model.track_rear_m = getTrackWidthM(P);
    model.h_roll_center_front_m = P.get("model.body.h1_roll");
    model.h_roll_center_rear_m = P.get("model.body.h2_roll");
    model.lambda_elastic_front =
        P.get("model.body.lambda_phi_elastic_lateral");
    model.minimum_wheel_load_N =
        P.get("model.mass_transfer.minimum_wheel_load_N");

    const dv_control_common::WheelLoadsN shared_loads =
        dv_control_common::computeWheelLoadsN(
            model,
            ax_mps2,
            ay_mps2,
            front_aero_N,
            rear_aero_N
        );

    WheelValues out;
    out.FL = shared_loads.FL;
    out.FR = shared_loads.FR;
    out.RL = shared_loads.RL;
    out.RR = shared_loads.RR;
    return out;
}


WheelValues estimateWheelSlipAnglesLikeTireModelRad(
    const ParamBank& P,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double delta_left_rad,
    double delta_right_rad
)
{
    const double l_f =
        P.get("model.body.l_f");

    const double l_r =
        P.get("model.body.l_r");

    const double half_track =
        0.5 * getTrackWidthM(P);

    /*
        Body frame:
            x forward
            y left

        Wheel positions:
            FL: x = +l_f, y = +track/2
            FR: x = +l_f, y = -track/2
            RL: x = -l_r, y = +track/2
            RR: x = -l_r, y = -track/2

        Velocity:
            vx_w = vx - yaw_rate * y_w
            vy_w = vy + yaw_rate * x_w
    */

    const double vx_for_slip_angle_mps =
        vxForSlipAngleCalculationMps(
            vx_mps
        );

    double vx_fl =
        vx_for_slip_angle_mps
        - yaw_rate_radps * half_track;

    double vy_fl =
        vy_mps
        + yaw_rate_radps * l_f;

    double vx_fr =
        vx_for_slip_angle_mps
        + yaw_rate_radps * half_track;

    double vy_fr =
        vy_mps
        + yaw_rate_radps * l_f;

    double vx_rl =
        vx_for_slip_angle_mps
        - yaw_rate_radps * half_track;

    double vy_rl =
        vy_mps
        - yaw_rate_radps * l_r;

    double vx_rr =
        vx_for_slip_angle_mps
        + yaw_rate_radps * half_track;

    double vy_rr =
        vy_mps
        - yaw_rate_radps * l_r;

    rotateVelocityToWheelFrame(delta_left_rad, vx_fl, vy_fl);
    rotateVelocityToWheelFrame(delta_right_rad, vx_fr, vy_fr);
    rotateVelocityToWheelFrame(0.0, vx_rl, vy_rl);
    rotateVelocityToWheelFrame(0.0, vx_rr, vy_rr);

    WheelValues alpha;

    alpha.FL =
        -std::atan2(
            vy_fl,
            safeVxSignedForSlip(vx_fl)
        );

    alpha.FR =
        -std::atan2(
            vy_fr,
            safeVxSignedForSlip(vx_fr)
        );

    alpha.RL =
        -std::atan2(
            vy_rl,
            safeVxSignedForSlip(vx_rl)
        );

    alpha.RR =
        -std::atan2(
            vy_rr,
            safeVxSignedForSlip(vx_rr)
        );

    constexpr double PI =
        3.14159265358979323846;

    const double alpha_min =
        -PI / 2.0 + 0.3;

    const double alpha_max =
        PI / 2.0 - 0.3;

    alpha.FL = clampLocalLong(alpha.FL, alpha_min, alpha_max);
    alpha.FR = clampLocalLong(alpha.FR, alpha_min, alpha_max);
    alpha.RL = clampLocalLong(alpha.RL, alpha_min, alpha_max);
    alpha.RR = clampLocalLong(alpha.RR, alpha_min, alpha_max);

    return alpha;
}


double estimateLateralForceFromSlipAngleN(double alpha_rad,
                                          double Fz_N,
                                          double B,
                                          double C,
                                          double D)
{
    return
        D
        * Fz_N
        * std::sin(
            C * std::atan(B * alpha_rad)
        );
}


WheelValues estimateWheelLateralForcesFromSlipAnglesN(
    const ParamBank& P,
    const WheelValues& slip_angle_rad,
    const WheelValues& Fz_N
)
{
    const double Bf =
        P.get("model.body.Bf");

    const double Cf =
        P.get("model.body.Cf");

    const double Df =
        P.get("model.body.Df");

    const double Br =
        P.get("model.body.Br");

    const double Cr =
        P.get("model.body.Cr");

    const double Dr =
        P.get("model.body.Dr");

    WheelValues Fy;

    Fy.FL =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.FL,
            Fz_N.FL,
            Bf,
            Cf,
            Df
        );

    Fy.FR =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.FR,
            Fz_N.FR,
            Bf,
            Cf,
            Df
        );

    Fy.RL =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.RL,
            Fz_N.RL,
            Br,
            Cr,
            Dr
        );

    Fy.RR =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.RR,
            Fz_N.RR,
            Br,
            Cr,
            Dr
        );

    return Fy;
}


double computeMaxAbsFxFromWheelFrictionEllipseN(const ParamBank& P,
                                                double Fz_N,
                                                double Fy_N)
{
    const double mu_x =
        P.get("general.mu_x");

    const double mu_y =
        P.get("general.mu_y");

    const double Fz_safe =
        std::max(1.0, Fz_N);

    const double Fy_limit =
        std::max(
            1.0e-6,
            mu_y * Fz_safe
        );

    const double lateral_usage =
        std::abs(Fy_N) / Fy_limit;

    const double remaining =
        std::max(
            0.0,
            1.0 - lateral_usage * lateral_usage
        );

    return
        mu_x
        * Fz_safe
        * std::sqrt(remaining);
}


WheelTorqueLimits computeWheelTorqueLimitsNm(const ParamBank& P,
                                             double vx_mps,
                                             double vy_mps,
                                             double yaw_rate_radps,
                                             double ax_mps2,
                                             double ay_mps2,
                                             double delta_bicycle_rad)
{
    WheelTorqueLimits limits;

    const double R_tire =
        getTireRadiusM(P);

    const SteeringAngles steering_angles =
        computeAntiAckermannSteeringAngles(
            delta_bicycle_rad,
            P
        );

    limits.Fz_N =
        estimateWheelNormalLoadsN(
            P,
            vx_mps,
            ax_mps2,
            ay_mps2
        );

    limits.slip_angle_rad =
        estimateWheelSlipAnglesLikeTireModelRad(
            P,
            vx_mps,
            vy_mps,
            yaw_rate_radps,
            steering_angles.delta_left_rad,
            steering_angles.delta_right_rad
        );

    limits.Fy_N =
        estimateWheelLateralForcesFromSlipAnglesN(
            P,
            limits.slip_angle_rad,
            limits.Fz_N
        );

    limits.max_abs_Fx_from_ellipse_N.FL =
        computeMaxAbsFxFromWheelFrictionEllipseN(P, limits.Fz_N.FL, limits.Fy_N.FL);

    limits.max_abs_Fx_from_ellipse_N.FR =
        computeMaxAbsFxFromWheelFrictionEllipseN(P, limits.Fz_N.FR, limits.Fy_N.FR);

    limits.max_abs_Fx_from_ellipse_N.RL =
        computeMaxAbsFxFromWheelFrictionEllipseN(P, limits.Fz_N.RL, limits.Fy_N.RL);

    limits.max_abs_Fx_from_ellipse_N.RR =
        computeMaxAbsFxFromWheelFrictionEllipseN(P, limits.Fz_N.RR, limits.Fy_N.RR);

    limits.max_abs_torque_from_ellipse_Nm.FL =
        limits.max_abs_Fx_from_ellipse_N.FL * R_tire;

    limits.max_abs_torque_from_ellipse_Nm.FR =
        limits.max_abs_Fx_from_ellipse_N.FR * R_tire;

    limits.max_abs_torque_from_ellipse_Nm.RL =
        limits.max_abs_Fx_from_ellipse_N.RL * R_tire;

    limits.max_abs_torque_from_ellipse_Nm.RR =
        limits.max_abs_Fx_from_ellipse_N.RR * R_tire;

    WheelValues mechanical_limit;

    mechanical_limit.FL = getMaxWheelTorqueNm(P);
    mechanical_limit.FR = getMaxWheelTorqueNm(P);
    mechanical_limit.RL = getMaxWheelTorqueNm(P);
    mechanical_limit.RR = getMaxWheelTorqueNm(P);

    constexpr double FRICTION_ELLIPSE_MIN_SPEED_MPS =
        3.0;

    const bool friction_ellipse_speed_active =
        std::abs(vx_mps) > FRICTION_ELLIPSE_MIN_SPEED_MPS;

    /*
        Mechanical torque limits are always active.

        Friction ellipse is active only above 3 m/s real vehicle speed.
        Below that, slip/Fy estimation is too noisy for launch/low-speed
        operation and can incorrectly reduce the available torque to zero.
    */
    if (friction_ellipse_speed_active)
    {
        limits.max_abs_torque_after_mech_Nm =
            minWheelValues(
                limits.max_abs_torque_from_ellipse_Nm,
                mechanical_limit
            );
    }
    else
    {
        limits.max_abs_torque_after_mech_Nm =
            mechanical_limit;
    }

    const double total_positive =
        clampTorqueByDrivetrainLimits(
            P,
            getMaxTotalWheelTorqueNm(P),
            vx_mps
        );

    const double total_negative =
        clampTorqueByDrivetrainLimits(
            P,
            -getMaxTotalWheelTorqueNm(P),
            vx_mps
        );

    limits.max_abs_total_torque_after_power_Nm =
        std::min(
            std::abs(total_positive),
            std::abs(total_negative)
        );

    return limits;
}


double computeMaxAvailableSignedTorqueNm(
    const ParamBank& P,
    const bool drive,
    const double vx_mps,
    const double vy_mps,
    const double yaw_rate_radps,
    const double ax_mps2,
    const double ay_mps2,
    const double delta_bicycle_rad
)
{
    const double sign =
        drive ? 1.0 : -1.0;

    const double total_drivetrain_limit_Nm =
        std::abs(
            clampTorqueByDrivetrainLimits(
                P,
                sign * getMaxTotalWheelTorqueNm(P),
                vx_mps
            )
        );

    const double total_friction_cap_Nm =
        computeTotalLongitudinalFrictionCapNm(
            P,
            vx_mps,
            ax_mps2,
            ay_mps2
        );

    const double total_available_limit_Nm =
        std::min(
            total_drivetrain_limit_Nm,
            total_friction_cap_Nm
        );

    if (!std::isfinite(total_available_limit_Nm) ||
        total_available_limit_Nm <= 1.0e-9)
    {
        return 0.0;
    }

    const WheelValues Fz_allocation_N =
        estimateWheelNormalLoadsN(
            P,
            vx_mps,
            ax_mps2,
            0.0
        );

    const double Fz_allocation_sum_N =
        std::max(
            1.0,
            std::max(0.0, Fz_allocation_N.FL)
            + std::max(0.0, Fz_allocation_N.FR)
            + std::max(0.0, Fz_allocation_N.RL)
            + std::max(0.0, Fz_allocation_N.RR)
        );

    WheelValues requested_abs_torque_Nm;

    requested_abs_torque_Nm.FL =
        total_available_limit_Nm
        * std::max(0.0, Fz_allocation_N.FL)
        / Fz_allocation_sum_N;

    requested_abs_torque_Nm.FR =
        total_available_limit_Nm
        * std::max(0.0, Fz_allocation_N.FR)
        / Fz_allocation_sum_N;

    requested_abs_torque_Nm.RL =
        total_available_limit_Nm
        * std::max(0.0, Fz_allocation_N.RL)
        / Fz_allocation_sum_N;

    requested_abs_torque_Nm.RR =
        total_available_limit_Nm
        * std::max(0.0, Fz_allocation_N.RR)
        / Fz_allocation_sum_N;

    WheelTorqueLimits limits =
        computeWheelTorqueLimitsNm(
            P,
            vx_mps,
            vy_mps,
            yaw_rate_radps,
            ax_mps2,
            ay_mps2,
            delta_bicycle_rad
        );

    const double per_wheel_engine_limit_Nm =
        drive
            ? getWheelDriveLimitFromEngineNm(P)
            : getWheelBrakeLimitFromEngineNm(P);

    WheelValues max_abs_torque_Nm;

    max_abs_torque_Nm.FL =
        std::min(limits.max_abs_torque_after_mech_Nm.FL, per_wheel_engine_limit_Nm);

    max_abs_torque_Nm.FR =
        std::min(limits.max_abs_torque_after_mech_Nm.FR, per_wheel_engine_limit_Nm);

    max_abs_torque_Nm.RL =
        std::min(limits.max_abs_torque_after_mech_Nm.RL, per_wheel_engine_limit_Nm);

    max_abs_torque_Nm.RR =
        std::min(limits.max_abs_torque_after_mech_Nm.RR, per_wheel_engine_limit_Nm);

    const double wheel_scale =
        computeGlobalTorqueScaleFromWheelLimits(
            requested_abs_torque_Nm,
            max_abs_torque_Nm
        );

    return
        sign
        * total_available_limit_Nm
        * wheel_scale;
}


double computeMaxAvailableDriveTorqueNm(
    const ParamBank& P,
    const double vx_mps,
    const double vy_mps,
    const double yaw_rate_radps,
    const double ax_mps2,
    const double ay_mps2,
    const double delta_bicycle_rad
)
{
    return computeMaxAvailableSignedTorqueNm(
        P,
        true,
        vx_mps,
        vy_mps,
        yaw_rate_radps,
        ax_mps2,
        ay_mps2,
        delta_bicycle_rad
    );
}


double computeMaxAvailableBrakeTorqueNm(
    const ParamBank& P,
    const double vx_mps,
    const double vy_mps,
    const double yaw_rate_radps,
    const double ax_mps2,
    const double ay_mps2,
    const double delta_bicycle_rad
)
{
    return computeMaxAvailableSignedTorqueNm(
        P,
        false,
        vx_mps,
        vy_mps,
        yaw_rate_radps,
        ax_mps2,
        ay_mps2,
        delta_bicycle_rad
    );
}


double computeFinalPerWheelEngineLimitScale(
    const ParamBank& P,
    const WheelValues& signed_wheel_torque_Nm
)
{
    double scale =
        1.0;

    const double drive_limit_Nm =
        getWheelDriveLimitFromEngineNm(P);

    const double brake_limit_Nm =
        getWheelBrakeLimitFromEngineNm(P);

    const auto update_scale =
        [&](const double torque_Nm)
        {
            const double requested_abs =
                std::abs(torque_Nm);

            if (requested_abs <= 1.0e-9)
            {
                return;
            }

            const double limit_Nm =
                torque_Nm >= 0.0
                    ? drive_limit_Nm
                    : brake_limit_Nm;

            scale =
                std::min(
                    scale,
                    limit_Nm / requested_abs
                );
        };

    update_scale(signed_wheel_torque_Nm.FL);
    update_scale(signed_wheel_torque_Nm.FR);
    update_scale(signed_wheel_torque_Nm.RL);
    update_scale(signed_wheel_torque_Nm.RR);

    return clampLocalLong(
        scale,
        0.0,
        1.0
    );
}


double estimateRequiredBrakeDistanceM(
    const ParamBank& P,
    const double initial_vx_mps,
    const double initial_ax_mps2,
    const double initial_total_torque_Nm
)
{
    const double m =
        P.get("model.body.m");

    const double R_tire =
        getTireRadiusM(P);

    if (!std::isfinite(m) || m <= 1.0e-9)
    {
        return 0.0;
    }

    double vx_mps =
        std::max(0.0, initial_vx_mps);

    const double target_speed_mps =
        std::max(0.0, P.get("general.coast_below_speed_mps"));

    if (vx_mps <= target_speed_mps)
    {
        return 0.0;
    }

    const double control_dt_s =
        getControlDtForAllocator(P);

    const double dt_s =
        std::min(
            0.01,
            std::max(0.001, control_dt_s)
        );

    constexpr int MAX_STEPS =
        30000;

    double s_m =
        0.0;

    double torque_cmd_Nm =
        clampTorqueByDrivetrainLimits(
            P,
            initial_total_torque_Nm,
            vx_mps
        );

    /*
        Important convention for brake-distance prediction:

        The available brake torque at step k is computed from the previous
        step acceleration estimate ax_prev_mps2, not from an algebraic fixed
        point inside the same step.

        This avoids the loop:
            brake torque -> ax -> Fz -> available brake torque
        being solved algebraically at one sample.

        For this prediction the car is assumed to be perfectly straight:
            vy = 0, yaw_rate = 0, ay = 0, delta = 0.

        Therefore the available brake torque depends only on:
            vx_mps and ax_prev_mps2.
    */
    double ax_prev_mps2 =
        std::isfinite(initial_ax_mps2)
            ? initial_ax_mps2
            : 0.0;

    for (int k = 0; k < MAX_STEPS; ++k)
    {
        if (vx_mps <= target_speed_mps)
        {
            break;
        }

        const double max_brake_torque_Nm =
            computeMaxAvailableBrakeTorqueNm(
                P,
                vx_mps,
                0.0,              // vy: straight-line prediction
                0.0,              // yaw_rate: straight-line prediction
                ax_prev_mps2,     // previous-step ax, no algebraic loop
                0.0,              // ay ignored for brake-distance prediction
                0.0               // steering: straight-line prediction
            );

        torque_cmd_Nm =
            rateLimitTorqueCommandNm(
                P,
                torque_cmd_Nm,
                max_brake_torque_Nm,
                dt_s
            );

        torque_cmd_Nm =
            clampTorqueByDrivetrainLimits(
                P,
                torque_cmd_Nm,
                vx_mps
            );

        const double Fx_tire_N =
            torque_cmd_Nm / R_tire;

        const double F_res_N =
            computeResistanceFeedforwardTorqueNm(P, vx_mps) / R_tire;

        const double ax_new_mps2 =
            (Fx_tire_N - F_res_N) / m;

        if (!std::isfinite(ax_new_mps2))
        {
            break;
        }

        const double vx_next_mps =
            std::max(0.0, vx_mps + ax_new_mps2 * dt_s);

        s_m +=
            0.5
            * (vx_mps + vx_next_mps)
            * dt_s;

        vx_mps =
            vx_next_mps;

        ax_prev_mps2 =
            ax_new_mps2;
    }

    if (!std::isfinite(s_m) || s_m < 0.0)
    {
        return 0.0;
    }

    return s_m;
}

bool shouldStartBrakeByDistance(
    const ParamBank& P,
    const double s_m,
    const double vx_mps,
    const double ax_mps2,
    const double current_total_torque_Nm,
    double* required_brake_distance_m_out,
    double* distance_available_for_brake_m_out
)
{
    const double s_total_m =
        P.get("general.s_total");

    const double brake_margin_m =
        P.get("general.brake_margin_m");

    const double distance_available_for_brake_m =
        s_total_m
        - s_m
        - brake_margin_m;

    const double required_brake_distance_m =
        estimateRequiredBrakeDistanceM(
            P,
            vx_mps,
            ax_mps2,
            current_total_torque_Nm
        );

    if (required_brake_distance_m_out != nullptr)
    {
        *required_brake_distance_m_out =
            required_brake_distance_m;
    }

    if (distance_available_for_brake_m_out != nullptr)
    {
        *distance_available_for_brake_m_out =
            distance_available_for_brake_m;
    }

    return
        distance_available_for_brake_m
        <= required_brake_distance_m;
}


WheelTorqueAllocation allocateWheelTorqueByNormalLoadNm(
    const ParamBank& P,
    double requested_total_torque_Nm,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double ax_mps2,
    double ay_mps2,
    double delta_bicycle_rad,
    double previous_total_limited_torque_Nm,
    bool previous_total_limited_torque_valid,
    bool bypass_torque_rate_limiter
)
{
    WheelTorqueAllocation out;

    out.requested_total_torque_Nm =
        requested_total_torque_Nm;

    const double drivetrain_limited_total_torque_Nm =
        clampTorqueByDrivetrainLimits(
            P,
            requested_total_torque_Nm,
            vx_mps
        );

    const double total_friction_cap_Nm =
        computeTotalLongitudinalFrictionCapNm(
            P,
            vx_mps,
            ax_mps2,
            ay_mps2
        );

    out.drivetrain_limited_total_torque_Nm =
        clampLocalLong(
            drivetrain_limited_total_torque_Nm,
            -total_friction_cap_Nm,
            total_friction_cap_Nm
        );

    out.limited_by_drivetrain =
        std::abs(
            out.drivetrain_limited_total_torque_Nm
            - requested_total_torque_Nm
        ) > 1.0e-9;

    out.previous_total_limited_torque_Nm =
        previous_total_limited_torque_valid
            ? previous_total_limited_torque_Nm
            : 0.0;

    out.normalized_M_dot_up =
        getNormalizedAllocatorTorqueRateUp(P);

    out.normalized_M_dot_down =
        getNormalizedAllocatorTorqueRateDown(P);

    const double allocator_dt_s =
        getControlDtForAllocator(P);

    /*
        M_dot runtime values are normalized total-torque rates [1/s].

        No division by 4:
            this allocator applies one global scale to all wheels,
            so the rate budget is for the whole available total torque.
    */
    out.max_total_torque_step_up_Nm =
        out.normalized_M_dot_up
        * getStaticTotalWheelDriveLimitNm(P)
        * allocator_dt_s;

    out.max_total_torque_step_down_Nm =
        out.normalized_M_dot_down
        * getStaticTotalWheelBrakeLimitNm(P)
        * allocator_dt_s;

    out.torque_rate_limiter_bypassed =
        bypass_torque_rate_limiter;

    double rate_limited_total_torque_Nm =
        out.drivetrain_limited_total_torque_Nm;

    bool input_torque_rate_limited =
        false;

    if (!out.torque_rate_limiter_bypassed)
    {
        const double allowed_low_Nm =
            out.previous_total_limited_torque_Nm
            - out.max_total_torque_step_down_Nm;

        const double allowed_high_Nm =
            out.previous_total_limited_torque_Nm
            + out.max_total_torque_step_up_Nm;

        rate_limited_total_torque_Nm =
            clampLocalLong(
                rate_limited_total_torque_Nm,
                allowed_low_Nm,
                allowed_high_Nm
            );

        input_torque_rate_limited =
            std::abs(
                rate_limited_total_torque_Nm
                - out.drivetrain_limited_total_torque_Nm
            ) > 1.0e-6;
    }

    out.steering_angles =
        computeAntiAckermannSteeringAngles(
            delta_bicycle_rad,
            P
        );

    out.limits =
        computeWheelTorqueLimitsNm(
            P,
            vx_mps,
            vy_mps,
            yaw_rate_radps,
            ax_mps2,
            ay_mps2,
            delta_bicycle_rad
        );

    const double sign =
        rate_limited_total_torque_Nm >= 0.0
        ? 1.0
        : -1.0;

    const double requested_abs =
        std::abs(
            rate_limited_total_torque_Nm
        );

    /*
        Base allocation for ACC classic:

            - proportional to normal loads,
            - but these allocation loads intentionally ignore ay,
            - they include static load, aero and longitudinal transfer from ax.

        This keeps launch/acceleration neutral left/right even if measured ay is
        noisy, while still moving torque rearward under positive ax.

        The limit/scale calculation below still uses out.limits, which was
        computed with the full measured ay_mps2.
    */

    const WheelValues Fz_allocation_N =
        estimateWheelNormalLoadsN(
            P,
            vx_mps,
            ax_mps2,
            0.0
        );

    const double Fz_allocation_sum_N =
        std::max(
            1.0,
            std::max(0.0, Fz_allocation_N.FL)
            + std::max(0.0, Fz_allocation_N.FR)
            + std::max(0.0, Fz_allocation_N.RL)
            + std::max(0.0, Fz_allocation_N.RR)
        );

    out.share.FL =
        std::max(0.0, Fz_allocation_N.FL) / Fz_allocation_sum_N;

    out.share.FR =
        std::max(0.0, Fz_allocation_N.FR) / Fz_allocation_sum_N;

    out.share.RL =
        std::max(0.0, Fz_allocation_N.RL) / Fz_allocation_sum_N;

    out.share.RR =
        std::max(0.0, Fz_allocation_N.RR) / Fz_allocation_sum_N;

    out.abs_torque_Nm.FL =
        requested_abs * out.share.FL;

    out.abs_torque_Nm.FR =
        requested_abs * out.share.FR;

    out.abs_torque_Nm.RL =
        requested_abs * out.share.RL;

    out.abs_torque_Nm.RR =
        requested_abs * out.share.RR;

    const WheelValues abs_torque_before_global_scale_Nm =
        out.abs_torque_Nm;

    const WheelValues lateral_usage =
        computeLateralUsageFromFyAndFz(
            P,
            out.limits.Fy_N,
            out.limits.Fz_N
        );

    const WheelValues scale_candidates =
        computeWheelScaleCandidates(
            abs_torque_before_global_scale_Nm,
            out.limits.max_abs_torque_after_mech_Nm
        );

    /*
        No saturation recovery.

        If one wheel exceeds its ellipse/mechanical limit, scale the whole
        vector globally. This preserves the base allocation proportions and
        does not generate accidental torque vectoring.
    */

    out.hard_limit_scale =
        computeGlobalTorqueScaleFromWheelLimits(
            out.abs_torque_Nm,
            out.limits.max_abs_torque_after_mech_Nm
        );

    if (out.torque_rate_limiter_bypassed)
    {
        out.total_torque_rate_scale_min =
            0.0;

        out.total_torque_rate_scale_max =
            std::numeric_limits<double>::infinity();

        out.total_torque_rate_limited_scale =
            1.0;

        out.limited_by_rate =
            false;

        out.rate_forced_over_friction_or_mech =
            false;
    }
    else
    {
        const double target_total_before_global_scale_Nm =
            sign * sumWheelTorquesNm(
                abs_torque_before_global_scale_Nm
            );

        out.total_torque_rate_scale_min =
            computeTotalTorqueRateScaleMin(
                out.previous_total_limited_torque_Nm,
                target_total_before_global_scale_Nm,
                out.max_total_torque_step_up_Nm,
                out.max_total_torque_step_down_Nm
            );

        out.total_torque_rate_scale_max =
            computeTotalTorqueRateScaleMax(
                out.previous_total_limited_torque_Nm,
                target_total_before_global_scale_Nm,
                out.max_total_torque_step_up_Nm,
                out.max_total_torque_step_down_Nm
            );

        out.total_torque_rate_limited_scale =
            clampLocalLong(
                1.0,
                out.total_torque_rate_scale_min,
                out.total_torque_rate_scale_max
            );

        const bool final_scale_rate_limited =
            std::abs(out.total_torque_rate_limited_scale - 1.0) > 1.0e-6;

        out.limited_by_rate =
            input_torque_rate_limited
            || final_scale_rate_limited;
    }

    /*
        RATE HAS PRIORITY OVER FRICTION / MECHANICAL LIMIT.

        Final scale is clamped into the rate-allowed interval.  If friction
        would reduce torque faster than the rate limiter allows, the scale is
        raised and rate_forced_over_friction_or_mech becomes true.
    */
    out.global_scale =
        clampLocalLong(
            out.hard_limit_scale,
            out.total_torque_rate_scale_min,
            out.total_torque_rate_scale_max
        );

    out.rate_forced_over_friction_or_mech =
        false;

    if (!out.torque_rate_limiter_bypassed &&
        out.global_scale > out.hard_limit_scale + 1.0e-9)
    {
        out.rate_forced_over_friction_or_mech =
            true;
    }

    out.abs_torque_Nm =
        scaleWheelValues(
            out.abs_torque_Nm,
            out.global_scale
        );

    const double final_per_wheel_engine_limit_Nm =
        sign >= 0.0
            ? getWheelDriveLimitFromEngineNm(P)
            : getWheelBrakeLimitFromEngineNm(P);

    WheelValues final_per_wheel_limit_Nm;

    final_per_wheel_limit_Nm.FL = final_per_wheel_engine_limit_Nm;
    final_per_wheel_limit_Nm.FR = final_per_wheel_engine_limit_Nm;
    final_per_wheel_limit_Nm.RL = final_per_wheel_engine_limit_Nm;
    final_per_wheel_limit_Nm.RR = final_per_wheel_engine_limit_Nm;

    const double final_engine_per_wheel_scale =
        computeGlobalTorqueScaleFromWheelLimits(
            out.abs_torque_Nm,
            final_per_wheel_limit_Nm
        );

    out.abs_torque_Nm =
        scaleWheelValues(
            out.abs_torque_Nm,
            final_engine_per_wheel_scale
        );

    out.global_scale *=
        final_engine_per_wheel_scale;

    out.hard_limit_scale *=
        final_engine_per_wheel_scale;

    out.limited_by_friction_or_mech =
        out.hard_limit_scale < 0.999999 ||
        final_engine_per_wheel_scale < 0.999999;

    out.torque_Nm.FL =
        sign * out.abs_torque_Nm.FL;

    out.torque_Nm.FR =
        sign * out.abs_torque_Nm.FR;

    out.torque_Nm.RL =
        sign * out.abs_torque_Nm.RL;

    out.torque_Nm.RR =
        sign * out.abs_torque_Nm.RR;

    out.allocated_total_torque_Nm =
        sumWheelTorquesNm(
            out.torque_Nm
        );

    if (P.getBool("general.print_console_debug_info"))
    {
        ROS_WARN_STREAM_THROTTLE(
            0.25,
            "\n[ACC ALLOCATOR DEBUG]"
            << "\n  input:"
            << " requested_total=" << requested_total_torque_Nm
            << " drivetrain_limited_or_mu_capped=" << out.drivetrain_limited_total_torque_Nm
            << " total_friction_cap_Nm=" << total_friction_cap_Nm
            << " rate_limited_total=" << rate_limited_total_torque_Nm
            << " sign=" << sign
            << " vx=" << vx_mps
            << " vx_for_slip=" << vxForSlipAngleCalculationMps(vx_mps)
            << " vy=" << vy_mps
            << " yaw_rate=" << yaw_rate_radps
            << " ax=" << ax_mps2
            << " ay=" << ay_mps2
            << " ay_for_allocation=0"
            << " delta_bicycle=" << delta_bicycle_rad
            << "\n  steering:"
            << " delta_left=" << out.steering_angles.delta_left_rad
            << " delta_right=" << out.steering_angles.delta_right_rad
            << "\n  Fz_allocation_N_ay0=" << wheelValuesToString(Fz_allocation_N)
            << "\n  Fz_limit_N_full_transfer=" << wheelValuesToString(out.limits.Fz_N)
            << "\n  slip_angle_deg=" << wheelValuesDegToString(out.limits.slip_angle_rad)
            << "\n  Fy_N=" << wheelValuesToString(out.limits.Fy_N)
            << "\n  lateral_usage_abs_Fy_over_muFz=" << wheelValuesToString(lateral_usage)
            << "\n  max_Fx_ellipse_N=" << wheelValuesToString(out.limits.max_abs_Fx_from_ellipse_N)
            << "\n  max_torque_ellipse_Nm=" << wheelValuesToString(out.limits.max_abs_torque_from_ellipse_Nm)
            << "\n  max_torque_after_mech_Nm=" << wheelValuesToString(out.limits.max_abs_torque_after_mech_Nm)
            << "\n  requested_abs_before_scale_Nm=" << wheelValuesToString(abs_torque_before_global_scale_Nm)
            << "\n  scale_candidates=max/requested=" << wheelValuesToString(scale_candidates)
            << "\n  previous_total_limited_Nm=" << out.previous_total_limited_torque_Nm
            << "\n  global_scale=" << out.global_scale
            << " hard_scale=" << out.hard_limit_scale
            << " rate_scale=" << out.total_torque_rate_limited_scale
            << " rate_scale_min=" << out.total_torque_rate_scale_min
            << " rate_scale_max=" << out.total_torque_rate_scale_max
            << " limited_by_rate=" << out.limited_by_rate
            << " rate_forced_over_friction_or_mech=" << out.rate_forced_over_friction_or_mech
            << " rate_bypass=" << out.torque_rate_limiter_bypassed
            << " Mdot_up_norm=" << out.normalized_M_dot_up
            << " Mdot_down_norm=" << out.normalized_M_dot_down
            << " max_total_step_up_Nm=" << out.max_total_torque_step_up_Nm
            << " max_total_step_down_Nm=" << out.max_total_torque_step_down_Nm
            << " final_engine_per_wheel_scale=" << final_engine_per_wheel_scale
            << " final_engine_per_wheel_limit_Nm=" << final_per_wheel_engine_limit_Nm
            << " limited_by_friction_or_mech=" << out.limited_by_friction_or_mech
            << " limited_by_drivetrain=" << out.limited_by_drivetrain
            << "\n  abs_after_scale_Nm=" << wheelValuesToString(out.abs_torque_Nm)
            << "\n  output_torque_Nm=" << wheelValuesToString(out.torque_Nm)
            << " allocated_total=" << out.allocated_total_torque_Nm
        );
    }

    return out;
}

} // namespace acc_launch_control
