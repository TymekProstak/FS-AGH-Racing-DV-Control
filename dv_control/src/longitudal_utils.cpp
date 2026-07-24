#include "longitudal_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dv_control
{

namespace
{

inline double safeAbs(const double x, const double eps = 1.0e-9)
{
    return std::max(std::abs(x), eps);
}

inline double getTireRadiusM(const ParamBank& P)
{
    const double R_tire =
        P.get("model.tire.R_tire");

    if (!std::isfinite(R_tire) || R_tire <= 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.tire.R_tire must be finite and > 0"
        );
    }

    return R_tire;
}

inline double getGearRatio(const ParamBank& P)
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

inline double getConfiguredTotalWheelDriveEnvelopeNm(const ParamBank& P)
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

inline double getConfiguredTotalWheelBrakeEnvelopeNm(const ParamBank& P)
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

inline double getTotalWheelDriveLimitFromEngineUsageNm(const ParamBank& P)
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

inline double getTotalWheelBrakeLimitFromEngineUsageNm(const ParamBank& P)
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

inline double getStaticTotalWheelDriveLimitNm(const ParamBank& P)
{
    return std::min(
        getConfiguredTotalWheelDriveEnvelopeNm(P),
        getTotalWheelDriveLimitFromEngineUsageNm(P)
    );
}

inline double getStaticTotalWheelBrakeLimitNm(const ParamBank& P)
{
    return std::min(
        getConfiguredTotalWheelBrakeEnvelopeNm(P),
        getTotalWheelBrakeLimitFromEngineUsageNm(P)
    );
}

inline bool useDrivePowerLimit(const ParamBank& P)
{
    return P.getBool("model.drivetrain.use_drive_power_limit");
}

inline bool useBrakePowerLimit(const ParamBank& P)
{
    return P.getBool("model.drivetrain.use_brake_power_limit");
}

inline double getDrivePowerLimitW(const ParamBank& P)
{
    return P.get("model.drivetrain.P_wheel_drive_max_W");
}

inline double getBrakePowerLimitW(const ParamBank& P)
{
    return P.get("model.drivetrain.P_wheel_brake_max_W");
}

inline double computeTotalAeroDownforceN(const ParamBank& P,
                                         const double vx_mps)
{
    if (!P.getBool("model.body.use_aero"))
    {
        return 0.0;
    }

    const double vx =
        std::isfinite(vx_mps) ? std::abs(vx_mps) : 0.0;

    const double vx2 =
        vx * vx;

    return
        (
            P.get("model.body.Cl1")
            + P.get("model.body.Cl2")
        )
        * vx2;
}

} // namespace

// =============================================================================
//                              PID CONFIG
// =============================================================================

PIDParams makeSpeedPidParams(const ParamBank& P)
{
    PIDParams params;

    params.Kp =
        P.get("speed_pid.Kp");

    params.Ki =
        P.get("speed_pid.Ki");

    params.Kd =
        P.get("speed_pid.Kd");

    params.saturation_lower =
        P.get("speed_pid.saturation_lower_Nm");

    params.saturation_upper =
        P.get("speed_pid.saturation_upper_Nm");

    params.anti_windup_gain =
        P.get("speed_pid.anti_windup_gain");

    params.leak_time_scale =
        P.get("speed_pid.leak_time_scale");

    params.use_output_rate_limit =
        P.getBool("speed_pid.use_output_rate_limit");

    params.output_rate_up =
        P.get("speed_pid.output_rate_up");

    params.output_rate_down =
        P.get("speed_pid.output_rate_down");

    return params;
}

// =============================================================================
//                              RESISTANCE FEEDFORWARD
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

    return
        F_res
        * R_tire;
}

double computeAccelerationFeedforwardTorqueNm(
    const ParamBank& P,
    double ax_mps2,
    double vx_mps)
{
    const double m =
        P.get("model.body.m");

    const double R_tire =
        getTireRadiusM(P);

    const double I_wheel =
        P.get("model.body.wheel_rot_inertia_kgm2");

    const double num_wheels =
        P.get("model.body.num_rotating_wheels");

    if (!std::isfinite(I_wheel) || I_wheel < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.body.wheel_rot_inertia_kgm2 "
            "must be finite and >= 0"
        );
    }

    if (!std::isfinite(num_wheels) || num_wheels < 0.0)
    {
        throw std::runtime_error(
            "[longitudal_utils] model.body.num_rotating_wheels "
            "must be finite and >= 0"
        );
    }

    const double equivalent_mass =
        m
        + num_wheels * I_wheel
            / (R_tire * R_tire);

    const double resistance_torque_Nm =
        ax_mps2 > 0.0
            ? computeResistanceFeedforwardTorqueNm(P, vx_mps)
            : 0.0;

    return
        equivalent_mass * ax_mps2 * R_tire
        + resistance_torque_Nm;
}

// =============================================================================
//                              DRIVETRAIN LIMITS
// =============================================================================

double clampTorqueByDrivetrainLimits(const ParamBank& P,
                                     double torque_cmd_Nm,
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

    if (useDrivePowerLimit(P) && omega_wheel > 1.0)
    {
        const double P_drive_max_W =
            getDrivePowerLimitW(P);

        if (!std::isfinite(P_drive_max_W) || P_drive_max_W < 0.0)
        {
            throw std::runtime_error(
                "[longitudal_utils] model.drivetrain.P_wheel_drive_max_W must be finite and >= 0"
            );
        }

        drive_limit_Nm =
            std::min(
                drive_limit_Nm,
                P_drive_max_W / omega_wheel
            );
    }

    if (useBrakePowerLimit(P) && omega_wheel > 1.0)
    {
        const double P_brake_max_W =
            getBrakePowerLimitW(P);

        if (!std::isfinite(P_brake_max_W) || P_brake_max_W < 0.0)
        {
            throw std::runtime_error(
                "[longitudal_utils] model.drivetrain.P_wheel_brake_max_W must be finite and >= 0"
            );
        }

        brake_limit_Nm =
            std::min(
                brake_limit_Nm,
                P_brake_max_W / omega_wheel
            );
    }

    drive_limit_Nm =
        std::max(0.0, drive_limit_Nm);

    brake_limit_Nm =
        std::max(0.0, brake_limit_Nm);

    return clampLocal(
        torque_cmd_Nm,
        -brake_limit_Nm,
        drive_limit_Nm
    );
}

// =============================================================================
//                              TORQUE TO PERCENT
// =============================================================================

double torqueNmToSignedPercent(const ParamBank& P,
                               double torque_Nm)
{
    /*
        Debug conversion only. The internal pipeline is Nm-only. The larger
        static total wheel-side limit is the 100% reference.
    */
    const double total_reference_Nm =
        std::max(
            getStaticTotalWheelDriveLimitNm(P),
            getStaticTotalWheelBrakeLimitNm(P)
        );

    return
        100.0
        * torque_Nm
        / safeAbs(total_reference_Nm);
}

} // namespace dv_control
