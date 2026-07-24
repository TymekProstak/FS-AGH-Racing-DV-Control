#include "longitudinal_utils.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace skidpad_control
{

// =============================================================================
//                              PID CONFIG
// =============================================================================

PIDParams makeSpeedPidParams(const ParamBank& P)
{
    PIDParams params;

    params.Kp = P.get("speed_pid.Kp");
    params.Ki = P.get("speed_pid.Ki");
    params.Kd = P.get("speed_pid.Kd");

    params.saturation_lower = P.get("speed_pid.saturation_lower_Nm");
    params.saturation_upper = P.get("speed_pid.saturation_upper_Nm");

    params.anti_windup_gain = P.get("speed_pid.anti_windup_gain");
    params.leak_time_scale = P.get("speed_pid.leak_time_scale");

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

    const bool use_drag =
        P.getBool("model.resistance.use_drag");

    const bool use_rolling_resistance =
        P.getBool("model.resistance.use_rolling_resistance");

    const bool use_aero =
        P.getBool("model.resistance.use_aero");

    const double Cd =
        P.get("model.resistance.Cd");

    const double Crr =
        P.get("model.resistance.Crr");

    const double Cl1 =
        P.get("model.resistance.Cl1");

    const double Cl2 =
        P.get("model.resistance.Cl2");

    const double extra_constant_N =
        P.get("model.resistance.extra_constant_N");

    const double extra_linear_N_per_mps =
        P.get("model.resistance.extra_linear_N_per_mps");

    const double R_tire =
        P.get("model.drivetrain.R_tire");

    const double v =
        std::abs(vx_mps);

    double F_res = 0.0;

    if (use_rolling_resistance)
    {
        double Fz_total =
            m * g;

        if (use_aero)
        {
            Fz_total +=
                (Cl1 + Cl2) * v * v;
        }

        F_res +=
            Crr * Fz_total;
    }

    if (use_drag)
    {
        /*
            Cd jest parametrem lumped:
                F_drag = Cd * v^2
        */
        F_res +=
            Cd * v * v;
    }

    /*
        Dodatkowy zmierzony / fitowany opór:
            F_extra = const + linear * |v|
    */
    F_res +=
        extra_constant_N
        + extra_linear_N_per_mps * v;

    return F_res * R_tire;
}


// =============================================================================
//                              DRIVETRAIN LIMITS
// =============================================================================

double clampTorqueByDrivetrainLimits(const ParamBank& P,
                                     double torque_cmd_Nm,
                                     double vx_mps)
{
    const double R_tire =
        std::max(P.get("model.drivetrain.R_tire"), 1.0e-6);

    const double omega_wheel =
        std::abs(vx_mps) / R_tire;

    /*
        torque_cmd_Nm is TOTAL wheel-side torque command.

        The physical command limit for the controller should not use the full
        installed wheel torque capability directly.  It uses the configured
        engine-side usage limits and gearbox ratio:

            drive_limit_total_wheel =
                M_engine_max_Nm_usage_drive_total * gear_ratio

            brake_limit_total_wheel =
                M_engine_max_Nm_usage_brake_total * gear_ratio
    */

    const double gear_ratio =
        P.get("model.drivetrain.gear_ratio");

    double drive_limit_Nm =
        std::abs(
            P.get("model.drivetrain.M_engine_max_Nm_usage_drive_total")
            * gear_ratio
        );

    double brake_limit_Nm =
        std::abs(
            P.get("model.drivetrain.M_engine_max_Nm_usage_brake_total")
            * gear_ratio
        );

    /*
        Total power limit on drivetrain.  Since torque_cmd_Nm is total
        wheel-side torque and omega_wheel is wheel angular speed:

            P_total = T_total_wheel * omega_wheel
    */

    const bool use_drive_power_limit =
        P.getBool("model.drivetrain.use_drive_power_limit");

    const bool use_brake_power_limit =
        P.getBool("model.drivetrain.use_brake_power_limit");

    if (use_drive_power_limit && omega_wheel > 1.0)
    {
        drive_limit_Nm =
            std::min(
                drive_limit_Nm,
                P.get("model.drivetrain.P_wheel_drive_max_W") /
                omega_wheel
            );
    }

    if (use_brake_power_limit && omega_wheel > 1.0)
    {
        brake_limit_Nm =
            std::min(
                brake_limit_Nm,
                P.get("model.drivetrain.P_wheel_brake_max_W") /
                omega_wheel
            );
    }

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
    const double M_drive =
        std::max(P.get("model.drivetrain.M_wheel_drive_max_Nm"), 1.0e-6);

    const double M_brake =
        std::max(P.get("model.drivetrain.M_wheel_brake_max_Nm"), 1.0e-6);

    if (torque_Nm >= 0.0)
    {
        return 100.0 * torque_Nm / M_drive;
    }

    return 100.0 * torque_Nm / M_brake;
}

} // namespace skidpad_control