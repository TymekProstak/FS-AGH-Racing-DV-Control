#pragma once

#include "ParamBank.hpp"
#include "dv_control_common/load_transfer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dv_control
{

struct WheelTorquesNm
{
    double FL = 0.0;
    double FR = 0.0;
    double RL = 0.0;
    double RR = 0.0;
};

struct SimpleTorqueAllocatorInput
{
    double total_torque_cmd_Nm = 0.0;
    double desired_yaw_moment_Nm = 0.0;

    double vx_mps = 0.0;
    double vy_mps = 0.0;
    double yaw_rate_radps = 0.0;

    double ax_mps2 = 0.0;
    double ay_mps2 = 0.0;

    double delta_bicycle_rad = 0.0;

    double previous_total_output_torque_Nm = 0.0;
    bool previous_total_output_valid = false;

    bool bypass_total_rate_limiter = false;
};

struct SimpleTorqueAllocatorResult
{
    WheelTorquesNm normal_load_N;

    /*
        Per-wheel slip angles and lateral-force estimates used by the optional
        friction-ellipse limiter. The calculation matches the longer allocator:

            1. velocity at each wheel center,
            2. rotation into the wheel frame,
            3. alpha = -atan2(vy_local, vx_local),
            4. Fy = D * Fz * sin(C * atan(B * alpha)).
    */
    double vx_for_slip_limiter_mps = 0.0;
    WheelTorquesNm slip_angle_rad;
    WheelTorquesNm estimated_lateral_force_N;
    WheelTorquesNm max_abs_torque_from_friction_Nm;

    WheelTorquesNm base_torque_Nm;
    WheelTorquesNm torque_after_tv_Nm;
    WheelTorquesNm torque_output_Nm;

    double requested_total_torque_Nm = 0.0;
    double total_after_direction_limit_Nm = 0.0;
    double total_after_power_limit_Nm = 0.0;
    double rate_limited_total_torque_Nm = 0.0;
    double output_total_torque_Nm = 0.0;

    bool direction_limited = false;
    bool power_limited = false;
    bool total_rate_limited = false;

    double tv_speed_scale = 0.0;
    double requested_tv_yaw_moment_Nm = 0.0;
    double scaled_tv_yaw_moment_Nm = 0.0;

    double delta_front_tv_Nm = 0.0;
    double delta_rear_tv_Nm = 0.0;

    bool friction_ellipse_active = false;
    bool friction_ellipse_speed_enabled = false;
    double friction_ellipse_limit_scale = 1.0;
    bool friction_ellipse_limited = false;

    double engine_per_wheel_limit_scale = 1.0;

    /*
        Final scale applied to the complete wheel-torque vector. It is the
        minimum of the engine per-wheel scale and, when enabled, the
        friction-ellipse scale.
    */
    double per_wheel_limit_scale = 1.0;
    bool per_wheel_limited = false;
};

namespace
{

constexpr double kEps = 1.0e-9;
constexpr double kTvDeadZoneEndMps = 2.0;
constexpr double kTvFullActivationMps = 4.0;
constexpr double kFrictionEllipseMinSpeedMps = 3.0;

inline double clampValue(
    const double value,
    const double low,
    const double high
)
{
    return std::max(low, std::min(value, high));
}

inline double finiteOrZero(const double value)
{
    return std::isfinite(value) ? value : 0.0;
}

inline double speedForSlipAndLimiterMps(
    const double vx_mps
)
{
    constexpr double kMinimumSlipSpeedMps =
        2.0;

    if (!std::isfinite(vx_mps))
    {
        return kMinimumSlipSpeedMps;
    }

    return std::max(
        vx_mps,
        kMinimumSlipSpeedMps
    );
}

inline double sumWheelTorques(const WheelTorquesNm& torque)
{
    return torque.FL + torque.FR + torque.RL + torque.RR;
}

inline WheelTorquesNm scaleWheelTorques(
    const WheelTorquesNm& torque,
    const double scale
)
{
    WheelTorquesNm out;

    out.FL = torque.FL * scale;
    out.FR = torque.FR * scale;
    out.RL = torque.RL * scale;
    out.RR = torque.RR * scale;

    return out;
}

inline double getGearRatio(const ParamBank& P)
{
    const double gear_ratio =
        P.get("model.drivetrain.gear_ratio");

    if (!std::isfinite(gear_ratio) ||
        std::abs(gear_ratio) <= kEps)
    {
        throw std::runtime_error(
            "model.drivetrain.gear_ratio must be finite and non-zero"
        );
    }

    return std::abs(gear_ratio);
}

inline double getTrackWidthM(const ParamBank& P)
{
    const double track_width =
        P.get("model.body.track_width");

    if (!std::isfinite(track_width) ||
        track_width <= kEps)
    {
        throw std::runtime_error(
            "model.body.track_width must be finite and > 0"
        );
    }

    return track_width;
}

inline double getFrontTrackWidthM(const ParamBank& P)
{
    return getTrackWidthM(P);
}

inline double getRearTrackWidthM(const ParamBank& P)
{
    return getTrackWidthM(P);
}

inline double getTireRadiusM(const ParamBank& P)
{
    const double radius =
        P.get("model.tire.R_tire");

    if (!std::isfinite(radius) ||
        radius <= kEps)
    {
        throw std::runtime_error(
            "model.tire.R_tire must be finite and > 0"
        );
    }

    return radius;
}

inline double getControlDtS(const ParamBank& P)
{
    const double frequency_hz =
        P.get("model.frequency.steer_cmd_loop_hz");

    if (!std::isfinite(frequency_hz) ||
        frequency_hz <= kEps)
    {
        throw std::runtime_error(
            "model.frequency.steer_cmd_loop_hz must be finite and > 0"
        );
    }

    return 1.0 / frequency_hz;
}

inline double getTotalDriveReferenceNm(const ParamBank& P)
{
    const double configured_total_wheel_Nm =
        P.get("model.drivetrain.M_wheel_drive_max_Nm");

    const double engine_usage_total_wheel_Nm =
        P.get("model.drivetrain.M_engine_max_Nm_usage_drive_total")
        * getGearRatio(P);

    if (!std::isfinite(configured_total_wheel_Nm) ||
        configured_total_wheel_Nm < 0.0 ||
        !std::isfinite(engine_usage_total_wheel_Nm) ||
        engine_usage_total_wheel_Nm < 0.0)
    {
        throw std::runtime_error(
            "Drive torque limits must be finite and >= 0"
        );
    }

    return std::min(
        configured_total_wheel_Nm,
        engine_usage_total_wheel_Nm
    );
}

inline double getTotalBrakeReferenceNm(const ParamBank& P)
{
    const double configured_total_wheel_Nm =
        P.get("model.drivetrain.M_wheel_brake_max_Nm");

    const double engine_usage_total_wheel_Nm =
        P.get("model.drivetrain.M_engine_max_Nm_usage_brake_total")
        * getGearRatio(P);

    if (!std::isfinite(configured_total_wheel_Nm) ||
        configured_total_wheel_Nm < 0.0 ||
        !std::isfinite(engine_usage_total_wheel_Nm) ||
        engine_usage_total_wheel_Nm < 0.0)
    {
        throw std::runtime_error(
            "Brake torque limits must be finite and >= 0"
        );
    }

    return std::min(
        configured_total_wheel_Nm,
        engine_usage_total_wheel_Nm
    );
}


inline double clampTotalTorqueByDirectionNm(
    const ParamBank& P,
    const double requested_total_Nm,
    bool& limited
)
{
    double output_Nm = requested_total_Nm;

    if (requested_total_Nm >= 0.0)
    {
        output_Nm =
            std::min(
                requested_total_Nm,
                getTotalDriveReferenceNm(P)
            );
    }
    else
    {
        output_Nm =
            std::max(
                requested_total_Nm,
                -getTotalBrakeReferenceNm(P)
            );
    }

    limited =
        std::abs(output_Nm - requested_total_Nm) > 1.0e-6;

    return output_Nm;
}

inline double clampTotalTorqueByPowerNm(
    const ParamBank& P,
    const double requested_total_Nm,
    const double vx_mps,
    bool& limited
)
{
    double output_Nm =
        requested_total_Nm;

    const double speed_for_power_mps =
        std::max(
            std::abs(finiteOrZero(vx_mps)),
            0.5
        );

    const double wheel_angular_speed_radps =
        speed_for_power_mps
        / getTireRadiusM(P);

    if (requested_total_Nm >= 0.0)
    {
        if (P.getBool("model.drivetrain.use_drive_power_limit"))
        {
            const double drive_power_W =
                P.get("model.drivetrain.P_wheel_drive_max_W");

            if (!std::isfinite(drive_power_W) ||
                drive_power_W < 0.0)
            {
                throw std::runtime_error(
                    "model.drivetrain.P_wheel_drive_max_W "
                    "must be finite and >= 0"
                );
            }

            const double max_drive_torque_from_power_Nm =
                drive_power_W
                / wheel_angular_speed_radps;

            output_Nm =
                std::min(
                    output_Nm,
                    max_drive_torque_from_power_Nm
                );
        }
    }
    else
    {
        if (P.getBool("model.drivetrain.use_brake_power_limit"))
        {
            const double brake_power_W =
                P.get("model.drivetrain.P_wheel_brake_max_W");

            if (!std::isfinite(brake_power_W) ||
                brake_power_W < 0.0)
            {
                throw std::runtime_error(
                    "model.drivetrain.P_wheel_brake_max_W "
                    "must be finite and >= 0"
                );
            }

            const double max_brake_torque_from_power_Nm =
                brake_power_W
                / wheel_angular_speed_radps;

            output_Nm =
                std::max(
                    output_Nm,
                    -max_brake_torque_from_power_Nm
                );
        }
    }

    limited =
        std::abs(output_Nm - requested_total_Nm) > 1.0e-6;

    return output_Nm;
}

inline double getPerWheelDriveLimitNm(const ParamBank& P)
{
    const double engine_side_limit_Nm =
        P.get("model.drivetrain.wheel_max_torque_engine_drive_Nm");

    if (!std::isfinite(engine_side_limit_Nm) ||
        engine_side_limit_Nm < 0.0)
    {
        throw std::runtime_error(
            "model.drivetrain.wheel_max_torque_engine_drive_Nm "
            "must be finite and >= 0"
        );
    }

    return engine_side_limit_Nm * getGearRatio(P);
}

inline double getPerWheelBrakeLimitNm(const ParamBank& P)
{
    const double engine_side_limit_Nm =
        P.get("model.drivetrain.wheel_max_torque_engine_brake_Nm");

    if (!std::isfinite(engine_side_limit_Nm) ||
        engine_side_limit_Nm < 0.0)
    {
        throw std::runtime_error(
            "model.drivetrain.wheel_max_torque_engine_brake_Nm "
            "must be finite and >= 0"
        );
    }

    return engine_side_limit_Nm * getGearRatio(P);
}

inline double rateLimitTotalTorqueNm(
    const ParamBank& P,
    const double requested_total_Nm,
    const double previous_total_Nm,
    const bool previous_valid,
    const bool bypass,
    bool& rate_limited
)
{
    rate_limited = false;

    if (bypass)
    {
        return requested_total_Nm;
    }

    const double previous =
        previous_valid
            ? finiteOrZero(previous_total_Nm)
            : 0.0;

    const double rate_up =
        P.get("model.drivetrain.M_dot_up");

    const double rate_down =
        P.get("model.drivetrain.M_dot_down");

    if (!std::isfinite(rate_up) ||
        rate_up < 0.0 ||
        !std::isfinite(rate_down) ||
        rate_down < 0.0)
    {
        throw std::runtime_error(
            "model.drivetrain.M_dot_up/down must be finite and >= 0"
        );
    }

    const double dt =
        getControlDtS(P);

    const double max_positive_step_Nm =
        rate_up
        * getTotalDriveReferenceNm(P)
        * dt;

    const double max_negative_step_Nm =
        rate_down
        * getTotalBrakeReferenceNm(P)
        * dt;

    double output =
        requested_total_Nm;

    if (requested_total_Nm > previous)
    {
        output =
            std::min(
                requested_total_Nm,
                previous + max_positive_step_Nm
            );
    }
    else if (requested_total_Nm < previous)
    {
        output =
            std::max(
                requested_total_Nm,
                previous - max_negative_step_Nm
            );
    }

    rate_limited =
        std::abs(output - requested_total_Nm) > 1.0e-6;

    return output;
}

struct SteeringAngles
{
    double left_rad = 0.0;
    double right_rad = 0.0;
};

inline SteeringAngles computeAntiAckermannAngles(
    const double delta_bicycle_rad
)
{
    constexpr double kQuadraticCoefficient =
        9.298947992255e-02;

    constexpr double kToeOffsetRad =
        3.242826792680e-03;

    const double delta =
        finiteOrZero(delta_bicycle_rad);

    const double delta_squared =
        delta * delta;

    SteeringAngles out;

    out.left_rad =
        delta
        - kQuadraticCoefficient * delta_squared
        - kToeOffsetRad;

    out.right_rad =
        delta
        + kQuadraticCoefficient * delta_squared
        + kToeOffsetRad;

    return out;
}

inline double safeVxSignedForSlip(
    const double vx_mps
)
{
    constexpr double kMinimumSignedVxMps =
        1.0e-4;

    if (!std::isfinite(vx_mps))
    {
        return kMinimumSignedVxMps;
    }

    return std::max(
        vx_mps,
        kMinimumSignedVxMps
    );
}

inline void rotateVelocityToWheelFrame(
    const double wheel_angle_rad,
    double& vx_mps,
    double& vy_mps
)
{
    const double c =
        std::cos(wheel_angle_rad);

    const double s =
        std::sin(wheel_angle_rad);

    const double vx_body_mps =
        vx_mps;

    const double vy_body_mps =
        vy_mps;

    vx_mps =
        c * vx_body_mps
        + s * vy_body_mps;

    vy_mps =
        -s * vx_body_mps
        + c * vy_body_mps;
}

inline WheelTorquesNm estimateWheelSlipAnglesLikeTireModelRad(
    const ParamBank& P,
    const double vx_mps,
    const double vy_mps,
    const double yaw_rate_radps,
    const double delta_left_rad,
    const double delta_right_rad
)
{
    const double l_f =
        P.get("model.body.l_f");

    const double l_r =
        P.get("model.body.l_r");

    const double half_track_front =
        0.5 * getFrontTrackWidthM(P);

    const double half_track_rear =
        0.5 * getRearTrackWidthM(P);

    const double vy =
        finiteOrZero(vy_mps);

    const double yaw_rate =
        finiteOrZero(yaw_rate_radps);

    /*
        Body frame:
            x forward
            y left

        Velocity at a wheel center:
            vx_w = vx - yaw_rate * y_w
            vy_w = vy + yaw_rate * x_w
    */
    double vx_fl =
        vx_mps
        - yaw_rate * half_track_front;

    double vy_fl =
        vy
        + yaw_rate * l_f;

    double vx_fr =
        vx_mps
        + yaw_rate * half_track_front;

    double vy_fr =
        vy
        + yaw_rate * l_f;

    double vx_rl =
        vx_mps
        - yaw_rate * half_track_rear;

    double vy_rl =
        vy
        - yaw_rate * l_r;

    double vx_rr =
        vx_mps
        + yaw_rate * half_track_rear;

    double vy_rr =
        vy
        - yaw_rate * l_r;

    rotateVelocityToWheelFrame(
        delta_left_rad,
        vx_fl,
        vy_fl
    );

    rotateVelocityToWheelFrame(
        delta_right_rad,
        vx_fr,
        vy_fr
    );

    WheelTorquesNm alpha;

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

    constexpr double kPi =
        3.14159265358979323846;

    const double alpha_min =
        -kPi / 2.0 + 0.3;

    const double alpha_max =
        +kPi / 2.0 - 0.3;

    alpha.FL = clampValue(alpha.FL, alpha_min, alpha_max);
    alpha.FR = clampValue(alpha.FR, alpha_min, alpha_max);
    alpha.RL = clampValue(alpha.RL, alpha_min, alpha_max);
    alpha.RR = clampValue(alpha.RR, alpha_min, alpha_max);

    return alpha;
}

inline WheelTorquesNm allocateBaseTorqueProportionalToLoadNm(
    const double total_torque_Nm,
    const WheelTorquesNm& normal_load_N
)
{
    const double left_total_Nm =
        0.5 * total_torque_Nm;

    const double right_total_Nm =
        0.5 * total_torque_Nm;

    const double left_load_sum_N =
        std::max(
            1.0,
            normal_load_N.FL + normal_load_N.RL
        );

    const double right_load_sum_N =
        std::max(
            1.0,
            normal_load_N.FR + normal_load_N.RR
        );

    WheelTorquesNm torque;

    torque.FL =
        left_total_Nm
        * normal_load_N.FL
        / left_load_sum_N;

    torque.RL =
        left_total_Nm
        * normal_load_N.RL
        / left_load_sum_N;

    torque.FR =
        right_total_Nm
        * normal_load_N.FR
        / right_load_sum_N;

    torque.RR =
        right_total_Nm
        * normal_load_N.RR
        / right_load_sum_N;

    return torque;
}

inline WheelTorquesNm estimateWheelNormalLoadsN(
    const ParamBank& P,
    const double ax_relaxed_mps2,
    const double ay_relaxed_mps2,
    const double vx_mps
)
{
    dv_control_common::WheelLoadModelParameters model;

    model.mass_kg = P.get("model.body.m");
    model.gravity_mps2 = P.get("model.body.g");
    model.lf_m = P.get("model.body.l_f");
    model.lr_m = P.get("model.body.l_r");
    model.h_cg_m = P.get("model.body.h_cg");
    model.track_front_m = getFrontTrackWidthM(P);
    model.track_rear_m = getRearTrackWidthM(P);
    model.h_roll_center_front_m = P.get("model.body.h1_roll");
    model.h_roll_center_rear_m = P.get("model.body.h2_roll");
    model.lambda_elastic_front =
        P.get("model.body.lambda_phi_elastic_lateral");
    model.minimum_wheel_load_N =
        P.get("model.mass_transfer.minimum_wheel_load_N");

    double front_aero_N = 0.0;
    double rear_aero_N = 0.0;

    if (P.getBool("model.body.use_aero"))
    {
        const double speed_mps =
            std::max(0.0, finiteOrZero(vx_mps));

        front_aero_N =
            P.get("model.body.Cl1") * speed_mps * speed_mps;

        rear_aero_N =
            P.get("model.body.Cl2") * speed_mps * speed_mps;
    }

    const dv_control_common::WheelLoadsN shared =
        dv_control_common::computeWheelLoadsN(
            model,
            ax_relaxed_mps2,
            ay_relaxed_mps2,
            front_aero_N,
            rear_aero_N
        );

    WheelTorquesNm out;
    out.FL = shared.FL;
    out.FR = shared.FR;
    out.RL = shared.RL;
    out.RR = shared.RR;
    return out;
}

inline double computeTvSpeedScale(const double vx_mps)
{
    const double speed =
        std::abs(finiteOrZero(vx_mps));

    if (speed <= kTvDeadZoneEndMps)
    {
        return 0.0;
    }

    if (speed >= kTvFullActivationMps)
    {
        return 1.0;
    }

    const double u =
        clampValue(
            (speed - kTvDeadZoneEndMps)
            / (kTvFullActivationMps - kTvDeadZoneEndMps),
            0.0,
            1.0
        );

    return
        u * u * (3.0 - 2.0 * u);
}

inline void computeLoadProportionalTvDeltasNm(
    const ParamBank& P,
    const WheelTorquesNm& normal_load_N,
    const double desired_yaw_moment_Nm,
    const double delta_left_rad,
    const double delta_right_rad,
    double& delta_front_tv_Nm,
    double& delta_rear_tv_Nm
)
{
    delta_front_tv_Nm = 0.0;
    delta_rear_tv_Nm = 0.0;

    if (std::abs(desired_yaw_moment_Nm) <= kEps)
    {
        return;
    }

    const double tire_radius =
        getTireRadiusM(P);

    const double l_f =
        P.get("model.body.l_f");

    const double track_front =
        getFrontTrackWidthM(P);

    const double track_rear =
        getRearTrackWidthM(P);

    const double half_track_front =
        0.5 * track_front;

    const double front_yaw_coefficient =
        (
            l_f
            * (
                std::sin(delta_left_rad)
                - std::sin(delta_right_rad)
            )
            - half_track_front
            * (
                std::cos(delta_left_rad)
                + std::cos(delta_right_rad)
            )
        )
        / tire_radius;

    const double rear_yaw_coefficient =
        -track_rear / tire_radius;

    const double front_load_N =
        normal_load_N.FL + normal_load_N.FR;

    const double rear_load_N =
        normal_load_N.RL + normal_load_N.RR;

    const double total_load_N =
        std::max(
            1.0,
            front_load_N + rear_load_N
        );

    const double front_share =
        front_load_N / total_load_N;

    const double rear_share =
        rear_load_N / total_load_N;

    const double yaw_moment_per_lambda =
        front_yaw_coefficient * front_share
        + rear_yaw_coefficient * rear_share;

    if (std::abs(yaw_moment_per_lambda) <= kEps)
    {
        return;
    }

    const double lambda =
        desired_yaw_moment_Nm
        / yaw_moment_per_lambda;

    delta_front_tv_Nm =
        lambda * front_share;

    delta_rear_tv_Nm =
        lambda * rear_share;
}

inline WheelTorquesNm applyTorqueVectoringNm(
    const WheelTorquesNm& base_torque_Nm,
    const double delta_front_tv_Nm,
    const double delta_rear_tv_Nm
)
{
    WheelTorquesNm out =
        base_torque_Nm;

    out.FL += delta_front_tv_Nm;
    out.FR -= delta_front_tv_Nm;

    out.RL += delta_rear_tv_Nm;
    out.RR -= delta_rear_tv_Nm;

    return out;
}

inline double estimateLateralForceFromSlipAngleN(
    const double alpha_rad,
    const double Fz_N,
    const double B,
    const double C,
    const double D
)
{
    return
        D
        * Fz_N
        * std::sin(
            C * std::atan(B * alpha_rad)
        );
}

inline WheelTorquesNm estimateWheelLateralForcesFromSlipAnglesN(
    const ParamBank& P,
    const WheelTorquesNm& slip_angle_rad,
    const WheelTorquesNm& normal_load_N
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

    WheelTorquesNm lateral_force;

    lateral_force.FL =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.FL,
            normal_load_N.FL,
            Bf,
            Cf,
            Df
        );

    lateral_force.FR =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.FR,
            normal_load_N.FR,
            Bf,
            Cf,
            Df
        );

    lateral_force.RL =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.RL,
            normal_load_N.RL,
            Br,
            Cr,
            Dr
        );

    lateral_force.RR =
        estimateLateralForceFromSlipAngleN(
            slip_angle_rad.RR,
            normal_load_N.RR,
            Br,
            Cr,
            Dr
        );

    return lateral_force;
}

inline double computeMaxAbsFxFromFrictionEllipseN(
    const ParamBank& P,
    const double normal_load_N,
    const double lateral_force_N
)
{
    const double mu_x =
        P.get("general.mu_x_friction_limit");

    const double mu_y =
        P.get("general.mu_y_friction_limit");

    if (!std::isfinite(mu_x) ||
        mu_x < 0.0 ||
        !std::isfinite(mu_y) ||
        mu_y <= kEps)
    {
        throw std::runtime_error(
            "general.mu_x_friction_limit must be finite and >= 0, "
            "general.mu_y_friction_limit must be finite and > 0"
        );
    }

    const double Fz =
        std::max(1.0, normal_load_N);

    const double lateral_limit_N =
        mu_y * Fz;

    const double lateral_usage =
        std::abs(lateral_force_N)
        / std::max(kEps, lateral_limit_N);

    const double remaining_squared =
        std::max(
            0.0,
            1.0 - lateral_usage * lateral_usage
        );

    return
        mu_x
        * Fz
        * std::sqrt(remaining_squared);
}

inline WheelTorquesNm computeMaxAbsWheelTorqueFromFrictionEllipseNm(
    const ParamBank& P,
    const WheelTorquesNm& normal_load_N,
    const WheelTorquesNm& lateral_force_N
)
{
    const double tire_radius =
        getTireRadiusM(P);

    WheelTorquesNm max_torque;

    max_torque.FL =
        computeMaxAbsFxFromFrictionEllipseN(
            P,
            normal_load_N.FL,
            lateral_force_N.FL
        )
        * tire_radius;

    max_torque.FR =
        computeMaxAbsFxFromFrictionEllipseN(
            P,
            normal_load_N.FR,
            lateral_force_N.FR
        )
        * tire_radius;

    max_torque.RL =
        computeMaxAbsFxFromFrictionEllipseN(
            P,
            normal_load_N.RL,
            lateral_force_N.RL
        )
        * tire_radius;

    max_torque.RR =
        computeMaxAbsFxFromFrictionEllipseN(
            P,
            normal_load_N.RR,
            lateral_force_N.RR
        )
        * tire_radius;

    return max_torque;
}

inline double computeGlobalAbsWheelLimitScale(
    const WheelTorquesNm& torque_Nm,
    const WheelTorquesNm& max_abs_torque_Nm
)
{
    double scale =
        1.0;

    const auto updateScale =
        [&](
            const double requested_torque_Nm,
            const double allowed_abs_torque_Nm
        )
        {
            const double requested_abs_Nm =
                std::abs(requested_torque_Nm);

            if (requested_abs_Nm <= kEps)
            {
                return;
            }

            scale =
                std::min(
                    scale,
                    std::max(0.0, allowed_abs_torque_Nm)
                    / requested_abs_Nm
                );
        };

    updateScale(torque_Nm.FL, max_abs_torque_Nm.FL);
    updateScale(torque_Nm.FR, max_abs_torque_Nm.FR);
    updateScale(torque_Nm.RL, max_abs_torque_Nm.RL);
    updateScale(torque_Nm.RR, max_abs_torque_Nm.RR);

    return clampValue(scale, 0.0, 1.0);
}

inline double computeGlobalPerWheelLimitScale(
    const WheelTorquesNm& torque_Nm,
    const double drive_limit_per_wheel_Nm,
    const double brake_limit_per_wheel_Nm
)
{
    double scale =
        1.0;

    const auto updateScale =
        [&](const double wheel_torque_Nm)
        {
            const double requested_abs_Nm =
                std::abs(wheel_torque_Nm);

            if (requested_abs_Nm <= kEps)
            {
                return;
            }

            const double allowed_abs_Nm =
                wheel_torque_Nm >= 0.0
                    ? drive_limit_per_wheel_Nm
                    : brake_limit_per_wheel_Nm;

            scale =
                std::min(
                    scale,
                    allowed_abs_Nm / requested_abs_Nm
                );
        };

    updateScale(torque_Nm.FL);
    updateScale(torque_Nm.FR);
    updateScale(torque_Nm.RL);
    updateScale(torque_Nm.RR);

    return clampValue(scale, 0.0, 1.0);
}

} // namespace

inline SimpleTorqueAllocatorResult allocateSimpleWheelTorques(
    const ParamBank& P,
    const SimpleTorqueAllocatorInput& input
)
{
    SimpleTorqueAllocatorResult result;

    result.requested_total_torque_Nm =
        finiteOrZero(input.total_torque_cmd_Nm);

    result.total_after_direction_limit_Nm =
        clampTotalTorqueByDirectionNm(
            P,
            result.requested_total_torque_Nm,
            result.direction_limited
        );

    result.total_after_power_limit_Nm =
        clampTotalTorqueByPowerNm(
            P,
            result.total_after_direction_limit_Nm,
            input.vx_mps,
            result.power_limited
        );

    result.rate_limited_total_torque_Nm =
        rateLimitTotalTorqueNm(
            P,
            result.total_after_power_limit_Nm,
            input.previous_total_output_torque_Nm,
            input.previous_total_output_valid,
            input.bypass_total_rate_limiter,
            result.total_rate_limited
        );

    result.normal_load_N =
        estimateWheelNormalLoadsN(
            P,
            input.ax_mps2,
            input.ay_mps2,
            input.vx_mps
        );

    result.base_torque_Nm =
        allocateBaseTorqueProportionalToLoadNm(
            result.rate_limited_total_torque_Nm,
            result.normal_load_N
        );

    result.tv_speed_scale =
        computeTvSpeedScale(
            input.vx_mps
        );

    result.requested_tv_yaw_moment_Nm =
        finiteOrZero(input.desired_yaw_moment_Nm);

    result.scaled_tv_yaw_moment_Nm =
        result.tv_speed_scale
        * result.requested_tv_yaw_moment_Nm;

    const SteeringAngles steering =
        computeAntiAckermannAngles(
            input.delta_bicycle_rad
        );

    computeLoadProportionalTvDeltasNm(
        P,
        result.normal_load_N,
        result.scaled_tv_yaw_moment_Nm,
        steering.left_rad,
        steering.right_rad,
        result.delta_front_tv_Nm,
        result.delta_rear_tv_Nm
    );

    result.torque_after_tv_Nm =
        applyTorqueVectoringNm(
            result.base_torque_Nm,
            result.delta_front_tv_Nm,
            result.delta_rear_tv_Nm
        );

    result.engine_per_wheel_limit_scale =
        computeGlobalPerWheelLimitScale(
            result.torque_after_tv_Nm,
            getPerWheelDriveLimitNm(P),
            getPerWheelBrakeLimitNm(P)
        );

    result.friction_ellipse_active =
        P.getBool("general.use_friction_elipse_limiter");

    /*
        Slip angle estimated from vx, vy and yaw rate is not reliable close
        to standstill. Do not use the friction ellipse below the configured
        raw longitudinal-speed threshold. In particular, do not replace a
        near-zero vx with an artificial positive speed and then limit torque
        from that artificial slip angle.
    */
    result.friction_ellipse_speed_enabled =
        std::isfinite(input.vx_mps)
        && input.vx_mps >= kFrictionEllipseMinSpeedMps;

    result.friction_ellipse_limit_scale =
        1.0;

    if (result.friction_ellipse_active &&
        result.friction_ellipse_speed_enabled)
    {
        result.vx_for_slip_limiter_mps =
            input.vx_mps;

        result.slip_angle_rad =
            estimateWheelSlipAnglesLikeTireModelRad(
                P,
                result.vx_for_slip_limiter_mps,
                input.vy_mps,
                input.yaw_rate_radps,
                steering.left_rad,
                steering.right_rad
            );

        result.estimated_lateral_force_N =
            estimateWheelLateralForcesFromSlipAnglesN(
                P,
                result.slip_angle_rad,
                result.normal_load_N
            );

        result.max_abs_torque_from_friction_Nm =
            computeMaxAbsWheelTorqueFromFrictionEllipseNm(
                P,
                result.normal_load_N,
                result.estimated_lateral_force_N
            );

        result.friction_ellipse_limit_scale =
            computeGlobalAbsWheelLimitScale(
                result.torque_after_tv_Nm,
                result.max_abs_torque_from_friction_Nm
            );
    }

    result.friction_ellipse_limited =
        result.friction_ellipse_active
        && result.friction_ellipse_speed_enabled
        && result.friction_ellipse_limit_scale < 1.0 - 1.0e-6;

    result.per_wheel_limit_scale =
        std::min(
            result.engine_per_wheel_limit_scale,
            result.friction_ellipse_limit_scale
        );

    result.per_wheel_limited =
        result.per_wheel_limit_scale < 1.0 - 1.0e-6;

    result.torque_output_Nm =
        scaleWheelTorques(
            result.torque_after_tv_Nm,
            result.per_wheel_limit_scale
        );

    result.output_total_torque_Nm =
        sumWheelTorques(
            result.torque_output_Nm
        );

    return result;
}

} // namespace dv_control
