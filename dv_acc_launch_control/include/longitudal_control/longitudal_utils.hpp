#pragma once

#include "ParamBank.hpp"
#include "pid.hpp"

namespace acc_launch_control
{

// =============================================================================
//                              BASIC STRUCTS
// =============================================================================

struct WheelValues
{
    double FL = 0.0;
    double FR = 0.0;
    double RL = 0.0;
    double RR = 0.0;
};

struct SteeringAngles
{
    double delta_left_rad = 0.0;
    double delta_right_rad = 0.0;
};

struct WheelTorqueLimits
{
    WheelValues Fz_N;
    WheelValues slip_angle_rad;
    WheelValues Fy_N;

    WheelValues max_abs_Fx_from_ellipse_N;
    WheelValues max_abs_torque_from_ellipse_Nm;
    WheelValues max_abs_torque_after_mech_Nm;

    /*
        Symmetric debug value from clampTorqueByDrivetrainLimits().
        It is not the requested torque. It is the smaller absolute value of
        available drive/brake total torque after total engine-side/power limits.
    */
    double max_abs_total_torque_after_power_Nm = 0.0;
};

struct WheelTorqueAllocation
{
    double requested_total_torque_Nm = 0.0;
    double drivetrain_limited_total_torque_Nm = 0.0;
    double allocated_total_torque_Nm = 0.0;

    WheelValues torque_Nm;
    WheelValues abs_torque_Nm;
    WheelValues share;

    /*
        Final allocator scaling.

        hard_limit_scale:
            scale from friction ellipse / mechanical wheel limits.

        total_torque_rate_*:
            scale interval from previous total torque and normalized runtime
            rate limit.

        Final policy:
            rate has priority over friction/mechanical scale.
    */
    double hard_limit_scale = 1.0;

    double previous_total_limited_torque_Nm = 0.0;
    double total_torque_rate_scale_min = 0.0;
    double total_torque_rate_scale_max = 1.0;
    double total_torque_rate_limited_scale = 1.0;

    double normalized_M_dot_up = 0.0;
    double normalized_M_dot_down = 0.0;
    double max_total_torque_step_up_Nm = 0.0;
    double max_total_torque_step_down_Nm = 0.0;

    double global_scale = 1.0;

    bool limited_by_drivetrain = false;
    bool limited_by_friction_or_mech = false;
    bool limited_by_rate = false;
    bool rate_forced_over_friction_or_mech = false;
    bool torque_rate_limiter_bypassed = false;

    SteeringAngles steering_angles;
    WheelTorqueLimits limits;
};


// =============================================================================
//                              PID CONFIG
// =============================================================================

PIDParams makeSpeedPidParams(const ParamBank& P);


// =============================================================================
//                          RESISTANCE / FEEDFORWARD
// =============================================================================
//
// All returned values are total wheel-side torque [Nm].
//

double computeResistanceFeedforwardTorqueNm(
    const ParamBank& P,
    double vx_mps
);

double computeAccelerationFeedforwardTorqueNm(
    const ParamBank& P,
    double ax_mps2,
    double vx_mps
);


// =============================================================================
//                              DRIVETRAIN LIMITS
// =============================================================================
//
// Input and output:
//      total wheel-side torque [Nm]
//
// Convention:
//      M_engine_max_Nm_usage_drive_total / brake_total are total motor-side
//      limits for the whole car. They are converted to wheel-side by gear_ratio.
//
//      wheel_max_torque_engine_drive_Nm / brake_Nm are motor-side limits for
//      one wheel. They are converted to wheel-side by gear_ratio and applied
//      later by one global vector scale.
//

double clampTorqueByDrivetrainLimits(
    const ParamBank& P,
    double total_torque_cmd_Nm,
    double vx_mps
);


// =============================================================================
//                              RUNTIME RATE LIMIT
// =============================================================================
//
// M_dot_up/down are normalized total wheel-side torque rates [1/s].
// Physical rate:
//      dM/dt = M_dot * static_total_wheel_limit
//

double getRuntimeTorqueRateLimitNmps(
    const ParamBank& P,
    bool torque_increasing
);

double rateLimitTorqueCommandNm(
    const ParamBank& P,
    double current_torque_Nm,
    double target_torque_Nm,
    double dt_s
);


// =============================================================================
//                              FINAL CAR INTERFACE
// =============================================================================
//
// Everything in the controller stays in wheel-side torque [Nm].
// Only at the final publish boundary:
//      movement = sum_wheel_torque_Nm / gear_ratio
// This is NOT percent.
//

double sumWheelTorquesNm(
    const WheelValues& wheel_torque_Nm
);

double totalWheelTorqueNmToVehicleInterfaceMovement(
    const ParamBank& P,
    double total_wheel_torque_Nm
);

double wheelTorqueAllocationToVehicleInterfaceMovement(
    const ParamBank& P,
    const WheelTorqueAllocation& allocation
);


// =============================================================================
//                              SLIP HELPERS
// =============================================================================

double slipDenominatorRegularizedNumeric(
    double vx,
    const ParamBank& P
);

double regularizedKappaFromWheelLinearSpeed(
    double vx,
    double v_wheel,
    const ParamBank& P
);


// =============================================================================
//              PER-WHEEL LOAD, SLIP, FY, ELLIPSE AND ALLOCATION
// =============================================================================

SteeringAngles computeAntiAckermannSteeringAngles(
    double delta_bicycle_rad,
    const ParamBank& P
);

WheelValues estimateWheelNormalLoadsN(
    const ParamBank& P,
    double vx_mps,
    double ax_mps2,
    double ay_mps2
);

WheelValues estimateWheelSlipAnglesLikeTireModelRad(
    const ParamBank& P,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double delta_left_rad,
    double delta_right_rad
);

double estimateLateralForceFromSlipAngleN(
    double alpha_rad,
    double Fz_N,
    double B,
    double C,
    double D
);

WheelValues estimateWheelLateralForcesFromSlipAnglesN(
    const ParamBank& P,
    const WheelValues& slip_angle_rad,
    const WheelValues& Fz_N
);

double computeMaxAbsFxFromWheelFrictionEllipseN(
    const ParamBank& P,
    double Fz_N,
    double Fy_N
);

WheelTorqueLimits computeWheelTorqueLimitsNm(
    const ParamBank& P,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double ax_mps2,
    double ay_mps2,
    double delta_bicycle_rad
);

/*
    Returns signed maximum available total wheel-side drive torque [Nm].

    Base distribution used for this estimate:
        share_i proportional to Fz_i computed with ay = 0.0
        i.e. static + aero + longitudinal transfer from ax.

    Limit/scale used for this estimate:
        full Fz/slip/Fy/friction calculation with real ay_mps2.

    Drivetrain total limit:
        M_engine_max_Nm_usage_drive_total * gear_ratio,
        also clipped by M_wheel_drive_max_Nm and optional power limit.

    Final per-wheel engine-side limit:
        wheel_max_torque_engine_drive_Nm * gear_ratio.
*/
double computeMaxAvailableDriveTorqueNm(
    const ParamBank& P,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double ax_mps2,
    double ay_mps2,
    double delta_bicycle_rad
);

/*
    Returns signed maximum available total wheel-side brake torque [Nm].
    The returned value is negative.

    Uses the same allocation/limit convention as computeMaxAvailableDriveTorqueNm,
    but with brake total/per-wheel engine-side limits.
*/
double computeMaxAvailableBrakeTorqueNm(
    const ParamBank& P,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double ax_mps2,
    double ay_mps2,
    double delta_bicycle_rad
);

/*
    Final hard limiter for already allocated signed per-wheel torques.
    Returns one global scale in [0, 1], so proportions and yaw balance are
    preserved. It does not clip wheels independently.
*/
double computeFinalPerWheelEngineLimitScale(
    const ParamBank& P,
    const WheelValues& signed_wheel_torque_Nm
);

WheelTorqueAllocation allocateWheelTorqueByNormalLoadNm(
    const ParamBank& P,
    double requested_total_torque_Nm,
    double vx_mps,
    double vy_mps,
    double yaw_rate_radps,
    double ax_mps2,
    double ay_mps2,
    double delta_bicycle_rad,
    double previous_total_limited_torque_Nm = 0.0,
    bool previous_total_limited_torque_valid = false,
    bool bypass_torque_rate_limiter = false
);


// =============================================================================
//                          BRAKE DISTANCE PREDICTION
// =============================================================================

/*
    Predicts distance needed to slow down from initial_vx_mps to
    general.coast_below_speed_mps.

    It simulates braking forward in time and at every step computes available
    brake torque using computeMaxAvailableBrakeTorqueNm().
*/
double estimateRequiredBrakeDistanceM(
    const ParamBank& P,
    double initial_vx_mps,
    double initial_ax_mps2,
    double initial_total_torque_Nm
);

/*
    Evaluates whether braking should start now.

    distance_available_for_brake = general.s_total - s_m - general.brake_margin_m
    start braking when:
        distance_available_for_brake <= estimateRequiredBrakeDistanceM(...)

    Optional output pointers can be nullptr.
*/
bool shouldStartBrakeByDistance(
    const ParamBank& P,
    double s_m,
    double vx_mps,
    double ax_mps2,
    double current_total_torque_Nm,
    double* required_brake_distance_m_out = nullptr,
    double* distance_available_for_brake_m_out = nullptr
);

} // namespace acc_launch_control