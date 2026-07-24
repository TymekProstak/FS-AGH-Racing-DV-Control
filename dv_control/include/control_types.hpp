#pragma once

#include <string>

namespace dv_control
{

// =============================================================================
//                              DRIVE PHASE
// =============================================================================

enum class DrivePhase
{
    WAITING = 0,
    DRIVE,
    EMERGENCY_BRAKE,
    FINAL_BRAKE,
    COASTING,
    FINISHED
};

enum class LongitudinalControllerMode
{
    NONE = 0,
    CONSTANT_SPEED,
    VELOCITY_PROFILE
};

enum class EmergencyBrakeMode
{
    NONE = 0,
    RAMP
};

// =============================================================================
//                         LATERAL CONTROLLER TYPE
// =============================================================================

enum class LateralControllerType
{
    NONE = 0,
    MPC_UNBOUNDED
};

struct State
{
    // -------------------------------------------------------------------------
    // Longitudinal / lateral vehicle states
    // -------------------------------------------------------------------------

    double vx = 0.0;
    double vy = 0.0;

    double ey = 0.0;
    double epsi = 0.0;

    double r = 0.0;

    // -------------------------------------------------------------------------
    // Steering states
    // -------------------------------------------------------------------------
    //
    // Fixed steering convention used by dv_control:
    //
    //     delta:
    //         physical steering angle from the encoder.
    //
    //     delta_dot:
    //         PT2 model steering-rate state propagated between controller
    //         iterations. The encoder is not differentiated.
    //
    //     delta_cmd:
    //         absolute steering command after integrating u_delta_cmd.
    //
    // PT2 is always active in the model.
    //
    double delta = 0.0;
    double delta_cmd = 0.0;
    double delta_dot = 0.0;
    double delta_enc = 0.0;
    double delta_vehicle_used = 0.0;

    // -------------------------------------------------------------------------
    // Encoder / measured speed
    // -------------------------------------------------------------------------

    double vx_enc = 0.0;

    // -------------------------------------------------------------------------
    // Global position / path state
    // -------------------------------------------------------------------------

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double s = 0.0;

    double acc_x = 0.0;
    double acc_y = 0.0;

    // -------------------------------------------------------------------------
    // Reference speed and acceleration for feedforward
    // -------------------------------------------------------------------------

    double v_ref = 0.0;
    double a_ref = 0.0;
};

// =============================================================================
//                         LATERAL CONTROLLER RESULT
// =============================================================================

struct Result
{
    bool valid = false;

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------

    // u_delta_cmd = d(delta_cmd)/dt
    double u_delta_cmd = 0.0;

    // Raw value before saturation.
    double u_delta_cmd_raw = 0.0;

    bool u_delta_cmd_saturated = false;
    bool rate_saturated = false;
    bool steering_angle_saturated = false;

    // -------------------------------------------------------------------------
    // Optimized torque-vectoring yaw moment
    // -------------------------------------------------------------------------

    double u_tv_yaw_moment_Nm = 0.0;
    double u_tv_yaw_moment_raw_Nm = 0.0;
    bool u_tv_yaw_moment_saturated = false;

    // -------------------------------------------------------------------------
    // Steering prediction / debug
    // -------------------------------------------------------------------------

    // Predicted PT2 physical angle after one controller step.
    double delta_act_next = 0.0;

    // Predicted PT2 steering-rate state after one controller step.
    double delta_dot_next = 0.0;

    // Predicted absolute command after one controller step.
    double delta_cmd_next = 0.0;

    // Steering angle used by the vehicle model for the current prediction.
    double delta_vehicle_used = 0.0;

    // -------------------------------------------------------------------------
    // Vehicle prediction / debug
    // -------------------------------------------------------------------------

    double vy_next = 0.0;
    double r_next = 0.0;
};

// =============================================================================
//                              CONVERSION
// =============================================================================

std::string phaseToString(DrivePhase phase);
std::string lateralControllerToString(LateralControllerType type);
std::string longitudinalControllerModeToString(LongitudinalControllerMode mode);
std::string emergencyBrakeModeToString(EmergencyBrakeMode mode);

} // namespace dv_control
