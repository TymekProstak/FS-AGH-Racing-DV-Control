#pragma once

#include <string>

namespace acc_launch_control
{

// =============================================================================
//                              DRIVE PHASE
// =============================================================================

enum class DrivePhase
{
    WAITING = 0,
    LAUNCH,
    ACCELERATION,
    SPEED_HOLD,
    BRAKING,
    COASTING,
    EMERGENCY_BRAKE,
    FINISHED
};


enum class LongitudinalControllerMode
{
    NONE = 0,
    MAP_LAUNCH_ACCELERATION_BRAKE
};


enum class EmergencyBrakeMode
{
    NONE = 0,
    FIXED_EMERGENCY_BRAKE
};


// =============================================================================
//                         LATERAL CONTROLLER TYPE
// =============================================================================

enum class LateralControllerType
{
    NONE = 0,
    LTV_MPC_UNBOUNDED
};


// =============================================================================
//                              VEHICLE STATE
// =============================================================================

struct State
{
    double vx = 0.0;
    double vy = 0.0;

    double ey = 0.0;
    double epsi = 0.0;

    double r = 0.0;

    /*
        ACC LTV MPC steering convention:

            delta:
                physical measured steering angle from encoder.

            delta_dot:
                steering-rate state propagated by the PT2 model.

            delta_cmd:
                absolute command obtained by integrating u_delta_cmd.

            delta_enc:
                raw/corrected encoder value retained for diagnostics.

            delta_vehicle_used:
                angle used by the vehicle model; equal to delta.
    */
    double delta = 0.0;
    double delta_dot = 0.0;
    double delta_cmd = 0.0;
    double delta_enc = 0.0;
    double delta_vehicle_used = 0.0;

    double vx_enc = 0.0;

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double s = 0.0;

    double ax_mps2 = 0.0;
    double ay_mps2 = 0.0;
};


// =============================================================================
//                            CONTROLLER RESULT
// =============================================================================

struct Result
{
    bool valid = false;

    double u_delta_cmd = 0.0;
    double u_delta_cmd_raw = 0.0;

    bool u_delta_cmd_saturated = false;
    bool rate_saturated = false;
    bool steering_angle_saturated = false;

    double u_tv_yaw_moment_Nm = 0.0;
    double u_tv_yaw_moment_raw_Nm = 0.0;
    bool u_tv_yaw_moment_saturated = false;

    double delta_act_next = 0.0;

    /*
        PT2 steering-rate state at the end of the first predicted sample.
        The wrapper carries this value to the next controller iteration.
    */
    double delta_dot_next = 0.0;

    double delta_cmd_next = 0.0;
    double delta_vehicle_used = 0.0;

    double vy_next = 0.0;
    double r_next = 0.0;
};


std::string phaseToString(DrivePhase phase);
std::string lateralControllerToString(LateralControllerType type);
std::string longitudinalControllerModeToString(LongitudinalControllerMode mode);
std::string emergencyBrakeModeToString(EmergencyBrakeMode mode);

} // namespace acc_launch_control
