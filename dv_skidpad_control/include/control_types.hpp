#pragma once

#include <string>

namespace skidpad_control
{

// =============================================================================
//                              VEHICLE STATE
// =============================================================================

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
    // Physical steering angle used by the model (encoder-closed PT2 state).
    double delta = 0.0;

    // delta_cmd:
    //     Current absolute steering command after rate integration.
    //     This is the value that the wrapper publishes as steeringAngle_rad.
    double delta_cmd = 0.0;

    // delta_dot:
    //     Actual / estimated steering rate.
    //
    //     Used by PT2 actuator model.
    double delta_dot = 0.0;

    // delta_enc:
    //     Optional encoder/debug value.
    //
    double delta_enc = 0.0;

    // delta_vehicle_used:
    //     Debug copy of the steering angle really used by the model.
    //
    //     Usually equal to delta.
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

    // -------------------------------------------------------------------------
    // IMU / estimated acceleration
    // -------------------------------------------------------------------------

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
    // Steering control
    // -------------------------------------------------------------------------
    //
    // u_delta_cmd = d(delta_cmd)/dt
    //
    double u_delta_cmd = 0.0;

    // Raw value before saturation.
    double u_delta_cmd_raw = 0.0;

    // Saturation debug.
    bool u_delta_cmd_saturated = false;
    bool rate_saturated = false;
    bool steering_angle_saturated = false;

    // -------------------------------------------------------------------------
    // Torque-vectoring output
    // -------------------------------------------------------------------------
    //
    // Used only by LTV-MPC when:
    //     general.general_use_torque_vectoring = true
    //     general.general_use_jaca_torque_vectoring = false
    //
    // Meaning:
    //     u_tv_yaw_moment_Nm     - limited/accepted optimized yaw moment [Nm]
    //     u_tv_yaw_moment_raw_Nm - raw optimizer output before yaw-moment limit [Nm]
    //
    // Wrapper then clamps/uses this as desired_yaw_moment_Nm for the 4-wheel
    // allocator. JACA mode does not use these fields; JACA yaw moment is computed
    // in the wrapper from yaw-rate target.
    //
    double u_tv_yaw_moment_Nm = 0.0;
    double u_tv_yaw_moment_raw_Nm = 0.0;
    bool u_tv_yaw_moment_saturated = false;

    // -------------------------------------------------------------------------
    // Steering prediction / debug
    // -------------------------------------------------------------------------

    // Predicted next steering angle used by the vehicle model.
    //
    // PT2:
    //     predicted actual steering angle.
    //
    double delta_act_next = 0.0;

    // Predicted PT2 steering rate after one control sample.
    double delta_dot_next = 0.0;

    // Predicted next command after rate integration.
    double delta_cmd_next = 0.0;

    // Steering angle used inside vehicle prediction.
    double delta_vehicle_used = 0.0;

    // -------------------------------------------------------------------------
    // Vehicle prediction / debug
    // -------------------------------------------------------------------------

    double vy_next = 0.0;
    double r_next = 0.0;

};


// =============================================================================
//                              CONTROL COMMAND
// =============================================================================

struct ControlCommand
{
    double steer_rad = 0.0;

    /*
        Aggregate command used by debug output. The vehicle command itself
        publishes the four allocated wheel torques.
    */
    double movement_percent = 0.0;

    bool finished = false;
};


// =============================================================================
//                              GLOBAL INFO
// =============================================================================

struct GlobalHandlingInfo
{
    bool cube_mars_initialization_finished = false;
    bool has_received_first_dv_board_message = false;
    bool has_valid_path_from_pp = false;

    /*
        Typo kept intentionally for compatibility with existing skidpad wrapper:
            has_recived_odometry
    */
    bool has_odometry_message = false;
    bool has_received_imu_message = false;

    bool ready_to_start_drive = false;
    bool as_finished = false;
};


// =============================================================================
//                          LONGITUDINAL DEBUG INFO
// =============================================================================

struct LongitudinalDebugInfo
{
    double v_curr_mps = 0.0;
    double v_ref_mps = 0.0;
    double v_error_mps = 0.0;

    double torque_cmd_Nm = 0.0;
    double throttle_cmd_percent = 0.0;

    std::string phase = "waiting";
};


// =============================================================================
//                            LATERAL DEBUG INFO
// =============================================================================

struct LateralDebugInfo
{
    double ey_m = 0.0;
    double epsi_rad = 0.0;

    // Steering measured/used/debug.
    double curr_steer_rad = 0.0;
    double ref_steer_rad = 0.0;
    double steer_error_rad = 0.0;

    // Actuator debug.
    double delta_cmd_rad = 0.0;
    double delta_vehicle_used_rad = 0.0;
    double delta_enc_rad = 0.0;

    double curr_yaw_rate_radps = 0.0;
    double ref_yaw_rate_radps = 0.0;
    double yaw_rate_error_radps = 0.0;

    double curr_vy_mps = 0.0;
    double ref_vy_mps = 0.0;
    double vy_error_mps = 0.0;

    // Torque-vectoring debug.
    bool optimized_torque_vectoring_used = false;
    bool jaca_torque_vectoring_used = false;

    double tv_yaw_moment_Nm = 0.0;
    double tv_yaw_moment_raw_Nm = 0.0;
    double tv_torque_diff_Nm = 0.0;
};

} // namespace skidpad_control
