#pragma once

#include "control_types.hpp"
#include "ParamBank.hpp"
#include "control_types.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <vector>

namespace skidpad_control
{

struct CubeMarsCheck
{
    bool is_intialized = false;
    bool is_emergency = false;
    bool is_hard_emergency = false;
    bool is_car_driving = false;
    bool was_car_driving = false;

    bool encoder_stopped_sending = false;
    bool encoder_reference_tracking_error_too_high = false;

    bool hard_encoder_stopped_sending = false;
    bool hard_encoder_reference_tracking_error_too_high = false;

    double time_since_last_encoder_update = 0.0;
    double hard_time_since_last_encoder_update = 0.0;
    double encoder_position_change = 0.0;

    double encoder_reference_tracking_error_abs = 0.0;
    double encoder_reference_tracking_error_avg = 0.0;
    double hard_encoder_reference_tracking_error_avg = 0.0;

    const double CUBE_MARS_TIME_WINDOW = 0.3;
    const double HARD_EMERGENCY_TIME_WINDOW = 1.25;
    const double CUBE_MARS_REFRENCE_TRACKING_ERROR_THRESHOLD = 0.2;

    double prev_encoder_position_rad = 0.0;
    double curr_encoder_position_rad = 0.0;

    bool encoder_message_check_initialized = false;
    bool encoder_tracking_check_initialized = false;

    double last_encoder_message_time_s = 0.0;

    double prev_encoder_tracking_check_time_s = 0.0;
    double encoder_tracking_window_start_time_s = 0.0;
    double hard_encoder_tracking_window_start_time_s = 0.0;

    double encoder_tracking_error_integral = 0.0;
    double encoder_tracking_window_elapsed_s = 0.0;

    double hard_encoder_tracking_error_integral = 0.0;
    double hard_encoder_tracking_window_elapsed_s = 0.0;

    inline void ClearEmergencyFlags()
    {
        is_emergency = false;
        is_hard_emergency = false;

        encoder_stopped_sending = false;
        encoder_reference_tracking_error_too_high = false;

        hard_encoder_stopped_sending = false;
        hard_encoder_reference_tracking_error_too_high = false;
    }

    inline void DisarmForNotDriving()
    {
        is_car_driving = false;
        was_car_driving = false;
        ClearEmergencyFlags();
    }

    inline void ArmSafetyBaselines(
        const double encoder_position_rad,
        const double current_time_s
    )
    {
        is_intialized = true;
        was_car_driving = true;

        prev_encoder_position_rad = encoder_position_rad;
        curr_encoder_position_rad = encoder_position_rad;

        last_encoder_message_time_s = current_time_s;

        prev_encoder_tracking_check_time_s = current_time_s;
        encoder_tracking_window_start_time_s = current_time_s;
        hard_encoder_tracking_window_start_time_s = current_time_s;

        time_since_last_encoder_update = 0.0;
        hard_time_since_last_encoder_update = 0.0;
        encoder_position_change = 0.0;

        encoder_reference_tracking_error_abs = 0.0;
        encoder_reference_tracking_error_avg = 0.0;
        hard_encoder_reference_tracking_error_avg = 0.0;

        encoder_tracking_error_integral = 0.0;
        encoder_tracking_window_elapsed_s = 0.0;
        hard_encoder_tracking_error_integral = 0.0;
        hard_encoder_tracking_window_elapsed_s = 0.0;

        encoder_message_check_initialized = true;
        encoder_tracking_check_initialized = true;

        ClearEmergencyFlags();
    }

    inline bool UpdateCubeMarsCheck(
        const bool new_encoder_message_get,
        const double encoder_position_rad,
        const double encoder_reference_position_rad,
        const bool is_car_driving_arg,
        const double current_time_s
    )
    {
        const bool entered_driving =
            is_car_driving_arg && !was_car_driving;

        is_car_driving = is_car_driving_arg;
        ClearEmergencyFlags();

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return false;
        }

        if (!is_intialized || entered_driving)
        {
            ArmSafetyBaselines(
                encoder_position_rad,
                current_time_s
            );
            return false;
        }

        was_car_driving = true;

        if (new_encoder_message_get)
        {
            prev_encoder_position_rad = curr_encoder_position_rad;
            curr_encoder_position_rad = encoder_position_rad;

            encoder_position_change =
                std::abs(curr_encoder_position_rad - prev_encoder_position_rad);

            last_encoder_message_time_s = current_time_s;
            time_since_last_encoder_update = 0.0;
            hard_time_since_last_encoder_update = 0.0;
            encoder_stopped_sending = false;
            hard_encoder_stopped_sending = false;
        }
        else
        {
            time_since_last_encoder_update =
                current_time_s - last_encoder_message_time_s;

            if (time_since_last_encoder_update < 0.0)
            {
                time_since_last_encoder_update = 0.0;
            }

            hard_time_since_last_encoder_update =
                time_since_last_encoder_update;

            encoder_stopped_sending =
                time_since_last_encoder_update > CUBE_MARS_TIME_WINDOW;

            hard_encoder_stopped_sending =
                hard_time_since_last_encoder_update > HARD_EMERGENCY_TIME_WINDOW;
        }

        const double dt_s =
            current_time_s - prev_encoder_tracking_check_time_s;

        if (dt_s > 0.0)
        {
            encoder_reference_tracking_error_abs =
                std::abs(curr_encoder_position_rad - encoder_reference_position_rad);

            encoder_tracking_error_integral +=
                encoder_reference_tracking_error_abs * dt_s;

            hard_encoder_tracking_error_integral +=
                encoder_reference_tracking_error_abs * dt_s;

            encoder_tracking_window_elapsed_s =
                current_time_s - encoder_tracking_window_start_time_s;

            hard_encoder_tracking_window_elapsed_s =
                current_time_s - hard_encoder_tracking_window_start_time_s;

            prev_encoder_tracking_check_time_s = current_time_s;
        }

        if (encoder_tracking_window_elapsed_s >= CUBE_MARS_TIME_WINDOW)
        {
            encoder_reference_tracking_error_avg =
                encoder_tracking_error_integral / encoder_tracking_window_elapsed_s;

            encoder_reference_tracking_error_too_high =
                encoder_reference_tracking_error_avg >
                CUBE_MARS_REFRENCE_TRACKING_ERROR_THRESHOLD;

            encoder_tracking_error_integral = 0.0;
            encoder_tracking_window_elapsed_s = 0.0;
            encoder_tracking_window_start_time_s = current_time_s;
        }

        if (hard_encoder_tracking_window_elapsed_s >= HARD_EMERGENCY_TIME_WINDOW)
        {
            hard_encoder_reference_tracking_error_avg =
                hard_encoder_tracking_error_integral /
                hard_encoder_tracking_window_elapsed_s;

            hard_encoder_reference_tracking_error_too_high =
                hard_encoder_reference_tracking_error_avg >
                CUBE_MARS_REFRENCE_TRACKING_ERROR_THRESHOLD;

            hard_encoder_tracking_error_integral = 0.0;
            hard_encoder_tracking_window_elapsed_s = 0.0;
            hard_encoder_tracking_window_start_time_s = current_time_s;
        }

        is_emergency =
            encoder_stopped_sending ||
            encoder_reference_tracking_error_too_high;

        is_hard_emergency =
            hard_encoder_stopped_sending ||
            hard_encoder_reference_tracking_error_too_high;

        return is_emergency;
    }
};


struct InsPoseCheck
{
    bool is_intialized = false;
    bool is_emergency = false;
    bool is_hard_emergency = false;
    bool is_car_driving = false;
    bool was_car_driving = false;

    bool speed_over_1mps = false;
    bool speed_over_2_5mps = false;
    bool speed_over_3mps = false;

    double time_since_last_ins_update = 0.0;
    double hard_time_since_last_ins_update = 0.0;
    double ins_position_change = 0.0;
    double hard_ins_position_change = 0.0;
    double ins_velocity_change = 0.0;

    bool ins_pose_is_changing = true;
    bool ins_pose_is_stable = true;
    bool ins_velocity_is_stable = true;
    bool ins_velocity_sliding = false;
    bool ins_stopped_sending = false;

    bool hard_ins_pose_is_changing = true;
    bool hard_ins_pose_is_stable = true;
    bool hard_ins_velocity_is_stable = true;
    bool hard_ins_velocity_sliding = false;
    bool hard_ins_stopped_sending = false;

    double prev_x_m = 0.0;
    double prev_y_m = 0.0;
    double curr_x_m = 0.0;
    double curr_y_m = 0.0;

    double prev_time_s = 0.0;
    double curr_time_s = 0.0;

    const double INS_TIME_WINDOW_IS_CHANGING = 0.3;
    const double INS_CHANGING_THRESHOLD = 0.015;

    const double INS_TIME_WINDOW_IS_STABLE = 0.02;
    const double INS_STABLE_THRESHOLD = 0.5;

    const double INS_VELOCITY_STABLE_THRESHOLD = 3.00;

    const double INS_NON_TIMEOUT_MIN_SPEED_MPS = 3.0;

    const double INS_TIME_WINDOW_SLIDING = 0.02;
    const double INS_SLIDING_MIN_SPEED_MPS = 3.0;
    const double INS_SLIDING_LATERAL_TO_LONGITUDINAL_RATIO = 1.0 / 3.0;
    const double INS_SLIDING_MAX_YAW_RATE_RAD_PER_SEC = 2.0;

    const double INS_NO_NEW_MESSAGE_THRESHOLD = 0.3;
    const double HARD_EMERGENCY_TIME_WINDOW = 1.25;

    bool pose_change_check_initialized = false;
    bool hard_pose_change_check_initialized = false;
    bool pose_stability_check_initialized = false;
    bool velocity_stability_check_initialized = false;
    bool velocity_sliding_check_initialized = false;
    bool ins_message_check_initialized = false;

    bool hard_pose_unstable_candidate_active = false;
    bool hard_velocity_unstable_candidate_active = false;
    bool hard_sliding_candidate_active = false;

    double prev_pose_change_check_time_s = 0.0;
    double hard_prev_pose_change_check_time_s = 0.0;
    double prev_pose_stability_check_time_s = 0.0;
    double prev_velocity_stability_check_time_s = 0.0;
    double prev_velocity_sliding_check_time_s = 0.0;
    double last_ins_message_time_s = 0.0;

    double hard_pose_unstable_candidate_start_time_s = 0.0;
    double hard_velocity_unstable_candidate_start_time_s = 0.0;
    double hard_sliding_candidate_start_time_s = 0.0;

    Eigen::Vector2d prev_pose_change_xy = Eigen::Vector2d::Zero();
    Eigen::Vector2d hard_prev_pose_change_xy = Eigen::Vector2d::Zero();
    Eigen::Vector2d prev_pose_stability_xy = Eigen::Vector2d::Zero();
    Eigen::Vector2d prev_velocity_xy = Eigen::Vector2d::Zero();

    inline void ClearEmergencyFlags()
    {
        is_emergency = false;
        is_hard_emergency = false;

        ins_stopped_sending = false;
        ins_pose_is_changing = true;
        ins_pose_is_stable = true;
        ins_velocity_is_stable = true;
        ins_velocity_sliding = false;

        hard_ins_stopped_sending = false;
        hard_ins_pose_is_changing = true;
        hard_ins_pose_is_stable = true;
        hard_ins_velocity_is_stable = true;
        hard_ins_velocity_sliding = false;
    }

    inline void ResetHardTimer(
        bool& candidate_active,
        double& candidate_start_time_s,
        const double current_time_s
    )
    {
        candidate_active = false;
        candidate_start_time_s = current_time_s;
    }

    inline bool UpdateHardHeldCondition(
        const bool condition_active,
        bool& candidate_active,
        double& candidate_start_time_s,
        const double current_time_s
    )
    {
        if (!condition_active)
        {
            ResetHardTimer(
                candidate_active,
                candidate_start_time_s,
                current_time_s
            );
            return false;
        }

        if (!candidate_active)
        {
            candidate_active = true;
            candidate_start_time_s = current_time_s;
            return false;
        }

        return
            (current_time_s - candidate_start_time_s) >=
            HARD_EMERGENCY_TIME_WINDOW;
    }

    inline void DisarmForNotDriving()
    {
        is_car_driving = false;
        was_car_driving = false;
        ClearEmergencyFlags();
    }

    inline void ArmSafetyBaselines(
        const Eigen::Vector2d& curr_ins_xy,
        const Eigen::Vector2d& curr_ins_velocity_body_xy_mps,
        const double vx_body_other_mps,
        const double current_time_s
    )
    {
        is_intialized = true;
        was_car_driving = true;

        speed_over_1mps = std::abs(vx_body_other_mps) > 1.0;
        speed_over_2_5mps = std::abs(vx_body_other_mps) > INS_SLIDING_MIN_SPEED_MPS;
        speed_over_3mps = std::abs(vx_body_other_mps) > INS_NON_TIMEOUT_MIN_SPEED_MPS;

        curr_x_m = curr_ins_xy.x();
        curr_y_m = curr_ins_xy.y();
        curr_time_s = current_time_s;

        prev_x_m = curr_x_m;
        prev_y_m = curr_y_m;
        prev_time_s = current_time_s;

        last_ins_message_time_s = current_time_s;
        time_since_last_ins_update = 0.0;
        hard_time_since_last_ins_update = 0.0;

        prev_pose_change_xy = curr_ins_xy;
        hard_prev_pose_change_xy = curr_ins_xy;
        prev_pose_stability_xy = curr_ins_xy;
        prev_velocity_xy = curr_ins_velocity_body_xy_mps;

        prev_pose_change_check_time_s = current_time_s;
        hard_prev_pose_change_check_time_s = current_time_s;
        prev_pose_stability_check_time_s = current_time_s;
        prev_velocity_stability_check_time_s = current_time_s;
        prev_velocity_sliding_check_time_s = current_time_s;

        hard_pose_unstable_candidate_start_time_s = current_time_s;
        hard_velocity_unstable_candidate_start_time_s = current_time_s;
        hard_sliding_candidate_start_time_s = current_time_s;

        hard_pose_unstable_candidate_active = false;
        hard_velocity_unstable_candidate_active = false;
        hard_sliding_candidate_active = false;

        ins_position_change = 0.0;
        hard_ins_position_change = 0.0;
        ins_velocity_change = 0.0;

        pose_change_check_initialized = true;
        hard_pose_change_check_initialized = true;
        pose_stability_check_initialized = true;
        velocity_stability_check_initialized = true;
        velocity_sliding_check_initialized = true;
        ins_message_check_initialized = true;

        ClearEmergencyFlags();
    }

    inline void ResetNonTimeoutMotionBaselines(
        const Eigen::Vector2d& curr_ins_xy,
        const Eigen::Vector2d& curr_ins_velocity_body_xy_mps,
        const double current_time_s
    )
    {
        /*
            Below the arming speed only INS timeout is meaningful.

            Do not accumulate "not moving", jump/stability, velocity jump or
            sliding checks while the car is slow.  Re-arm their baselines to
            the current sample so that crossing the speed threshold later does
            not immediately trigger on stale low-speed data.
        */
        speed_over_3mps = false;

        ins_pose_is_changing = true;
        ins_pose_is_stable = true;
        ins_velocity_is_stable = true;
        ins_velocity_sliding = false;

        hard_ins_pose_is_changing = true;
        hard_ins_pose_is_stable = true;
        hard_ins_velocity_is_stable = true;
        hard_ins_velocity_sliding = false;

        ins_position_change = 0.0;
        hard_ins_position_change = 0.0;
        ins_velocity_change = 0.0;

        prev_pose_change_xy = curr_ins_xy;
        hard_prev_pose_change_xy = curr_ins_xy;
        prev_pose_stability_xy = curr_ins_xy;
        prev_velocity_xy = curr_ins_velocity_body_xy_mps;

        prev_pose_change_check_time_s = current_time_s;
        hard_prev_pose_change_check_time_s = current_time_s;
        prev_pose_stability_check_time_s = current_time_s;
        prev_velocity_stability_check_time_s = current_time_s;
        prev_velocity_sliding_check_time_s = current_time_s;

        hard_pose_unstable_candidate_active = false;
        hard_velocity_unstable_candidate_active = false;
        hard_sliding_candidate_active = false;

        hard_pose_unstable_candidate_start_time_s = current_time_s;
        hard_velocity_unstable_candidate_start_time_s = current_time_s;
        hard_sliding_candidate_start_time_s = current_time_s;

        pose_change_check_initialized = true;
        hard_pose_change_check_initialized = true;
        pose_stability_check_initialized = true;
        velocity_stability_check_initialized = true;
        velocity_sliding_check_initialized = true;
    }

    inline bool CheckInsNoNewMessage(
        const bool new_ins_message_get,
        const bool is_car_driving_arg,
        const double current_time_s
    )
    {
        is_car_driving = is_car_driving_arg;

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return false;
        }

        curr_time_s = current_time_s;

        if (!ins_message_check_initialized)
        {
            ins_message_check_initialized = true;
            last_ins_message_time_s = current_time_s;
            time_since_last_ins_update = 0.0;
            hard_time_since_last_ins_update = 0.0;
            ins_stopped_sending = false;
            hard_ins_stopped_sending = false;
            return false;
        }

        if (new_ins_message_get)
        {
            last_ins_message_time_s = current_time_s;
            time_since_last_ins_update = 0.0;
            hard_time_since_last_ins_update = 0.0;
            ins_stopped_sending = false;
            hard_ins_stopped_sending = false;
            return false;
        }

        time_since_last_ins_update =
            current_time_s - last_ins_message_time_s;

        if (time_since_last_ins_update < 0.0)
        {
            time_since_last_ins_update = 0.0;
        }

        hard_time_since_last_ins_update =
            time_since_last_ins_update;

        ins_stopped_sending =
            time_since_last_ins_update > INS_NO_NEW_MESSAGE_THRESHOLD;

        hard_ins_stopped_sending =
            hard_time_since_last_ins_update > HARD_EMERGENCY_TIME_WINDOW;

        return ins_stopped_sending;
    }

    inline void CheckInsPoseChange(
        const Eigen::Vector2d& curr_ins_xy,
        const double vx_body_other_mps,
        const bool is_car_driving_arg,
        const double current_time_s
    )
    {
        is_car_driving = is_car_driving_arg;

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return;
        }

        speed_over_1mps = std::abs(vx_body_other_mps) > 1.0;
        speed_over_3mps = std::abs(vx_body_other_mps) > INS_NON_TIMEOUT_MIN_SPEED_MPS;

        curr_x_m = curr_ins_xy.x();
        curr_y_m = curr_ins_xy.y();
        curr_time_s = current_time_s;

        if (!pose_change_check_initialized)
        {
            pose_change_check_initialized = true;
            prev_pose_change_xy = curr_ins_xy;
            prev_pose_change_check_time_s = current_time_s;
            ins_pose_is_changing = true;
            ins_position_change = 0.0;
        }
        else
        {
            const double time_window_s =
                current_time_s - prev_pose_change_check_time_s;

            if (time_window_s >= INS_TIME_WINDOW_IS_CHANGING)
            {
                ins_position_change =
                    (curr_ins_xy - prev_pose_change_xy).norm();

                if (!speed_over_3mps)
                {
                    ins_pose_is_changing = true;
                }
                else
                {
                    ins_pose_is_changing =
                        ins_position_change > INS_CHANGING_THRESHOLD;
                }

                prev_x_m = prev_pose_change_xy.x();
                prev_y_m = prev_pose_change_xy.y();
                prev_time_s = prev_pose_change_check_time_s;

                prev_pose_change_xy = curr_ins_xy;
                prev_pose_change_check_time_s = current_time_s;
            }
        }

        if (!hard_pose_change_check_initialized)
        {
            hard_pose_change_check_initialized = true;
            hard_prev_pose_change_xy = curr_ins_xy;
            hard_prev_pose_change_check_time_s = current_time_s;
            hard_ins_pose_is_changing = true;
            hard_ins_position_change = 0.0;
            return;
        }

        const double hard_time_window_s =
            current_time_s - hard_prev_pose_change_check_time_s;

        if (hard_time_window_s < HARD_EMERGENCY_TIME_WINDOW)
        {
            return;
        }

        hard_ins_position_change =
            (curr_ins_xy - hard_prev_pose_change_xy).norm();

        if (!speed_over_3mps)
        {
            hard_ins_pose_is_changing = true;
        }
        else
        {
            hard_ins_pose_is_changing =
                hard_ins_position_change > INS_CHANGING_THRESHOLD;
        }

        hard_prev_pose_change_xy = curr_ins_xy;
        hard_prev_pose_change_check_time_s = current_time_s;
    }

    inline void CheckInsPoseStability(
        const Eigen::Vector2d& curr_ins_xy,
        const bool is_car_driving_arg,
        const double current_time_s
    )
    {
        is_car_driving = is_car_driving_arg;

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return;
        }

        curr_x_m = curr_ins_xy.x();
        curr_y_m = curr_ins_xy.y();
        curr_time_s = current_time_s;

        if (!pose_stability_check_initialized)
        {
            pose_stability_check_initialized = true;
            prev_pose_stability_xy = curr_ins_xy;
            prev_pose_stability_check_time_s = current_time_s;
            ins_pose_is_stable = true;
        }
        else
        {
            const double time_window_s =
                current_time_s - prev_pose_stability_check_time_s;

            if (time_window_s >= INS_TIME_WINDOW_IS_STABLE)
            {
                const double pose_jump_m =
                    (curr_ins_xy - prev_pose_stability_xy).norm();

                ins_pose_is_stable =
                    pose_jump_m <= INS_STABLE_THRESHOLD;

                prev_pose_stability_xy = curr_ins_xy;
                prev_pose_stability_check_time_s = current_time_s;
            }
        }

        hard_ins_pose_is_stable =
            !UpdateHardHeldCondition(
                !ins_pose_is_stable,
                hard_pose_unstable_candidate_active,
                hard_pose_unstable_candidate_start_time_s,
                current_time_s
            );
    }

    inline void CheckInsVelocityStability(
        const Eigen::Vector2d& curr_velocity_xy_mps,
        const bool is_car_driving_arg,
        const double current_time_s
    )
    {
        is_car_driving = is_car_driving_arg;

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return;
        }

        if (!velocity_stability_check_initialized)
        {
            velocity_stability_check_initialized = true;
            prev_velocity_xy = curr_velocity_xy_mps;
            prev_velocity_stability_check_time_s = current_time_s;
            ins_velocity_is_stable = true;
            ins_velocity_change = 0.0;
        }
        else
        {
            const double time_window_s =
                current_time_s - prev_velocity_stability_check_time_s;

            if (time_window_s >= INS_TIME_WINDOW_IS_STABLE)
            {
                ins_velocity_change =
                    (curr_velocity_xy_mps - prev_velocity_xy).norm();

                ins_velocity_is_stable =
                    ins_velocity_change <= INS_VELOCITY_STABLE_THRESHOLD;

                prev_velocity_xy = curr_velocity_xy_mps;
                prev_velocity_stability_check_time_s = current_time_s;
            }
        }

        hard_ins_velocity_is_stable =
            !UpdateHardHeldCondition(
                !ins_velocity_is_stable,
                hard_velocity_unstable_candidate_active,
                hard_velocity_unstable_candidate_start_time_s,
                current_time_s
            );
    }

    inline bool CheckInsVelocitySliding(
        const double vx_body_ins_mps,
        const double vy_body_ins_mps,
        const double vx_body_other_mps,
        const double yaw_rate_rad_per_sec,
        const bool is_car_driving_arg,
        const double current_time_s
    )
    {
        /*
            Skidpad hard override:
            sliding/yaw-rate based INS check is disabled.

            Keep all related fields in safe state even if this function is
            invoked while a configuration path accidentally enables it.
        */
        (void)vx_body_ins_mps;
        (void)vy_body_ins_mps;
        (void)vx_body_other_mps;
        (void)yaw_rate_rad_per_sec;

        is_car_driving = is_car_driving_arg;

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return false;
        }

        velocity_sliding_check_initialized = true;
        prev_velocity_sliding_check_time_s = current_time_s;

        ins_velocity_sliding = false;
        hard_ins_velocity_sliding = false;

        hard_sliding_candidate_active = false;
        hard_sliding_candidate_start_time_s = current_time_s;

        return false;
    }

    inline bool UpdateInsPoseCheck(
        const bool new_ins_message_get,
        const Eigen::Vector2d& curr_ins_xy,
        const Eigen::Vector2d& curr_ins_velocity_body_xy_mps,
        const double vx_body_other_mps,
        const double yaw_rate_rad_per_sec,
        const bool is_car_driving_arg,
        const double current_time_s,
        const bool use_ins_pose_check,
        const bool use_ins_stability_check,
        const bool use_ins_sliding_velocity_check
    )
    {
        const bool entered_driving =
            is_car_driving_arg && !was_car_driving;

        is_car_driving = is_car_driving_arg;
        is_emergency = false;
        is_hard_emergency = false;

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return false;
        }

        if (!is_intialized || entered_driving)
        {
            ArmSafetyBaselines(
                curr_ins_xy,
                curr_ins_velocity_body_xy_mps,
                vx_body_other_mps,
                current_time_s
            );
            return false;
        }

        was_car_driving = true;

        /*
            INS timeout is checked whenever the car is in driving mode.

            Motion-quality checks are armed only above 3 m/s based on
            vx_body_other_mps:
                - INS not moving,
                - INS pose jumps/stability,
                - INS velocity jumps/stability,
                - INS sliding check.

            This avoids false emergencies while the car is crawling, starting,
            stopping or nearly stationary.
        */
        const bool ins_motion_checks_active =
            std::abs(vx_body_other_mps) > INS_NON_TIMEOUT_MIN_SPEED_MPS;

        speed_over_3mps =
            ins_motion_checks_active;

        if (use_ins_pose_check)
        {
            CheckInsNoNewMessage(
                new_ins_message_get,
                is_car_driving,
                current_time_s
            );
        }
        else
        {
            ins_stopped_sending = false;
            hard_ins_stopped_sending = false;
        }

        if (!ins_motion_checks_active)
        {
            ResetNonTimeoutMotionBaselines(
                curr_ins_xy,
                curr_ins_velocity_body_xy_mps,
                current_time_s
            );
        }
        else
        {
            if (use_ins_pose_check)
            {
                CheckInsPoseChange(
                    curr_ins_xy,
                    vx_body_other_mps,
                    is_car_driving,
                    current_time_s
                );
            }
            else
            {
                ins_pose_is_changing = true;
                hard_ins_pose_is_changing = true;
            }

            if (use_ins_stability_check)
            {
                CheckInsPoseStability(
                    curr_ins_xy,
                    is_car_driving,
                    current_time_s
                );

                CheckInsVelocityStability(
                    curr_ins_velocity_body_xy_mps,
                    is_car_driving,
                    current_time_s
                );
            }
            else
            {
                ins_pose_is_stable = true;
                ins_velocity_is_stable = true;
                hard_ins_pose_is_stable = true;
                hard_ins_velocity_is_stable = true;
            }

            /*
                Sliding/yaw-rate based INS check is intentionally disabled for
                skidpad. Large lateral velocity and yaw rate can be normal here.
                Keep the flags safe regardless of config.
            */
            (void)use_ins_sliding_velocity_check;
            (void)yaw_rate_rad_per_sec;

            ins_velocity_sliding = false;
            hard_ins_velocity_sliding = false;
            hard_sliding_candidate_active = false;
            hard_sliding_candidate_start_time_s = current_time_s;
        }

        /*
            Skidpad hard override:
            keep INS stability/velocity/sliding fields safe.

            INS pose-not-changing is still active together with INS timeout,
            because it catches the case where INS keeps publishing but the
            pose is frozen while the car is driving.
        */
        ins_pose_is_stable = true;
        hard_ins_pose_is_stable = true;

        ins_velocity_is_stable = true;
        hard_ins_velocity_is_stable = true;

        ins_velocity_sliding = false;
        hard_ins_velocity_sliding = false;

        hard_pose_unstable_candidate_active = false;
        hard_velocity_unstable_candidate_active = false;
        hard_sliding_candidate_active = false;

        is_emergency =
            (use_ins_pose_check && ins_stopped_sending) ||
            (use_ins_pose_check && !ins_pose_is_changing);

        is_hard_emergency =
            (use_ins_pose_check && hard_ins_stopped_sending) ||
            (use_ins_pose_check && !hard_ins_pose_is_changing);

        return is_emergency;
    }
};


struct DynamicStateCheck
{
    bool is_intialized = false;
    bool is_emergency = false;

    bool is_car_driving = false;

    bool ey_over_1_5m = false;
    bool epsi_over_75_deg = false;
    bool beta_angle_over_20_deg = false;
    bool yaw_rate_over_2_5_rad_per_sec = false;
    bool speed_over_3mps = false;

    double curr_vx_mps = 0.0;
    double curr_vy_mps = 0.0;
    double curr_ey_m = 0.0;
    double curr_epsi_rad = 0.0;
    double curr_beta_rad = 0.0;
    double curr_yaw_rate_rad_per_sec = 0.0;

    const double EY_THRESHOLD_M = 1.5;
    const double EPSI_THRESHOLD_RAD = 1.3089969389957472; // 75 deg
    const double BETA_THRESHOLD_RAD = 0.3490658503988659;
    const double YAW_RATE_THRESHOLD_RAD_PER_SEC = 2.5;
    const double DYNAMIC_CHECK_MIN_SPEED_MPS = 3.0;

    inline void ClearEmergencyFlags()
    {
        is_emergency = false;
        ey_over_1_5m = false;
        epsi_over_75_deg = false;
        beta_angle_over_20_deg = false;
        yaw_rate_over_2_5_rad_per_sec = false;
        speed_over_3mps = false;
    }

    inline void DisarmForNotDriving()
    {
        is_car_driving = false;
        ClearEmergencyFlags();
    }

    inline bool UpdateDynamicStateCheck(
        const State& state,
        const bool in_as_driving_mode,
        const double vx_body_other_mps
    )
    {
        is_car_driving = in_as_driving_mode;
        ClearEmergencyFlags();

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return false;
        }

        is_intialized = true;

        curr_vx_mps = vx_body_other_mps;
        curr_vy_mps = state.vy;
        curr_ey_m = state.ey;
        curr_epsi_rad = state.epsi;
        curr_yaw_rate_rad_per_sec = state.r;

        speed_over_3mps =
            std::abs(vx_body_other_mps) > DYNAMIC_CHECK_MIN_SPEED_MPS;

        if (!speed_over_3mps)
        {
            /*
                Do not run dynamic geometry/beta/yaw checks at low speed.
                The speed source is vx_body_other_mps, usually encoder-based,
                so the arming decision does not depend on possibly broken INS.
            */
            return false;
        }

        curr_beta_rad =
            std::atan2(curr_vy_mps, vx_body_other_mps);

        ey_over_1_5m =
            std::abs(curr_ey_m) > EY_THRESHOLD_M;

        epsi_over_75_deg =
            std::abs(curr_epsi_rad) > EPSI_THRESHOLD_RAD;

        /*
            Skidpad can naturally generate large sideslip and yaw rate.
            Do not emergency on beta/yaw-rate here. Keep diagnostic flags
            explicitly safe/false.
        */
        beta_angle_over_20_deg = false;
        yaw_rate_over_2_5_rad_per_sec = false;

        is_emergency =
            ey_over_1_5m ||
            epsi_over_75_deg;

        return is_emergency;
    }
};


struct PathPlannerCheck
{
    bool is_intialized = false;
    bool is_emergency = false;
    bool is_hard_emergency = false;

    bool is_car_driving = false;
    bool was_car_driving = false;

    bool is_track_closed = false;
    bool path_planner_is_valid = false;

    bool path_planner_stopped_sending = false;
    bool hard_path_planner_stopped_sending = false;

    bool bolide_in_break_between_first_and_last = false;
    bool bolide_before_first_point = false;
    bool bolide_after_last_point = false;

    double bolide_heading_dot_path_dir = 0.0;

    bool is_spline_valid = false;

    const double PATH_PLANNER_NO_NEW_MESSAGE_THRESHOLD = 0.5;
    const double HARD_EMERGENCY_TIME_WINDOW = 1.25;
    const double PATH_PLANNER_MAX_DIST_TO_BREAK_M = 2.0;

    double time_since_last_path_planner_update = 0.0;
    double hard_time_since_last_path_planner_update = 0.0;
    double last_path_planner_message_time_s = 0.0;

    static inline bool CheckBolideBeforeFirstPoint(
        const Eigen::Vector2d& bolide_xy,
        const std::vector<Eigen::Vector2d>& path_xy
    )
    {
        if (path_xy.size() < 2)
        {
            return false;
        }

        const Eigen::Vector2d& first = path_xy[0];
        const Eigen::Vector2d& second = path_xy[1];

        const Eigen::Vector2d path_dir = second - first;
        const double len2 = path_dir.squaredNorm();

        if (len2 < 1.0e-8)
        {
            return false;
        }

        const double projection =
            (bolide_xy - first).dot(path_dir) / len2;

        return projection < 0.0;
    }

    static inline bool CheckBolideAfterLastPoint(
        const Eigen::Vector2d& bolide_xy,
        const std::vector<Eigen::Vector2d>& path_xy
    )
    {
        if (path_xy.size() < 2)
        {
            return false;
        }

        const int last_id =
            static_cast<int>(path_xy.size()) - 1;

        const Eigen::Vector2d& before_last = path_xy[last_id - 1];
        const Eigen::Vector2d& last = path_xy[last_id];

        const Eigen::Vector2d path_dir = last - before_last;
        const double len2 = path_dir.squaredNorm();

        if (len2 < 1.0e-8)
        {
            return false;
        }

        const double projection =
            (bolide_xy - before_last).dot(path_dir) / len2;

        return projection > 1.0;
    }

    static inline bool CheckBolideInBreakBetweenLastAndFirst(
        const double bolide_yaw_rad,
        const std::vector<Eigen::Vector2d>& path_xy,
        double& heading_dot_path_dir_out
    )
    {
        heading_dot_path_dir_out =
            0.0;

        if (path_xy.size() < 2)
        {
            return false;
        }

        const Eigen::Vector2d& first =
            path_xy.front();

        const Eigen::Vector2d& last =
            path_xy.back();

        const Eigen::Vector2d path_dir =
            last - first;

        const double path_len =
            path_dir.norm();

        if (path_len < 1.0e-8)
        {
            return false;
        }

        const Eigen::Vector2d bolide_heading(
            std::cos(bolide_yaw_rad),
            std::sin(bolide_yaw_rad)
        );

        heading_dot_path_dir_out =
            bolide_heading.dot(path_dir / path_len);

        return
            heading_dot_path_dir_out < 0.0;
    }

    inline bool PathIsAlwaysSafeBecauseTrackClosed() const
    {
        return is_track_closed && is_spline_valid;
    }

    inline void ClearEmergencyFlags()
    {
        is_emergency = false;
        is_hard_emergency = false;

        path_planner_stopped_sending = false;
        hard_path_planner_stopped_sending = false;

        bolide_in_break_between_first_and_last = false;
        bolide_before_first_point = false;
        bolide_after_last_point = false;

        bolide_heading_dot_path_dir = 0.0;
    }

    inline void DisarmForNotDriving()
    {
        is_car_driving = false;
        was_car_driving = false;
        ClearEmergencyFlags();
    }

    inline void ArmSafetyBaselines(
        const double current_time_s
    )
    {
        is_intialized = true;
        was_car_driving = true;

        last_path_planner_message_time_s = current_time_s;
        time_since_last_path_planner_update = 0.0;
        hard_time_since_last_path_planner_update = 0.0;

        path_planner_is_valid = true;

        ClearEmergencyFlags();
    }

    inline bool UpdatePathPlannerCheck(
        const Eigen::Vector2d& bolide_xy,
        const std::vector<Eigen::Vector2d>& path_xy,
        const bool spline_is_valid,
        const bool new_path_planner_message_received,
        const double current_time_s,
        const bool is_car_driving_arg,
        const bool is_track_closed_arg,
        const double bolide_yaw_rad
    )
    {
        const bool entered_driving =
            is_car_driving_arg && !was_car_driving;

        is_car_driving = is_car_driving_arg;
        ClearEmergencyFlags();

        if (!is_car_driving)
        {
            DisarmForNotDriving();
            return false;
        }

        is_track_closed = is_track_closed_arg;
        is_spline_valid = spline_is_valid;

        if (!is_intialized || entered_driving)
        {
            ArmSafetyBaselines(current_time_s);
            return false;
        }

        was_car_driving = true;

        /*
            Important project rule:
            closed track + valid spline is always safe from the path-planner
            point of view. In that case, do not check timeout, before/after,
            break between first and last, or path size.
        */
        if (is_track_closed && is_spline_valid)
        {
            path_planner_is_valid = true;
            time_since_last_path_planner_update = 0.0;
            hard_time_since_last_path_planner_update = 0.0;
            is_emergency = false;
            is_hard_emergency = false;
            return false;
        }

        if (new_path_planner_message_received)
        {
            last_path_planner_message_time_s = current_time_s;
            time_since_last_path_planner_update = 0.0;
            hard_time_since_last_path_planner_update = 0.0;
            path_planner_stopped_sending = false;
            hard_path_planner_stopped_sending = false;
        }
        else
        {
            time_since_last_path_planner_update =
                current_time_s - last_path_planner_message_time_s;

            if (time_since_last_path_planner_update < 0.0)
            {
                time_since_last_path_planner_update = 0.0;
            }

            hard_time_since_last_path_planner_update =
                time_since_last_path_planner_update;

            path_planner_stopped_sending =
                time_since_last_path_planner_update >
                PATH_PLANNER_NO_NEW_MESSAGE_THRESHOLD;

            hard_path_planner_stopped_sending =
                hard_time_since_last_path_planner_update >
                HARD_EMERGENCY_TIME_WINDOW;
        }

        path_planner_is_valid =
            path_xy.size() >= 2;

        is_hard_emergency =
            hard_path_planner_stopped_sending;

        if (!is_spline_valid)
        {
            is_emergency = true;
            return true;
        }

        if (path_planner_stopped_sending)
        {
            is_emergency = true;
            return true;
        }

        if (!path_planner_is_valid)
        {
            is_emergency = true;
            return true;
        }

        bolide_before_first_point =
            CheckBolideBeforeFirstPoint(
                bolide_xy,
                path_xy
            );

        bolide_after_last_point =
            CheckBolideAfterLastPoint(
                bolide_xy,
                path_xy
            );

        bolide_in_break_between_first_and_last =
            CheckBolideInBreakBetweenLastAndFirst(
                bolide_yaw_rad,
                path_xy,
                bolide_heading_dot_path_dir
            );

        is_emergency =
            bolide_before_first_point ||
            bolide_after_last_point ||
            bolide_in_break_between_first_and_last;

        return is_emergency;
    }
};


struct EmergencyCheck
{
    CubeMarsCheck cube_mars_check;
    InsPoseCheck ins_pose_check;
    DynamicStateCheck dynamic_state_check;
    PathPlannerCheck path_planner_check;

    bool is_emergency = false;
    bool is_hard_emergency = false;
    bool is_car_driving = false;

    std::string emergency_reason = "";
    std::string hard_emergency_reason = "";

    skidpad_control::ParamBank param_bank;

    inline void AppendEmergencyReason(
        const std::string& reason
    )
    {
        if (!emergency_reason.empty())
        {
            emergency_reason += " | ";
        }

        emergency_reason += reason;
    }

    inline void AppendHardEmergencyReason(
        const std::string& reason
    )
    {
        if (!hard_emergency_reason.empty())
        {
            hard_emergency_reason += " | ";
        }

        hard_emergency_reason += reason;
    }

    inline bool UpdateEmergencyCheck(
        const skidpad_control::State& state,
        const bool in_as_driving_mode,
        const double current_time_s,

        const bool new_encoder_message_get,
        const double encoder_position_rad,
        const double encoder_reference_position_rad,

        const bool new_ins_message_get,
        const Eigen::Vector2d& curr_ins_xy,
        const Eigen::Vector2d& curr_ins_velocity_body_xy_mps,
        const double vx_body_other_mps,

        const Eigen::Vector2d& bolide_xy,
        const std::vector<Eigen::Vector2d>& path_xy,
        const bool spline_is_valid,
        const bool new_path_planner_message_received,
        const bool path_is_track_closed
    )
    {
        is_car_driving = in_as_driving_mode;
        is_emergency = false;
        is_hard_emergency = false;
        emergency_reason.clear();
        hard_emergency_reason.clear();

        const bool use_emergency_check =
            param_bank.getBool("general.use_emergency_check");

        if (!use_emergency_check)
        {
            return false;
        }

        if (!is_car_driving)
        {
            cube_mars_check.DisarmForNotDriving();
            ins_pose_check.DisarmForNotDriving();
            dynamic_state_check.DisarmForNotDriving();
            path_planner_check.DisarmForNotDriving();

            emergency_reason.clear();
            hard_emergency_reason.clear();
            is_emergency = false;
            is_hard_emergency = false;

            return false;
        }

        const bool use_cube_mars_encoder_check =
            param_bank.getBool("general.use_cube_mars_encoder_check");

        const bool use_cube_mars_following_check =
            param_bank.getBool("general.use_cube_mars_following_check");

        const bool use_path_planner_check =
            param_bank.getBool("general.use_path_planner_check");

        const bool use_dynamic_state_check =
            param_bank.getBool("general.use_dynamic_state_check");

        const bool use_ins_pose_check =
            param_bank.getBool("general.use_ins_pose_check");

        const bool use_ins_stability_check =
            param_bank.getBool("general.use_ins_stability_check");

        const bool use_ins_sliding_velocity_check =
            param_bank.getBool("general.use_ins_sliding_velocity_check");

        /*
            Skidpad hard override:
            stability/sliding config flags can stay in JSON/ParamBank, but
            their diagnostic fields are forced safe below.
        */
        (void)use_ins_stability_check;
        (void)use_ins_sliding_velocity_check;

        if (use_cube_mars_encoder_check || use_cube_mars_following_check)
        {
            cube_mars_check.UpdateCubeMarsCheck(
                new_encoder_message_get,
                encoder_position_rad,
                encoder_reference_position_rad,
                is_car_driving,
                current_time_s
            );

            if (use_cube_mars_encoder_check &&
                cube_mars_check.encoder_stopped_sending)
            {
                AppendEmergencyReason(
                    "CubeMars encoder stopped sending, time_since_last_encoder_update=" +
                    std::to_string(cube_mars_check.time_since_last_encoder_update)
                );
            }

            if (use_cube_mars_encoder_check &&
                cube_mars_check.hard_encoder_stopped_sending)
            {
                AppendHardEmergencyReason(
                    "HARD CubeMars encoder stopped sending, hard_time_since_last_encoder_update=" +
                    std::to_string(cube_mars_check.hard_time_since_last_encoder_update)
                );
            }

            if (use_cube_mars_following_check &&
                cube_mars_check.encoder_reference_tracking_error_too_high)
            {
                AppendEmergencyReason(
                    "CubeMars reference tracking error too high, avg_error_rad=" +
                    std::to_string(cube_mars_check.encoder_reference_tracking_error_avg)
                );
            }

            if (use_cube_mars_following_check &&
                cube_mars_check.hard_encoder_reference_tracking_error_too_high)
            {
                AppendHardEmergencyReason(
                    "HARD CubeMars reference tracking error too high, hard_avg_error_rad=" +
                    std::to_string(cube_mars_check.hard_encoder_reference_tracking_error_avg)
                );
            }
        }

        if (use_dynamic_state_check)
        {
            dynamic_state_check.UpdateDynamicStateCheck(
                state,
                is_car_driving,
                vx_body_other_mps
            );

            /*
                Skidpad hard override before reason aggregation.
                Keep beta/yaw-rate diagnostic flags safe even if they were
                updated inside DynamicStateCheck.
            */
            dynamic_state_check.beta_angle_over_20_deg = false;
            dynamic_state_check.yaw_rate_over_2_5_rad_per_sec = false;

            if (dynamic_state_check.ey_over_1_5m)
            {
                AppendEmergencyReason(
                    "Dynamic state lateral error over 1.5 m, ey_m=" +
                    std::to_string(dynamic_state_check.curr_ey_m)
                );
            }

            if (dynamic_state_check.epsi_over_75_deg)
            {
                AppendEmergencyReason(
                    "Dynamic state heading error over 75 deg, epsi_rad=" +
                    std::to_string(dynamic_state_check.curr_epsi_rad)
                );
            }
        }

        if (use_ins_pose_check ||
            use_ins_stability_check ||
            use_ins_sliding_velocity_check)
        {
            ins_pose_check.UpdateInsPoseCheck(
                new_ins_message_get,
                curr_ins_xy,
                curr_ins_velocity_body_xy_mps,
                vx_body_other_mps,
                state.r,
                is_car_driving,
                current_time_s,
                use_ins_pose_check,
                use_ins_stability_check,
                use_ins_sliding_velocity_check
            );

            /*
                Skidpad hard override before reason aggregation.
                Stability/velocity/sliding fields may be updated internally,
                but they are forced safe before any later checks.

                Pose-not-changing is intentionally NOT forced safe.
            */
            ins_pose_check.ins_pose_is_stable = true;
            ins_pose_check.hard_ins_pose_is_stable = true;

            ins_pose_check.ins_velocity_is_stable = true;
            ins_pose_check.hard_ins_velocity_is_stable = true;

            ins_pose_check.ins_velocity_sliding = false;
            ins_pose_check.hard_ins_velocity_sliding = false;

            if (use_ins_pose_check &&
                ins_pose_check.ins_stopped_sending)
            {
                AppendEmergencyReason(
                    "INS stopped sending, time_since_last_ins_update=" +
                    std::to_string(ins_pose_check.time_since_last_ins_update)
                );
            }

            if (use_ins_pose_check &&
                !ins_pose_check.ins_pose_is_changing)
            {
                AppendEmergencyReason(
                    "INS pose is not changing while car is driving, position_change_m=" +
                    std::to_string(ins_pose_check.ins_position_change)
                );
            }

            if (use_ins_pose_check &&
                ins_pose_check.hard_ins_stopped_sending)
            {
                AppendHardEmergencyReason(
                    "HARD INS stopped sending, hard_time_since_last_ins_update=" +
                    std::to_string(ins_pose_check.hard_time_since_last_ins_update)
                );
            }

            if (use_ins_pose_check &&
                !ins_pose_check.hard_ins_pose_is_changing)
            {
                AppendHardEmergencyReason(
                    "HARD INS pose is not changing while car is driving, hard_position_change_m=" +
                    std::to_string(ins_pose_check.hard_ins_position_change)
                );
            }
        }

        if (use_path_planner_check)
        {
            path_planner_check.UpdatePathPlannerCheck(
                bolide_xy,
                path_xy,
                spline_is_valid,
                new_path_planner_message_received,
                current_time_s,
                is_car_driving,
                path_is_track_closed,
                state.yaw
            );

            const bool path_is_always_safe =
                path_planner_check.PathIsAlwaysSafeBecauseTrackClosed();

            if (!path_is_always_safe)
            {
                if (!path_planner_check.is_spline_valid)
                {
                    AppendEmergencyReason(
                        "Path planner spline invalid"
                    );
                }

                if (path_planner_check.path_planner_stopped_sending)
                {
                    AppendEmergencyReason(
                        "Path planner stopped sending, time_since_last_path_update=" +
                        std::to_string(path_planner_check.time_since_last_path_planner_update)
                    );
                }

                if (path_planner_check.hard_path_planner_stopped_sending)
                {
                    AppendHardEmergencyReason(
                        "HARD Path planner stopped sending, hard_time_since_last_path_update=" +
                        std::to_string(path_planner_check.hard_time_since_last_path_planner_update)
                    );
                }

                if (!path_planner_check.path_planner_is_valid)
                {
                    AppendEmergencyReason(
                        "Path planner path invalid / too short"
                    );
                }

                if (path_planner_check.bolide_before_first_point)
                {
                    AppendEmergencyReason(
                        "Bolide before first path point"
                    );
                }

                if (path_planner_check.bolide_after_last_point)
                {
                    AppendEmergencyReason(
                        "Bolide after last path point"
                    );
                }

                if (path_planner_check.bolide_in_break_between_first_and_last)
                {
                    AppendEmergencyReason(
                        "Bolide heading opposite to path direction, heading_dot_path_dir=" +
                        std::to_string(path_planner_check.bolide_heading_dot_path_dir)
                    );
                }
            }
        }

        is_hard_emergency =
            !hard_emergency_reason.empty();

        if (is_hard_emergency)
        {
            AppendEmergencyReason(hard_emergency_reason);
        }

        is_emergency =
            !emergency_reason.empty();

        if (!is_car_driving)
        {
            is_emergency = false;
            is_hard_emergency = false;
            emergency_reason.clear();
            hard_emergency_reason.clear();
        }

        return is_emergency;
    }
};

} // namespace skidpad_control
