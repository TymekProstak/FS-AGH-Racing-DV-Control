#pragma once

#include "ParamBank.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace dv_control
{

struct EmergencyCheckResult
{
    bool is_safe = true;
    std::string reason;
};

struct EmergencyCheck
{
    ParamBank param_bank;

    static constexpr double MIN_ACTIVE_SPEED_MPS = 3.0;

    /*
        Only two active emergency checks remain:

            1. INS message timeout.
            2. CubeMars encoder message timeout.
    */
    static constexpr double MESSAGE_TIMEOUT_S = 0.30;

    bool initialized = false;
    bool was_driving = false;

    double last_encoder_message_time_s = 0.0;
    double last_ins_message_time_s = 0.0;

    static inline EmergencyCheckResult Safe()
    {
        return {true, ""};
    }

    static inline EmergencyCheckResult Emergency(
        const std::string& reason
    )
    {
        return {false, reason};
    }

    inline void Reset()
    {
        initialized = false;
        was_driving = false;

        last_encoder_message_time_s = 0.0;
        last_ins_message_time_s = 0.0;
    }

    inline void ArmBaselines(
        const double now_s
    )
    {
        initialized = true;

        /*
            Start both timeout counters from the moment the checker becomes
            active. This prevents an immediate timeout when entering DRIVE.
        */
        last_encoder_message_time_s = now_s;
        last_ins_message_time_s = now_s;
    }

    inline void ResetActiveChecksForLowSpeed(
        const double now_s
    )
    {
        /*
            Below 3 m/s the checker is always safe.

            Reset both timeout baselines so crossing 3 m/s cannot immediately
            trigger an emergency because of stale low-speed timestamps.
        */
        last_encoder_message_time_s = now_s;
        last_ins_message_time_s = now_s;
    }

    inline EmergencyCheckResult UpdateEmergencyCheck(
        const bool is_driving,
        const double now_s,

        const bool new_encoder_message,
        const double encoder_position_rad,
        const double encoder_reference_position_rad,

        const bool new_ins_message,
        const Eigen::Vector2d& ins_xy,

        const double vx_other_mps,

        const Eigen::Vector2d& bolide_xy,
        const std::vector<Eigen::Vector2d>& path_xy
    )
    {
        /*
            These arguments are intentionally unused now.

            CubeMars following, INS freeze and path-distance checks are
            disabled. They are kept in the function signature so wrapper.cpp
            does not need to be changed.
        */
        (void)encoder_position_rad;
        (void)encoder_reference_position_rad;
        (void)ins_xy;
        (void)bolide_xy;
        (void)path_xy;

        if (!param_bank.getBool("general.use_emergency_check"))
        {
            Reset();
            return Safe();
        }

        if (!is_driving)
        {
            Reset();
            return Safe();
        }

        const bool entered_driving =
            !was_driving;

        was_driving = true;

        if (!initialized || entered_driving)
        {
            ArmBaselines(now_s);
            return Safe();
        }

        /*
            Project rule:
                below 3 m/s everything is safe.

            No INS timeout and no CubeMars encoder timeout are evaluated below
            this speed.
        */
        if (std::abs(vx_other_mps) < MIN_ACTIVE_SPEED_MPS)
        {
            ResetActiveChecksForLowSpeed(now_s);
            return Safe();
        }

        /*
            Update timestamps only when a genuinely new message arrived.
        */
        if (new_encoder_message)
        {
            last_encoder_message_time_s = now_s;
        }

        if (new_ins_message)
        {
            last_ins_message_time_s = now_s;
        }

        // =====================================================================
        //                          INS MESSAGE TIMEOUT
        // =====================================================================

        const double ins_timeout_s =
            std::max(
                0.0,
                now_s - last_ins_message_time_s
            );

        if (ins_timeout_s > MESSAGE_TIMEOUT_S)
        {
            return Emergency(
                "INS_TIMEOUT dt_s="
                + std::to_string(ins_timeout_s)
            );
        }

        // =====================================================================
        //                    CUBEMARS ENCODER MESSAGE TIMEOUT
        // =====================================================================

        const double encoder_timeout_s =
            std::max(
                0.0,
                now_s - last_encoder_message_time_s
            );

        if (encoder_timeout_s > MESSAGE_TIMEOUT_S)
        {
            return Emergency(
                "CUBEMARS_TIMEOUT dt_s="
                + std::to_string(encoder_timeout_s)
            );
        }

        /*
            Disabled checks:

                - INS frozen / insufficient movement.
                - CubeMars not following steering reference.
                - Distance to closest path point.
        */

        return Safe();
    }
};

} // namespace dv_control