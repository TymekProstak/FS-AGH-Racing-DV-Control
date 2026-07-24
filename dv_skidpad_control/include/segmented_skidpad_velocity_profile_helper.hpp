#pragma once

#include "ParamBank.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace skidpad_control
{

/*
    Header-only segmented skidpad reference-speed helper.

    Progress-based sequence:

        ENTRY:
            right_down

        ALPHA1:
            right_down -> right_up

        RIGHT_MIDDLE:
            right_up

        ALPHA2 + ALPHA3:
            one continuous braking ramp:
                right_up -> left_down

            ALPHA2 is the final part of the right circle.
            ALPHA3 is the first part of the left circle.

        ALPHA4:
            left_down -> left_up

        LEFT_MIDDLE:
            left_up

    Required ParamBank keys:

        general.v_ref
        general.use_segmented_velocity_profile
        general.segment_ref_ramped

        Shared fallback keys:
            general.v_ref_segmented_up_mps
            general.v_ref_segmented_down_mps
            general.segment_angle_length_deg

        New split profile keys:
            general.v_ref_segmented_right_down_mps
            general.v_ref_segmented_right_up_mps
            general.v_ref_segmented_left_down_mps
            general.v_ref_segmented_left_up_mps

            general.segment_alpha1_deg
            general.segment_alpha2_deg
            general.segment_alpha3_deg
            general.segment_alpha4_deg

    Skidpad handbook reference radius:
        R_ref = 9.125 m

    This is sequence/progress based, not XY-sector based.
*/

enum class SegmentedSkidpadSpeedRegion
{
    CONSTANT = 0,

    ENTRY_STRAIGHT_RIGHT_DOWN,

    ALPHA1_RIGHT_DOWN_TO_RIGHT_UP,
    RIGHT_UP,

    ALPHA2_RIGHT_FINAL_BRAKING,
    ALPHA3_LEFT_ENTRY_BRAKING,

    ALPHA4_LEFT_DOWN_TO_LEFT_UP,
    LEFT_UP,

    AFTER_PROFILE_LEFT_UP,

    /*
        Aliases used by diagnostic output.
    */
    ENTRY_STRAIGHT_DOWN = ENTRY_STRAIGHT_RIGHT_DOWN,
    RIGHT_ENTRY_ALPHA = ALPHA1_RIGHT_DOWN_TO_RIGHT_UP,
    RIGHT_FINAL_ALPHA = ALPHA2_RIGHT_FINAL_BRAKING,
    LEFT_ENTRY_ALPHA = ALPHA3_LEFT_ENTRY_BRAKING,
    AFTER_PROFILE_UP = AFTER_PROFILE_LEFT_UP
};

struct SegmentedSkidpadSpeedSample
{
    double s_m = 0.0;
    double v_ref_mps = 0.0;
    SegmentedSkidpadSpeedRegion region =
        SegmentedSkidpadSpeedRegion::CONSTANT;
};

struct SegmentedSkidpadSpeedProfile
{
    bool enabled = false;
    bool ramped = false;

    double v_default_mps = 0.0;

    /*
        Shared fallback values and diagnostic reference. evaluate() uses the
        split right/left values below.
    */
    double v_up_mps = 0.0;
    double v_down_mps = 0.0;
    double alpha_deg = 30.0;

    double v_right_down_mps = 0.0;
    double v_right_up_mps = 0.0;
    double v_left_down_mps = 0.0;
    double v_left_up_mps = 0.0;

    double alpha1_deg = 30.0;
    double alpha2_deg = 30.0;
    double alpha3_deg = 30.0;
    double alpha4_deg = 30.0;

    /*
        If your path includes a real approach straight before the first right
        circle, pass its length from the caller. If your path starts at the
        right circle entry, leave this as 0.
    */
    double entry_straight_length_m = 0.0;

    double reference_radius_m = 9.125;

    static SegmentedSkidpadSpeedProfile fromParamBank(
        const ParamBank& P,
        double entry_straight_length_m_arg = 0.0
    )
    {
        SegmentedSkidpadSpeedProfile out;

        out.enabled =
            P.getBool("general.use_segmented_velocity_profile");

        out.ramped =
            P.getBool("general.segment_ref_ramped");

        out.v_default_mps =
            P.get("general.v_ref");

        // // out.v_up_mps =
        // //     P.get("general.v_ref_segmented_up_mps");

        // // out.v_down_mps =
        // //     P.get("general.v_ref_segmented_down_mps");

        // out.alpha_deg =
        //     P.get("general.segment_angle_length_deg");

        out.v_right_down_mps =
            P.get("general.v_ref_segmented_right_down_mps");

        out.v_right_up_mps =
            P.get("general.v_ref_segmented_right_up_mps");

        out.v_left_down_mps =
            P.get("general.v_ref_segmented_left_down_mps");

        out.v_left_up_mps =
            P.get("general.v_ref_segmented_left_up_mps");

        out.alpha1_deg =
            P.get("general.segment_alpha1_deg");

        out.alpha2_deg =
            P.get("general.segment_alpha2_deg");

        out.alpha3_deg =
            P.get("general.segment_alpha3_deg");

        out.alpha4_deg =
            P.get("general.segment_alpha4_deg");

        out.entry_straight_length_m =
            entry_straight_length_m_arg;

        out.validate();

        return out;
    }

    void validate() const
    {
        requireFinitePositive("v_default_mps", v_default_mps);

       // requireFinitePositive("v_up_mps", v_up_mps);
        //requireFinitePositive("v_down_mps", v_down_mps);

        requireFinitePositive("v_right_down_mps", v_right_down_mps);
        requireFinitePositive("v_right_up_mps", v_right_up_mps);
        requireFinitePositive("v_left_down_mps", v_left_down_mps);
        requireFinitePositive("v_left_up_mps", v_left_up_mps);

        requireFinitePositive("reference_radius_m", reference_radius_m);
        requireFiniteNonNegative("entry_straight_length_m", entry_straight_length_m);

       // validateAlphaDeg("segment_angle_length_deg", alpha_deg);
        validateAlphaDeg("segment_alpha1_deg", alpha1_deg);
        validateAlphaDeg("segment_alpha2_deg", alpha2_deg);
        validateAlphaDeg("segment_alpha3_deg", alpha3_deg);
        validateAlphaDeg("segment_alpha4_deg", alpha4_deg);

        /*
            With every alpha in (0, 180), the right-up and left-up plateaus
            are guaranteed to remain non-negative for 2 circles = 720 deg.
            This check is still kept explicit for clearer error messages.
        */
        if (alpha1_deg + alpha2_deg >= 720.0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] alpha1 + alpha2 must be < 720 deg"
            );
        }

        if (alpha3_deg + alpha4_deg >= 720.0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] alpha3 + alpha4 must be < 720 deg"
            );
        }
    }

    double alpha1LengthM() const
    {
        return reference_radius_m * deg2rad(alpha1_deg);
    }

    double alpha2LengthM() const
    {
        return reference_radius_m * deg2rad(alpha2_deg);
    }

    double alpha3LengthM() const
    {
        return reference_radius_m * deg2rad(alpha3_deg);
    }

    double alpha4LengthM() const
    {
        return reference_radius_m * deg2rad(alpha4_deg);
    }

    double circleLengthM() const
    {
        return 2.0 * pi() * reference_radius_m;
    }

    double totalModeledLengthM() const
    {
        return entry_straight_length_m + 4.0 * circleLengthM();
    }

    SegmentedSkidpadSpeedSample evaluate(double s_query_m) const
    {
        validateFinite("s_query_m", s_query_m);

        SegmentedSkidpadSpeedSample out;
        out.s_m =
            std::max(0.0, s_query_m);

        if (!enabled)
        {
            out.v_ref_mps =
                v_default_mps;

            out.region =
                SegmentedSkidpadSpeedRegion::CONSTANT;

            return out;
        }

        const double L_entry =
            entry_straight_length_m;

        const double L_alpha1 =
            alpha1LengthM();

        const double L_alpha2 =
            alpha2LengthM();

        const double L_alpha3 =
            alpha3LengthM();

        const double L_alpha4 =
            alpha4LengthM();

        const double L_circle =
            circleLengthM();

        /*
            Two right circles, then two left circles.

            Boundaries:

                b0 = start alpha1
                b1 = end alpha1

                b2 = start alpha2, final part of right circle
                b3 = right->left transition point

                b4 = end alpha3, reached left_down
                b5 = end alpha4, reached left_up

                b6 = end modeled skidpad profile

            alpha2 + alpha3 is one continuous ramp:
                right_up -> left_down
        */
        const double b0 =
            L_entry;

        const double b1 =
            b0 + L_alpha1;

        const double b2 =
            b0 + 2.0 * L_circle - L_alpha2;

        const double b3 =
            b0 + 2.0 * L_circle;

        const double b4 =
            b3 + L_alpha3;

        const double b5 =
            b4 + L_alpha4;

        const double b6 =
            b0 + 4.0 * L_circle;

        const double s =
            out.s_m;

        if (s < b0)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::ENTRY_STRAIGHT_RIGHT_DOWN;

            out.v_ref_mps =
                v_right_down_mps;

            return out;
        }

        if (s < b1)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::ALPHA1_RIGHT_DOWN_TO_RIGHT_UP;

            if (ramped)
            {
                const double a =
                    normalized01(s, b0, b1);

                out.v_ref_mps =
                    lerp(v_right_down_mps, v_right_up_mps, a);
            }
            else
            {
                out.v_ref_mps =
                    v_right_down_mps;
            }

            return out;
        }

        if (s < b2)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::RIGHT_UP;

            out.v_ref_mps =
                v_right_up_mps;

            return out;
        }

        if (s < b3)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::ALPHA2_RIGHT_FINAL_BRAKING;

            if (ramped)
            {
                const double a =
                    normalized01(s, b2, b4);

                out.v_ref_mps =
                    lerp(v_right_up_mps, v_left_down_mps, a);
            }
            else
            {
                out.v_ref_mps =
                    v_left_down_mps;
            }

            return out;
        }

        if (s < b4)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::ALPHA3_LEFT_ENTRY_BRAKING;

            if (ramped)
            {
                const double a =
                    normalized01(s, b2, b4);

                out.v_ref_mps =
                    lerp(v_right_up_mps, v_left_down_mps, a);
            }
            else
            {
                out.v_ref_mps =
                    v_left_down_mps;
            }

            return out;
        }

        if (s < b5)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::ALPHA4_LEFT_DOWN_TO_LEFT_UP;

            if (ramped)
            {
                const double a =
                    normalized01(s, b4, b5);

                out.v_ref_mps =
                    lerp(v_left_down_mps, v_left_up_mps, a);
            }
            else
            {
                out.v_ref_mps =
                    v_left_down_mps;
            }

            return out;
        }

        if (s < b6)
        {
            out.region =
                SegmentedSkidpadSpeedRegion::LEFT_UP;

            out.v_ref_mps =
                v_left_up_mps;

            return out;
        }

        out.region =
            SegmentedSkidpadSpeedRegion::AFTER_PROFILE_LEFT_UP;

        out.v_ref_mps =
            v_left_up_mps;

        return out;
    }

    std::vector<double> buildVelocityHorizon(
        double s0_m,
        double vx_mps,
        int N,
        double dt_s
    ) const
    {
        validateFinite("s0_m", s0_m);
        validateFinite("vx_mps", vx_mps);
        validateFinitePositive("dt_s", dt_s);

        if (N <= 0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] N must be > 0"
            );
        }

        const double ds =
            std::max(std::abs(vx_mps), 0.5) * dt_s;

        std::vector<double> out;
        out.reserve(static_cast<std::size_t>(N));

        for (int k = 0; k < N; ++k)
        {
            const double s_query =
                s0_m + static_cast<double>(k) * ds;

            out.push_back(
                evaluate(s_query).v_ref_mps
            );
        }

        return out;
    }

    std::vector<SegmentedSkidpadSpeedSample> buildSampleHorizon(
        double s0_m,
        double vx_mps,
        int N,
        double dt_s
    ) const
    {
        validateFinite("s0_m", s0_m);
        validateFinite("vx_mps", vx_mps);
        validateFinitePositive("dt_s", dt_s);

        if (N <= 0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] N must be > 0"
            );
        }

        const double ds =
            std::max(std::abs(vx_mps), 0.5) * dt_s;

        std::vector<SegmentedSkidpadSpeedSample> out;
        out.reserve(static_cast<std::size_t>(N));

        for (int k = 0; k < N; ++k)
        {
            const double s_query =
                s0_m + static_cast<double>(k) * ds;

            out.push_back(
                evaluate(s_query)
            );
        }

        return out;
    }

    static const char* regionName(SegmentedSkidpadSpeedRegion r)
    {
        switch (r)
        {
            case SegmentedSkidpadSpeedRegion::CONSTANT:
                return "CONSTANT";

            case SegmentedSkidpadSpeedRegion::ENTRY_STRAIGHT_RIGHT_DOWN:
                return "ENTRY_STRAIGHT_RIGHT_DOWN";

            case SegmentedSkidpadSpeedRegion::ALPHA1_RIGHT_DOWN_TO_RIGHT_UP:
                return "ALPHA1_RIGHT_DOWN_TO_RIGHT_UP";

            case SegmentedSkidpadSpeedRegion::RIGHT_UP:
                return "RIGHT_UP";

            case SegmentedSkidpadSpeedRegion::ALPHA2_RIGHT_FINAL_BRAKING:
                return "ALPHA2_RIGHT_FINAL_BRAKING";

            case SegmentedSkidpadSpeedRegion::ALPHA3_LEFT_ENTRY_BRAKING:
                return "ALPHA3_LEFT_ENTRY_BRAKING";

            case SegmentedSkidpadSpeedRegion::ALPHA4_LEFT_DOWN_TO_LEFT_UP:
                return "ALPHA4_LEFT_DOWN_TO_LEFT_UP";

            case SegmentedSkidpadSpeedRegion::LEFT_UP:
                return "LEFT_UP";

            case SegmentedSkidpadSpeedRegion::AFTER_PROFILE_LEFT_UP:
                return "AFTER_PROFILE_LEFT_UP";
        }

        return "UNKNOWN";
    }

private:
    static double pi()
    {
        return 3.141592653589793238462643383279502884;
    }

    static double deg2rad(double deg)
    {
        return deg * pi() / 180.0;
    }

    static void validateAlphaDeg(const std::string& name, double v)
    {
        if (!std::isfinite(v) || v <= 0.0 || v >= 180.0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] " + name + " must be finite and in (0, 180)"
            );
        }
    }

    static double normalized01(double x, double x0, double x1)
    {
        const double dx =
            x1 - x0;

        if (!std::isfinite(dx) || std::abs(dx) < 1.0e-12)
        {
            return 0.0;
        }

        const double a =
            (x - x0) / dx;

        return std::max(0.0, std::min(1.0, a));
    }

    static double lerp(double a, double b, double t)
    {
        return a + (b - a) * std::max(0.0, std::min(1.0, t));
    }

    static void validateFinite(const std::string& name, double v)
    {
        if (!std::isfinite(v))
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] " + name + " must be finite"
            );
        }
    }

    static void validateFinitePositive(const std::string& name, double v)
    {
        if (!std::isfinite(v) || v <= 0.0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] " + name + " must be finite and > 0"
            );
        }
    }

    static void requireFinitePositive(const std::string& name, double v)
    {
        validateFinitePositive(name, v);
    }

    static void requireFiniteNonNegative(const std::string& name, double v)
    {
        if (!std::isfinite(v) || v < 0.0)
        {
            throw std::runtime_error(
                "[SegmentedSkidpadSpeedProfile] " + name + " must be finite and >= 0"
            );
        }
    }
};


/*
    Read N exactly like the active lateral-controller horizon selection.

    This is intentionally header-only and independent of the solver. If you
    later move it into path_utils.cpp, use the same logic as readActiveHorizonN.
*/
inline int readActiveSegmentedVelocityHorizonN(const ParamBank& P)
{
    return static_cast<int>(
        std::llround(P.get("model.ltv_mpc_unbounded.N"))
    );
}


/*
    Main helper for current code:

        const auto v_ref_horizon =
            buildSegmentedSkidpadVelocityRefHorizon(
                param_,
                along_skidpad_ref_path_m_,
                current_state_.vx,
                entry_straight_length_m
            );

    It computes N from the currently selected controller and dt from
    model.frequency.steer_cmd_loop_hz, mirroring buildCurvatureHorizon.
*/
inline std::vector<double> buildSegmentedSkidpadVelocityRefHorizon(
    const ParamBank& P,
    double s0_m,
    double vx_mps,
    double entry_straight_length_m = 0.0
)
{
    const SegmentedSkidpadSpeedProfile profile =
        SegmentedSkidpadSpeedProfile::fromParamBank(
            P,
            entry_straight_length_m
        );

    const int N =
        std::max(1, readActiveSegmentedVelocityHorizonN(P));

    const double dt =
        1.0 / std::max(P.get("model.frequency.steer_cmd_loop_hz"), 1.0e-6);

    return profile.buildVelocityHorizon(
        s0_m,
        vx_mps,
        N,
        dt
    );
}


inline std::vector<SegmentedSkidpadSpeedSample> buildSegmentedSkidpadVelocityRefSampleHorizon(
    const ParamBank& P,
    double s0_m,
    double vx_mps,
    double entry_straight_length_m = 0.0
)
{
    const SegmentedSkidpadSpeedProfile profile =
        SegmentedSkidpadSpeedProfile::fromParamBank(
            P,
            entry_straight_length_m
        );

    const int N =
        std::max(1, readActiveSegmentedVelocityHorizonN(P));

    const double dt =
        1.0 / std::max(P.get("model.frequency.steer_cmd_loop_hz"), 1.0e-6);

    return profile.buildSampleHorizon(
        s0_m,
        vx_mps,
        N,
        dt
    );
}


inline double segmentedSkidpadVelocityRefAtS(
    const ParamBank& P,
    double s_m,
    double entry_straight_length_m = 0.0
)
{
    const SegmentedSkidpadSpeedProfile profile =
        SegmentedSkidpadSpeedProfile::fromParamBank(
            P,
            entry_straight_length_m
        );

    return profile.evaluate(s_m).v_ref_mps;
}

} // namespace skidpad_control
