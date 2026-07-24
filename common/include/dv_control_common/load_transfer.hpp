#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dv_control_common
{

struct RelaxedAcceleration
{
    double ax_mps2 = 0.0;
    double ay_mps2 = 0.0;
};

/*
 * First-order load-transfer relaxation:
 *
 *     tau * da_f/dt + a_f = a_measured
 *
 * The exact zero-order-hold update is used, so the result does not depend on
 * a forward-Euler approximation:
 *
 *     a_f[k+1] = a_f[k] + (1 - exp(-dt/tau)) * (a[k] - a_f[k]).
 */
class LoadTransferRelaxation
{
public:
    RelaxedAcceleration update(
        double ax_target_mps2,
        double ay_target_mps2,
        double dt_s,
        double tau_s
    )
    {
        if (!std::isfinite(dt_s) || dt_s <= 0.0)
        {
            throw std::invalid_argument(
                "load-transfer sample time must be finite and > 0"
            );
        }

        if (!std::isfinite(tau_s) || tau_s <= 0.0)
        {
            throw std::invalid_argument(
                "model.mass_transfer.tau_load_s must be finite and > 0"
            );
        }

        ax_target_mps2 =
            std::isfinite(ax_target_mps2) ? ax_target_mps2 : 0.0;

        ay_target_mps2 =
            std::isfinite(ay_target_mps2) ? ay_target_mps2 : 0.0;

        const double alpha =
            std::clamp(
                -std::expm1(-dt_s / tau_s),
                0.0,
                1.0
            );

        state_.ax_mps2 +=
            alpha * (ax_target_mps2 - state_.ax_mps2);

        state_.ay_mps2 +=
            alpha * (ay_target_mps2 - state_.ay_mps2);

        return state_;
    }

    void reset(const RelaxedAcceleration state = {})
    {
        state_.ax_mps2 =
            std::isfinite(state.ax_mps2) ? state.ax_mps2 : 0.0;

        state_.ay_mps2 =
            std::isfinite(state.ay_mps2) ? state.ay_mps2 : 0.0;
    }

    const RelaxedAcceleration& state() const noexcept
    {
        return state_;
    }

private:
    RelaxedAcceleration state_;
};

struct WheelLoadModelParameters
{
    double mass_kg = 0.0;
    double gravity_mps2 = 0.0;
    double lf_m = 0.0;
    double lr_m = 0.0;
    double h_cg_m = 0.0;
    double track_front_m = 0.0;
    double track_rear_m = 0.0;
    double h_roll_center_front_m = 0.0;
    double h_roll_center_rear_m = 0.0;
    double lambda_elastic_front = 0.5;
    double minimum_wheel_load_N = 50.0;
};

struct WheelLoadsN
{
    double FL = 0.0;
    double FR = 0.0;
    double RL = 0.0;
    double RR = 0.0;
};

/*
 * Shared quasi-static four-wheel load model used by all three allocators.
 *
 * Longitudinal transfer:
 *     DeltaFz_x = m * ax * h_cg / L
 *
 * Roll-axis height at the CG:
 *     h_RA = (lr*h_RC,f + lf*h_RC,r) / L
 *
 * Elastic roll moment:
 *     M_phi,el = m * ay * (h_cg - h_RA)
 *
 * Front/rear lateral transfers:
 *     DeltaFz_y,f =
 *       (m*ay*lr/L*h_RC,f + lambda*M_phi,el) / t_f
 *
 *     DeltaFz_y,r =
 *       (m*ay*lf/L*h_RC,r + (1-lambda)*M_phi,el) / t_r
 *
 * DeltaFz_y is the load moved from the inside wheel to the outside wheel.
 * There is deliberately no additional factor 1/2.
 */
inline WheelLoadsN computeWheelLoadsN(
    const WheelLoadModelParameters& p,
    double ax_relaxed_mps2,
    double ay_relaxed_mps2,
    double front_aero_load_N = 0.0,
    double rear_aero_load_N = 0.0
)
{
    const double wheelbase_m =
        p.lf_m + p.lr_m;

    const bool valid =
        std::isfinite(p.mass_kg) && p.mass_kg > 0.0 &&
        std::isfinite(p.gravity_mps2) && p.gravity_mps2 > 0.0 &&
        std::isfinite(p.lf_m) && p.lf_m >= 0.0 &&
        std::isfinite(p.lr_m) && p.lr_m >= 0.0 &&
        std::isfinite(wheelbase_m) && wheelbase_m > 0.0 &&
        std::isfinite(p.h_cg_m) && p.h_cg_m >= 0.0 &&
        std::isfinite(p.track_front_m) && p.track_front_m > 0.0 &&
        std::isfinite(p.track_rear_m) && p.track_rear_m > 0.0 &&
        std::isfinite(p.h_roll_center_front_m) &&
        std::isfinite(p.h_roll_center_rear_m) &&
        std::isfinite(p.lambda_elastic_front) &&
        p.lambda_elastic_front >= 0.0 &&
        p.lambda_elastic_front <= 1.0 &&
        std::isfinite(p.minimum_wheel_load_N) &&
        p.minimum_wheel_load_N >= 0.0;

    if (!valid)
    {
        throw std::invalid_argument("invalid shared wheel-load model parameters");
    }

    ax_relaxed_mps2 =
        std::isfinite(ax_relaxed_mps2) ? ax_relaxed_mps2 : 0.0;

    ay_relaxed_mps2 =
        std::isfinite(ay_relaxed_mps2) ? ay_relaxed_mps2 : 0.0;

    front_aero_load_N =
        std::max(
            0.0,
            std::isfinite(front_aero_load_N) ? front_aero_load_N : 0.0
        );

    rear_aero_load_N =
        std::max(
            0.0,
            std::isfinite(rear_aero_load_N) ? rear_aero_load_N : 0.0
        );

    double front_axle_N =
        p.mass_kg * p.gravity_mps2 * p.lr_m / wheelbase_m
        + front_aero_load_N;

    double rear_axle_N =
        p.mass_kg * p.gravity_mps2 * p.lf_m / wheelbase_m
        + rear_aero_load_N;

    const double longitudinal_transfer_N =
        p.mass_kg * ax_relaxed_mps2 * p.h_cg_m / wheelbase_m;

    front_axle_N -= longitudinal_transfer_N;
    rear_axle_N += longitudinal_transfer_N;

    const double minimum_axle_load_N =
        2.0 * p.minimum_wheel_load_N;

    front_axle_N =
        std::max(minimum_axle_load_N, front_axle_N);

    rear_axle_N =
        std::max(minimum_axle_load_N, rear_axle_N);

    const double h_roll_axis_at_cg_m =
        (
            p.lr_m * p.h_roll_center_front_m
            + p.lf_m * p.h_roll_center_rear_m
        ) / wheelbase_m;

    const double front_lateral_force_N =
        p.mass_kg * ay_relaxed_mps2 * p.lr_m / wheelbase_m;

    const double rear_lateral_force_N =
        p.mass_kg * ay_relaxed_mps2 * p.lf_m / wheelbase_m;

    const double elastic_roll_moment_Nm =
        p.mass_kg
        * ay_relaxed_mps2
        * (p.h_cg_m - h_roll_axis_at_cg_m);

    double front_lateral_transfer_N =
        (
            front_lateral_force_N * p.h_roll_center_front_m
            + p.lambda_elastic_front * elastic_roll_moment_Nm
        ) / p.track_front_m;

    double rear_lateral_transfer_N =
        (
            rear_lateral_force_N * p.h_roll_center_rear_m
            + (1.0 - p.lambda_elastic_front) * elastic_roll_moment_Nm
        ) / p.track_rear_m;

    const double front_transfer_limit_N =
        std::max(
            0.0,
            0.5 * front_axle_N - p.minimum_wheel_load_N
        );

    const double rear_transfer_limit_N =
        std::max(
            0.0,
            0.5 * rear_axle_N - p.minimum_wheel_load_N
        );

    front_lateral_transfer_N =
        std::clamp(
            front_lateral_transfer_N,
            -front_transfer_limit_N,
            front_transfer_limit_N
        );

    rear_lateral_transfer_N =
        std::clamp(
            rear_lateral_transfer_N,
            -rear_transfer_limit_N,
            rear_transfer_limit_N
        );

    WheelLoadsN out;

    out.FL = 0.5 * front_axle_N - front_lateral_transfer_N;
    out.FR = 0.5 * front_axle_N + front_lateral_transfer_N;
    out.RL = 0.5 * rear_axle_N - rear_lateral_transfer_N;
    out.RR = 0.5 * rear_axle_N + rear_lateral_transfer_N;

    return out;
}

} // namespace dv_control_common
