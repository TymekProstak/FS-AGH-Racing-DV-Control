#include "dv_control_common/load_transfer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace
{

bool near(double a, double b, double tolerance = 1.0e-9)
{
    return std::abs(a - b) <= tolerance;
}

} // namespace

int main()
{
    using namespace dv_control_common;

    LoadTransferRelaxation relaxation;

    const RelaxedAcceleration first =
        relaxation.update(4.0, -2.0, 0.05, 0.05);

    const double expected_alpha =
        1.0 - std::exp(-1.0);

    assert(near(first.ax_mps2, 4.0 * expected_alpha));
    assert(near(first.ay_mps2, -2.0 * expected_alpha));

    WheelLoadModelParameters p;
    p.mass_kg = 228.0;
    p.gravity_mps2 = 9.81;
    p.lf_m = 0.912967105;
    p.lr_m = 0.617032895;
    p.h_cg_m = 0.3;
    p.track_front_m = 1.2;
    p.track_rear_m = 1.2;
    p.h_roll_center_front_m = 0.08;
    p.h_roll_center_rear_m = 0.11;
    p.lambda_elastic_front = 0.4794651384909265;
    p.minimum_wheel_load_N = 50.0;

    const WheelLoadsN static_load =
        computeWheelLoadsN(p, 0.0, 0.0);

    assert(near(static_load.FL, static_load.FR));
    assert(near(static_load.RL, static_load.RR));
    assert(
        near(
            static_load.FL + static_load.FR
            + static_load.RL + static_load.RR,
            p.mass_kg * p.gravity_mps2
        )
    );

    const WheelLoadsN cornering =
        computeWheelLoadsN(p, 0.0, 4.0);

    assert(cornering.FR > cornering.FL);
    assert(cornering.RR > cornering.RL);
    assert(
        near(
            cornering.FL + cornering.FR,
            static_load.FL + static_load.FR
        )
    );
    assert(
        near(
            cornering.RL + cornering.RR,
            static_load.RL + static_load.RR
        )
    );

    std::cout << "load_transfer_test: PASS\n";
    return 0;
}
