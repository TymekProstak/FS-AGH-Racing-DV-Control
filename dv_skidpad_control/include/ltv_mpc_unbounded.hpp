#pragma once

#include "ParamBank.hpp"
#include "control_types.hpp"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace skidpad_control
{

class UnboundedLtvMpc
{
public:
    /*
        Fixed eight-state actuator/vehicle layout:

            x = [ey, epsi, vy, r,
                 delta, delta_dot,
                 delta_cmd,
                 Mtv]

        Inputs:
            u(0) = delta_cmd_dot      [rad/s]
            u(1) = dMtv               [Nm/s]

        Steering convention:
            - PT2 is always active;
            - delta is the physical steering angle supplied by the wrapper
              from the encoder at the beginning of each solve;
            - delta_dot is the PT2 model steering-rate state;
            - delta_cmd drives the PT2 actuator directly.

        Output convention:
            out.u_delta_cmd          = first u(0)
            out.delta_act_next       = predicted PT2 delta after one step
            out.delta_dot_next       = predicted PT2 delta_dot after one step
            out.u_tv_yaw_moment_Nm   = Mtv after the first horizon step

        Optimized torque vectoring:
            - if JACA is enabled, JACA has priority and Mtv/dMtv has no
              influence on dynamics;
            - if optimized TV is enabled:
                  r_dot   += Mtv / Iz
                  Mtv_dot  = dMtv

        QP TV cost:
            0.5 * R_tv_yaw_moment   * sum(Mtv_k^2)
          + 0.5 * R_d_tv_yaw_moment * sum(dMtv_k^2)
    */
    static constexpr int NX = 8;
    static constexpr int NU = 2;

    static constexpr int IX_EY = 0;
    static constexpr int IX_EPSI = 1;
    static constexpr int IX_VY = 2;
    static constexpr int IX_R = 3;
    static constexpr int IX_DELTA = 4;
    static constexpr int IX_DELTA_DOT = 5;
    static constexpr int IX_DELTA_CMD = 6;
    static constexpr int IX_MTV = 7;

    static constexpr int IU_DELTA_CMD_DOT = 0;
    static constexpr int IU_DMTV = 1;

public:
    UnboundedLtvMpc() = delete;
    explicit UnboundedLtvMpc(const ParamBank& P);

    void setParamBank(const ParamBank& P);
    void reset();

    double getDt() const;

    Result solve(const State& x0,
                 const std::vector<double>& kappa_ref_horizon,
                 const std::vector<double>& v_body_ref_horizon);

    Result solveWithBodyAcceleration(
        const State& x0,
        const std::vector<double>& kappa_ref_horizon,
        double v_body_initial_mps,
        const std::vector<double>& ax_body_ref_horizon);

private:
    ParamBank param_;

    int N_ = 15;
    bool is_initialized_ = false;

    /*
        Warm start stores N_ pairs:
            [delta_cmd_dot_0, dMtv_0,
             delta_cmd_dot_1, dMtv_1, ...]
    */
    std::vector<double> last_output_;

    /*
        State does not carry Mtv, so keep the last published optimized TV
        yaw moment here as x0(IX_MTV) for the next rollout.
    */
    double last_tv_yaw_moment_state_Nm_ = 0.0;

private:
    void validateParams() const;
    int readHorizonN() const;

    bool useJacaTorqueVectoring() const;
    bool useOptimizedTorqueVectoring() const;

    double readTvYawMomentCost() const;
    double maxOptimizedTvYawMomentNm() const;

    std::vector<double> integrateBodySpeedHorizon(
        double v_body_initial_mps,
        const std::vector<double>& ax_body_ref_horizon,
        double dt) const;

    Eigen::Matrix<double, NX, 1> continuousDynamics(
        const Eigen::Matrix<double, NX, 1>& x,
        const Eigen::Matrix<double, NU, 1>& u,
        double v_body_model,
        double kappa_ref) const;

    void calculateContinuousJacobian(
        const Eigen::Matrix<double, NX, 1>& x,
        const Eigen::Matrix<double, NU, 1>& u,
        double v_body_model,
        double kappa_ref,
        Eigen::Matrix<double, NX, NX>& Ac,
        Eigen::Matrix<double, NX, NU>& Bc) const;

    Eigen::Matrix<double, NX, 1> rk4Step(
        const Eigen::Matrix<double, NX, 1>& x,
        const Eigen::Matrix<double, NU, 1>& u,
        double v_body_model,
        double kappa_ref,
        double dt) const;

    void buildDiscreteLinearization(
        const Eigen::Matrix<double, NX, 1>& x_nom,
        const Eigen::Matrix<double, NU, 1>& u_nom,
        double v_body_model,
        double kappa_ref,
        double dt,
        Eigen::Matrix<double, NX, NX>& Ad,
        Eigen::Matrix<double, NX, NU>& Bd,
        Eigen::Matrix<double, NX, 1>& gd,
        Eigen::Matrix<double, NX, 1>& x_next_nom) const;

    void requireFinite(const std::string& key) const;
    void requirePositive(const std::string& key) const;
    void requireNonNegative(const std::string& key) const;
    void requireBoolLike(const std::string& key) const;

    void resetOutput();

    static void discretizeExpmAB(
        const Eigen::Matrix<double, NX, NX>& A,
        const Eigen::Matrix<double, NX, NU>& B,
        double dt,
        Eigen::Matrix<double, NX, NX>& Ad,
        Eigen::Matrix<double, NX, NU>& Bd);
};

} // namespace skidpad_control
