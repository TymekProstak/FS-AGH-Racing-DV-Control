#include "ltv_mpc_unbounded.hpp"

#if defined(DV_CONTROL_USE_TUSTIN_DISCRETIZATION)
#include <Eigen/LU>
#else
#include <unsupported/Eigen/MatrixFunctions>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace acc_launch_control
{

namespace
{

using Clock = std::chrono::steady_clock;

static constexpr double kFrenetDenomMin = 0.2;
static constexpr int kMpcDebugPrintEvery = 20;

inline double msBetween(const Clock::time_point& a,
                        const Clock::time_point& b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

struct SteeringLimitResult
{
    double u = 0.0;

    bool rate_saturated = false;
    bool steering_angle_saturated = false;

    bool saturated() const
    {
        return rate_saturated || steering_angle_saturated;
    }
};


struct TorqueVectoringYawTarget
{
    double target_yaw_rate = 0.0;
    double d_target_yaw_rate_d_delta = 0.0;
    double p_tv_gain = 0.0;
};

inline double smoothTurnaroundGain(double v,
                                   double v_low,
                                   double v_high,
                                   double transition_gain,
                                   double low_speed_gain)
{
    const double dv = v_high - v_low;

    if (!std::isfinite(dv) || std::abs(dv) < 1.0e-9)
    {
        return 1.0;
    }

    const double z =
        (2.0 * v - v_low - v_high) / dv;

    const double blend =
        0.5 * (1.0 - std::tanh(transition_gain * z));

    // Smooth C-infinity equivalent of the old low-speed yaw boost.
    // The gain is about 1.5 below v_low and about 1.0 above v_high.
    return 1.0 + low_speed_gain * blend;
}

TorqueVectoringYawTarget calculateTorqueVectoringYawTarget(
    const ParamBank& param,
    double vx_body,
    double delta_act,
    double wheelbase)
{
    TorqueVectoringYawTarget out;

    if (!param.getBool("general.general_use_jaca_torque_vectoring"))
    {
        out.p_tv_gain = 0.0;
        return out;
    }

    out.p_tv_gain =
                param.get("model.torque_vectoring_jaca.p_tv_gain")*param.get("model.drivetrain.gear_ratio")*param.get("model.torque_vectoring_jaca.jaca_magic_number")*4*param.get("model.torque_vectoring_jaca.track_width")/2.0/(param.get("model.tire.R_tire"));

    if (out.p_tv_gain <= 0.0)
    {
        out.p_tv_gain = 0.0;
        return out;
    }

    const double v_low =
        param.get("model.torque_vectoring_jaca.speed_bled_low");

    const double v_high =
        param.get("model.torque_vectoring_jaca.speed_bled_high");

    const double turn_radius_speed_gain =
        param.get("model.torque_vectoring_jaca.turn_radius_speed_gain");

    const double over_steer_gain =
        param.get("model.torque_vectoring_jaca.over_steer_gain");

    const double transition_gain =
        param.get("model.torque_vectoring_jaca.transition_gain");

    const double k_turn =
        smoothTurnaroundGain(
            vx_body,
            v_low,
            v_high,
            transition_gain,
            param.get("model.torque_vectoring_jaca.low_speed_gain")
        );

    const double effective_wheelbase =
        wheelbase + turn_radius_speed_gain * vx_body * vx_body;

    const double yaw_target_gain =
        (1.0 + over_steer_gain) * k_turn;

    out.target_yaw_rate =
        vx_body * std::sin(delta_act) * yaw_target_gain / effective_wheelbase;

    out.d_target_yaw_rate_d_delta =
        vx_body * std::cos(delta_act) * yaw_target_gain / effective_wheelbase;

    return out;
}

inline double mpcClamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

inline double safeFrenetDenom(double denom_raw)
{
    if (!std::isfinite(denom_raw))
    {
        return kFrenetDenomMin;
    }

    if (std::abs(denom_raw) < kFrenetDenomMin)
    {
        return (denom_raw < 0.0) ? -kFrenetDenomMin : kFrenetDenomMin;
    }

    return denom_raw;
}

inline double safeForwardBodySpeed(double v_raw, double v_min)
{
    if (!std::isfinite(v_raw))
    {
        return v_min;
    }

    return std::max(v_raw, v_min);
}

struct AxleNormalLoads
{
    double front_N = 0.0;
    double rear_N = 0.0;
};

inline AxleNormalLoads computeAeroAwareAxleNormalLoadsN(
    const ParamBank& param,
    const double vx_body_mps,
    const double m,
    const double g,
    const double lf,
    const double lr)
{
    /*
        Normal loads used by the LTV-MPC lateral tire model.

        Static load:
            Fz_front = m*g*l_r/(l_f+l_r)
            Fz_rear  = m*g*l_f/(l_f+l_r)

        Optional aero load:
            active only if model.body.use_aero == true

            Fz_front += model.body.Cl1 * vx^2
            Fz_rear  += model.body.Cl2 * vx^2

        IMPORTANT:
            model.body.C_l_f / model.body.C_l_r are not used here.
            Those names are easy to confuse with aero coefficients, but this
            controller convention uses model.body.Cl1/Cl2 guarded by
            model.body.use_aero.
    */
    const double L =
        lf + lr;

    if (!std::isfinite(L) || std::abs(L) < 1.0e-9)
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] wheelbase l_f + l_r must be finite and non-zero"
        );
    }

    const double Fz_static_total_N =
        m * g;

    AxleNormalLoads out;

    out.front_N =
        Fz_static_total_N * lr / L;

    out.rear_N =
        Fz_static_total_N * lf / L;

    if (param.getBool("model.body.use_aero"))
    {
        const double Cl1 =
            param.get("model.body.Cl1");

        const double Cl2 =
            param.get("model.body.Cl2");

        const double vx_safe =
            std::isfinite(vx_body_mps)
                ? std::max(0.0, vx_body_mps)
                : 0.0;

        const double vx2 =
            vx_safe * vx_safe;

        const double Fz_aero_front_N =
            Cl1 * vx2;

        const double Fz_aero_rear_N =
            Cl2 * vx2;

        if (std::isfinite(Fz_aero_front_N))
        {
            out.front_N += Fz_aero_front_N;
        }

        if (std::isfinite(Fz_aero_rear_N))
        {
            out.rear_N += Fz_aero_rear_N;
        }
    }

    out.front_N =
        std::max(1.0, out.front_N);

    out.rear_N =
        std::max(1.0, out.rear_N);

    return out;
}

SteeringLimitResult applySteeringLimits(double u_raw,
                                        double delta_cmd_now,
                                        double dt,
                                        double max_rate,
                                        double max_steer,
                                        double eps)
{
    SteeringLimitResult out;

    double u =
        mpcClamp(u_raw, -max_rate, max_rate);

    out.rate_saturated =
        std::abs(u - u_raw) > eps;

    if (delta_cmd_now >= max_steer)
    {
        out.steering_angle_saturated = true;

        if (u > 0.0)
        {
            u = 0.0;
        }
    }
    else if (delta_cmd_now <= -max_steer)
    {
        out.steering_angle_saturated = true;

        if (u < 0.0)
        {
            u = 0.0;
        }
    }
    else
    {
        const double delta_cmd_next_rate_only =
            delta_cmd_now + dt * u;

        if (delta_cmd_next_rate_only > max_steer)
        {
            u =
                (max_steer - delta_cmd_now) / dt;

            out.steering_angle_saturated =
                true;
        }
        else if (delta_cmd_next_rate_only < -max_steer)
        {
            u =
                (-max_steer - delta_cmd_now) / dt;

            out.steering_angle_saturated =
                true;
        }
    }

    out.u =
        u;

    return out;
}

void validateCurvatureHorizon(const std::vector<double>& kappa_ref_horizon,
                              int N,
                              const char* tag)
{
    if (static_cast<int>(kappa_ref_horizon.size()) < N)
    {
        throw std::runtime_error(
            std::string("[") + tag + "] kappa_ref_horizon.size() < N"
        );
    }

    for (int k = 0; k < N; ++k)
    {
        if (!std::isfinite(kappa_ref_horizon[static_cast<std::size_t>(k)]))
        {
            throw std::runtime_error(
                std::string("[") + tag + "] kappa_ref_horizon contains NaN/Inf"
            );
        }
    }
}

void validateBodySpeedHorizon(const std::vector<double>& v_body_ref_horizon,
                              int N,
                              const char* tag)
{
    if (static_cast<int>(v_body_ref_horizon.size()) < N)
    {
        throw std::runtime_error(
            std::string("[") + tag + "] v_body_ref_horizon.size() < N"
        );
    }

    for (int k = 0; k < N; ++k)
    {
        const double v_body =
            v_body_ref_horizon[static_cast<std::size_t>(k)];

        if (!std::isfinite(v_body))
        {
            throw std::runtime_error(
                std::string("[") + tag + "] v_body_ref_horizon contains NaN/Inf"
            );
        }
    }
}

void validateBodyAccelerationHorizon(const std::vector<double>& ax_body_ref_horizon,
                                     int N,
                                     const char* tag)
{
    if (static_cast<int>(ax_body_ref_horizon.size()) < N)
    {
        throw std::runtime_error(
            std::string("[") + tag + "] ax_body_ref_horizon.size() < N"
        );
    }

    for (int k = 0; k < N; ++k)
    {
        const double ax_body =
            ax_body_ref_horizon[static_cast<std::size_t>(k)];

        if (!std::isfinite(ax_body))
        {
            throw std::runtime_error(
                std::string("[") + tag + "] ax_body_ref_horizon contains NaN/Inf"
            );
        }
    }
}

} // anonymous namespace

// =============================================================================
//                              CONSTRUCTION
// =============================================================================

UnboundedLtvMpc::UnboundedLtvMpc(const ParamBank& P)
    : param_(P)
{
    validateParams();

    N_ = readHorizonN();
    last_output_.assign(static_cast<std::size_t>(N_ * NU), 0.0);
    last_tv_yaw_moment_state_Nm_ = 0.0;
}

void UnboundedLtvMpc::setParamBank(const ParamBank& P)
{
    param_ = P;

    validateParams();

    N_ = readHorizonN();
    reset();
}

void UnboundedLtvMpc::reset()
{
    is_initialized_ = false;
    last_output_.assign(static_cast<std::size_t>(N_ * NU), 0.0);
    last_tv_yaw_moment_state_Nm_ = 0.0;
}

double UnboundedLtvMpc::getDt() const
{
    const double frequency_hz =
        param_.get("model.frequency.steer_cmd_loop_hz");

    if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0)
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] model.frequency.steer_cmd_loop_hz must be finite and > 0"
        );
    }

    return 1.0 / frequency_hz;
}

// =============================================================================
//                              SOLVE
// =============================================================================

Result UnboundedLtvMpc::solveWithBodyAcceleration(
    const State& x0,
    const std::vector<double>& kappa_ref_horizon,
    double v_body_initial_mps,
    const std::vector<double>& ax_body_ref_horizon)
{
    validateCurvatureHorizon(kappa_ref_horizon, N_, "UnboundedLtvMpc");
    validateBodyAccelerationHorizon(ax_body_ref_horizon, N_, "UnboundedLtvMpc");

    const double dt =
        getDt();

    const std::vector<double> v_body_ref_horizon =
        integrateBodySpeedHorizon(
            v_body_initial_mps,
            ax_body_ref_horizon,
            dt
        );

    return solve(
        x0,
        kappa_ref_horizon,
        v_body_ref_horizon
    );
}

Result UnboundedLtvMpc::solve(const State& x0,
                              const std::vector<double>& kappa_ref_horizon,
                              const std::vector<double>& v_body_ref_horizon)
{
    const auto t_total_begin = Clock::now();

    Result out;

    validateCurvatureHorizon(kappa_ref_horizon, N_, "UnboundedLtvMpc");
    validateBodySpeedHorizon(v_body_ref_horizon, N_, "UnboundedLtvMpc");

    const double dt =
        getDt();

    const auto t_after_validation = Clock::now();

    /*
        Reusable thread-local workspace.

        The controller normally runs from one control thread. Keeping these
        buffers between solve() calls removes repeated heap allocations without
        changing the MPC model, cost, horizon or numerical algorithm.

        thread_local also keeps independent workspaces when solve() is called
        from different threads.
    */
    struct SolveWorkspace
    {
        std::vector<Eigen::Matrix<double, NX, NX>> Ad_list;
        std::vector<Eigen::Matrix<double, NX, NU>> Bd_list;
        std::vector<Eigen::Matrix<double, NX, 1>> gd_list;
        std::vector<Eigen::Matrix<double, NX, 1>> x_nominal;

        Eigen::MatrixXd H;
        Eigen::VectorXd g;
        Eigen::MatrixXd S;

        std::vector<double> z_limited;
    };

    static thread_local SolveWorkspace workspace;

    workspace.Ad_list.resize(static_cast<std::size_t>(N_));
    workspace.Bd_list.resize(static_cast<std::size_t>(N_));
    workspace.gd_list.resize(static_cast<std::size_t>(N_));
    workspace.x_nominal.resize(static_cast<std::size_t>(N_ + 1));

    auto& Ad_list = workspace.Ad_list;
    auto& Bd_list = workspace.Bd_list;
    auto& gd_list = workspace.gd_list;
    auto& x_nominal = workspace.x_nominal;

    const bool optimized_tv_active =
        useOptimizedTorqueVectoring();

    const double tv_yaw_moment_initial_Nm =
        optimized_tv_active
            ? last_tv_yaw_moment_state_Nm_
            : 0.0;

    /*
        MPC state:

            x = [ey, epsi, vy, r,
                 delta, delta_dot,
                 delta_cmd,
                 Mtv]

    */
    x_nominal[0] << x0.ey,
                    x0.epsi,
                    x0.vy,
                    x0.r,
                    x0.delta,
                    x0.delta_dot,
                    x0.delta_cmd,
                    tv_yaw_moment_initial_Nm;

    if (!is_initialized_ || static_cast<int>(last_output_.size()) != N_ * NU)
    {
        last_output_.assign(static_cast<std::size_t>(N_ * NU), 0.0);
        is_initialized_ = true;
    }

    const auto t_linearization_begin = Clock::now();

    for (int k = 0; k < N_; ++k)
    {
        Eigen::Matrix<double, NU, 1> u_nom;
        u_nom.setZero();

        for (int j = 0; j < NU; ++j)
        {
            u_nom(j) =
                last_output_[static_cast<std::size_t>(k * NU + j)];
        }

        const double v_body_model =
            v_body_ref_horizon[static_cast<std::size_t>(k)];

        const double kappa_ref =
            kappa_ref_horizon[static_cast<std::size_t>(k)];

        Eigen::Matrix<double, NX, 1> x_next_nom;

        buildDiscreteLinearization(
            x_nominal[static_cast<std::size_t>(k)],
            u_nom,
            v_body_model,
            kappa_ref,
            dt,
            Ad_list[static_cast<std::size_t>(k)],
            Bd_list[static_cast<std::size_t>(k)],
            gd_list[static_cast<std::size_t>(k)],
            x_next_nom
        );

        const bool lin_finite =
            Ad_list[static_cast<std::size_t>(k)].allFinite() &&
            Bd_list[static_cast<std::size_t>(k)].allFinite() &&
            gd_list[static_cast<std::size_t>(k)].allFinite() &&
            x_next_nom.allFinite();

        if (!lin_finite)
        {
            resetOutput();
            return out;
        }

        x_nominal[static_cast<std::size_t>(k + 1)] =
            x_next_nom;
    }

    const auto t_linearization_end = Clock::now();

    const auto t_qp_build_begin = Clock::now();

    Eigen::Matrix<double, NX, NX> Q =
        Eigen::Matrix<double, NX, NX>::Zero();

    Q(0, 0) = param_.get("model.ltv_mpc_unbounded.Q_ey");
    Q(1, 1) = param_.get("model.ltv_mpc_unbounded.Q_psi");
    Q(2, 2) = param_.get("model.ltv_mpc_unbounded.Q_vy");
    Q(3, 3) = param_.get("model.ltv_mpc_unbounded.Q_r");

    const double terminal_scale =
        param_.get("model.ltv_mpc_unbounded.terminal_scale");

    const double R_d_delta =
        param_.get("model.ltv_mpc_unbounded.R_d_delta");

    /*
        TV actuator convention in this LTV-MPC:

            x(IX_MTV) = Mtv       [Nm]
            u(IU_DMTV) = dMtv     [Nm/s]

        Dynamics:
            r_dot   += Mtv / Iz
            Mtv_dot  = dMtv

        TV cost:
            J_tv =
                0.5 * R_tv_yaw_moment
                    * sum_k Mtv_k^2
              + 0.5 * R_d_tv_yaw_moment
                    * sum_k dMtv_k^2

        last_output_ is only shifted warm-start / nominal input sequence.
        It is not a TV reference in the cost.
    */
    const double R_tv_yaw_moment =
        readTvYawMomentCost();

    const double R_d_tv_yaw_moment =
        optimized_tv_active
            ? param_.get("model.ltv_mpc_unbounded.R_d_tv_yaw_moment")
            : 0.0;

    Q(IX_MTV, IX_MTV) =
        R_tv_yaw_moment;

    Eigen::Matrix<double, NX, NX> P =
        terminal_scale * Q;

    const int nz =
        N_ * NU;

    workspace.H.setZero(nz, nz);
    workspace.g.setZero(nz);
    workspace.S.setZero(NX, nz);

    Eigen::MatrixXd& H =
        workspace.H;

    Eigen::VectorXd& g =
        workspace.g;

    Eigen::MatrixXd& S =
        workspace.S;

    for (int k = 0; k < N_; ++k)
    {
        const int idx_delta =
            k * NU + IU_DELTA_CMD_DOT;

        const int idx_dmtv =
            k * NU + IU_DMTV;

        H(idx_delta, idx_delta) +=
            R_d_delta;

        /*
            dMtv input regularization:

                0.5 * R_d_tv_yaw_moment * dMtv_k^2
        */
        H(idx_dmtv, idx_dmtv) +=
            R_d_tv_yaw_moment;
    }

    Eigen::Matrix<double, NX, 1> x_base =
        x_nominal[0];

    for (int k = 0; k < N_; ++k)
    {
        const auto& Ad =
            Ad_list[static_cast<std::size_t>(k)];

        const auto& Bd =
            Bd_list[static_cast<std::size_t>(k)];

        const auto& gd =
            gd_list[static_cast<std::size_t>(k)];

        x_base =
            Ad * x_base + gd;

        S =
            Ad * S;

        S.block(0, k * NU, NX, NU) += Bd;

        const Eigen::Matrix<double, NX, NX>& Qk =
            (k == N_ - 1) ? P : Q;

        /*
            Qk is diagonal. Build the same quadratic form using weighted
            rank-one updates and only the lower triangle of H:

                H += S' * Qk * S
                g += S' * Qk * x_base

            Eigen::LLT below reads the lower triangle, so computing the unused
            upper triangle is unnecessary. This preserves the mathematical
            problem while substantially reducing temporary matrices and work.
        */
        for (int state_idx = 0; state_idx < NX; ++state_idx)
        {
            const double q_weight =
                Qk(state_idx, state_idx);

            if (q_weight == 0.0)
            {
                continue;
            }

            const auto sensitivity_row =
                S.row(state_idx);

            H.selfadjointView<Eigen::Lower>().rankUpdate(
                sensitivity_row.transpose(),
                q_weight
            );

            g.noalias() +=
                (q_weight * x_base(state_idx))
                * sensitivity_row.transpose();
        }
    }

    H.diagonal().array() +=
        param_.get("model.ltv_mpc_unbounded.hessian_regularization");

    const auto t_qp_build_end = Clock::now();

    if (!H.allFinite() || !g.allFinite() || !x_base.allFinite())
    {
        resetOutput();
        return out;
    }

    const auto t_qp_solve_begin = Clock::now();

    Eigen::LLT<Eigen::MatrixXd> llt(H);

    if (llt.info() != Eigen::Success)
    {
        resetOutput();
        return out;
    }

    Eigen::VectorXd z =
        llt.solve(-g);

    if (llt.info() != Eigen::Success || !z.allFinite())
    {
        resetOutput();
        return out;
    }

    const auto t_qp_solve_end = Clock::now();

    const auto t_limit_begin = Clock::now();

    const double u_delta_cmd_raw =
        z(IU_DELTA_CMD_DOT);

    const double u_dmtv_raw =
        z(IU_DMTV);

    const double tv_yaw_moment_raw_Nm =
        tv_yaw_moment_initial_Nm + dt * u_dmtv_raw;

    const double max_rate =
        param_.get("model.steering_limit.max_steer_rate");

    const double max_steer =
        param_.get("model.steering_limit.max_steer");

    const double max_tv_yaw_moment =
        maxOptimizedTvYawMomentNm();

    workspace.z_limited.assign(
        static_cast<std::size_t>(nz),
        0.0
    );

    std::vector<double>& z_limited =
        workspace.z_limited;

    double delta_cmd_roll =
        x0.delta_cmd;

    double tv_yaw_moment_roll =
        optimized_tv_active
            ? tv_yaw_moment_initial_Nm
            : 0.0;

    SteeringLimitResult sat;

    bool tv_yaw_moment_saturated =
        false;

    double u_dmtv_first_limited =
        0.0;

    for (int k = 0; k < N_; ++k)
    {
        const int idx_delta =
            k * NU + IU_DELTA_CMD_DOT;

        const int idx_dmtv =
            k * NU + IU_DMTV;

        const SteeringLimitResult sat_k =
            applySteeringLimits(
                z(idx_delta),
                delta_cmd_roll,
                dt,
                max_rate,
                max_steer,
                1.0e-12
            );

        const double u_delta_limited =
            sat_k.u;

        z_limited[static_cast<std::size_t>(idx_delta)] =
            u_delta_limited;

        double u_dmtv_limited =
            0.0;

        if (optimized_tv_active)
        {
            const double tv_yaw_moment_next_raw =
                tv_yaw_moment_roll + dt * z(idx_dmtv);

            const double tv_yaw_moment_next_limited =
                mpcClamp(
                    tv_yaw_moment_next_raw,
                    -max_tv_yaw_moment,
                    max_tv_yaw_moment
                );

            u_dmtv_limited =
                (tv_yaw_moment_next_limited - tv_yaw_moment_roll)
                / std::max(dt, 1.0e-9);

            if (std::abs(tv_yaw_moment_next_limited - tv_yaw_moment_next_raw) > 1.0e-9)
            {
                tv_yaw_moment_saturated =
                    true;
            }

            tv_yaw_moment_roll =
                tv_yaw_moment_next_limited;
        }

        z_limited[static_cast<std::size_t>(idx_dmtv)] =
            u_dmtv_limited;

        if (k == 0)
        {
            sat =
                sat_k;

            u_dmtv_first_limited =
                u_dmtv_limited;
        }

        delta_cmd_roll =
            mpcClamp(
                delta_cmd_roll + dt * u_delta_limited,
                -max_steer,
                max_steer
            );
    }

    const auto t_limit_end = Clock::now();

    const double u_delta_cmd =
        z_limited[static_cast<std::size_t>(IU_DELTA_CMD_DOT)];

    const double u_dmtv =
        u_dmtv_first_limited;

    const auto t_prediction_begin = Clock::now();

    Eigen::Matrix<double, NX, 1> x_now =
        x_nominal[0];

    const double v_body_model_now =
        v_body_ref_horizon[0];

    Eigen::Matrix<double, NU, 1> u_now;
    u_now.setZero();

    u_now(IU_DELTA_CMD_DOT) =
        u_delta_cmd;

    u_now(IU_DMTV) =
        u_dmtv;

    Eigen::Matrix<double, NX, 1> x_next =
        rk4Step(
            x_now,
            u_now,
            v_body_model_now,
            kappa_ref_horizon[0],
            dt
        );

    if (!x_next.allFinite())
    {
        resetOutput();
        return out;
    }

    const auto t_prediction_end = Clock::now();

    if (static_cast<int>(last_output_.size()) != N_ * NU)
    {
        last_output_.resize(static_cast<std::size_t>(N_ * NU));
    }

    std::fill(
        last_output_.begin(),
        last_output_.end(),
        0.0
    );

    for (int k = 0; k + 1 < N_; ++k)
    {
        for (int j = 0; j < NU; ++j)
        {
            last_output_[static_cast<std::size_t>(k * NU + j)] =
                z_limited[static_cast<std::size_t>((k + 1) * NU + j)];
        }
    }

    for (int j = 0; j < NU; ++j)
    {
        last_output_[static_cast<std::size_t>((N_ - 1) * NU + j)] =
            z_limited[static_cast<std::size_t>((N_ - 1) * NU + j)];
    }

    out.valid =
        true;

    out.u_delta_cmd =
        u_delta_cmd;

    out.u_delta_cmd_raw =
        u_delta_cmd_raw;

    out.u_delta_cmd_saturated =
        sat.saturated();

    const double tv_yaw_moment_next_Nm =
        optimized_tv_active ? x_next(IX_MTV) : 0.0;

    last_tv_yaw_moment_state_Nm_ =
        tv_yaw_moment_next_Nm;

    out.u_tv_yaw_moment_Nm =
        tv_yaw_moment_next_Nm;

    out.u_tv_yaw_moment_raw_Nm =
        optimized_tv_active ? tv_yaw_moment_raw_Nm : 0.0;

    out.u_tv_yaw_moment_saturated =
        tv_yaw_moment_saturated;

    out.rate_saturated =
        sat.rate_saturated;

    out.steering_angle_saturated =
        sat.steering_angle_saturated;

    out.delta_act_next =
        x_next(IX_DELTA);

    /*
        Use the PT2 state directly. The wrapper must carry this value into
        current_state_.delta_dot on the next control iteration.
    */
    out.delta_dot_next =
        x_next(IX_DELTA_DOT);

    out.delta_cmd_next =
        x_next(IX_DELTA_CMD);

    out.delta_vehicle_used =
        x_now(IX_DELTA);

    out.vy_next =
        x_next(2);

    out.r_next =
        x_next(3);

    const auto t_total_end = Clock::now();

    static int debug_counter = 0;
    ++debug_counter;

    const bool should_print =
        param_.getBool("general.print_console_debug_info") &&
        (
            (debug_counter % kMpcDebugPrintEvery == 0) ||
            out.u_delta_cmd_saturated
        );

    if (should_print)
    {
        std::cout
            << std::fixed << std::setprecision(3)
            << "[LTV_MPC_DEBUG] "
#if defined(DV_CONTROL_USE_TUSTIN_DISCRETIZATION)
            << "discretization=tustin, "
#else
            << "discretization=exact_expm, "
#endif
            << "total=" << msBetween(t_total_begin, t_total_end) << " ms, "
            << "validation=" << msBetween(t_total_begin, t_after_validation) << " ms, "
            << "linearization=" << msBetween(t_linearization_begin, t_linearization_end) << " ms, "
            << "qp_build=" << msBetween(t_qp_build_begin, t_qp_build_end) << " ms, "
            << "qp_solve=" << msBetween(t_qp_solve_begin, t_qp_solve_end) << " ms, "
            << "limits=" << msBetween(t_limit_begin, t_limit_end) << " ms, "
            << "prediction=" << msBetween(t_prediction_begin, t_prediction_end) << " ms"
            << " | raw_u_delta=" << out.u_delta_cmd_raw
            << ", limited_u_delta=" << out.u_delta_cmd
            << ", dMtv=" << u_dmtv
            << ", rate_sat=" << out.rate_saturated
            << ", steer_angle_sat=" << out.steering_angle_saturated
            << ", any_sat=" << out.u_delta_cmd_saturated
            << ", delta_cmd_now=" << x0.delta_cmd
            << ", delta_cmd_next=" << out.delta_cmd_next
            << ", delta_vehicle_used=" << out.delta_vehicle_used
            << ", Mz_tv_raw=" << out.u_tv_yaw_moment_raw_Nm
            << ", Mz_tv=" << out.u_tv_yaw_moment_Nm
            << ", Mz_tv_sat=" << out.u_tv_yaw_moment_saturated
            << ", vy_next=" << out.vy_next
            << ", r_next=" << out.r_next
            << std::endl;
    }

    return out;
}

// =============================================================================
//                              DYNAMICS / LINEARIZATION
// =============================================================================

Eigen::Matrix<double, UnboundedLtvMpc::NX, 1>
UnboundedLtvMpc::continuousDynamics(
    const Eigen::Matrix<double, NX, 1>& x,
    const Eigen::Matrix<double, NU, 1>& u,
    double v_body_model,
    double kappa_ref) const
{
    const double m  = param_.get("model.body.m");
    const double g  = param_.get("model.body.g");
    const double Iz = param_.get("model.body.Iz");

    const double lf = param_.get("model.body.l_f");
    const double lr = param_.get("model.body.l_r");

    const double C_f_smf = param_.get("model.body.Cf");
    const double D_f_smf = param_.get("model.body.Df");
    const double B_f_smf = param_.get("model.body.Bf");

    const double C_r_smf = param_.get("model.body.Cr");
    const double D_r_smf = param_.get("model.body.Dr");
    const double B_r_smf = param_.get("model.body.Br");

    const double v_min =
        param_.get("model.ltv_mpc_unbounded.v_min");

    const double vx_body =
        safeForwardBodySpeed(v_body_model, v_min);

    const double ey =
        x(0);

    const double epsi =
        x(1);

    const double vy =
        x(2);

    const double r =
        x(3);

    const double delta_vehicle =
        x(IX_DELTA);

    const double s_epsi =
        std::sin(epsi);

    const double c_epsi =
        std::cos(epsi);

    const double denom =
        safeFrenetDenom(1.0 - kappa_ref * ey);

    const double v_path_projection =
        vx_body * c_epsi - vy * s_epsi;

    const double s_dot =
        v_path_projection / denom;

    const double ey_dot_time =
        vx_body * s_epsi + vy * c_epsi;

    const double L =
        lf + lr;

    const AxleNormalLoads Fz_axle =
        computeAeroAwareAxleNormalLoadsN(
            param_,
            vx_body,
            m,
            g,
            lf,
            lr
        );

    const double Fz_front_axle =
        Fz_axle.front_N;

    const double Fz_rear_axle =
        Fz_axle.rear_N;

    const double alpha_f =
        delta_vehicle - std::atan2(vy + lf * r, vx_body);

    const double alpha_r =
        -std::atan2(vy - lr * r, vx_body);

    const double Fyf =
        Fz_front_axle *
        D_f_smf *
        std::sin(C_f_smf * std::atan(B_f_smf * alpha_f));

    const double Fyr =
        Fz_rear_axle *
        D_r_smf *
        std::sin(C_r_smf * std::atan(B_r_smf * alpha_r));

    Eigen::Matrix<double, NX, 1> xdot;
    xdot.setZero();

    xdot(0) =
        ey_dot_time;

    xdot(1) =
        r - kappa_ref * s_dot;

    xdot(2) =
        (Fyf + Fyr) / m - vx_body * r;

    double tv_yaw_moment =
        0.0;

    if (useJacaTorqueVectoring())
    {
        const TorqueVectoringYawTarget tv_yaw_target =
            calculateTorqueVectoringYawTarget(
                param_,
                vx_body,
                delta_vehicle,
                L
            );

        tv_yaw_moment =
            tv_yaw_target.p_tv_gain *
            (tv_yaw_target.target_yaw_rate - r);
    }
    else if (useOptimizedTorqueVectoring())
    {
        /*
            Optimized TV yaw moment is now a state:
                x(IX_MTV) = Mz [Nm]
                u(IU_DMTV) = dMz/dt [Nm/s]

            Yaw dynamics:
                r_dot += Mz / Iz
        */
        tv_yaw_moment =
            x(IX_MTV);
    }

    xdot(3) =
        (lf * Fyf - lr * Fyr) / Iz + tv_yaw_moment / Iz;

    /*
        PT2 steering actuator is always active.

        State:
            x(IX_DELTA)     = measured/physical steering angle supplied by
                              the wrapper at the beginning of this solve,
            x(IX_DELTA_DOT) = model steering-rate state,
            x(IX_DELTA_CMD) = integrated absolute steering command.

        The PT2 actuator is driven directly by delta_cmd.
    */
    const double steering_omega =
        param_.get("model.steering_system.steer_natural_freq");

    const double steering_damping =
        param_.get("model.steering_system.steer_damping");

    const double steering_omega_sq =
        steering_omega * steering_omega;

    xdot(IX_DELTA) =
        x(IX_DELTA_DOT);

    xdot(IX_DELTA_DOT) =
        -steering_omega_sq * x(IX_DELTA)
        -2.0 * steering_damping * steering_omega * x(IX_DELTA_DOT)
        +steering_omega_sq * x(IX_DELTA_CMD);

    xdot(IX_DELTA_CMD) =
        u(IU_DELTA_CMD_DOT);

    if (useOptimizedTorqueVectoring())
    {
        xdot(IX_MTV) =
            u(IU_DMTV);
    }

    return xdot;
}

void UnboundedLtvMpc::calculateContinuousJacobian(
    const Eigen::Matrix<double, NX, 1>& x,
    const Eigen::Matrix<double, NU, 1>& /*u*/,
    double v_body_model,
    double kappa_ref,
    Eigen::Matrix<double, NX, NX>& Ac,
    Eigen::Matrix<double, NX, NU>& Bc) const
{
    Ac.setZero();
    Bc.setZero();

    const double m  = param_.get("model.body.m");
    const double g  = param_.get("model.body.g");
    const double Iz = param_.get("model.body.Iz");

    const double lf = param_.get("model.body.l_f");
    const double lr = param_.get("model.body.l_r");

    const double C_f_smf = param_.get("model.body.Cf");
    const double D_f_smf = param_.get("model.body.Df");
    const double B_f_smf = param_.get("model.body.Bf");

    const double C_r_smf = param_.get("model.body.Cr");
    const double D_r_smf = param_.get("model.body.Dr");
    const double B_r_smf = param_.get("model.body.Br");

    const double v_min =
        param_.get("model.ltv_mpc_unbounded.v_min");

    const double vx_body =
        safeForwardBodySpeed(v_body_model, v_min);

    const double ey =
        x(0);

    const double epsi =
        x(1);

    const double vy =
        x(2);

    const double r =
        x(3);

    const double delta_vehicle =
        x(IX_DELTA);

    const double s_epsi =
        std::sin(epsi);

    const double c_epsi =
        std::cos(epsi);

    // ============================================================
    // Frenet kinematics:
    //
    // ey_dot   = vx*sin(epsi) + vy*cos(epsi)
    // epsi_dot = r - kappa*s_dot
    //
    // s_dot = (vx*cos(epsi) - vy*sin(epsi)) / (1 - kappa*ey)
    // ============================================================
    Ac(0, 1) =
        vx_body * c_epsi - vy * s_epsi;

    Ac(0, 2) =
        c_epsi;

    const double denom_raw =
        1.0 - kappa_ref * ey;

    const double denom =
        safeFrenetDenom(denom_raw);

    const bool denom_clamped =
        (!std::isfinite(denom_raw)) ||
        (std::abs(denom_raw) < kFrenetDenomMin);

    const double v_path_projection =
        vx_body * c_epsi - vy * s_epsi;

    if (!denom_clamped)
    {
        const double ds_dot_dey =
            v_path_projection * kappa_ref / (denom * denom);

        Ac(1, 0) =
            -kappa_ref * ds_dot_dey;
    }

    const double ds_dot_depsi =
        (-vx_body * s_epsi - vy * c_epsi) / denom;

    const double ds_dot_dvy =
        -s_epsi / denom;

    Ac(1, 1) =
        -kappa_ref * ds_dot_depsi;

    Ac(1, 2) =
        -kappa_ref * ds_dot_dvy;

    Ac(1, 3) =
        1.0;

    // ============================================================
    // Tire model:
    //
    // alpha_f = delta_vehicle - atan2(vy + lf*r, vx)
    // alpha_r =              - atan2(vy - lr*r, vx)
    //
    // Fy = Fz * D * sin(C * atan(B * alpha))
    // ============================================================
    const double L =
        lf + lr;

    const AxleNormalLoads Fz_axle =
        computeAeroAwareAxleNormalLoadsN(
            param_,
            vx_body,
            m,
            g,
            lf,
            lr
        );

    const double Fz_front_axle =
        Fz_axle.front_N;

    const double Fz_rear_axle =
        Fz_axle.rear_N;

    const double yf =
        vy + lf * r;

    const double yr =
        vy - lr * r;

    const double alpha_f =
        delta_vehicle - std::atan2(yf, vx_body);

    const double alpha_r =
        -std::atan2(yr, vx_body);

    const auto dFy_dAlpha = [](double Fz,
                               double D,
                               double C,
                               double B,
                               double alpha) -> double
    {
        const double B_alpha =
            B * alpha;

        const double atan_B_alpha =
            std::atan(B_alpha);

        return Fz * D *
               std::cos(C * atan_B_alpha) *
               C *
               B /
               (1.0 + B_alpha * B_alpha);
    };

    const double dFyf_dalpha =
        dFy_dAlpha(
            Fz_front_axle,
            D_f_smf,
            C_f_smf,
            B_f_smf,
            alpha_f
        );

    const double dFyr_dalpha =
        dFy_dAlpha(
            Fz_rear_axle,
            D_r_smf,
            C_r_smf,
            B_r_smf,
            alpha_r
        );

    const double denom_f =
        vx_body * vx_body + yf * yf;

    const double denom_r =
        vx_body * vx_body + yr * yr;

    const double datan_f_dyf =
        vx_body / denom_f;

    const double datan_r_dyr =
        vx_body / denom_r;

    const double dalpha_f_dvy =
        -datan_f_dyf;

    const double dalpha_f_dr =
        -lf * datan_f_dyf;

    const double dalpha_f_ddelta =
        1.0;

    const double dalpha_r_dvy =
        -datan_r_dyr;

    const double dalpha_r_dr =
        lr * datan_r_dyr;

    const double dFyf_dvy =
        dFyf_dalpha * dalpha_f_dvy;

    const double dFyf_dr =
        dFyf_dalpha * dalpha_f_dr;

    const double dFyf_ddelta =
        dFyf_dalpha * dalpha_f_ddelta;

    const double dFyr_dvy =
        dFyr_dalpha * dalpha_r_dvy;

    const double dFyr_dr =
        dFyr_dalpha * dalpha_r_dr;

    // ============================================================
    // Lateral dynamics:
    //
    // vy_dot = (Fyf + Fyr)/m - vx*r
    // r_dot  = (lf*Fyf - lr*Fyr)/Iz
    //
    // Steering state used by the vehicle: x(IX_DELTA).
    // ============================================================
    Ac(2, 2) =
        (dFyf_dvy + dFyr_dvy) / m;

    Ac(2, 3) =
        (dFyf_dr + dFyr_dr) / m - vx_body;

    Ac(2, IX_DELTA) =
        dFyf_ddelta / m;

    TorqueVectoringYawTarget tv_yaw_target;

    if (useJacaTorqueVectoring())
    {
        tv_yaw_target =
            calculateTorqueVectoringYawTarget(
                param_,
                vx_body,
                delta_vehicle,
                L
            );
    }

    Ac(3, 2) =
        (lf * dFyf_dvy - lr * dFyr_dvy) / Iz;

    Ac(3, 3) =
        (lf * dFyf_dr - lr * dFyr_dr) / Iz
        - tv_yaw_target.p_tv_gain / Iz;

    Ac(3, IX_DELTA) =
        lf * dFyf_ddelta / Iz
        + tv_yaw_target.p_tv_gain *
          tv_yaw_target.d_target_yaw_rate_d_delta / Iz;

    if (useOptimizedTorqueVectoring())
    {
        /*
            r_dot += Mz / Iz,
            Mz_dot = dMtv.
        */
        Ac(3, IX_MTV) =
            1.0 / Iz;

        Bc(IX_MTV, IU_DMTV) =
            1.0;
    }

    // ============================================================
    // Steering PT2:
    //
    // delta_dot  = delta_rate
    // delta_ddot = -omega^2*delta
    //              -2*zeta*omega*delta_rate
    //              +omega^2*delta_input
    //
    // PT2 input is the current command state x(IX_DELTA_CMD).
    // ============================================================
    const double steering_omega =
        param_.get("model.steering_system.steer_natural_freq");

    const double steering_damping =
        param_.get("model.steering_system.steer_damping");

    const double steering_omega_sq =
        steering_omega * steering_omega;

    Ac(IX_DELTA, IX_DELTA_DOT) =
        1.0;

    Ac(IX_DELTA_DOT, IX_DELTA) =
        -steering_omega_sq;

    Ac(IX_DELTA_DOT, IX_DELTA_DOT) =
        -2.0 * steering_damping * steering_omega;

    Ac(IX_DELTA_DOT, IX_DELTA_CMD) =
        steering_omega_sq;

    // ============================================================
    // Input:
    //
    // delta_cmd_dot = u
    // ============================================================
    Bc(IX_DELTA_CMD, IU_DELTA_CMD_DOT) =
        1.0;
}

Eigen::Matrix<double, UnboundedLtvMpc::NX, 1>
UnboundedLtvMpc::rk4Step(
    const Eigen::Matrix<double, NX, 1>& x,
    const Eigen::Matrix<double, NU, 1>& u,
    double v_body_model,
    double kappa_ref,
    double dt) const
{
    const auto k1 =
        continuousDynamics(x, u, v_body_model, kappa_ref);

    const auto k2 =
        continuousDynamics(x + 0.5 * dt * k1, u, v_body_model, kappa_ref);

    const auto k3 =
        continuousDynamics(x + 0.5 * dt * k2, u, v_body_model, kappa_ref);

    const auto k4 =
        continuousDynamics(x + dt * k3, u, v_body_model, kappa_ref);

    Eigen::Matrix<double, NX, 1> x_next =
        x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    /*
        Keep command integration exactly discrete:

            delta_cmd[k+1] =
                delta_cmd[k] + dt * delta_cmd_dot[k]

    */
    x_next(IX_DELTA_CMD) =
        x(IX_DELTA_CMD)
        + dt * u(IU_DELTA_CMD_DOT);

    return x_next;
}

void UnboundedLtvMpc::buildDiscreteLinearization(
    const Eigen::Matrix<double, NX, 1>& x_nom,
    const Eigen::Matrix<double, NU, 1>& u_nom,
    double v_body_model,
    double kappa_ref,
    double dt,
    Eigen::Matrix<double, NX, NX>& Ad,
    Eigen::Matrix<double, NX, NU>& Bd,
    Eigen::Matrix<double, NX, 1>& gd,
    Eigen::Matrix<double, NX, 1>& x_next_nom) const
{
    Eigen::Matrix<double, NX, NX> Ac;
    Eigen::Matrix<double, NX, NU> Bc;

    calculateContinuousJacobian(
        x_nom,
        u_nom,
        v_body_model,
        kappa_ref,
        Ac,
        Bc
    );

    discretizeExpmAB(
        Ac,
        Bc,
        dt,
        Ad,
        Bd
    );

    /*
        Exact discrete override for the command integrator:

            delta_cmd[k+1] =
                delta_cmd[k] + dt * delta_cmd_dot[k]

        Do not overwrite IX_MTV: it is now state index 7.
    */
    Ad.row(IX_DELTA_CMD).setZero();
    Bd.row(IX_DELTA_CMD).setZero();

    Ad(IX_DELTA_CMD, IX_DELTA_CMD) =
        1.0;

    Bd(IX_DELTA_CMD, IU_DELTA_CMD_DOT) =
        dt;

    x_next_nom =
        rk4Step(
            x_nom,
            u_nom,
            v_body_model,
            kappa_ref,
            dt
        );

    gd =
        x_next_nom - Ad * x_nom - Bd * u_nom;
}

std::vector<double> UnboundedLtvMpc::integrateBodySpeedHorizon(
    double v_body_initial_mps,
    const std::vector<double>& ax_body_ref_horizon,
    double dt) const
{
    validateBodyAccelerationHorizon(
        ax_body_ref_horizon,
        N_,
        "UnboundedLtvMpc"
    );

    const double v_min =
        param_.get("model.ltv_mpc_unbounded.v_min");

    std::vector<double> v_body_ref_horizon;
    v_body_ref_horizon.reserve(static_cast<std::size_t>(N_));

    double v_body =
        safeForwardBodySpeed(v_body_initial_mps, v_min);

    for (int k = 0; k < N_; ++k)
    {
        v_body_ref_horizon.push_back(v_body);

        const double ax_body =
            ax_body_ref_horizon[static_cast<std::size_t>(k)];

        v_body =
            safeForwardBodySpeed(v_body + ax_body * dt, v_min);
    }

    return v_body_ref_horizon;
}

// =============================================================================
//                              PARAMS / MODE HELPERS
// =============================================================================


bool UnboundedLtvMpc::useJacaTorqueVectoring() const
{
    return param_.getBool("general.general_use_jaca_torque_vectoring");
}

bool UnboundedLtvMpc::useOptimizedTorqueVectoring() const
{
    /*
        I give JACA priority.

        If JACA is enabled, the second decision variable Mz is still present
        in the QP, but it has zero influence on the dynamics and is driven to
        zero by its positive quadratic cost.
    */
    if (useJacaTorqueVectoring())
    {
        return false;
    }

    return param_.getBool("general.general_use_torque_vectoring");
}

double UnboundedLtvMpc::readTvYawMomentCost() const
{
    if (useOptimizedTorqueVectoring())
    {
        return param_.get("model.ltv_mpc_unbounded.R_tv_yaw_moment");
    }

    /*
        Small positive regularization for the unused Mz input.
        I do not require an extra config key when optimized TV is disabled.
    */
    return 1.0e-6;
}

double UnboundedLtvMpc::maxOptimizedTvYawMomentNm() const
{
    if (!useOptimizedTorqueVectoring())
    {
        return 0.0;
    }

    /*
        Optimized-TV limit convention used by the current config:

            model.torque_vectoring.max_torque_diff_Nm_engine

        is the maximum EXTRA motor-side torque that TV may add/subtract
        on one wheel [Nm engine-side].

        For 4WD TV, the yaw moment from a symmetric left/right torque split is:

            left wheels:  -dT_engine on FL and RL
            right wheels: +dT_engine on FR and RR

            wheel-side delta per wheel:
                dT_wheel = dT_engine * gear_ratio

            total right-left force difference:
                4 * dT_wheel / R_tire

            yaw moment:
                Mz_max = 0.5 * track_width * (4 * dT_wheel / R_tire)

            so:
                Mz_max = 2 * track_width * dT_engine * gear_ratio / R_tire
    */
    const double max_torque_diff_engine_per_wheel_Nm =
        param_.get("model.torque_vectoring.max_torque_diff_Nm_engine");

    const double gear_ratio =
        param_.get("model.drivetrain.gear_ratio");

    const double track_width =
        param_.get("model.torque_vectoring_jaca.track_width");

    const double tire_radius =
        param_.get("model.tire.R_tire");

    if (!std::isfinite(max_torque_diff_engine_per_wheel_Nm) ||
        !std::isfinite(gear_ratio) ||
        !std::isfinite(track_width) ||
        !std::isfinite(tire_radius) ||
        track_width <= 0.0 ||
        std::abs(gear_ratio) <= 1.0e-9 ||
        std::abs(tire_radius) <= 1.0e-9)
    {
        return 0.0;
    }

    return
        2.0
        * track_width
        * std::abs(max_torque_diff_engine_per_wheel_Nm)
        * std::abs(gear_ratio)
        / std::abs(tire_radius);
}


void UnboundedLtvMpc::validateParams() const
{
    requirePositive("model.ltv_mpc_unbounded.N");

    requireNonNegative("model.ltv_mpc_unbounded.Q_ey");
    requireNonNegative("model.ltv_mpc_unbounded.Q_psi");
    requireNonNegative("model.ltv_mpc_unbounded.Q_vy");
    requireNonNegative("model.ltv_mpc_unbounded.Q_r");

    requirePositive("model.ltv_mpc_unbounded.R_d_delta");

    requirePositive("model.ltv_mpc_unbounded.v_min");
    requirePositive("model.ltv_mpc_unbounded.hessian_regularization");
    requireNonNegative("model.ltv_mpc_unbounded.terminal_scale");

    /*
        The PT2 steering model is always active.
    */
    requirePositive("model.steering_system.steer_natural_freq");
    requireNonNegative("model.steering_system.steer_damping");

    requirePositive("model.frequency.steer_cmd_loop_hz");

    requirePositive("model.steering_limit.max_steer");
    requirePositive("model.steering_limit.max_steer_rate");

    requirePositive("model.body.m");
    requirePositive("model.body.g");
    requirePositive("model.body.Iz");
    requirePositive("model.body.l_f");
    requirePositive("model.body.l_r");

    requirePositive("model.body.Cf");
    requirePositive("model.body.Df");
    requirePositive("model.body.Bf");

    requirePositive("model.body.Cr");
    requirePositive("model.body.Dr");
    requirePositive("model.body.Br");

    (void)param_.getBool("model.body.use_aero");

    if (param_.getBool("model.body.use_aero"))
    {
        requireNonNegative("model.body.Cl1");
        requireNonNegative("model.body.Cl2");
    }

    (void)param_.getBool("general.general_use_jaca_torque_vectoring");
    (void)param_.getBool("general.general_use_torque_vectoring");

    if (param_.getBool("general.general_use_jaca_torque_vectoring"))
    {
        requireNonNegative("model.torque_vectoring_jaca.p_tv_gain");

        requireFinite("model.torque_vectoring_jaca.speed_bled_low");
        requireFinite("model.torque_vectoring_jaca.speed_bled_high");

        const double tv_speed_low =
            param_.get("model.torque_vectoring_jaca.speed_bled_low");

        const double tv_speed_high =
            param_.get("model.torque_vectoring_jaca.speed_bled_high");

        if (tv_speed_high <= tv_speed_low)
        {
            throw std::runtime_error(
                "[UnboundedLtvMpc] model.torque_vectoring_jaca.speed_bled_high must be > speed_bled_low"
            );
        }

        requireNonNegative("model.torque_vectoring_jaca.turn_radius_speed_gain");
        requireFinite("model.torque_vectoring_jaca.over_steer_gain");

        if (param_.get("model.torque_vectoring_jaca.over_steer_gain") <= -1.0)
        {
            throw std::runtime_error(
                "[UnboundedLtvMpc] model.torque_vectoring_jaca.over_steer_gain must be > -1"
            );
        }

        requirePositive("model.torque_vectoring_jaca.transition_gain");
    }

    if (useOptimizedTorqueVectoring())
    {
        requirePositive("model.ltv_mpc_unbounded.R_tv_yaw_moment");
        requireNonNegative("model.ltv_mpc_unbounded.R_d_tv_yaw_moment");
        requirePositive("model.torque_vectoring.max_torque_diff_Nm_engine");
        requirePositive("model.drivetrain.gear_ratio");
        requirePositive("model.torque_vectoring_jaca.track_width");
        requirePositive("model.tire.R_tire");
    }

    const int N =
        param_.getInt("model.ltv_mpc_unbounded.N");

    if (N <= 0)
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] model.ltv_mpc_unbounded.N must be > 0"
        );
    }
}

int UnboundedLtvMpc::readHorizonN() const
{
    const int N =
        param_.getInt("model.ltv_mpc_unbounded.N");

    if (N <= 0)
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] model.ltv_mpc_unbounded.N must be > 0"
        );
    }

    return N;
}

void UnboundedLtvMpc::requireFinite(const std::string& key) const
{
    const double v =
        param_.get(key);

    if (!std::isfinite(v))
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] parameter '" + key + "' is not finite"
        );
    }
}

void UnboundedLtvMpc::requirePositive(const std::string& key) const
{
    requireFinite(key);

    const double v =
        param_.get(key);

    if (v <= 0.0)
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] parameter '" + key + "' must be > 0"
        );
    }
}

void UnboundedLtvMpc::requireNonNegative(const std::string& key) const
{
    requireFinite(key);

    const double v =
        param_.get(key);

    if (v < 0.0)
    {
        throw std::runtime_error(
            "[UnboundedLtvMpc] parameter '" + key + "' must be >= 0"
        );
    }
}

void UnboundedLtvMpc::requireBoolLike(const std::string& key) const
{
    (void)param_.getBool(key);
}

// =============================================================================
//                              INTERNAL HELPERS
// =============================================================================

void UnboundedLtvMpc::resetOutput()
{
    is_initialized_ = false;
    last_output_.assign(static_cast<std::size_t>(N_ * NU), 0.0);
    last_tv_yaw_moment_state_Nm_ = 0.0;
}

void UnboundedLtvMpc::discretizeExpmAB(
    const Eigen::Matrix<double, NX, NX>& A,
    const Eigen::Matrix<double, NX, NU>& B,
    double dt,
    Eigen::Matrix<double, NX, NX>& Ad,
    Eigen::Matrix<double, NX, NU>& Bd)
{
#if defined(DV_CONTROL_USE_TUSTIN_DISCRETIZATION)
    /*
        Optional bilinear/Tustin discretization:

            Ad = (I - 0.5*dt*A)^(-1) * (I + 0.5*dt*A)
            Bd = (I - 0.5*dt*A)^(-1) * dt*B

        This is substantially faster than the exact augmented matrix
        exponential, but it is an approximation of zero-order hold.

        The CMake option is OFF by default, so the exact controller
        discretization remains the default.
    */
    const Eigen::Matrix<double, NX, NX> I =
        Eigen::Matrix<double, NX, NX>::Identity();

    const Eigen::Matrix<double, NX, NX> lhs =
        I - 0.5 * dt * A;

    const Eigen::Matrix<double, NX, NX> rhs_A =
        I + 0.5 * dt * A;

    Eigen::PartialPivLU<Eigen::Matrix<double, NX, NX>> lu(lhs);

    Ad =
        lu.solve(rhs_A);

    Bd =
        lu.solve(dt * B);
#else
    /*
        Exact zero-order-hold discretization using the augmented matrix
        exponential.
    */
    Eigen::Matrix<double, NX + NU, NX + NU> M =
        Eigen::Matrix<double, NX + NU, NX + NU>::Zero();

    M.template block<NX, NX>(0, 0) =
        A;

    M.template block<NX, NU>(0, NX) =
        B;

    const Eigen::Matrix<double, NX + NU, NX + NU> E =
        (M * dt).exp();

    Ad =
        E.template block<NX, NX>(0, 0);

    Bd =
        E.template block<NX, NU>(0, NX);
#endif
}

} // namespace acc_launch_control
