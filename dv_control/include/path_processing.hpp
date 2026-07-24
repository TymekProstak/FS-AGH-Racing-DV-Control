#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "ParamBank.hpp"
#include "Vec2.hpp"
#include "control_types.hpp"
#include "spline.hpp"

namespace dv_control
{

static constexpr double kPlannerPi = 3.141592653589793238462643383279502884;
static constexpr double kClosedTrackBfForwardDistanceM = 80.0;
static constexpr double kPlannerMinPowerSpeedMps = 0.5;
static constexpr double kTwoPointPathTargetSpeedMps = 3.0;

// =============================================================================
//                              RESULT STRUCTS
// =============================================================================

struct LocalPlannerResult
{
    Eigen::VectorXd X_ref;
    Eigen::VectorXd Y_ref;

    std::vector<double> curvature_ref;
    std::vector<double> acceleration_ref;
    std::vector<double> velocity_ref;

    bool valid = false;
    bool before_bolide = false;
    bool after_bolide = false;

    double ey = 0.0;
    double epsi = 0.0;

    /*
        Nearest reference/projection point.

        Normal case:
            x_ref_point, y_ref_point = projection of bolide onto spline
            s = spline arc length of this projection

        Before-start case:
            x_ref_point, y_ref_point = orthogonal projection of the bolide
                                       onto the straight continuation of the
                                       first raw path segment P0 -> P1
            s = signed distance from P0 to that projection along P0 -> P1,
                therefore s < 0

        After-end case:
            x_ref_point, y_ref_point = orthogonal projection of the bolide
                                       onto the straight continuation of the
                                       open spline endpoint tangent
            s = spline length + signed distance from the endpoint to that
                projection along the endpoint tangent
    */
    double x_ref_point = 0.0;
    double y_ref_point = 0.0;
    double s = 0.0;
};

struct BfProfile
{
    double s0 = 0.0;
    double ds = 0.1;
    double s_end = 0.0;

    std::vector<double> s;
    std::vector<double> kappa;
    std::vector<double> v;
    std::vector<double> a;
};

struct PlannerLongitudinalLimits
{
    double m = 1.0;
    double g = 9.81;

    double ax_max = 0.0;
    double ay_max = 0.0;

    double drive_power_limit_W = 0.0;
    double brake_power_limit_W = 0.0;

    /*
        In this planner I treat model.body.Cd as a lumped quadratic drag
        coefficient, i.e. F_drag = Cd * v^2.
        If later Cd is a dimensionless aerodynamic coefficient, replace this
        with 0.5 * rho * Cd * A * v^2 and add rho/A to the config.
    */
    double drag_coeff = 0.0;
    double rolling_resistance_coeff = 0.0;
};


// =============================================================================
//                              BASIC HELPERS
// =============================================================================

static inline double safeSqrt(double x)
{
    return std::sqrt(std::max(0.0, x));
}

static inline double wrapPi(double a)
{
    while (a > kPlannerPi)
    {
        a -= 2.0 * kPlannerPi;
    }

    while (a < -kPlannerPi)
    {
        a += 2.0 * kPlannerPi;
    }

    return a;
}

static inline Vec2 eigenPoint(const Eigen::VectorXd& X,
                              const Eigen::VectorXd& Y,
                              int i)
{
    return Vec2(
        static_cast<float>(X(i)),
        static_cast<float>(Y(i))
    );
}

struct PreparedLocalPath
{
    TrackSpline2D spline;
    bool valid = false;
};

static inline PreparedLocalPath prepareLocalPath(
    const Eigen::VectorXd& X_raw,
    const Eigen::VectorXd& Y_raw,
    bool closed_track)
{
    PreparedLocalPath prepared;

    if (X_raw.size() != Y_raw.size())
    {
        return prepared;
    }

    const int R =
        static_cast<int>(X_raw.size());

    if (R < 2 || (closed_track && R < 3))
    {
        return prepared;
    }

    std::vector<Vec2> raw_points;
    raw_points.reserve(static_cast<std::size_t>(R));

    for (int i = 0; i < R; ++i)
    {
        raw_points.push_back(
            eigenPoint(X_raw, Y_raw, i)
        );
    }

    prepared.spline.build(
        raw_points,
        closed_track
    );

    prepared.valid =
        prepared.spline.valid();

    return prepared;
}

// =============================================================================
//                              BF PROFILE HELPERS
// =============================================================================

static inline PlannerLongitudinalLimits loadPlannerLongitudinalLimits(
    const ParamBank& P)
{
    PlannerLongitudinalLimits lim;

    lim.m =
        std::max(1.0e-6, P.get("model.body.m"));

    lim.g =
        P.get("model.body.g");

    lim.ax_max =
        std::max(0.0, P.get("general.ax_max"));

    lim.ay_max =
        std::max(0.0, P.get("general.ay_max"));

    lim.drive_power_limit_W =
        std::max(0.0, P.get("model.drivetrain.P_wheel_drive_max_W"));

    lim.brake_power_limit_W =
        std::max(0.0, P.get("model.drivetrain.P_wheel_brake_max_W"));

    lim.drag_coeff =
        std::max(0.0, P.get("model.body.Cd"));

    lim.rolling_resistance_coeff =
        std::max(0.0, P.get("model.body.rolling_resistance_coeff"));

    return lim;
}

static inline double longitudinalResistanceForce(
    double v,
    const PlannerLongitudinalLimits& lim)
{
    const double v_abs =
        std::abs(v);

    const double F_drag =
        lim.drag_coeff * v_abs * v_abs;

    const double F_roll =
        lim.rolling_resistance_coeff * lim.m * lim.g;

    return std::max(0.0, F_drag + F_roll);
}

static inline double longitudinalLimitFromEllipse(
    double v,
    double kappa,
    const PlannerLongitudinalLimits& lim)
{
    const double ay =
        v * v * std::abs(kappa);

    if (lim.ay_max <= 1.0e-9)
    {
        return 0.0;
    }

    const double ratio =
        std::min(1.0, ay / lim.ay_max);

    const double free_part =
        std::max(0.0, 1.0 - ratio * ratio);

    return lim.ax_max * std::sqrt(free_part);
}

static inline double driveAccelerationLimit(
    double v,
    double kappa,
    const PlannerLongitudinalLimits& lim)
{
    const double ellipse_ax =
        longitudinalLimitFromEllipse(v, kappa, lim);

    const double v_power =
        std::max(std::abs(v), kPlannerMinPowerSpeedMps);

    /*
        Power condition for acceleration:
            P_drive >= F_drive * v ~= m * ax * v

        I also subtract drag and rolling resistance, because only the
        remaining longitudinal force can increase kinetic energy.
    */
    const double F_from_power =
        lim.drive_power_limit_W / v_power;

    const double F_resistance =
        longitudinalResistanceForce(v, lim);

    const double power_ax =
        std::max(0.0, (F_from_power - F_resistance) / lim.m);

    return std::min(ellipse_ax, power_ax);
}

static inline double brakeDecelerationLimit(
    double v,
    double kappa,
    const PlannerLongitudinalLimits& lim)
{
    const double ellipse_ax =
        longitudinalLimitFromEllipse(v, kappa, lim);

    const double v_power =
        std::max(std::abs(v), kPlannerMinPowerSpeedMps);

    /*
        Power condition for braking / slowing down:
            P_brake >= F_brake * v

        Drag and rolling resistance help the car slow down, so they are added
        to the available braking force in the backward pass.
    */
    const double F_from_power =
        lim.brake_power_limit_W / v_power;

    const double F_resistance =
        longitudinalResistanceForce(v, lim);

    const double power_ax =
        std::max(0.0, (F_from_power + F_resistance) / lim.m);

    return std::min(ellipse_ax, power_ax);
}

static inline double curvatureSpeedLimit(double kappa,
                                         double v_min,
                                         double v_max,
                                         double ay_max)
{
    const double kk =
        std::abs(kappa);

    if (kk < 1.0e-9)
    {
        return v_max;
    }

    const double v_kappa =
        std::sqrt(std::max(0.0, ay_max / kk));

    return std::clamp(v_kappa, v_min, v_max);
}

static inline double profileTime(const BfProfile& prof)
{
    double T = 0.0;

    for (int i = 0; i + 1 < static_cast<int>(prof.v.size()); ++i)
    {
        const double v_sum =
            std::max(1.0e-3, prof.v[i] + prof.v[i + 1]);

        T += 2.0 * prof.ds / v_sum;
    }

    return T;
}

static inline void computeAcceleration(BfProfile& prof)
{
    const int M =
        static_cast<int>(prof.v.size());

    prof.a.assign(M, 0.0);

    if (M < 2)
    {
        return;
    }

    for (int i = 0; i + 1 < M; ++i)
    {
        prof.a[i] =
            (prof.v[i + 1] * prof.v[i + 1] - prof.v[i] * prof.v[i]) /
            (2.0 * prof.ds);
    }

    prof.a[M - 1] =
        prof.a[M - 2];
}


static inline void runBackwardPass(
    BfProfile& prof,
    double v_min,
    double v_max,
    const PlannerLongitudinalLimits& long_lim)
{
    const int M =
        static_cast<int>(prof.v.size());

    if (M < 2 || prof.ds <= 1.0e-9)
    {
        return;
    }

    /*
        Index 0 is the measured current speed of the bolide.

        The backward pass may limit only future profile samples. It must not
        rewrite the current state to a lower, fictitious speed merely because
        the car cannot satisfy a future curvature/terminal constraint within
        the available distance.

        Therefore the pass stops at i = 1. Sample v[0] remains exactly the
        measured/clamped initial speed set by the caller.
    */
    for (int i = M - 2; i >= 1; --i)
    {
        const double ax =
            brakeDecelerationLimit(
                prof.v[i + 1],
                prof.kappa[i + 1],
                long_lim
            );

        const double v_reachable_before =
            safeSqrt(
                prof.v[i + 1] * prof.v[i + 1] +
                2.0 * ax * prof.ds
            );

        prof.v[i] =
            std::min(prof.v[i], v_reachable_before);

        prof.v[i] =
            std::clamp(prof.v[i], v_min, v_max);
    }
}

static inline BfProfile runForwardBackwardOpenSpline(
    const TrackSpline2D& spline,
    double s0,
    double v0,
    double v_min,
    double v_max,
    const PlannerLongitudinalLimits& long_lim,
    double ds)
{
    BfProfile prof;

    if (!spline.valid() || ds <= 1.0e-6)
    {
        return prof;
    }

    const double L =
        spline.totalLength();

    if (L <= s0 + 1.0e-9)
    {
        return prof;
    }

    const int M =
        std::max(
            2,
            static_cast<int>(
                std::ceil((L - s0) / ds)
            ) + 1
        );

    const double ds_effective =
        (L - s0) / static_cast<double>(M - 1);

    prof.s0 = s0;
    prof.ds = ds_effective;
    prof.s_end = L;

    prof.s.resize(M);
    prof.kappa.resize(M);
    prof.v.resize(M);
    prof.a.resize(M);

    for (int i = 0; i < M; ++i)
    {
        const double si =
            s0 + static_cast<double>(i) * ds_effective;

        prof.s[i] = si;
        prof.kappa[i] = spline.getCurvature(si);
        prof.v[i] = curvatureSpeedLimit(
            prof.kappa[i],
            v_min,
            v_max,
            long_lim.ay_max
        );
    }

    /*
        Open BF starts from the measured current speed, even if that speed is
        temporarily above the local curvature speed limit. The backward pass
        may reduce earlier profile values, but the initial state is not
        invented by the planner.
    */
    prof.v[0] =
        std::clamp(v0, v_min, v_max);

    for (int i = 0; i + 1 < M; ++i)
    {
        const double ax =
            driveAccelerationLimit(
                prof.v[i],
                prof.kappa[i],
                long_lim
            );

        const double v_reachable =
            safeSqrt(
                prof.v[i] * prof.v[i] +
                2.0 * ax * prof.ds
            );

        prof.v[i + 1] =
            std::min(prof.v[i + 1], v_reachable);

        prof.v[i + 1] =
            std::clamp(prof.v[i + 1], v_min, v_max);
    }

    runBackwardPass(
        prof,
        v_min,
        v_max,
        long_lim
    );

    computeAcceleration(prof);

    return prof;
}


static inline BfProfile runForwardBackwardClosedSpline(
    const TrackSpline2D& spline,
    double s0,
    double v0,
    double v_min,
    double v_max,
    const PlannerLongitudinalLimits& long_lim,
    double ds,
    double forward_distance_m)
{
    BfProfile prof;

    if (!spline.valid() || !spline.isClosed())
    {
        return prof;
    }

    if (forward_distance_m <= ds || ds <= 1.0e-6)
    {
        return prof;
    }

    /*
        Dla zamkniętego toru nie liczę pełnego okrążenia.
        Robię lokalny BF od aktualnej projekcji s0 tylko kilka/kilkadziesiąt
        metrów do przodu, np. 40 m.

        Sam spline może dostawać s większe niż totalLength(), bo closed backend
        zawija s wewnętrznie. Natomiast profil BF jest lokalny i monotoniczny:
            [s0, s0 + forward_distance_m]
    */
    const int M =
        std::max(3, static_cast<int>(std::ceil(forward_distance_m / ds)) + 1);

    const double ds_eff =
        forward_distance_m / static_cast<double>(M - 1);

    prof.s0 = s0;
    prof.ds = ds_eff;
    prof.s_end = s0 + forward_distance_m;

    prof.s.resize(M);
    prof.kappa.resize(M);
    prof.v.resize(M);
    prof.a.resize(M);

    for (int i = 0; i < M; ++i)
    {
        const double si =
            s0 + static_cast<double>(i) * ds_eff;

        prof.s[i] = si;
        prof.kappa[i] = spline.getCurvature(si);
        prof.v[i] = curvatureSpeedLimit(
            prof.kappa[i],
            v_min,
            v_max,
            long_lim.ay_max
        );
    }

    /*
        Start profilu jest w aktualnym położeniu bolidu i musi zachować
        zmierzoną aktualną prędkość, nawet jeśli jest ona chwilowo wyższa od
        lokalnego limitu krzywizny. BF może zaplanować hamowanie w kolejnych
        próbkach, ale nie może przepisać bieżącego stanu na niższą prędkość.

        Nie spinam końca z początkiem, bo to nie jest pełne okrążenie, tylko
        lokalny horyzont prędkości. Nie wymuszam też final_to_short_bf_speed.
    */
    prof.v[0] =
        std::clamp(v0, v_min, v_max);

    for (int i = 0; i + 1 < M; ++i)
    {
        const double ax =
            driveAccelerationLimit(
                prof.v[i],
                prof.kappa[i],
                long_lim
            );

        const double v_reachable =
            safeSqrt(prof.v[i] * prof.v[i] + 2.0 * ax * ds_eff);

        prof.v[i + 1] =
            std::min(prof.v[i + 1], v_reachable);

        prof.v[i + 1] =
            std::clamp(prof.v[i + 1], v_min, v_max);
    }

    runBackwardPass(
        prof,
        v_min,
        v_max,
        long_lim
    );

    computeAcceleration(prof);

    return prof;
}

static inline void profileAtS(const BfProfile& prof,
                              double s_query,
                              double fallback_v,
                              double& kappa,
                              double& v,
                              double& a)
{
    const int M =
        static_cast<int>(prof.s.size());

    kappa = 0.0;
    v = fallback_v;
    a = 0.0;

    if (M < 2)
    {
        return;
    }

    if (s_query >= prof.s.back())
    {
        kappa = prof.kappa.back();
        v = prof.v.back();
        a = prof.a.back();
        return;
    }

    const double u =
        (s_query - prof.s0) / prof.ds;

    int i =
        static_cast<int>(std::floor(u));

    i =
        std::clamp(i, 0, M - 2);

    const double alpha =
        std::clamp(u - static_cast<double>(i), 0.0, 1.0);

    const auto lerp = [alpha](double p0, double p1)
    {
        return (1.0 - alpha) * p0 + alpha * p1;
    };

    kappa =
        lerp(prof.kappa[i], prof.kappa[i + 1]);

    v =
        lerp(prof.v[i], prof.v[i + 1]);

    a =
        lerp(prof.a[i], prof.a[i + 1]);
}

// =============================================================================
//                            BEFORE-START HANDLER
// =============================================================================

static inline void fillBeforeStartResult(
    const Eigen::VectorXd& X_raw,
    const Eigen::VectorXd& Y_raw,
    const TrackSpline2D& spline,
    double projected_s,
    const State& bolide_state,
    int N,
    const ParamBank& P,
    LocalPlannerResult& out)
{
    out =
        LocalPlannerResult{};

    if (N <= 0 ||
        X_raw.size() < 2 ||
        X_raw.size() != Y_raw.size() ||
        !spline.valid() ||
        spline.isClosed() ||
        !std::isfinite(projected_s) ||
        !std::isfinite(bolide_state.x) ||
        !std::isfinite(bolide_state.y) ||
        !std::isfinite(bolide_state.yaw))
    {
        return;
    }

    const double L =
        spline.totalLength();

    const double s_epsilon =
        std::max(1.0e-5, 1.0e-6 * L);

    if (!std::isfinite(L) ||
        L <= 1.0e-9 ||
        projected_s < -s_epsilon ||
        projected_s > s_epsilon)
    {
        return;
    }

    const double p0_x =
        X_raw(0);

    const double p0_y =
        Y_raw(0);

    int second_index =
        -1;

    constexpr double kDuplicateEpsilonSquared =
        1.0e-12;

    for (int i = 1; i < static_cast<int>(X_raw.size()); ++i)
    {
        const double dx =
            X_raw(i) - p0_x;

        const double dy =
            Y_raw(i) - p0_y;

        if (dx * dx + dy * dy > kDuplicateEpsilonSquared)
        {
            second_index =
                i;

            break;
        }
    }

    if (second_index < 0)
    {
        return;
    }

    const double segment_x =
        X_raw(second_index) - p0_x;

    const double segment_y =
        Y_raw(second_index) - p0_y;

    const double segment_length =
        std::hypot(segment_x, segment_y);

    if (!std::isfinite(segment_length) ||
        segment_length <= 1.0e-9)
    {
        return;
    }

    const double ux =
        segment_x / segment_length;

    const double uy =
        segment_y / segment_length;

    const double p0_to_bolide_x =
        bolide_state.x - p0_x;

    const double p0_to_bolide_y =
        bolide_state.y - p0_y;

    const double longitudinal_from_start =
        p0_to_bolide_x * ux +
        p0_to_bolide_y * uy;

    const double heading_alignment =
        std::cos(bolide_state.yaw) * ux +
        std::sin(bolide_state.yaw) * uy;

    /*
        Recheck the two geometric BEFORE_START conditions used by the spline
        classifier. There is intentionally no distance threshold:
            longitudinal_from_start < 0.0
        is sufficient.
    */
    if (!(longitudinal_from_start < 0.0) ||
        !(heading_alignment > 0.0))
    {
        return;
    }

    const double projection_x =
        p0_x + longitudinal_from_start * ux;

    const double projection_y =
        p0_y + longitudinal_from_start * uy;

    const double yaw_ref =
        std::atan2(uy, ux);

    const double loop_hz =
        P.get("model.frequency.steer_cmd_loop_hz");

    if (!std::isfinite(loop_hz) ||
        loop_hz <= 0.0)
    {
        return;
    }

    const double controller_dt =
        1.0 / loop_hz;

    const double ds_step =
        std::max(
            0.05,
            kTwoPointPathTargetSpeedMps * controller_dt
        );

    out.X_ref =
        Eigen::VectorXd::Zero(N);

    out.Y_ref =
        Eigen::VectorXd::Zero(N);

    out.curvature_ref.assign(
        static_cast<std::size_t>(N),
        0.0
    );

    out.acceleration_ref.assign(
        static_cast<std::size_t>(N),
        0.0
    );

    out.velocity_ref.assign(
        static_cast<std::size_t>(N),
        kTwoPointPathTargetSpeedMps
    );

    /*
        Same artificial straight behavior as the dedicated two-point mode.
        The whole controller horizon remains on the P0 -> P1 line. BF is
        bypassed by returning this result immediately.
    */
    for (int i = 0; i < N; ++i)
    {
        const double distance_ahead =
            ds_step * static_cast<double>(i);

        out.X_ref(i) =
            projection_x + distance_ahead * ux;

        out.Y_ref(i) =
            projection_y + distance_ahead * uy;
    }

    out.x_ref_point =
        projection_x;

    out.y_ref_point =
        projection_y;

    out.s =
        longitudinal_from_start;

    const double projection_to_bolide_x =
        bolide_state.x - projection_x;

    const double projection_to_bolide_y =
        bolide_state.y - projection_y;

    out.ey =
        -uy * projection_to_bolide_x +
         ux * projection_to_bolide_y;

    out.epsi =
        wrapPi(bolide_state.yaw - yaw_ref);

    out.before_bolide =
        true;

    out.after_bolide =
        false;

    out.valid =
        out.X_ref.allFinite() &&
        out.Y_ref.allFinite() &&
        std::isfinite(out.x_ref_point) &&
        std::isfinite(out.y_ref_point) &&
        std::isfinite(out.s) &&
        std::isfinite(out.ey) &&
        std::isfinite(out.epsi);
}

// =============================================================================
//                              AFTER-END HANDLER
// =============================================================================

static inline void fillAfterEndResult(
    const TrackSpline2D& spline,
    double projected_s,
    const State& bolide_state,
    int N,
    const ParamBank& P,
    LocalPlannerResult& out)
{
    out =
        LocalPlannerResult{};

    if (N <= 0 ||
        !spline.valid() ||
        spline.isClosed() ||
        !std::isfinite(projected_s) ||
        !std::isfinite(bolide_state.x) ||
        !std::isfinite(bolide_state.y) ||
        !std::isfinite(bolide_state.yaw))
    {
        return;
    }

    const double L =
        spline.totalLength();

    const double s_epsilon =
        std::max(1.0e-5, 1.0e-6 * L);

    /*
        This handler may only be used after the global projection landed
        at the end of an open spline, within the same endpoint tolerance
        used by the spline-side AFTER_END classifier.
    */
    if (!std::isfinite(L) ||
        L <= 1.0e-9 ||
        projected_s < L - s_epsilon ||
        projected_s > L + s_epsilon)
    {
        return;
    }

    const Vec2 p_end =
        spline.eval(L);

    const double yaw_ref =
        spline.getYaw(L);

    if (!std::isfinite(static_cast<double>(p_end.x)) ||
        !std::isfinite(static_cast<double>(p_end.y)) ||
        !std::isfinite(yaw_ref))
    {
        return;
    }

    const double tx =
        std::cos(yaw_ref);

    const double ty =
        std::sin(yaw_ref);

    const double end_to_bolide_x =
        bolide_state.x -
        static_cast<double>(p_end.x);

    const double end_to_bolide_y =
        bolide_state.y -
        static_cast<double>(p_end.y);

    /*
        Orthogonal projection of the bolide onto the infinite straight line
        defined by the endpoint point and endpoint tangent.

        The heading-alignment condition deciding whether this is AFTER_END
        is checked by TrackSpline2D::projectPointGlobally(). This function
        only constructs the artificial straight reference.
    */
    const double distance_along_end_tangent =
        end_to_bolide_x * tx +
        end_to_bolide_y * ty;

    const double projection_x =
        static_cast<double>(p_end.x) +
        distance_along_end_tangent * tx;

    const double projection_y =
        static_cast<double>(p_end.y) +
        distance_along_end_tangent * ty;

    const double extended_projected_s =
        L + distance_along_end_tangent;

    if (!std::isfinite(projection_x) ||
        !std::isfinite(projection_y) ||
        !std::isfinite(extended_projected_s))
    {
        return;
    }

    const double v_final =
        std::min(
            std::max(0.0, bolide_state.vx),
            P.get("general.final_to_short_bf_speed")
        );

    const double loop_hz =
        std::max(
            1.0,
            P.get("model.frequency.steer_cmd_loop_hz")
        );

    const double controller_dt =
        1.0 / loop_hz;

    const double ds_step =
        std::max(
            0.05,
            std::abs(v_final) * controller_dt
        );

    out.X_ref =
        Eigen::VectorXd::Zero(N);

    out.Y_ref =
        Eigen::VectorXd::Zero(N);

    out.curvature_ref.assign(
        static_cast<std::size_t>(N),
        0.0
    );

    out.acceleration_ref.assign(
        static_cast<std::size_t>(N),
        0.0
    );

    out.velocity_ref.assign(
        static_cast<std::size_t>(N),
        v_final
    );

    /*
        Do not call spline.getX/getY for s > L. OpenAkimaSpline2D clamps
        such values to L. Build the horizon explicitly on the endpoint
        tangent continuation instead.
    */
    for (int i = 0; i < N; ++i)
    {
        const double distance_ahead =
            ds_step * static_cast<double>(i);

        out.X_ref(i) =
            projection_x +
            distance_ahead * tx;

        out.Y_ref(i) =
            projection_y +
            distance_ahead * ty;
    }

    out.x_ref_point =
        projection_x;

    out.y_ref_point =
        projection_y;

    out.s =
        extended_projected_s;

    const double projection_to_bolide_x =
        bolide_state.x - projection_x;

    const double projection_to_bolide_y =
        bolide_state.y - projection_y;

    out.ey =
        -ty * projection_to_bolide_x +
         tx * projection_to_bolide_y;

    out.epsi =
        wrapPi(bolide_state.yaw - yaw_ref);

    out.before_bolide =
        false;

    out.after_bolide =
        true;

    out.valid =
        out.X_ref.allFinite() &&
        out.Y_ref.allFinite() &&
        std::isfinite(out.x_ref_point) &&
        std::isfinite(out.y_ref_point) &&
        std::isfinite(out.ey) &&
        std::isfinite(out.epsi) &&
        std::isfinite(out.s);
}

// =============================================================================
//                         TWO-POINT STRAIGHT MODE
// =============================================================================

static inline LocalPlannerResult buildTwoPointStraightReference(
    const Eigen::VectorXd& X_raw,
    const Eigen::VectorXd& Y_raw,
    const State& bolide_state,
    int N,
    const ParamBank& P)
{
    LocalPlannerResult out;

    if (N <= 0 ||
        X_raw.size() != 2 ||
        Y_raw.size() != 2)
    {
        return out;
    }

    const double ax =
        X_raw(0);

    const double ay =
        Y_raw(0);

    const double bx =
        X_raw(1);

    const double by =
        Y_raw(1);

    if (!std::isfinite(ax) ||
        !std::isfinite(ay) ||
        !std::isfinite(bx) ||
        !std::isfinite(by) ||
        !std::isfinite(bolide_state.x) ||
        !std::isfinite(bolide_state.y) ||
        !std::isfinite(bolide_state.yaw))
    {
        return out;
    }

    const double abx =
        bx - ax;

    const double aby =
        by - ay;

    const double line_length =
        std::hypot(abx, aby);

    if (!std::isfinite(line_length) ||
        line_length <= 1.0e-9)
    {
        return out;
    }

    const double ux =
        abx / line_length;

    const double uy =
        aby / line_length;

    const double apx =
        bolide_state.x - ax;

    const double apy =
        bolide_state.y - ay;

    /*
        Projection onto the infinite line through the two path-planning
        points. The reference therefore remains a straight line also before
        the first point and after the second point.
    */
    const double s_projection =
        apx * ux + apy * uy;

    const double projection_x =
        ax + s_projection * ux;

    const double projection_y =
        ay + s_projection * uy;

    const double yaw_ref =
        std::atan2(uy, ux);

    out.X_ref =
        Eigen::VectorXd::Zero(N);

    out.Y_ref =
        Eigen::VectorXd::Zero(N);

    out.curvature_ref.assign(
        static_cast<std::size_t>(N),
        0.0
    );

    out.acceleration_ref.assign(
        static_cast<std::size_t>(N),
        0.0
    );

    out.velocity_ref.assign(
        static_cast<std::size_t>(N),
        kTwoPointPathTargetSpeedMps
    );

    const double loop_hz =
        P.get("model.frequency.steer_cmd_loop_hz");

    if (!std::isfinite(loop_hz) ||
        loop_hz <= 0.0)
    {
        return LocalPlannerResult{};
    }

    const double controller_dt =
        1.0 / loop_hz;

    const double ds_step =
        std::max(
            0.05,
            kTwoPointPathTargetSpeedMps * controller_dt
        );

    for (int i = 0; i < N; ++i)
    {
        const double s_ahead =
            ds_step * static_cast<double>(i);

        out.X_ref(i) =
            projection_x + s_ahead * ux;

        out.Y_ref(i) =
            projection_y + s_ahead * uy;
    }

    out.x_ref_point =
        projection_x;

    out.y_ref_point =
        projection_y;

    out.s =
        s_projection;

    const double dx =
        bolide_state.x - projection_x;

    const double dy =
        bolide_state.y - projection_y;

    out.ey =
        -std::sin(yaw_ref) * dx +
         std::cos(yaw_ref) * dy;

    out.epsi =
        wrapPi(bolide_state.yaw - yaw_ref);

    out.after_bolide =
        false;

    out.valid =
        out.X_ref.allFinite() &&
        out.Y_ref.allFinite() &&
        std::isfinite(out.x_ref_point) &&
        std::isfinite(out.y_ref_point) &&
        std::isfinite(out.s) &&
        std::isfinite(out.ey) &&
        std::isfinite(out.epsi);

    return out;
}

// =============================================================================
//                              MAIN ENTRY
// =============================================================================

static inline LocalPlannerResult buildLocalPlannerReferenceFromSpline(
    const Eigen::VectorXd& X_raw,
    const Eigen::VectorXd& Y_raw,
    const State& bolide_state,
    int N,
    const ParamBank& P,
    bool closed_track,
    const TrackSpline2D& spline,
    bool allow_before_start = true)
{
    LocalPlannerResult out;

    if (N <= 0 || X_raw.size() != Y_raw.size())
    {
        return out;
    }

    const int R =
        static_cast<int>(X_raw.size());

    if (R < 2 || (closed_track && R < 3))
    {
        return out;
    }

    out.X_ref = Eigen::VectorXd::Zero(N);
    out.Y_ref = Eigen::VectorXd::Zero(N);
    out.curvature_ref.assign(N, 0.0);
    out.acceleration_ref.assign(N, 0.0);
    out.velocity_ref.assign(N, 0.0);

    const Vec2 q(
        static_cast<float>(bolide_state.x),
        static_cast<float>(bolide_state.y)
    );

    if (!spline.valid() ||
        spline.isClosed() != closed_track)
    {
        return out;
    }

    /*
        Exactly two path-planning points define a dedicated infinite straight
        reference. The BF profile is intentionally bypassed:
            curvature_ref    = 0,
            acceleration_ref = 0,
            velocity_ref     = 3 m/s.

        The wrapper selects the speed PID for this mode, while lateral control
        receives ey, epsi and a zero-curvature horizon computed from the same
        line.
    */
    if (!closed_track && R == 2)
    {
        return buildTwoPointStraightReference(
            X_raw,
            Y_raw,
            bolide_state,
            N,
            P
        );
    }

    /*
        Stateless global projection.

        Every invocation searches the complete spline geometry. No previous s,
        local window or projection state is carried between controller cycles.

        For an open spline the global classifier may return:

        BEFORE_START when:
            1. the global position projection lands at s ~= 0,
            2. bolide heading agrees with the first raw segment P0 -> P1,
            3. projection of (bolide - P0) onto P0 -> P1 is negative.

        AFTER_END when:
            1. the global position projection lands at s ~= L,
            2. bolide heading agrees with the endpoint spline tangent.

        Both endpoint modes bypass BF. BEFORE_START also behaves
        longitudinally exactly like the two-point mode:
            curvature_ref = 0,
            acceleration_ref = 0,
            velocity_ref = 3 m/s.
    */
    const SplineProjectionResult projection =
        spline.projectPointGlobally(
            q,
            bolide_state.yaw
        );

    if (!projection.valid)
    {
        return out;
    }

    const double s0 =
        projection.s_proj;

    const double L =
        spline.totalLength();

    if (!std::isfinite(s0) ||
        !std::isfinite(L) ||
        L <= 1.0e-9)
    {
        return out;
    }

    /*
        The wrapper owns the one-way BEFORE_START latch.

        When allow_before_start is false, a global BEFORE_START classification
        is treated as the normal beginning of the real spline. The artificial
        straight handler is skipped and the code continues below with s0 == 0,
        BF and the real spline curvature/reference.
    */
    if (!closed_track &&
        allow_before_start &&
        projection.region == SplineProjectionRegion::BEFORE_START)
    {
        fillBeforeStartResult(
            X_raw,
            Y_raw,
            spline,
            s0,
            bolide_state,
            N,
            P,
            out
        );

        return out;
    }

    if (!closed_track &&
        projection.region == SplineProjectionRegion::AFTER_END)
    {
        fillAfterEndResult(
            spline,
            s0,
            bolide_state,
            N,
            P,
            out
        );

        return out;
    }

    const double ds =
        P.get("general.bf_spatial_step");

    const double v_min =
        P.get("general.v_min");

    const double v_max =
        P.get("general.v_max");

    const PlannerLongitudinalLimits long_lim =
        loadPlannerLongitudinalLimits(P);

    const double minimal_forward_time =
        P.get("general.minimal_planer_forward_time");

    const double final_to_short_bf_speed =
        P.get("general.final_to_short_bf_speed");

    const double controller_dt =
        1.0 / P.get("model.frequency.steer_cmd_loop_hz");

    const double v0 =
        std::clamp(bolide_state.vx, v_min, v_max);

    BfProfile prof;

    if (closed_track)
    {
        prof =
            runForwardBackwardClosedSpline(
                spline,
                s0,
                v0,
                v_min,
                v_max,
                long_lim,
                ds,
                kClosedTrackBfForwardDistanceM
            );
    }
    else
    {
        prof =
            runForwardBackwardOpenSpline(
                spline,
                s0,
                v0,
                v_min,
                v_max,
                long_lim,
                ds
            );

        if (prof.v.size() >= 2 &&
            profileTime(prof) < minimal_forward_time)
        {
            const double forced_terminal_speed =
                std::min(
                    prof.v.back(),
                    std::clamp(
                        final_to_short_bf_speed,
                        v_min,
                        v_max
                    )
                );

            if (forced_terminal_speed < prof.v.back())
            {
                prof.v.back() = forced_terminal_speed;

                /*
                    Curvature sampling and the forward pass are already done.
                    Only propagate the new terminal limit backward and refresh
                    acceleration; do not recompute the full BF profile.
                */
                runBackwardPass(
                    prof,
                    v_min,
                    v_max,
                    long_lim
                );

                computeAcceleration(prof);
            }
        }
    }

    if (prof.v.size() < 2)
    {
        return out;
    }

    const double terminal_velocity =
        prof.v.back();

    const Vec2 p_proj =
        spline.eval(s0);

    const double yaw_ref =
        spline.getYaw(s0);

    out.x_ref_point =
        static_cast<double>(p_proj.x);

    out.y_ref_point =
        static_cast<double>(p_proj.y);

    out.s =
        s0;

    const double dx =
        static_cast<double>(bolide_state.x) -
        static_cast<double>(p_proj.x);

    const double dy =
        static_cast<double>(bolide_state.y) -
        static_cast<double>(p_proj.y);

    out.ey =
        -std::sin(yaw_ref) * dx +
         std::cos(yaw_ref) * dy;

    out.epsi =
        wrapPi(bolide_state.yaw - yaw_ref);

    const Vec2 p_end =
        spline.eval(L);

    const double yaw_end =
        spline.getYaw(L);

    double s_horizon =
        s0;

    for (int i = 0; i < N; ++i)
    {
        double kappa_i = 0.0;
        double v_i = closed_track ? v0 : terminal_velocity;
        double a_i = 0.0;

        if (closed_track)
        {
            profileAtS(
                prof,
                s_horizon,
                prof.v.back(),
                kappa_i,
                v_i,
                a_i
            );

            out.X_ref(i) =
                spline.getX(s_horizon);

            out.Y_ref(i) =
                spline.getY(s_horizon);

            out.curvature_ref[static_cast<std::size_t>(i)] =
                kappa_i;

            out.velocity_ref[static_cast<std::size_t>(i)] =
                v_i;

            out.acceleration_ref[static_cast<std::size_t>(i)] =
                a_i;
        }
        else
        {
            profileAtS(
                prof,
                s_horizon,
                terminal_velocity,
                kappa_i,
                v_i,
                a_i
            );

            if (s_horizon <= L)
            {
                out.X_ref(i) =
                    spline.getX(s_horizon);

                out.Y_ref(i) =
                    spline.getY(s_horizon);

                out.curvature_ref[static_cast<std::size_t>(i)] =
                    kappa_i;

                out.velocity_ref[static_cast<std::size_t>(i)] =
                    v_i;

                out.acceleration_ref[static_cast<std::size_t>(i)] =
                    a_i;
            }
            else
            {
                const double extra_s =
                    s_horizon - L;

                out.X_ref(i) =
                    static_cast<double>(p_end.x) +
                    extra_s * std::cos(yaw_end);

                out.Y_ref(i) =
                    static_cast<double>(p_end.y) +
                    extra_s * std::sin(yaw_end);

                out.curvature_ref[static_cast<std::size_t>(i)] =
                    0.0;

                out.velocity_ref[static_cast<std::size_t>(i)] =
                    terminal_velocity;

                out.acceleration_ref[static_cast<std::size_t>(i)] =
                    0.0;
            }
        }

        s_horizon +=
            out.velocity_ref[static_cast<std::size_t>(i)] * controller_dt +
            0.5 *
            out.acceleration_ref[static_cast<std::size_t>(i)] *
            controller_dt *
            controller_dt;
    }

    out.valid = true;
    out.before_bolide = false;
    out.after_bolide = false;

    return out;
}


static inline LocalPlannerResult buildLocalPlannerReference(
    const Eigen::VectorXd& X_raw,
    const Eigen::VectorXd& Y_raw,
    const State& bolide_state,
    int N,
    const ParamBank& P,
    bool closed_track,
    bool allow_before_start = true)
{
    PreparedLocalPath prepared =
        prepareLocalPath(
            X_raw,
            Y_raw,
            closed_track
        );

    if (!prepared.valid)
    {
        return LocalPlannerResult{};
    }

    return buildLocalPlannerReferenceFromSpline(
        X_raw,
        Y_raw,
        bolide_state,
        N,
        P,
        closed_track,
        prepared.spline,
        allow_before_start
    );
}


} // namespace dv_control
