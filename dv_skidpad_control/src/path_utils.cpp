#include "path_utils.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace skidpad_control
{

namespace
{


double dist2ToSplineAtS(const PathSpline& spline,
                        double s_query_m,
                        double px,
                        double py)
{
    const SplinePathPoint p =
        spline.evaluate(s_query_m);

    if (!p.valid)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double dx =
        px - p.x_m;

    const double dy =
        py - p.y_m;

    return dx * dx + dy * dy;
}

} // anonymous namespace

// =============================================================================
//                              ARC LENGTH
// =============================================================================

Eigen::VectorXd buildArcLength(const Eigen::VectorXd& X,
                               const Eigen::VectorXd& Y)
{
    const int n = static_cast<int>(X.size());

    Eigen::VectorXd S(n);
    S.setZero();

    for (int i = 1; i < n; ++i)
    {
        S(i) = S(i - 1) + norm2(X(i) - X(i - 1),
                                Y(i) - Y(i - 1));
    }

    return S;
}


// =============================================================================
//                              CURVATURE
// =============================================================================

double estimateCurvatureAtSegment(const Eigen::VectorXd& X,
                                  const Eigen::VectorXd& Y,
                                  int segment_index)
{
    const int n = static_cast<int>(X.size());

    if (n < 3)
    {
        return 0.0;
    }

    const int i_curr =
        std::max(0, std::min(segment_index, n - 2));

    const int i_prev =
        std::max(0, i_curr - 1);

    const int i_next =
        std::min(n - 2, i_curr + 1);

    const double yaw_prev =
        std::atan2(Y(i_prev + 1) - Y(i_prev),
                   X(i_prev + 1) - X(i_prev));

    const double yaw_next =
        std::atan2(Y(i_next + 1) - Y(i_next),
                   X(i_next + 1) - X(i_next));

    const double s_prev =
        norm2(X(i_prev + 1) - X(i_prev),
              Y(i_prev + 1) - Y(i_prev));

    const double s_next =
        norm2(X(i_next + 1) - X(i_next),
              Y(i_next + 1) - Y(i_next));

    const double ds =
        0.5 * (s_prev + s_next);

    if (ds < 1.0e-6)
    {
        return 0.0;
    }

    return wrapAngle(yaw_next - yaw_prev) / ds;
}


// =============================================================================
//                              RAW MONOTONIC PATH PROJECTION
// =============================================================================

PathProjection projectToPathMonotonic(const Eigen::VectorXd& X,
                                      const Eigen::VectorXd& Y,
                                      const Eigen::VectorXd& S,
                                      double px,
                                      double py,
                                      double yaw_rad,
                                      int last_segment_index,
                                      double last_s_m,
                                      bool has_previous_projection)
{
    (void)yaw_rad;

    PathProjection best;

    const int n = static_cast<int>(X.size());

    if (n < 2 || Y.size() != X.size() || S.size() != X.size())
    {
        return best;
    }

    int i_begin = 0;
    int i_end = n - 2;

    double s_min_allowed = S(0);
    double s_max_allowed = S(n - 1);

    if (has_previous_projection)
    {
        // Strictly monotonic RAW projection: no backward window.
        constexpr double kForwardSearchDistanceM = 4.00;

        last_segment_index =
            std::max(0, std::min(last_segment_index, n - 2));

        last_s_m =
            clampLocal(last_s_m, S(0), S(n - 1));

        s_min_allowed =
            last_s_m;

        s_max_allowed =
            std::min(S(n - 1), last_s_m + kForwardSearchDistanceM);

        i_begin = last_segment_index;

        while (i_begin > 0 && S(i_begin) > s_min_allowed)
        {
            --i_begin;
        }

        i_end = last_segment_index;

        while (i_end < n - 2 && S(i_end) <= s_max_allowed)
        {
            ++i_end;
        }

        i_begin = std::max(0, std::min(i_begin, n - 2));
        i_end = std::max(i_begin, std::min(i_end, n - 2));
    }

    double best_dist2 =
        std::numeric_limits<double>::infinity();

    for (int i = i_begin; i <= i_end; ++i)
    {
        const double ax = X(i);
        const double ay = Y(i);

        const double bx = X(i + 1);
        const double by = Y(i + 1);

        const double sx = bx - ax;
        const double sy = by - ay;

        const double len2 = sx * sx + sy * sy;

        if (len2 < 1.0e-12)
        {
            continue;
        }

        const double seg_len = std::sqrt(len2);

        const double tx = sx / seg_len;
        const double ty = sy / seg_len;

        const double wx = px - ax;
        const double wy = py - ay;

        const double t =
            clampLocal((wx * sx + wy * sy) / len2, 0.0, 1.0);

        const double qx = ax + t * sx;
        const double qy = ay + t * sy;

        const double ex = px - qx;
        const double ey = py - qy;

        const double dist2 = ex * ex + ey * ey;

        const double s_candidate =
            S(i) + t * seg_len;

        if (has_previous_projection)
        {
            if (s_candidate + 1.0e-9 < s_min_allowed)
            {
                continue;
            }

            if (s_candidate > s_max_allowed + 1.0e-9)
            {
                continue;
            }
        }

        if (dist2 < best_dist2)
        {
            best_dist2 = dist2;

            const double yaw_ref =
                std::atan2(ty, tx);

            const double nx_left = -ty;
            const double ny_left =  tx;

            best.valid = true;

            best.segment_index = i;
            best.raw_segment_index = i;

            best.s_m = s_candidate;
            best.raw_s_m = s_candidate;

            best.x_ref_m = qx;
            best.y_ref_m = qy;
            best.yaw_ref_rad = yaw_ref;

            best.ey_m = ex * nx_left + ey * ny_left;
            best.epsi_rad = wrapAngle(yaw_rad - yaw_ref);

            best.kappa_ref = estimateCurvatureAtSegment(X, Y, i);

            best.refined_by_spline = false;
        }
    }

    return best;
}


// =============================================================================
//                              LOCAL SPLINE REFINEMENT
// =============================================================================

PathProjection refineProjectionOnSplineLocal(const PathSpline& spline,
                                             const PathProjection& coarse,
                                             double px,
                                             double py,
                                             double yaw_rad,
                                             double min_allowed_s_m,
                                             double half_window_m)
{
    PathProjection out;

    if (!spline.isValid() || !coarse.valid)
    {
        return out;
    }

    const double half_window =
        std::max(half_window_m, 1.0e-4);

    const double s_min_allowed =
        clampLocal(min_allowed_s_m, spline.sMin(), spline.sMax());

    double s_lo =
        std::max(spline.sMin(), coarse.s_m - half_window);

    double s_hi =
        std::min(spline.sMax(), coarse.s_m + half_window);

    s_lo =
        std::max(s_lo, s_min_allowed);

    if (s_hi < s_lo)
    {
        s_hi = s_lo;
    }

    double best_s =
        clampLocal(coarse.s_m, s_lo, s_hi);

    double best_dist2 =
        dist2ToSplineAtS(spline, best_s, px, py);

    // Coarse local scan. This is intentionally distance-only.
    constexpr int kCoarseSamples = 41;

    for (int i = 0; i < kCoarseSamples; ++i)
    {
        const double ratio =
            (kCoarseSamples <= 1)
                ? 0.0
                : static_cast<double>(i) / static_cast<double>(kCoarseSamples - 1);

        const double s =
            s_lo + ratio * (s_hi - s_lo);

        const double d2 =
            dist2ToSplineAtS(spline, s, px, py);

        if (d2 < best_dist2)
        {
            best_dist2 = d2;
            best_s = s;
        }
    }

    // Small fine scan around the best coarse sample.
    const double coarse_step =
        (s_hi - s_lo) / static_cast<double>(std::max(1, kCoarseSamples - 1));

    const double fine_lo =
        std::max(s_lo, best_s - coarse_step);

    const double fine_hi =
        std::min(s_hi, best_s + coarse_step);

    constexpr int kFineSamples = 21;

    for (int i = 0; i < kFineSamples; ++i)
    {
        const double ratio =
            (kFineSamples <= 1)
                ? 0.0
                : static_cast<double>(i) / static_cast<double>(kFineSamples - 1);

        const double s =
            fine_lo + ratio * (fine_hi - fine_lo);

        const double d2 =
            dist2ToSplineAtS(spline, s, px, py);

        if (d2 < best_dist2)
        {
            best_dist2 = d2;
            best_s = s;
        }
    }

    const SplinePathPoint p =
        spline.evaluate(best_s);

    if (!p.valid)
    {
        return out;
    }

    const double ex =
        px - p.x_m;

    const double ey =
        py - p.y_m;

    const double nx_left =
        -std::sin(p.yaw_rad);

    const double ny_left =
         std::cos(p.yaw_rad);

    out.valid = true;

    out.segment_index =
        coarse.segment_index;

    out.raw_segment_index =
        coarse.raw_segment_index;

    out.raw_s_m =
        coarse.raw_s_m;

    out.s_m =
        best_s;

    out.x_ref_m =
        p.x_m;

    out.y_ref_m =
        p.y_m;

    out.yaw_ref_rad =
        p.yaw_rad;

    out.ey_m =
        ex * nx_left + ey * ny_left;

    out.epsi_rad =
        wrapAngle(yaw_rad - p.yaw_rad);

    out.kappa_ref =
        p.kappa;

    out.refined_by_spline = true;

    return out;
}


// =============================================================================
//                              CURVATURE AT S
// =============================================================================

double curvatureAtS(const Eigen::VectorXd& X,
                    const Eigen::VectorXd& Y,
                    const Eigen::VectorXd& S,
                    double s_query_m)
{
    const int n = static_cast<int>(X.size());

    if (n < 3 || Y.size() != X.size() || S.size() != X.size())
    {
        return 0.0;
    }

    if (s_query_m <= S(0))
    {
        return estimateCurvatureAtSegment(X, Y, 0);
    }

    if (s_query_m >= S(n - 1))
    {
        return estimateCurvatureAtSegment(X, Y, n - 2);
    }

    int segment_index = 0;

    for (int i = 0; i < n - 1; ++i)
    {
        if (S(i) <= s_query_m && s_query_m <= S(i + 1))
        {
            segment_index = i;
            break;
        }
    }

    return estimateCurvatureAtSegment(X, Y, segment_index);
}


// =============================================================================
//                              HORIZON
// =============================================================================

int readActiveHorizonN(const ParamBank& P)
{
    return static_cast<int>(
        std::llround(P.get("model.ltv_mpc_unbounded.N"))
    );
}

std::vector<double> buildCurvatureHorizon(const ParamBank& P,
                                          const Eigen::VectorXd& X,
                                          const Eigen::VectorXd& Y,
                                          const Eigen::VectorXd& S,
                                          double s0_m,
                                          double vx_mps)
{
    const int N =
        std::max(1, readActiveHorizonN(P));

    std::vector<double> kappa_horizon(static_cast<std::size_t>(N), 0.0);

    if (X.size() < 3 || Y.size() != X.size() || S.size() != X.size())
    {
        return kappa_horizon;
    }

    const double dt =
        1.0 / std::max(P.get("model.frequency.steer_cmd_loop_hz"), 1.0e-6);

    const double ds =
        std::max(std::abs(vx_mps), 0.5) * dt;

    const double s_min =
        S(0);

    const double s_max =
        S(S.size() - 1);

    for (int k = 0; k < N; ++k)
    {
        const double s_raw =
            s0_m + static_cast<double>(k) * ds;

        const double s_query =
            clampLocal(s_raw, s_min, s_max);

        kappa_horizon[static_cast<std::size_t>(k)] =
            curvatureAtS(X, Y, S, s_query);
    }

    return kappa_horizon;
}

PathPoint pointAtS(const Eigen::VectorXd& X,
                   const Eigen::VectorXd& Y,
                   const Eigen::VectorXd& S,
                   double s_query_m)
{
    PathPoint out;

    const int n = static_cast<int>(X.size());

    if (n < 2 || Y.size() != X.size() || S.size() != X.size())
    {
        return out;
    }

    const double s_min = S(0);
    const double s_max = S(n - 1);

    s_query_m = clampLocal(s_query_m, s_min, s_max);

    int i = 0;

    for (int k = 0; k < n - 1; ++k)
    {
        if (S(k) <= s_query_m && s_query_m <= S(k + 1))
        {
            i = k;
            break;
        }
    }

    const double ds = S(i + 1) - S(i);

    double t = 0.0;

    if (ds > 1.0e-9)
    {
        t = (s_query_m - S(i)) / ds;
    }

    t = clampLocal(t, 0.0, 1.0);

    const double ax = X(i);
    const double ay = Y(i);

    const double bx = X(i + 1);
    const double by = Y(i + 1);

    const double sx = bx - ax;
    const double sy = by - ay;

    out.valid = true;
    out.segment_index = i;
    out.s_m = s_query_m;

    out.x_m = ax + t * sx;
    out.y_m = ay + t * sy;

    out.yaw_rad = std::atan2(sy, sx);

    return out;
}


std::vector<double> buildCurvatureHorizonFromSpline(
    const ParamBank& P,
    const PathSpline& spline,
    double s0_m,
    double vx_mps)
{
    const int N =
        std::max(1, readActiveHorizonN(P));

    std::vector<double> kappa_horizon(static_cast<std::size_t>(N), 0.0);

    if (!spline.isValid())
    {
        return kappa_horizon;
    }

    const double dt =
        1.0 / std::max(P.get("model.frequency.steer_cmd_loop_hz"), 1.0e-6);

    const double ds =
        std::max(std::abs(vx_mps), 0.5) * dt;

    const double s_min =
        spline.sMin();

    const double s_max =
        spline.sMax();

    for (int k = 0; k < N; ++k)
    {
        const double s_raw =
            s0_m + static_cast<double>(k) * ds;

        const double s_query =
            clampLocal(s_raw, s_min, s_max);

        kappa_horizon[static_cast<std::size_t>(k)] =
            spline.curvatureAtS(s_query);
    }

    return kappa_horizon;
}


std::vector<double> buildSplineCurvatureHorizon(
    const ParamBank& P,
    const PathSpline& spline,
    double s0_m,
    double vx_mps)
{
    return buildCurvatureHorizonFromSpline(P, spline, s0_m, vx_mps);
}

} // namespace skidpad_control
