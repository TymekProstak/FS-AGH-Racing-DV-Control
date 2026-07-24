#include "path_spline.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace skidpad_control
{

namespace
{

double clampLocal(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

} // anonymous namespace


// =============================================================================
//                              CUBIC 1D
// =============================================================================

bool PathSpline::Cubic1D::fitNatural(const Eigen::VectorXd& s_in,
                                     const Eigen::VectorXd& y_in)
{
    const int n = static_cast<int>(s_in.size());

    if (n < 3 || y_in.size() != s_in.size())
    {
        valid = false;
        return false;
    }

    for (int i = 1; i < n; ++i)
    {
        if (!(s_in(i) > s_in(i - 1)))
        {
            valid = false;
            return false;
        }
    }

    s = s_in;
    a = y_in;

    b = Eigen::VectorXd::Zero(n - 1);
    c = Eigen::VectorXd::Zero(n);
    d = Eigen::VectorXd::Zero(n - 1);

    Eigen::VectorXd h(n - 1);

    for (int i = 0; i < n - 1; ++i)
    {
        h(i) = s(i + 1) - s(i);

        if (h(i) <= 1.0e-9)
        {
            valid = false;
            return false;
        }
    }

    /*
        Natural cubic spline:
            second derivative at both endpoints = 0

        Solving tridiagonal system for c coefficients.
    */

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);

    A(0, 0) = 1.0;
    A(n - 1, n - 1) = 1.0;

    for (int i = 1; i < n - 1; ++i)
    {
        A(i, i - 1) = h(i - 1);
        A(i, i) = 2.0 * (h(i - 1) + h(i));
        A(i, i + 1) = h(i);

        rhs(i) =
            3.0 * ((a(i + 1) - a(i)) / h(i) -
                   (a(i) - a(i - 1)) / h(i - 1));
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver(A);

    c = solver.solve(rhs);

    if (!c.allFinite())
    {
        valid = false;
        return false;
    }

    for (int i = 0; i < n - 1; ++i)
    {
        b(i) =
            (a(i + 1) - a(i)) / h(i) -
            h(i) * (2.0 * c(i) + c(i + 1)) / 3.0;

        d(i) =
            (c(i + 1) - c(i)) / (3.0 * h(i));
    }

    valid = true;
    return true;
}


int PathSpline::Cubic1D::findSegment(double s_query) const
{
    const int n = static_cast<int>(s.size());

    if (n < 2)
    {
        return 0;
    }

    if (s_query <= s(0))
    {
        return 0;
    }

    if (s_query >= s(n - 1))
    {
        return n - 2;
    }

    int lo = 0;
    int hi = n - 1;

    while (hi - lo > 1)
    {
        const int mid = (lo + hi) / 2;

        if (s(mid) <= s_query)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    return lo;
}


double PathSpline::Cubic1D::value(double s_query) const
{
    if (!valid)
    {
        return 0.0;
    }

    const int i = findSegment(s_query);

    const double ds =
        s_query - s(i);

    return a(i) + b(i) * ds + c(i) * ds * ds + d(i) * ds * ds * ds;
}


double PathSpline::Cubic1D::firstDerivative(double s_query) const
{
    if (!valid)
    {
        return 0.0;
    }

    const int i = findSegment(s_query);

    const double ds =
        s_query - s(i);

    return b(i) + 2.0 * c(i) * ds + 3.0 * d(i) * ds * ds;
}


double PathSpline::Cubic1D::secondDerivative(double s_query) const
{
    if (!valid)
    {
        return 0.0;
    }

    const int i = findSegment(s_query);

    const double ds =
        s_query - s(i);

    return 2.0 * c(i) + 6.0 * d(i) * ds;
}


// =============================================================================
//                              PATH SPLINE
// =============================================================================

bool PathSpline::fit(const Eigen::VectorXd& S,
                     const Eigen::VectorXd& X,
                     const Eigen::VectorXd& Y)
{
    valid_ = false;
    has_raw_path_ = false;

    const int n = static_cast<int>(S.size());

    if (n < 3 || X.size() != S.size() || Y.size() != S.size())
    {
        std::cerr << "[PathSpline] Invalid input size." << std::endl;
        return false;
    }

    if (!S.allFinite() || !X.allFinite() || !Y.allFinite())
    {
        std::cerr << "[PathSpline] Input contains NaN/Inf." << std::endl;
        return false;
    }

    for (int i = 1; i < n; ++i)
    {
        if (!(S(i) > S(i - 1)))
        {
            std::cerr << "[PathSpline] S must be strictly increasing." << std::endl;
            return false;
        }
    }

    if (!sx_.fitNatural(S, X))
    {
        std::cerr << "[PathSpline] Failed to fit x(s)." << std::endl;
        return false;
    }

    if (!sy_.fitNatural(S, Y))
    {
        std::cerr << "[PathSpline] Failed to fit y(s)." << std::endl;
        return false;
    }

    raw_S_ = S;
    raw_X_ = X;
    raw_Y_ = Y;
    has_raw_path_ = true;

    s_min_ = S(0);
    s_max_ = S(n - 1);

    valid_ = true;
    return true;
}


bool PathSpline::isValid() const
{
    return valid_;
}


double PathSpline::sMin() const
{
    return s_min_;
}


double PathSpline::sMax() const
{
    return s_max_;
}


SplinePathPoint PathSpline::evaluate(double s_query_m) const
{
    SplinePathPoint out;

    if (!valid_)
    {
        return out;
    }

    const double s_query =
        clampLocal(s_query_m, s_min_, s_max_);

    const double x =
        sx_.value(s_query);

    const double y =
        sy_.value(s_query);

    const double dx =
        sx_.firstDerivative(s_query);

    const double dy =
        sy_.firstDerivative(s_query);

    const double ddx =
        sx_.secondDerivative(s_query);

    const double ddy =
        sy_.secondDerivative(s_query);

    const double speed2 =
        dx * dx + dy * dy;

    double kappa = 0.0;

    if (speed2 > 1.0e-12)
    {
        const double denom =
            std::pow(speed2, 1.5);

        kappa =
            (dx * ddy - dy * ddx) / denom;
    }

    out.valid = true;

    out.s_m = s_query;

    out.x_m = x;
    out.y_m = y;

    out.dx_ds = dx;
    out.dy_ds = dy;

    out.d2x_ds2 = ddx;
    out.d2y_ds2 = ddy;

    out.yaw_rad =
        std::atan2(dy, dx);

    out.kappa =
        kappa;

    return out;
}


double PathSpline::curvatureAtS(double s_query_m) const
{
    const SplinePathPoint p =
        evaluate(s_query_m);

    if (!p.valid)
    {
        return 0.0;
    }

    return p.kappa;
}


bool PathSpline::hasRawPath() const
{
    return has_raw_path_;
}


const Eigen::VectorXd& PathSpline::rawS() const
{
    return raw_S_;
}


const Eigen::VectorXd& PathSpline::rawX() const
{
    return raw_X_;
}


const Eigen::VectorXd& PathSpline::rawY() const
{
    return raw_Y_;
}


int PathSpline::rawSegmentIndexAtS(double s_query_m) const
{
    const int n = static_cast<int>(raw_S_.size());

    if (!has_raw_path_ || n < 2)
    {
        return 0;
    }

    const double s_query =
        clampLocal(s_query_m, raw_S_(0), raw_S_(n - 1));

    if (s_query <= raw_S_(0))
    {
        return 0;
    }

    if (s_query >= raw_S_(n - 1))
    {
        return n - 2;
    }

    int lo = 0;
    int hi = n - 1;

    while (hi - lo > 1)
    {
        const int mid =
            (lo + hi) / 2;

        if (raw_S_(mid) <= s_query)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    return std::max(0, std::min(lo, n - 2));
}

} // namespace skidpad_control