#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dv_control
{

struct SmoothedXY
{
    Eigen::VectorXd x;
    Eigen::VectorXd y;
};

namespace detail
{

inline double dotVec(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    return a.dot(b);
}

// A = I + lambda * D^T D
// gdzie D to operator drugiej różnicy dla open path:
// (Dq)_i = q_i - 2 q_{i+1} + q_{i+2}
inline Eigen::VectorXd applyOpenSmoothNormal(const Eigen::VectorXd& v,
                                             double lambda)
{
    const Eigen::Index n =
        v.size();

    Eigen::VectorXd out =
        v;

    if (lambda <= 0.0 || n < 3)
    {
        return out;
    }

    for (Eigen::Index i = 0; i + 2 < n; ++i)
    {
        const double w =
            v[i] - 2.0 * v[i + 1] + v[i + 2];

        out[i] +=
            lambda * w;

        out[i + 1] +=
            -2.0 * lambda * w;

        out[i + 2] +=
            lambda * w;
    }

    return out;
}

template <typename ApplyA>
inline Eigen::VectorXd solveCG(const Eigen::VectorXd& b,
                               ApplyA applyA,
                               int max_iters = 100,
                               double tol = 1.0e-10)
{
    const Eigen::Index n =
        b.size();

    if (n == 0)
    {
        return Eigen::VectorXd();
    }

    Eigen::VectorXd x =
        b;

    Eigen::VectorXd Ax =
        applyA(x);

    Eigen::VectorXd r =
        b - Ax;

    Eigen::VectorXd p =
        r;

    double rsold =
        dotVec(r, r);

    const double bnorm =
        std::max(1.0, b.norm());

    const double tol2 =
        (tol * bnorm) * (tol * bnorm);

    if (rsold <= tol2)
    {
        return x;
    }

    for (int it = 0; it < max_iters; ++it)
    {
        Eigen::VectorXd Ap =
            applyA(p);

        const double denom =
            dotVec(p, Ap);

        if (std::abs(denom) < 1.0e-20)
        {
            break;
        }

        const double alpha =
            rsold / denom;

        x.noalias() +=
            alpha * p;

        r.noalias() -=
            alpha * Ap;

        const double rsnew =
            dotVec(r, r);

        if (rsnew <= tol2)
        {
            break;
        }

        const double beta =
            rsnew / rsold;

        p =
            r + beta * p;

        rsold =
            rsnew;
    }

    return x;
}

inline void forceOpenPathEndpoints(const Eigen::Ref<const Eigen::VectorXd>& x_in,
                                   const Eigen::Ref<const Eigen::VectorXd>& y_in,
                                   Eigen::VectorXd& x_out,
                                   Eigen::VectorXd& y_out)
{
    const Eigen::Index n =
        x_in.size();

    if (n <= 0)
    {
        return;
    }

    // Trzymam pierwszy punkt dokładnie jak w raw path.
    x_out(0) =
        x_in(0);

    y_out(0) =
        y_in(0);

    // Trzymam ostatni punkt dokładnie jak w raw path.
    // To jest kluczowe, żeby smoothing nie skracał końca ścieżki.
    x_out(n - 1) =
        x_in(n - 1);

    y_out(n - 1) =
        y_in(n - 1);
}

} // namespace detail

inline SmoothedXY smoothXY(const Eigen::Ref<const Eigen::VectorXd>& x_in,
                           const Eigen::Ref<const Eigen::VectorXd>& y_in,
                           double lambda,
                           int max_iters = 100,
                           double tol = 1.0e-10)
{
    if (x_in.size() != y_in.size())
    {
        throw std::invalid_argument(
            "smoothXY: x_in and y_in must have the same size."
        );
    }

    SmoothedXY result;

    result.x =
        x_in;

    result.y =
        y_in;

    const Eigen::Index n =
        x_in.size();

    if (n < 3 || lambda <= 0.0)
    {
        return result;
    }

    auto applyA = [lambda](const Eigen::VectorXd& v) -> Eigen::VectorXd
    {
        return detail::applyOpenSmoothNormal(v, lambda);
    };

    result.x =
        detail::solveCG(result.x, applyA, max_iters, tol);

    result.y =
        detail::solveCG(result.y, applyA, max_iters, tol);

    // Najważniejsza poprawka:
    // smoothing wygładza środek, ale nie może przesuwać początku i końca patha.
    detail::forceOpenPathEndpoints(
        x_in,
        y_in,
        result.x,
        result.y
    );

    return result;
}

} // namespace dv_control
