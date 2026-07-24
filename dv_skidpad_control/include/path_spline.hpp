#pragma once

#include <Eigen/Dense>

namespace skidpad_control
{

struct SplinePathPoint
{
    bool valid = false;

    double s_m = 0.0;

    double x_m = 0.0;
    double y_m = 0.0;

    double dx_ds = 0.0;
    double dy_ds = 0.0;

    double d2x_ds2 = 0.0;
    double d2y_ds2 = 0.0;

    double yaw_rad = 0.0;
    double kappa = 0.0;
};

class PathSpline
{
public:
    PathSpline() = default;

    bool fit(const Eigen::VectorXd& S,
             const Eigen::VectorXd& X,
             const Eigen::VectorXd& Y);

    bool isValid() const;

    double sMin() const;
    double sMax() const;

    SplinePathPoint evaluate(double s_query_m) const;

    double curvatureAtS(double s_query_m) const;

    // -------------------------------------------------------------------------
    // RAW path memory.
    //
    // The spline is only the smooth geometric representation. The RAW path from
    // path planning is still kept here so that code can relate a refined spline
    // s-value back to the original monotonic RAW path segments.
    // -------------------------------------------------------------------------
    bool hasRawPath() const;

    const Eigen::VectorXd& rawS() const;
    const Eigen::VectorXd& rawX() const;
    const Eigen::VectorXd& rawY() const;

    int rawSegmentIndexAtS(double s_query_m) const;

private:
    struct Cubic1D
    {
        Eigen::VectorXd s;
        Eigen::VectorXd a;
        Eigen::VectorXd b;
        Eigen::VectorXd c;
        Eigen::VectorXd d;

        bool valid = false;

        bool fitNatural(const Eigen::VectorXd& s_in,
                        const Eigen::VectorXd& y_in);

        double value(double s_query) const;
        double firstDerivative(double s_query) const;
        double secondDerivative(double s_query) const;

        int findSegment(double s_query) const;
    };

private:
    Cubic1D sx_;
    Cubic1D sy_;

    Eigen::VectorXd raw_S_;
    Eigen::VectorXd raw_X_;
    Eigen::VectorXd raw_Y_;

    double s_min_ = 0.0;
    double s_max_ = 0.0;

    bool valid_ = false;
    bool has_raw_path_ = false;
};

} // namespace skidpad_control