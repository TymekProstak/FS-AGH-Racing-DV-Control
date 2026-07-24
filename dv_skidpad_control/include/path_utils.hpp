#pragma once

#include <vector>

#include <Eigen/Dense>

#include "ParamBank.hpp"
#include "path_spline.hpp"

namespace skidpad_control
{

// =============================================================================
//                              PATH PROJECTION
// =============================================================================

struct PathProjection
{
    bool valid = false;

    // RAW segment used as monotonic anchor.
    int segment_index = 0;

    // If refined_by_spline == false, s_m is the RAW polyline projection s.
    // If refined_by_spline == true, s_m is the locally refined spline s.
    double s_m = 0.0;

    double x_ref_m = 0.0;
    double y_ref_m = 0.0;
    double yaw_ref_rad = 0.0;

    double ey_m = 0.0;
    double epsi_rad = 0.0;

    double kappa_ref = 0.0;

    // Diagnostic / pipeline fields.
    bool refined_by_spline = false;
    int raw_segment_index = 0;
    double raw_s_m = 0.0;
};


// =============================================================================
//                              PATH HELPERS
// =============================================================================

Eigen::VectorXd buildArcLength(const Eigen::VectorXd& X,
                               const Eigen::VectorXd& Y);


double estimateCurvatureAtSegment(const Eigen::VectorXd& X,
                                  const Eigen::VectorXd& Y,
                                  int segment_index);


// Step 1 of the pipeline:
//     RAW monotonic segment search on path-planning polyline.
//
// This function intentionally uses distance-only segment projection. No yaw
// score, no global spline nearest point, no branch-jump logic.
PathProjection projectToPathMonotonic(const Eigen::VectorXd& X,
                                      const Eigen::VectorXd& Y,
                                      const Eigen::VectorXd& S,
                                      double px,
                                      double py,
                                      double yaw_rad,
                                      int last_segment_index,
                                      double last_s_m,
                                      bool has_previous_projection);


// Step 2 of the pipeline:
//     local spline refinement near the monotonic RAW projection.
//
// The search is limited to a small window around coarse.raw_s/coarse.s. The
// lower bound can be set to the previous accepted s to keep the final refined
// s monotonic as well.
PathProjection refineProjectionOnSplineLocal(const PathSpline& spline,
                                             const PathProjection& coarse,
                                             double px,
                                             double py,
                                             double yaw_rad,
                                             double min_allowed_s_m,
                                             double half_window_m);


double curvatureAtS(const Eigen::VectorXd& X,
                    const Eigen::VectorXd& Y,
                    const Eigen::VectorXd& S,
                    double s_query_m);


int readActiveHorizonN(const ParamBank& P);


std::vector<double> buildCurvatureHorizon(const ParamBank& P,
                                          const Eigen::VectorXd& X,
                                          const Eigen::VectorXd& Y,
                                          const Eigen::VectorXd& S,
                                          double s0_m,
                                          double vx_mps);


struct PathPoint
{
    bool valid = false;

    int segment_index = 0;

    double s_m = 0.0;

    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

PathPoint pointAtS(const Eigen::VectorXd& X,
                   const Eigen::VectorXd& Y,
                   const Eigen::VectorXd& S,
                   double s_query_m);


std::vector<double> buildCurvatureHorizonFromSpline(
    const ParamBank& P,
    const PathSpline& spline,
    double s0_m,
    double vx_mps);

// Compatibility name used by current wrapper.cpp.
std::vector<double> buildSplineCurvatureHorizon(
    const ParamBank& P,
    const PathSpline& spline,
    double s0_m,
    double vx_mps);

} // namespace skidpad_control