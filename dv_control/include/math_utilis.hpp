#pragma once

namespace dv_control
{

// =============================================================================
//                              MATH HELPERS
// =============================================================================

double clampLocal(double x, double lo, double hi);

double wrapAngle(double angle_rad);

double norm2(double x, double y);

} // namespace dv_control