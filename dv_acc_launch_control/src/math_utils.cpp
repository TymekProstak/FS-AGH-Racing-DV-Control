#include "math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace acc_launch_control
{

namespace
{
    constexpr double kPi = 3.141592653589793238462643383279502884;
}


// =============================================================================
//                              MATH HELPERS
// =============================================================================

double clampLocal(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}


double wrapAngle(double angle_rad)
{
    while (angle_rad > kPi)
    {
        angle_rad -= 2.0 * kPi;
    }

    while (angle_rad < -kPi)
    {
        angle_rad += 2.0 * kPi;
    }

    return angle_rad;
}


double norm2(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

} // namespace acc_launch_control