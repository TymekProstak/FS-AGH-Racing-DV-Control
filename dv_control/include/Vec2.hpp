#pragma once
#include <cmath>

namespace dv_control
{
struct Vec2 {
    float x;
    float y;

    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float xx, float yy) : x(xx), y(yy) {}

    inline Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    inline Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    inline Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    inline Vec2& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    inline float length() const {
        return std::sqrt(x * x + y * y);
    }
};
} // namespace dv_control