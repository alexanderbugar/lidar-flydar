#pragma once

#include <cmath>

struct Vec3 {
    float x, y, z;

    Vec3 operator+(Vec3 v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(Vec3 v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float k) const { return {x*k, y*k, z*k}; }
    float dot(Vec3 v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(Vec3 v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
};

struct Quat {
    float w = 1, x = 0, y = 0, z = 0;

    Quat operator*(Quat q) const {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w,
        };
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    Quat unit() const {
        const float norm = sqrtf(w*w + x*x + y*y + z*z);
        if (!(norm > 1e-9f) || !std::isfinite(norm)) return {};
        return {w/norm, x/norm, y/norm, z/norm};
    }

    Vec3 rotate(Vec3 v) const {
        const Vec3 axis{x, y, z};
        const Vec3 t = axis.cross(v) * 2;
        return v + t*w + axis.cross(t);
    }

    // callers pass one of the unit basis axes here.
    static Quat about(Vec3 axis, float radians) {
        const float h = radians * 0.5f;
        const float s = sinf(h);
        return {cosf(h), axis.x*s, axis.y*s, axis.z*s};
    }
};
