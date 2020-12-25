#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <ZorkLib.hpp>
#include <ZorkLibWindow.hpp>

#undef max
#undef min

#ifdef USE_FLOAT64
using Float = double;
using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Matrix2 = Eigen::Matrix2d;
using Matrix3 = Eigen::Matrix3d;
#else
using Float = float;
using Vector2 = Eigen::Vector2f;
using Vector3 = Eigen::Vector3f;
using Matrix2 = Eigen::Matrix2f;
using Matrix3 = Eigen::Matrix3f;
#endif

constexpr Float cGravity = -9.81;
constexpr Float cInfiniteMass = std::numeric_limits<Float>::infinity();
constexpr Float cPI = 3.14159265358979323846;
constexpr Float cEpsilon = 1e-05;

constexpr size_t sMaxCollisionSteps = 30;
constexpr Float sSpeedFactor = 10.0;
constexpr Float cSingleStepTime = 0.001; // 1ms
constexpr Float cSingleStepTime2 = 0.01; // 1ms

inline Float deg2rad(Float deg) {
	return deg * cPI / 180.f;
}

inline Float rad2deg(Float rad) {
	return rad * 180.f / cPI;
}

template <typename T>
inline T max(T a, T b) {
	return (a < b) ? b : a;
}

template <typename T>
inline T min(T a, T b) {
	return (a < b) ? a : b;
}

inline void __sincospid(double a, double* sp, double* cp) {
    double c, r, s, t, az;
    int64_t i;

    az = a * 0.0; // must be evaluated with IEEE-754 semantics
    /* for |a| >= 2**53, cospi(a) = 1.0, but cospi(Inf) = NaN */
    a = (fabs(a) < 9.0071992547409920e+15) ? a : az;  // 0x1.0p53
    /* reduce argument to primary approximation interval (-0.25, 0.25) */
    r = nearbyint(a + a); // must use IEEE-754 "to nearest" rounding
    i = (int64_t)r;
    t = fma(-0.5, r, a);
    /* compute core approximations */
    s = t * t;
    /* Approximate cos(pi*x) for x in [-0.25,0.25] */
    r = -1.0369917389758117e-4;
    r = fma(r, s, 1.9294935641298806e-3);
    r = fma(r, s, -2.5806887942825395e-2);
    r = fma(r, s, 2.3533063028328211e-1);
    r = fma(r, s, -1.3352627688538006e+0);
    r = fma(r, s, 4.0587121264167623e+0);
    r = fma(r, s, -4.9348022005446790e+0);
    c = fma(r, s, 1.0000000000000000e+0);
    /* Approximate sin(pi*x) for x in [-0.25,0.25] */
    r = 4.6151442520157035e-4;
    r = fma(r, s, -7.3700183130883555e-3);
    r = fma(r, s, 8.2145868949323936e-2);
    r = fma(r, s, -5.9926452893214921e-1);
    r = fma(r, s, 2.5501640398732688e+0);
    r = fma(r, s, -5.1677127800499516e+0);
    s = s * t;
    r = r * s;
    s = fma(t, 3.1415926535897931e+0, r);
    /* map results according to quadrant */
    if (i & 2) {
        s = 0.0 - s; // must be evaluated with IEEE-754 semantics
        c = 0.0 - c; // must be evaluated with IEEE-754 semantics
    }
    if (i & 1) {
        t = 0.0 - s; // must be evaluated with IEEE-754 semantics
        s = c;
        c = t;
    }
    /* IEEE-754: sinPi(+n) is +0 and sinPi(-n) is -0 for positive integers n */
    if (a == floor(a)) s = az;
    *sp = s;
    *cp = c;
}

inline void __sincospif(float a, float* sp, float* cp) {
    float az, t, c, r, s;
    int32_t i;

    az = a * 0.0f; // must be evaluated with IEEE-754 semantics
    /* for |a| > 2**24, cospi(a) = 1.0f, but cospi(Inf) = NaN */
    a = (fabsf(a) < 0x1.0p24f) ? a : az;
    r = nearbyintf(a + a); // must use IEEE-754 "to nearest" rounding
    i = (int32_t)r;
    t = fmaf(-0.5f, r, a);
    /* compute core approximations */
    s = t * t;
    /* Approximate cos(pi*x) for x in [-0.25,0.25] */
    r = 0x1.d9e000p-3f;
    r = fmaf(r, s, -0x1.55c400p+0f);
    r = fmaf(r, s, 0x1.03c1cep+2f);
    r = fmaf(r, s, -0x1.3bd3ccp+2f);
    c = fmaf(r, s, 0x1.000000p+0f);
    /* Approximate sin(pi*x) for x in [-0.25,0.25] */
    r = -0x1.310000p-1f;
    r = fmaf(r, s, 0x1.46737ep+1f);
    r = fmaf(r, s, -0x1.4abbfep+2f);
    r = (t * s) * r;
    s = fmaf(t, 0x1.921fb6p+1f, r);
    if (i & 2) {
        s = 0.0f - s; // must be evaluated with IEEE-754 semantics
        c = 0.0f - c; // must be evaluated with IEEE-754 semantics
    }
    if (i & 1) {
        t = 0.0f - s; // must be evaluated with IEEE-754 semantics
        s = c;
        c = t;
    }
    /* IEEE-754: sinPi(+n) is +0 and sinPi(-n) is -0 for positive integers n */
    if (a == floorf(a)) s = az;
    *sp = s;
    *cp = c;
}

inline void sincospi(Float a, Float& sp, Float& cp) {
#ifdef USE_FLOAT64
    return __sincospid(a, &sp, &cp);
#else
    return __sincospif(a, &sp, &cp);
#endif // USE_FLOAT64
}