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