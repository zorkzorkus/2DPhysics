#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <ZorkLib.hpp>
#include <ZorkLibWindow.hpp>

#undef max
#undef min

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;

constexpr float cGravity = -9.81f;
constexpr float cInfiniteMass = std::numeric_limits<float>::infinity();
constexpr float cPI = 3.14159265358979323846f;
constexpr float cEpsilon = 1e-05;

constexpr size_t sMaxCollisionSteps = 30;
constexpr float sSpeedFactor = 10.f;

inline float deg2rad(float deg) {
	return deg * cPI / 180.f;
}

inline float rad2deg(float rad) {
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