#pragma once

#include "Object.hpp"

class Box : public Object {

public:

	Box(Vector2 halfSize, Float mass = 1.f, Vector2 position = Vector2::Zero(), Vector2 velocity = Vector2::Zero(), Vector2 acceleration = Vector2::Zero(), Float angle = 0.f, Float angularVelocity = 0.f)
		: Object(mass, position, velocity, acceleration, angle, angularVelocity), m_HalfSize(halfSize) {
		m_MomentOfInertia = m_Mass / 12.f * (m_HalfSize.squaredNorm());
		m_Type = Object::ObjectType::Box;
	}

	Vector2& HalfSize();

protected:

private:

	Vector2 m_HalfSize;

};
