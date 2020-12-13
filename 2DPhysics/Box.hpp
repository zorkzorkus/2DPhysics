#pragma once

#include "Object.hpp"

class Box : public Object {

public:

	Box(Vector2f halfSize, float mass = 1.f, Vector2f position = Vector2f::Zero(), Vector2f velocity = Vector2f::Zero(), Vector2f acceleration = Vector2f::Zero(), float angle = 0.f, float angularVelocity = 0.f)
		: Object(mass, position, velocity, acceleration, angle, angularVelocity), m_HalfSize(halfSize) {
		m_MomentOfInertia = m_Mass / 12.f * (m_HalfSize.squaredNorm());
		m_Type = Object::ObjectType::Box;
	}

	Vector2f& HalfSize();

protected:

private:

	Vector2f m_HalfSize;

};
