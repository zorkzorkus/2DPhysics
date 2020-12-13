#pragma once

#include "Object.hpp"

class Circle : public Object {

public:

	Circle(float radius, float mass = 1.f, Vector2f position = Vector2f::Zero(), Vector2f velocity = Vector2f::Zero(), Vector2f acceleration = Vector2f::Zero(), float angle = 0.f, float angularVelocity  = 0.f) : Object(mass, position, velocity, acceleration, angle, angularVelocity), m_Radius(radius){
		m_MomentOfInertia = 2.f / 5.f * m_Mass * pow(m_Radius, 2);
		m_Type = Object::ObjectType::Circle;
	}

	float& Radius();

protected:

private:

	float m_Radius;

};