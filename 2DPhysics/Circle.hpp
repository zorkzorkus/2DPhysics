#pragma once

#include "Object.hpp"

class Circle : public Object {

public:

	Circle(Float radius, Float mass = 1.f, Vector2 position = Vector2::Zero(), Vector2 velocity = Vector2::Zero(), Vector2 acceleration = Vector2::Zero(), Float angle = 0.f, Float angularVelocity  = 0.f) : Object(mass, position, velocity, acceleration, angle, angularVelocity), m_Radius(radius){
		m_MomentOfInertia = 2.f / 5.f * m_Mass * pow(m_Radius, 2);
		m_Type = Object::ObjectType::Circle;
	}

	Float& Radius();

protected:

private:

	Float m_Radius;

};