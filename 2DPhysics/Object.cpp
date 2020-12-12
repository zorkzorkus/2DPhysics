#include "Object.hpp"

void Object::Update(float timeDelta) {

	if (m_InvertedMass == 0.f) return;

	m_Position += timeDelta * m_Velocity + (timeDelta*timeDelta * 0.5f) * m_Acceleration;
	m_Velocity += timeDelta * m_Acceleration;

	m_Angle += timeDelta * m_AngularVelocity;
	m_Angle = fmodf(m_Angle, 360.f);

}

Vector2f & Object::Position() {
	return m_Position;
}

Vector2f & Object::Velocity() {
	return m_Velocity;
}

Vector2f & Object::Acceleration() {
	return m_Acceleration;
}

float & Object::Angle() {
	return m_Angle;
}

float & Object::AngularVelocity() {
	return m_AngularVelocity;
}

float Object::GetMass() {
	return m_Mass;
}

float Object::GetInvertedMass() {
	return m_InvertedMass;
}

void Object::SetMass(float mass) {

	if (mass == 0.f) {
		m_Mass = 0.f;
		m_InvertedMass = cInfiniteMass;
		return;
	}

	if (mass == cInfiniteMass) {
		m_Mass = cInfiniteMass;
		m_InvertedMass = 0.f;
		return;
	}

	m_Mass = mass;
	m_InvertedMass = 1.f / mass;
}

void Object::SetInvertedMass(float invMass) {

	if (invMass == 0.f) {
		m_Mass = cInfiniteMass;
		m_InvertedMass = 0.f;
		return;
	}

	if (invMass == cInfiniteMass) {
		m_Mass = 0.f;
		m_InvertedMass = cInfiniteMass;
		return;
	}

	m_Mass = 1.f / invMass;
	m_InvertedMass = invMass;
}

bool Object::IsMovable() {
	return m_InvertedMass != 0.f;
}

Object::ObjectType Object::GetType() {
	return m_Type;
}

Object::Object(float mass, Vector2f position, Vector2f velocity, Vector2f acceleration, float angle, float angularVelocity) : m_Position(position), m_Velocity(velocity), m_Acceleration(acceleration), m_Angle(angle), m_AngularVelocity(angularVelocity), m_Type(Object::ObjectType::Invalid) {
	SetMass(mass);
	m_Color = rand() | 0xFF000000; // shit code
}
