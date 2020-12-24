#include "Object.hpp"

void Object::Update(Float timeDelta) {

	if (m_InvertedMass == 0.f) return;

	m_Position += timeDelta * m_Velocity + (timeDelta*timeDelta * 0.5f) * m_Acceleration;
	m_Velocity += timeDelta * m_Acceleration;

	m_Angle += timeDelta * m_AngularVelocity;
	m_Angle = fmodf(m_Angle, 360.f);

}

Vector2 & Object::Position() {
	return m_Position;
}

Vector2 & Object::Velocity() {
	return m_Velocity;
}

Vector2 & Object::Acceleration() {
	return m_Acceleration;
}

Float & Object::Angle() {
	return m_Angle;
}

Float & Object::AngularVelocity() {
	return m_AngularVelocity;
}

Float& Object::MomentOfInertia() {
	return m_MomentOfInertia;
}

Float Object::GetMass() {
	return m_Mass;
}

Float Object::GetInvertedMass() {
	return m_InvertedMass;
}

void Object::SetMass(Float mass) {

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

void Object::SetInvertedMass(Float invMass) {

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

Object::Object(Float mass, Vector2 position, Vector2 velocity, Vector2 acceleration, Float angle, Float angularVelocity) : m_Position(position), m_Velocity(velocity), m_Acceleration(acceleration), m_Angle(angle), m_AngularVelocity(angularVelocity), m_Type(Object::ObjectType::Invalid) {
	SetMass(mass);
	m_Color = rand() | 0xFF000000; // shit code
}
