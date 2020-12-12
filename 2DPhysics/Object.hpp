#pragma once

#include "Common.hpp"

class Object {

public:

	enum class ObjectType { Invalid, Box, Circle };

	virtual void Update(float timeDelta);

	// Reference
	Vector2f& Position();
	Vector2f& Velocity();
	Vector2f& Acceleration();
	float& Angle();
	float& AngularVelocity();

	// Mass can not be accessed by reference since we cache it's inverted value.
	float GetMass();
	float GetInvertedMass();
	void SetMass(float mass);
	void SetInvertedMass(float invMass);

	bool IsMovable();
	ObjectType GetType();

	UINT32 m_Color;

protected:

	Object(float mass = 1.f, Vector2f position = Vector2f::Zero(), Vector2f velocity = Vector2f::Zero(), Vector2f acceleration = Vector2f::Zero(), float angle = 0.f, float angularVelocity = 0.f);

	Vector2f m_Position;
	Vector2f m_Velocity;
	Vector2f m_Acceleration;
	float m_Mass;
	float m_InvertedMass;
	float m_Angle;
	float m_AngularVelocity;


	ObjectType m_Type;

private:

};