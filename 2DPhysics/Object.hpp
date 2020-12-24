#pragma once

#include "Common.hpp"

class Object {

public:

	enum class ObjectType { Invalid, Box, Circle };

	virtual void Update(Float timeDelta);

	// Reference
	Vector2& Position();
	Vector2& Velocity();
	Vector2& Acceleration();
	Float& Angle();
	Float& AngularVelocity();
	Float& MomentOfInertia();

	// Mass can not be accessed by reference since we cache it's inverted value.
	Float GetMass();
	Float GetInvertedMass();
	void SetMass(Float mass);
	void SetInvertedMass(Float invMass);

	bool IsMovable();
	ObjectType GetType();

	UINT32 m_Color;

protected:

	Object(Float mass = 1.f, Vector2 position = Vector2::Zero(), Vector2 velocity = Vector2::Zero(), Vector2 acceleration = Vector2::Zero(), Float angle = 0.f, Float angularVelocity = 0.f);

	Vector2 m_Position;
	Vector2 m_Velocity;
	Vector2 m_Acceleration;
	Float m_MomentOfInertia;
	Float m_Mass;
	Float m_InvertedMass;
	Float m_Angle;
	Float m_AngularVelocity;


	ObjectType m_Type;

private:

};