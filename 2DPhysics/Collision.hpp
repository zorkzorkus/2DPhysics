#pragma once

#include "Object.hpp"

struct Collision {

public:

	Collision(Object* obj1, Object* obj2, Vector2f point1, Vector2f point2, Vector2f normal, float depth) : m_Object1(obj1), m_Object2(obj2), m_CollisionPoint1(point1), m_CollisionPoint2(point2), m_CollisionNormal(normal), m_Depth(depth) {}

	Object* m_Object1;
	Vector2f m_CollisionPoint1;

	Object* m_Object2;
	Vector2f m_CollisionPoint2;

	float m_Depth;
	Vector2f m_CollisionNormal;

};