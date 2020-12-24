#pragma once

#include "Object.hpp"

struct Collision {

public:

	Collision(Object* obj1, Object* obj2, Vector2 point1, Vector2 point2, Vector2 normal, Float depth) : m_Object1(obj1), m_Object2(obj2), m_CollisionPoint1(point1), m_CollisionPoint2(point2), m_CollisionNormal(normal), m_Depth(depth) {}

	Object* m_Object1;
	Vector2 m_CollisionPoint1;

	Object* m_Object2;
	Vector2 m_CollisionPoint2;

	Float m_Depth;
	Vector2 m_CollisionNormal;

};