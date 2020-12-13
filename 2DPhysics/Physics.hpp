#pragma once

#include "ObjectUtility.hpp"
#include "Collision.hpp"

class Physics {

public:

	Physics(std::vector<Object*>& objects);

	void Update(float timeDelta);

private:

	void HandleCollisions();
	bool CheckCollisions();
	void ResolveCollision();
	void ApplyImpulse();
	void ApplyImpulseSimple();
	void ApplyImpulseRotation();

	bool TestCollision(Object* o1, Object* o2);
	bool TestCollisionCircleCircle(Circle* c1, Circle* c2);
	bool TestCollisionCircleBox(Circle* c1, Box* c2, bool swapped);
	bool TestCollisionBoxBox(Box* c1, Box* c2);

	std::vector<Object*>& m_Objects;
	std::vector<Collision> m_Collisions;

};