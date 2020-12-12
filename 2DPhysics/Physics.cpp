#include "Physics.hpp"

Physics::Physics(std::vector<Object*>& objects) : m_Objects(objects) {
}

void Physics::Update(float timeDelta) {

	for (auto& o : m_Objects) {
		o->Update(timeDelta);
	}

	HandleCollisions();

}

void Physics::HandleCollisions() {

	size_t steps = 1;

	if (CheckCollisions()) {
		ApplyImpulse();
		ResolveCollision();
	}

	while (CheckCollisions()) {
		if (steps >= sMaxCollisionSteps) break;
		ResolveCollision();
		steps++;
	}

}

bool Physics::CheckCollisions() {

	m_Collisions.clear();
	bool collisionPresent = false;

	for (size_t it1 = 0; it1 < m_Objects.size(); ++it1) {
		for (size_t it2 = it1 + 1; it2 < m_Objects.size(); ++it2) {
			if (TestCollision(m_Objects[it1], m_Objects[it2])) collisionPresent = true;
		}
	}

	return collisionPresent;

}

void Physics::ResolveCollision() {

	for (auto& c : m_Collisions) {

		Object* obj1 = c.m_Object1;
		Object* obj2 = c.m_Object2;

		float sumInvMass = obj1->GetInvertedMass() + obj2->GetInvertedMass();
		float massFactor1 = obj1->GetInvertedMass() / sumInvMass;
		float massFactor2 = obj2->GetInvertedMass() / sumInvMass;

		// Resolve Interpenetration

		if (obj1->IsMovable()) {
			obj1->Position() -= c.m_CollisionNormal * massFactor1 * c.m_Depth;
		}

		if (obj2->IsMovable()) {
			obj2->Position() += c.m_CollisionNormal * massFactor2 * c.m_Depth;
		}

	}

}

void Physics::ApplyImpulse() {

	for (auto& c : m_Collisions) {

		Object* obj1 = c.m_Object1;
		Object* obj2 = c.m_Object2;

		float sumInvMass = obj1->GetInvertedMass() + obj2->GetInvertedMass();
		float massFactor1 = obj1->GetInvertedMass() / sumInvMass;
		float massFactor2 = obj2->GetInvertedMass() / sumInvMass;

		// Apply Impulse (without rotation / friction)

		Vector2f velocityParallel1 = obj1->Velocity().dot(c.m_CollisionNormal) * c.m_CollisionNormal;
		Vector2f velocityParallel2 = obj2->Velocity().dot(c.m_CollisionNormal) * c.m_CollisionNormal;
		Vector2f closingVelocity = velocityParallel2 - velocityParallel1;

		// TODO: coefficient of resitution = 30%
		float coeff = 1.f + 1.0f;

		obj1->Velocity() += massFactor1 * closingVelocity * coeff;
		obj2->Velocity() -= massFactor2 * closingVelocity * coeff;

	}
}

bool Physics::TestCollision(Object* o1, Object* o2) {

	// two immovable objects dont cause collisions
	if (!o1->IsMovable() && !o2->IsMovable()) return false;

	if (Circle* obj1Circle = IsObjectType<Circle>(o1)) {
		if (Circle* obj2Circle = IsObjectType<Circle>(o2)) {
			return TestCollisionCircleCircle(obj1Circle, obj2Circle);
		}

		if (Box* obj2Box = IsObjectType<Box>(o2)) {
			return TestCollisionCircleBox(obj1Circle, obj2Box, false);
		}

		__debugbreak();

	}

	if (Box* obj1Box = IsObjectType<Box>(o1)) {

		if (Circle* obj2Circle = IsObjectType<Circle>(o2)) {
			return TestCollisionCircleBox(obj2Circle, obj1Box, true);
		}

		if (Box* obj2Box = IsObjectType<Box>(o2)) {
			return TestCollisionBoxBox(obj1Box, obj2Box);
		}

		__debugbreak();

	}

	// obj1 is neither Box nor Circle.
	__debugbreak();
	return false;
}

bool Physics::TestCollisionCircleCircle(Circle * c1, Circle * c2) {
	Vector2f diff = c2->Position() - c1->Position();
	float depth = (c1->Radius() + c2->Radius()) - diff.norm();
	if (depth > 0) {
		diff.normalize();
		Vector2f p1 = c1->Position() + diff * c1->Radius();
		Vector2f p2 = c2->Position() - diff * c2->Radius();
		m_Collisions.push_back(Collision(c1, c2, p1, p2, diff, depth));
		return true;
	}
	return false;
}

bool Physics::TestCollisionCircleBox(Circle * c1, Box * c2, bool swapped) {

	// Boundary Test
	float maxBox1 = c1->Radius();
	float maxBox2 = c2->HalfSize().maxCoeff();
	float distance = (c2->Position() - c1->Position()).norm();
	if (distance > maxBox1 + maxBox2) return false;


	Matrix2f rot;
	float radBoxAligned = deg2rad(c2->Angle());
	rot(0, 0) = cos(radBoxAligned);
	rot(0, 1) = -sin(radBoxAligned);
	rot(1, 0) = sin(radBoxAligned);
	rot(1, 1) = cos(radBoxAligned);
	Matrix2f invRot = rot.inverse();

	Vector2f sPos = c1->Position();
	sPos -= c2->Position();
	sPos = rot * sPos;

	Vector2f cBox = sPos;
	float xsize = c2->HalfSize()(0);
	float ysize = c2->HalfSize()(1);
	if (cBox(0) < -xsize) {
		cBox(0) = -xsize;
	} else if (cBox(0) > xsize) {
		cBox(0) = xsize;
	}
	if (cBox(1) < -ysize) {
		cBox(1) = -ysize;
	} else if (cBox(1) > ysize) {
		cBox(1) = ysize;
	}

	cBox = invRot * cBox;
	sPos = invRot * sPos;

	Vector2f box2circle = sPos - cBox;
	float depth = c1->Radius() - box2circle.norm();

	if (depth > 0) {
		box2circle.normalize();
		Vector2f cSp = c1->Position() - box2circle * c1->Radius();
		if (swapped) {
			m_Collisions.push_back(Collision(c1, c2, cSp, cBox, -box2circle, depth));
		} else {
			m_Collisions.push_back(Collision(c2, c1, cBox, cSp, box2circle, depth));
		}
		return true;
	}

	return false;

}

bool Physics::TestCollisionBoxBox(Box * c1, Box * c2) {

	// BoundaryTest
	float maxBox1 = c1->HalfSize().norm();
	float maxBox2 = c2->HalfSize().norm();
	float distance = (c2->Position() - c1->Position()).norm();
	if (distance > maxBox1 + maxBox2) return false;

	// Potential collision, initialized with infinite depth.
	Collision coll(c1, c2, Vector2f(), Vector2f(), Vector2f(), std::numeric_limits<float>::infinity());

	// x is length of box on projection in origin
	// halfsize is width,length of other box
	// pos is projected position of otherbox with respect to origin of box 1
	// anglerad is angle in radians with respect to rotation of box 1	
	auto projectionHelper = [](float x, Vector2f halfsize, float pos, float angleRad) -> float {

		float d1 = abs(sin(angleRad)) * halfsize(0) + abs(cos(angleRad)) * halfsize(1);
		float d2 = abs(sin(angleRad)) * halfsize(1) + abs(cos(angleRad)) * halfsize(0);
		float d = max(d1, d2);
		return x - abs(pos) + d;

	};

	auto CollisionCalculation = [&](Box* aligned, Box* other) {

		if (coll.m_Depth < 0) return;

		Matrix2f rotMatrixToAligned = Matrix2f::Zero();
		float radBoxAligned = deg2rad(aligned->Angle());
		float radBoxOther = deg2rad(other->Angle());

		rotMatrixToAligned(0, 0) = cos(radBoxAligned);
		rotMatrixToAligned(0, 1) = -sin(radBoxAligned);
		rotMatrixToAligned(1, 0) = sin(radBoxAligned);
		rotMatrixToAligned(1, 1) = cos(radBoxAligned);

		Vector2f positionOfBoxOtherInAlignedSpace = other->Position();
		positionOfBoxOtherInAlignedSpace -= aligned->Position();
		positionOfBoxOtherInAlignedSpace = rotMatrixToAligned * positionOfBoxOtherInAlignedSpace;

		Vector2f center2center = other->Position() - aligned->Position();

		float rotOtherInAlignedSpace = radBoxOther - radBoxAligned; // TODO: is correct?

		float depth1 = projectionHelper(aligned->HalfSize()(0), other->HalfSize(), positionOfBoxOtherInAlignedSpace(0), rotOtherInAlignedSpace);

		if (depth1 < coll.m_Depth) {

			coll.m_Depth = depth1;
			if (depth1 < 0) return;
			coll.m_Object1 = aligned;
			coll.m_Object2 = other;
			coll.m_CollisionNormal(0) = cos(radBoxAligned);
			coll.m_CollisionNormal(1) = -sin(radBoxAligned);
			if (coll.m_CollisionNormal.dot(center2center) < 0) coll.m_CollisionNormal *= -1.f;
			Vector2f widthAxisBox2(cos(radBoxOther), -sin(radBoxOther));
			Vector2f heightAxisBox2(sin(radBoxOther), cos(radBoxOther));
			coll.m_CollisionPoint2 = c2->Position();
			if (widthAxisBox2.dot(coll.m_CollisionNormal) > 0) {
				coll.m_CollisionPoint2 += widthAxisBox2 * c2->HalfSize()(0);
			} else {
				coll.m_CollisionPoint2 -= widthAxisBox2 * c2->HalfSize()(0);
			}
			if (heightAxisBox2.dot(coll.m_CollisionNormal) > 0) {
				coll.m_CollisionPoint2 += heightAxisBox2 * c2->HalfSize()(0);
			} else {
				coll.m_CollisionPoint2 -= heightAxisBox2 * c2->HalfSize()(0);
			}
			coll.m_CollisionPoint1 = coll.m_CollisionPoint2 - (coll.m_Depth * coll.m_CollisionNormal);

		}

		float depth2 = projectionHelper(aligned->HalfSize()(1), other->HalfSize(), positionOfBoxOtherInAlignedSpace(1), rotOtherInAlignedSpace);

		if (depth2 < coll.m_Depth) {

			coll.m_Depth = depth2;
			if (depth2 < 0) return;
			coll.m_Object1 = aligned;
			coll.m_Object2 = other;
			coll.m_CollisionNormal(0) = sin(radBoxAligned);
			coll.m_CollisionNormal(1) = cos(radBoxAligned);
			if (coll.m_CollisionNormal.dot(center2center) < 0) coll.m_CollisionNormal *= -1.f;
			Vector2f widthAxisBox2(cos(radBoxOther), -sin(radBoxOther));
			Vector2f heightAxisBox2(sin(radBoxOther), cos(radBoxOther));
			coll.m_CollisionPoint2 = c2->Position();
			float direction = widthAxisBox2.dot(coll.m_CollisionNormal);
			if (direction > cEpsilon) {
				coll.m_CollisionPoint2 += widthAxisBox2 * c2->HalfSize()(0);
			} else if (direction < -cEpsilon) {
				coll.m_CollisionPoint2 -= widthAxisBox2 * c2->HalfSize()(0);
			}
			direction = heightAxisBox2.dot(coll.m_CollisionNormal);
			if (direction > cEpsilon) {
				coll.m_CollisionPoint2 += heightAxisBox2 * c2->HalfSize()(1);
			} else if (direction < -cEpsilon) {
				coll.m_CollisionPoint2 -= heightAxisBox2 * c2->HalfSize()(1);
			}
			coll.m_CollisionPoint1 = coll.m_CollisionPoint2 - (coll.m_Depth * coll.m_CollisionNormal);

		}

	};

	CollisionCalculation(c1, c2);
	CollisionCalculation(c2, c1);

	if (coll.m_Depth > 0 && !isinf(coll.m_Depth)) {
		m_Collisions.push_back(coll);
		return true;
	}

	return false;
}
