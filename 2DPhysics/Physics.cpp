#include "Physics.hpp"

Physics::Physics(std::vector<Object*>& objects) : m_Objects(objects) {
}

void Physics::Update(Float timeDelta) {

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

		Float sumInvMass = obj1->GetInvertedMass() + obj2->GetInvertedMass();
		Float massFactor1 = obj1->GetInvertedMass() / sumInvMass;
		Float massFactor2 = obj2->GetInvertedMass() / sumInvMass;

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
	ApplyImpulseRotation();
}

void Physics::ApplyImpulseSimple() {

	for (auto& c : m_Collisions) {

		Object* obj1 = c.m_Object1;
		Object* obj2 = c.m_Object2;

		Float sumInvMass = obj1->GetInvertedMass() + obj2->GetInvertedMass();
		Float massFactor1 = obj1->GetInvertedMass() / sumInvMass;
		Float massFactor2 = obj2->GetInvertedMass() / sumInvMass;

		// Apply Impulse (without rotation / friction)

		Vector2 velocityParallel1 = obj1->Velocity().dot(c.m_CollisionNormal) * c.m_CollisionNormal;
		Vector2 velocityParallel2 = obj2->Velocity().dot(c.m_CollisionNormal) * c.m_CollisionNormal;
		Vector2 closingVelocity = velocityParallel2 - velocityParallel1;

		// TODO: coefficient of resitution = 30%
		Float coeff = 1.f + 0.3f;

		obj1->Velocity() += massFactor1 * closingVelocity * coeff;
		obj2->Velocity() -= massFactor2 * closingVelocity * coeff;

	}
}

void Physics::ApplyImpulseRotation() {

	for (auto& c : m_Collisions) {

		Object* obj1 = c.m_Object1;
		Object* obj2 = c.m_Object2;

		Float sumInvMass = obj1->GetInvertedMass() + obj2->GetInvertedMass();
		Float massFactor1 = obj1->GetInvertedMass() / sumInvMass;
		Float massFactor2 = obj2->GetInvertedMass() / sumInvMass;

		// Apply Impulse (without rotation / friction)

		// WORK IN PROGRESS
		// here is some information about impulse with rotation

		// leverArm is the vector from CenterOfGravity (= position) to CollisionPoint
		// rotating the leverArm 90� CW (negative angularVelocity CCW) gives the direction of the velocity component of the angularVelocity
		// leverArmCW.norm * 2 * deg2rad(angVel) gives the absoulute velocity component of the angularVelocity.
		//   Example:
		//     leverArm is 5 units long and does one rotation in a 2 seconds (angVel == 180�)
		//     a circle with 5 units radius has a circumference of 2*pi*5
		//     that distance was covered in 2 seconds -> 2*pi*5 / 2 == pi*5 == deg2rad(angularVelocity) * 5
		// difference between velocities (mapped to normal) at the collisionPoints gives the closingVelocity
		// components perpendicular to the normals are left out
		// objects are affected by the proportion of the inverseMass
		// 2 physical laws help us solve the system:
		//   conservation of momentum: v1*m1 + v2*m2 == v1'*m1 + v2'*m2
		//   conservation of energy: 0.5*v1�*m1 + 0.5*v2�*m2 == 0.5*v1'�*m1+0.5*v2'�*m2
		// Cr is the coefficient of restitution and tells us how much velocity energy (infact velocity) is absorbed by the objects (as heat, deformation, ...)
		// Cr 100% (1 + 1.0f) means 100% of velocity is retained in the collision (elastic)
		// Cr 40% (1 + 0.4f): 40% of the velocity is retained als velocity
		// Cr 0% (1 + 0.f): 0% velocity is retained, both objects completely stop
		// therefore the velocity in the formulas above should use the seperatingVelocity
		// where seperatingVelocity = Cr * closingVelocity

		// Now the interesting part: change in angularVelocity:
		// a 'tensor' (rotationalInverseInertia) gives information about how the object has its mass distributed:
		// mass close to the centerOfGravity means it is easier to rotate, less force required
		// mass spread far means the rotation contains a lot more energy (think of spinning and pulling your arms out and in. you speed down and speed up!)
		// some diabolical mumbojumbo with the tensor and you know the change in transitionalVelocity and angularVelocity :^)

		Vector2 temp = c.m_Object1->Position() - c.m_CollisionPoint1;
		Vector2 leverArm1 = {-temp[1], temp[0]};
		Vector2 velocityPoint1 = obj1->Velocity() + leverArm1 * deg2rad(obj1->AngularVelocity());

		temp = c.m_Object2->Position() - c.m_CollisionPoint2;
		Vector2 leverArm2 = {-temp[1], temp[0]};
		Vector2 velocityPoint2 = obj2->Velocity() + leverArm2 * deg2rad(obj2->AngularVelocity());

		Vector2 velocityParallel1 = velocityPoint1.dot(c.m_CollisionNormal) * c.m_CollisionNormal;
		Vector2 velocityParallel2 = velocityPoint2.dot(c.m_CollisionNormal) * c.m_CollisionNormal;

		Vector2 closingVelocity = velocityParallel2 - velocityParallel1;
		Float cr = 1 + 0.7f; // TODO: cr usually is a property of the colliding materials
		Vector2 seperatingVelocity = closingVelocity * cr;

		Float numerator = seperatingVelocity.dot(c.m_CollisionNormal);
		Float denumerator = c.m_CollisionNormal.squaredNorm() * sumInvMass;
		Float denum1 = pow(c.m_CollisionNormal.dot(leverArm1), 2) / obj1->MomentOfInertia();
		Float denum2 = pow(c.m_CollisionNormal.dot(leverArm2), 2) / obj2->MomentOfInertia();
		Float factorJ = numerator / (denumerator + denum1 + denum2);

		Float newAngular1 = obj1->AngularVelocity() + rad2deg(factorJ * c.m_CollisionNormal.dot(leverArm1) / obj1->MomentOfInertia());
		Float newAngular2 = obj2->AngularVelocity() - rad2deg(factorJ * c.m_CollisionNormal.dot(leverArm2) / obj2->MomentOfInertia());

		Vector2 newTrans1 = obj1->Velocity() + factorJ * obj1->GetInvertedMass() * c.m_CollisionNormal;
		Vector2 newTrans2 = obj2->Velocity() - factorJ * obj2->GetInvertedMass() * c.m_CollisionNormal;

		obj1->Velocity() = newTrans1;
		obj2->Velocity() = newTrans2;
		obj1->AngularVelocity() = newAngular1;
		obj2->AngularVelocity() = newAngular2;


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

bool Physics::TestCollisionCircleCircle(Circle* c1, Circle* c2) {
	Vector2 diff = c2->Position() - c1->Position();
	Float depth = (c1->Radius() + c2->Radius()) - diff.norm();
	if (depth > 0) {
		diff.normalize();
		Vector2 p1 = c1->Position() + diff * c1->Radius();
		Vector2 p2 = c2->Position() - diff * c2->Radius();
		m_Collisions.push_back(Collision(c1, c2, p1, p2, diff, depth));
		return true;
	}
	return false;
}

bool Physics::TestCollisionCircleBox(Circle* c1, Box* c2, bool swapped) {

	// Boundary Test
	Float maxBox1 = c1->Radius();
	Float maxBox2 = c2->HalfSize().maxCoeff();
	Float distance = (c2->Position() - c1->Position()).norm();
	if (distance > maxBox1 + maxBox2) return false;


	Matrix2 rot;
	Float radBoxAligned = deg2rad(c2->Angle());
	rot(0, 0) = cos(radBoxAligned);
	rot(0, 1) = -sin(radBoxAligned);
	rot(1, 0) = sin(radBoxAligned);
	rot(1, 1) = cos(radBoxAligned);
	Matrix2 invRot = rot.inverse();

	Vector2 sPos = c1->Position();
	sPos -= c2->Position();
	sPos = rot * sPos;

	Vector2 cBox = sPos;
	Float xsize = c2->HalfSize()(0);
	Float ysize = c2->HalfSize()(1);
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

	Vector2 box2circle = sPos - cBox;
	Float depth = c1->Radius() - box2circle.norm();

	if (depth > 0) {
		box2circle.normalize();
		Vector2 cSp = c1->Position() - box2circle * c1->Radius();
		if (swapped) {
			m_Collisions.push_back(Collision(c1, c2, cSp, cBox, -box2circle, depth));
		} else {
			m_Collisions.push_back(Collision(c2, c1, cBox, cSp, box2circle, depth));
		}
		return true;
	}

	return false;

}

bool Physics::TestCollisionBoxBox(Box* c1, Box* c2) {

	// BoundaryTest
	Float maxBox1 = c1->HalfSize().norm();
	Float maxBox2 = c2->HalfSize().norm();
	Float distance = (c2->Position() - c1->Position()).norm();
	if (distance > maxBox1 + maxBox2) return false;

	// Potential collision, initialized with infinite depth.
	Collision coll(c1, c2, Vector2(), Vector2(), Vector2(), std::numeric_limits<Float>::infinity());

	// angle in radians of both objects
	Float angle1 = deg2rad(c1->Angle());
	Float angle2 = deg2rad(c2->Angle());

	Float cosAngle1, sinAngle1, cosAngle2, sinAngle2;
	sincospi(c1->Angle() / 180., sinAngle1, cosAngle1);
	sincospi(c2->Angle() / 180., sinAngle2, cosAngle2);

	//cosAngle1 = cos(angle1);
	//sinAngle1 = sin(angle1);
	//cosAngle2 = cos(angle2);
	//sinAngle2 = sin(angle2);

	// normals of (width-direction, height-direction) of objects 1 and 2
	Vector2 nw1 = {cosAngle1, -sinAngle1};
	Vector2 nh1 = {sinAngle1, cosAngle1};
	Vector2 nw2 = {cosAngle2, -sinAngle2};
	Vector2 nh2 = {sinAngle2, cosAngle2};

	// vectors from object center to face
	Vector2 w1 = c1->HalfSize()[0] * nw1;
	Vector2 h1 = c1->HalfSize()[1] * nh1;
	Vector2 w2 = c2->HalfSize()[0] * nw2;
	Vector2 h2 = c2->HalfSize()[1] * nh2;

	// center2center vector from obj 1 to obj 2
	Vector2 c2c = c2->Position() - c1->Position();

	// for each normal:
	//     we map w1,h2,w2,h2 summed together then substract mapped c2c vector
	//     since normal is taken from one box, we don't need to map w1 and h1
	//     if result is negative, then objects do not intersect
	//     if positive, result is the distance the objects intersect
	// since projection is performed the same, we use this helper function:

	auto projectionHelper = [&] (Vector2& normal, Float boxSize, Vector2& w2, Vector2& h2, Vector2 c2c, Box* obj1, Box* obj2) {
		Float depth = abs(boxSize) + abs(w2.dot(normal)) + abs(h2.dot(normal));
		depth -= abs(c2c.dot(normal));
		return depth;
	};

	// code looks unnessecary bloated, but should lead to better performance <.<

	int collisionCase = 1;
	coll.m_Depth = projectionHelper(nw1, c1->HalfSize()[0], w2, h2, c2c, c1, c2);
	if (coll.m_Depth < 0) {
		return false;
	}

	Float newDepth = projectionHelper(nh1, c1->HalfSize()[1], w2, h2, c2c, c1, c2);
	if (newDepth < 0) {
		return false;
	} else if (newDepth < coll.m_Depth) {
		coll.m_Depth = newDepth;
		collisionCase = 2;
	}

	newDepth = projectionHelper(nw2, c2->HalfSize()[0], w1, h1, -c2c, c2, c1);
	if (newDepth < 0) {
		return false;
	} else if (newDepth < coll.m_Depth) {
		coll.m_Depth = newDepth;
		collisionCase = 3;
	}

	newDepth = projectionHelper(nh2, c2->HalfSize()[1], w1, h1, -c2c, c2, c1);
	if (newDepth < 0) {
		return false;
	} else if (newDepth < coll.m_Depth) {
		coll.m_Depth = newDepth;
		collisionCase = 4;
	}

	// collisionNormal goes from Obj1 -> Obj2
	// penetrating vertex is from Obj2
	// first calculate CollisionPoint2 (vertex): position + vertexVector (dot product with normal < 0)
	//   if dot product == 0 then ignore this direction, CP2 is in the center of the face
	// then CollisionPoint1: CP2 + normal * depth

	Vector2 rotWidth;
	Vector2 rotHeight;

	switch (collisionCase) {
		case 1: // normal horizontal (width) of box 1
			coll.m_CollisionNormal = nw1;
			rotWidth = w2;
			rotHeight = h2;
			coll.m_Object1 = c1;
			coll.m_Object2 = c2;
			if (coll.m_CollisionNormal.dot(c2c) < 0) coll.m_CollisionNormal = -coll.m_CollisionNormal;
			break;

		case 2: // normal vertial (height) of box 1
			coll.m_CollisionNormal = nh1;
			rotWidth = w2;
			rotHeight = h2;
			coll.m_Object1 = c1;
			coll.m_Object2 = c2;
			if (coll.m_CollisionNormal.dot(c2c) < 0) coll.m_CollisionNormal = -coll.m_CollisionNormal;
			break;

		case 3: // normal horizontal (width) of box 2
			coll.m_CollisionNormal = nw2;
			rotWidth = w1;
			rotHeight = h1;
			coll.m_Object1 = c2;
			coll.m_Object2 = c1;
			if (coll.m_CollisionNormal.dot(c2c) > 0) coll.m_CollisionNormal = -coll.m_CollisionNormal;
			break;

		case 4: // normal vertical (height) of box 2
			coll.m_CollisionNormal = nh2;
			rotWidth = w1;
			rotHeight = h1;
			coll.m_Object1 = c2;
			coll.m_Object2 = c1;
			if (coll.m_CollisionNormal.dot(c2c) > 0) coll.m_CollisionNormal = -coll.m_CollisionNormal;
			break;
	}

	coll.m_CollisionPoint2 = coll.m_Object2->Position();

	if ((coll.m_CollisionNormal.dot(rotWidth)) < -cEpsilon) {
		coll.m_CollisionPoint2 += rotWidth;
	} else if ((coll.m_CollisionNormal.dot(rotWidth)) > cEpsilon) {
		coll.m_CollisionPoint2 -= rotWidth;
	}

	if ((coll.m_CollisionNormal.dot(rotHeight)) < -cEpsilon) {
		coll.m_CollisionPoint2 += rotHeight;
	} else if ((coll.m_CollisionNormal.dot(rotHeight)) > cEpsilon) {
		coll.m_CollisionPoint2 -= rotHeight;
	}

	coll.m_CollisionPoint1 = coll.m_CollisionPoint2 + coll.m_CollisionNormal * coll.m_Depth;

	m_Collisions.push_back(coll);
	return true;
}
