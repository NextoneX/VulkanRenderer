//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact ) {
	// TODO: Add Code

	const Vec3 ab = bodyB->m_position - bodyA->m_position;

	const ShapeSphere * sphereA = (const ShapeSphere *)bodyA->m_shape;
	const ShapeSphere * sphereB = (const ShapeSphere *)bodyB->m_shape;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float distSquare = ab.GetLengthSqr();
	if( distSquare > radiusAB * radiusAB ) {
		return false;
	}

	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	contact.normal = ab;
	contact.normal.Normalize();

	contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
	contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphereB->m_radius;

	return true;
}

// d = rayDir = rayEnd - rayStart, a = rayStart, c = sphereCenter
// d^2t^2 + d.Dot(c - a)t + (c - a)^2 - R^2 = 0
bool RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float& sphereRadius,
	float& t1, float& t2) {
	const Vec3 dis = sphereCenter - rayStart;
	// for: at^2 + 2b^t + c = 0
	const float a = rayDir.Dot(rayDir);
	const float b = dis.Dot(rayDir);
	const float c = dis.Dot(dis) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	const float invA = 1.0f / a;
	
	if (delta < 0)
		return false;

	const float deltaRoot = sqrtf(delta);
	t1 = invA * (b - deltaRoot);
	t2 = invA * (b + deltaRoot);

	return true;
}

bool SphereSphereDynamic(const ShapeSphere* shapeA, const ShapeSphere* shapeB, const Vec3& posA, const Vec3& posB,
	const Vec3& velA, const Vec3& velB, const float& dt, Vec3& ptOnA, Vec3& ptOnB, float& toi) {
	const Vec3 relativeVelocity = velA - velB;

	return true;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	// TODO: Add Code

	return false;
}






















