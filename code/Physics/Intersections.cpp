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

	const Vec3 startPtA = posA;
	const Vec3 endPtA = posA + relativeVelocity * dt;
	const Vec3 rayDir = endPtA - startPtA;
	const float radiusA = shapeA->m_radius;
	const float radiusB = shapeB->m_radius;

	float t0 = 0;
	float t1 = 0;
	if (rayDir.GetLengthSqr() < 0.000001f) {
		Vec3 ab = posB - posA;
		float radius = radiusA + radiusB + 0.001f;
		if (ab.GetLengthSqr() > radius * radius) {
			return false;
		}
	}
	else if (!RaySphere(posA, rayDir, posB, radiusA + radiusB, t0, t1)) {
		return false;
	}

	t0 *= dt;
	t1 *= dt;

	if (t1 < 0.0f)
		return false;

	toi = (t0 < 0.0f) ? 0.0f : t0;

	if (toi > dt)
		return false;

	Vec3 newPosA = posA + velA * toi;
	Vec3 newPosB = posB + velB * toi;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * radiusA;
	ptOnB = newPosB - ab * radiusB;
	return true;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	// TODO: Add Code
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;
	// SphereSphere
	if ( bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE) {
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

		const Vec3 posA = bodyA->m_position;
		const Vec3 posB = bodyB->m_position;

		const Vec3 velA = bodyA->m_linearVelocity;
		const Vec3 velB = bodyB->m_linearVelocity;

		if ( SphereSphereDynamic( sphereA, sphereB, posA, posB, velA, velB, dt, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact.timeOfImpact)) {
			bodyA->Update(contact.timeOfImpact);
			bodyB->Update(contact.timeOfImpact);

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			contact.normal = bodyA->m_position - bodyB->m_position;
			contact.normal.Normalize();

			bodyA->Update(-contact.timeOfImpact);
			bodyB->Update(-contact.timeOfImpact);

			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			contact.separationDistance = r;
			return true;
		}
	}
	return false;
}






















