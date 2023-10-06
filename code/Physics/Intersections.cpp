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

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	// TODO: Add Code

	return false;
}






















