//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact ) {
	// TODO: Add Code
	Body * bodyA = contact.bodyA;
	Body * bodyB = contact.bodyB;
	
	const Vec3 ptOnA = contact.ptOnA_WorldSpace;
	const Vec3 ptOnB = contact.ptOnB_WorldSpace;

	const float elasticity = bodyA->m_elasticity * bodyB->m_elasticity;

	const float sumInvMass = bodyA->m_invMass + bodyB->m_invMass;
	// const float invMassA = bodyA->m_invMass;
	// const float invMassB = bodyB->m_invMass;

	const Mat3 invWorldInertiaA = bodyA->GetInverseInertiaTensorWorldSpace();
	const Mat3 invWorldInertiaB = bodyB->GetInverseInertiaTensorWorldSpace();

	const Vec3 & n = contact.normal;

	const Vec3 ra = ptOnA - bodyA->GetCenterOfMassWorldSpace();
	const Vec3 rb = ptOnB - bodyB->GetCenterOfMassWorldSpace();

	const Vec3 angularJA = (invWorldInertiaA * ra.Cross( n )).Cross( ra );
	const Vec3 angularJB = (invWorldInertiaB * rb.Cross( n )).Cross( rb );
	const float angularFactor = (angularJA + angularJB).Dot( n );

	// Get the world space velocity of the motion and rotation
	const Vec3 velA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross( ra );
	const Vec3 velB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross( rb );

	// Calculate the collision impulse
	const Vec3 vab = velA - velB;
	const float impulseJ = (1.0f + elasticity) * vab.Dot( n ) / (sumInvMass + angularFactor);
	const Vec3 vectorImpulseJ = n * impulseJ;

	// Calculate the impulse caused by friction

	const float friction = bodyA->m_friction * bodyB->m_friction;
	const Vec3 velTang = vab - n * n.Dot(vab);

	Vec3 relativeVelTang = velTang;
	relativeVelTang.Normalize();

	const Vec3 inertiaA = (invWorldInertiaA * ra.Cross(relativeVelTang)).Cross(ra);
	const Vec3 inertiaB = (invWorldInertiaB * rb.Cross(relativeVelTang)).Cross(rb);

	const Vec3 impulseFriction = velTang * friction / (sumInvMass + (inertiaA + inertiaB).Dot(velTang));

	bodyA->ApplyImpulse( ptOnA, (vectorImpulseJ + impulseFriction) * -1.0f);
	bodyB->ApplyImpulse( ptOnB, vectorImpulseJ + impulseFriction );

	if (0.0f == contact.timeOfImpact) {
		const Vec3 ds = ptOnB - ptOnA;
		bodyA->m_position += ds * bodyA->m_invMass / sumInvMass;
		bodyB->m_position -= ds * bodyB->m_invMass / sumInvMass;
	}
}