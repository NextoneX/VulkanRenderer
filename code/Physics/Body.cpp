//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_linearVelocity( 0.0f, 0.0f, 0.0f),
m_invMass( 0.0f ),
m_elasticity( 0.0f ),
m_shape( NULL ) {
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
    return m_position + m_orientation.RotatePoint( m_shape->GetCenterOfMass());
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
    return m_shape->GetCenterOfMass();
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const
{
    Vec3 temp = worldPt - GetCenterOfMassModelSpace();
    Quat inverseOrient = m_orientation.Inverse();
    Vec3 bodySpace = inverseOrient.RotatePoint(temp);
    return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& bodyPt) const
{
    return GetCenterOfMassModelSpace() + m_orientation.RotatePoint(bodyPt);
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const {
	Mat3 inertiaTensor		= m_shape->InertiaTensor();
	Mat3 invInertiaTensor	= inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const {
	Mat3 inertiaTensor		= m_shape->InertiaTensor();
	Mat3 invInertiaTensor	= inertiaTensor.Inverse() * m_invMass;
	Mat3 orient				= m_orientation.ToMat3();
	invInertiaTensor		= orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}

void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (0.0f == m_invMass) {
		return;
	}

	ApplyImpulseLinear(impulse);

	Vec3 r = impulsePoint - GetCenterOfMassWorldSpace();
	Vec3 dL = r.Cross(impulse);// in world space
	ApplyImpulseAngular(dL);
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (0.0f == m_invMass) {
		return;
	}

	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (0.0f == m_invMass) {
		return;
	}

	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float maxAngularVelocity = 30.0f;
	if (m_angularVelocity.GetLengthSqr() > maxAngularVelocity * maxAngularVelocity) {
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularVelocity;
	}
}

void Body::Update(const float dt_sec)
{
	m_position += m_linearVelocity * dt_sec;

    Vec3 positionCM = GetCenterOfMassWorldSpace();
	Vec3 cmToPos = m_position - positionCM;

	Mat3 orientation = m_orientation.ToMat3();
	Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
    Vec3 alpha = inertiaTensor.Inverse() * (m_angularVelocity.Cross(
        inertiaTensor * m_angularVelocity));
    m_angularVelocity += alpha * dt_sec;

    Vec3 deltaTheta = m_angularVelocity * dt_sec;
    Quat deltaOrient = Quat(deltaTheta, deltaTheta.GetMagnitude());
    m_orientation = deltaOrient * m_orientation;
    m_orientation.Normalize();

    m_position = positionCM + deltaOrient.RotatePoint(cmToPos);
}
