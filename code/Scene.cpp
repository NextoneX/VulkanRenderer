//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	body.m_position = Vec3(-3, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(100, 0, 0);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.0f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(0.5f);
	m_bodies.push_back(body);

	body.m_position = Vec3(0, 0, 3);
	//body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.0f;
	//body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(0.5f);
	m_bodies.push_back(body);

	//Ground
	body.m_position = Vec3(0, 0, -1000);
	//body.m_orientation = Quat(0, 0, 0, 1);
	//body.m_linearVelocity = Vec3(0, 0, 0);
	//body.m_invMass = 0.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeSphere(1000.0f);
	m_bodies.push_back(body);

	// TODO: Add code
}

int CompareContacts(const void* p1, const void* p2) {
	contact_t a = *(contact_t*)p1;
	contact_t b = *(contact_t*)p2;

	if (a.timeOfImpact < b.timeOfImpact)
		return -1;

	if (a.timeOfImpact == b.timeOfImpact)
		return 0;

	return 1;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	// TODO: Add code
	auto bodySize = m_bodies.size();
	for (int i = 0; i < bodySize; i++) {
		Body * body = &m_bodies[ i ];

		// gravity
		float mass = 1.0f / body->m_invMass;
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body->ApplyImpulseLinear( impulseGravity );
	}

	int numContacts = 0;
	const int maxContacts = bodySize * bodySize;
	// alloca在栈上连续分配空间
	contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts);
	for (int i = 0; i < bodySize; i++) {
		Body* bodyA = &m_bodies[i];
		for (int j = i++; j < bodySize; j++) {
			Body* bodyB = &m_bodies[j];

			if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass) {
				continue;
			}

			contact_t contact;
			if (Intersect(bodyA, bodyB, dt_sec, contact)) {
				contacts[numContacts] = contact;
				numContacts++;
			}
		}
	}

	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

	float accumulatedTime = 0.0f;
	// 按碰撞单步模拟
	for (int i = 0; i < numContacts; i++) {
		contact_t& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;

		Body* bodyA = contact.bodyA;
		Body* bodyB = contact.bodyB;

		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass) {
			continue;
		}

		for (int j = 0; j < bodySize; j++) {
			m_bodies[j].Update(dt);
		}

		ResolveContact(contact);
		accumulatedTime += dt;
	}

	const float timeRemaining = dt_sec - accumulatedTime;
	if(timeRemaining > 0.0f)
		for (int i = 0; i < bodySize; i++)
		{
			m_bodies[ i ].Update(timeRemaining);
		}
}