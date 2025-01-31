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
	//body.m_position = Vec3(-3, 0, 3);
	//body.m_orientation = Quat(0, 0, 0, 1);
	//body.m_linearVelocity = Vec3(100, 0, 0);
	//body.m_invMass = 1.0f;
	//body.m_elasticity = 0.0f;
	//body.m_friction = 0.5f;
	//body.m_shape = new ShapeSphere(0.5f);
	//m_bodies.push_back(body);

	//body.m_position = Vec3(0, 0, 3);
	////body.m_orientation = Quat(0, 0, 0, 1);
	//body.m_linearVelocity = Vec3(0, 0, 0);
	//body.m_invMass = 0.0f;
	//body.m_elasticity = 0.0f;
	////body.m_friction = 0.5f;
	//body.m_shape = new ShapeSphere(0.5f);
	//m_bodies.push_back(body);

	////Ground
	//body.m_position = Vec3(0, 0, -1000);
	////body.m_orientation = Quat(0, 0, 0, 1);
	////body.m_linearVelocity = Vec3(0, 0, 0);
	////body.m_invMass = 0.0f;
	//body.m_elasticity = 1.0f;
	//body.m_friction = 0.0f;
	//body.m_shape = new ShapeSphere(1000.0f);
	//m_bodies.push_back(body);

	// TODO: Add code
	// Dynamic Bodies
	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {
			float radius = 0.5f;
			float xx = float(x - 1) * radius * 1.5f;
			float yy = float(y - 1) * radius * 1.5f;
			body.m_position = Vec3(xx, yy, 10.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}

	// Static "floor"
	for (int x = 0; x < 3; x++) {
		for (int y = 0; y < 3; y++) {
			float radius = 80.0f;
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}
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

	//
	// BroadPhase
	//
	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

	//
	// NarrowPhase
	//
	int numContacts = 0;
	const int collisionPairsSize = collisionPairs.size();
	// alloca在栈上连续分配空间
	contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * collisionPairsSize);
	for (int i = 0; i < collisionPairsSize; i++) {
		const collisionPair_t& pair = collisionPairs[i];
		Body* bodyA = &m_bodies[pair.a];
		Body* bodyB = &m_bodies[pair.b];

		// Skip body pairs with infinite mass
		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass) {
			continue;
		}

		contact_t contact;
		if (Intersect(bodyA, bodyB, dt_sec, contact)) {
			contacts[numContacts] = contact;
			numContacts++;
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