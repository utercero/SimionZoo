/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#include "Drone4.h"

//#define RIGID 1

Drone4::Drone4(btDynamicsWorld* ownerWorld, const btVector3& positionOffset,
	btScalar scale_ragdoll) : m_ownerWorld(ownerWorld)
{


	// Setup the geometry
	m_shapes[BODYPART_BASE] = new btBoxShape(btVector3(btScalar(3.0*scale_ragdoll), btScalar(0.01*scale_ragdoll), btScalar(scale_ragdoll*3.0)));
	m_shapes[BODYPART_LEFT_UPP] = new btBoxShape(btVector3(btScalar(0.5 * scale_ragdoll), btScalar(0.01 * scale_ragdoll), btScalar(scale_ragdoll * 0.5)));
	m_shapes[BODYPART_RIGHT_UPP] = new btBoxShape(btVector3(btScalar(0.5 * scale_ragdoll), btScalar(0.01 * scale_ragdoll), btScalar(scale_ragdoll * 0.5)));
	m_shapes[BODYPART_LEFT_LOW] = new btBoxShape(btVector3(btScalar(0.5 * scale_ragdoll), btScalar(0.01 * scale_ragdoll), btScalar(scale_ragdoll * 0.5)));
	m_shapes[BODYPART_RIGHT_LOW] = new btBoxShape(btVector3(btScalar(0.5 * scale_ragdoll), btScalar(0.01 * scale_ragdoll), btScalar(scale_ragdoll * 0.5)));

	// Setup all the rigid bodies
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	m_bodies[BODYPART_BASE] = localCreateRigidBody(btScalar(10.), offset*transform, m_shapes[BODYPART_BASE]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	m_bodies[BODYPART_LEFT_UPP] = localCreateRigidBody(btScalar(0.1), transform, m_shapes[BODYPART_LEFT_UPP]);
	//transform.setIdentity();

	//transform.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	m_bodies[BODYPART_LEFT_LOW] = localCreateRigidBody(btScalar(0.1), transform, m_shapes[BODYPART_LEFT_UPP]);

	//transform.setIdentity();
	//transform.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	m_bodies[BODYPART_RIGHT_UPP] = localCreateRigidBody(btScalar(0.1), transform, m_shapes[BODYPART_LEFT_UPP]);

	//transform.setIdentity();
	//transform.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	m_bodies[BODYPART_RIGHT_LOW] = localCreateRigidBody(btScalar(0.1), transform, m_shapes[BODYPART_LEFT_UPP]);







	///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
		// Now setup the constraints
		//btGeneric6DofConstraint * joint6DOF;
	btPoint2PointConstraint* joint;
	//btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
	/// ******* SPINE HEAD ******** ///
		/*
		{
			localA.setIdentity(); localB.setIdentity();

			localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30*scale_ragdoll), btScalar(0.)));

			localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14*scale_ragdoll), btScalar(0.)));

			joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB,useLinearReferenceFrameA);

	#ifdef RIGID
			joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
			joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
	#else
			joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
			joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.5f,SIMD_EPSILON,SIMD_PI*0.3f));
	#endif
			m_joints[JOINT_SPINE_HEAD] = joint6DOF;
			m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
		}*/
		/// *************************** ///






		/// *************************** ///


		/// ******* LEFT UP ******** ///

	{


		btVector3 posicionA = btVector3(btScalar(3 * scale_ragdoll), btScalar(0.0*scale_ragdoll), btScalar(3 * scale_ragdoll));
		btVector3 posicionB = btVector3(btScalar(-0.0 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(-0.0 * scale_ragdoll));
		joint = new btPoint2PointConstraint(*m_bodies[BODYPART_BASE], *m_bodies[BODYPART_LEFT_UPP], posicionA, posicionB);
		m_joints[JOINT_LEFT_UP] = joint;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_UP], true);
	}



	/// ******* LEFT DOWN ******** ///

	{
		btVector3 posicionA = btVector3(btScalar(-3 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(3 * scale_ragdoll));
		btVector3 posicionB = btVector3(btScalar(0.0 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(-0.0 * scale_ragdoll));
		joint = new btPoint2PointConstraint(*m_bodies[BODYPART_BASE], *m_bodies[BODYPART_LEFT_LOW], posicionA, posicionB);
		m_joints[JOINT_LEFT_DOWN] = joint;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_DOWN], true);
	}
	/// ******* RIGHT UP ******** ///

	{
		btVector3 posicionA = btVector3(btScalar(3 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(-3 * scale_ragdoll));
		btVector3 posicionB = btVector3(btScalar(-0.0 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(0.0 * scale_ragdoll));
		joint = new btPoint2PointConstraint(*m_bodies[BODYPART_BASE], *m_bodies[BODYPART_RIGHT_UPP], posicionA, posicionB);
		m_joints[JOINT_RIGHT_UP] = joint;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_UP], true);
	}
	/// ******* RIGHT DOWN ******** ///

	{
		btVector3 posicionA = btVector3(btScalar(-3 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(-3 * scale_ragdoll));
		btVector3 posicionB = btVector3(btScalar(0.0 * scale_ragdoll), btScalar(0.0 * scale_ragdoll), btScalar(0.0 * scale_ragdoll));
		joint = new btPoint2PointConstraint(*m_bodies[BODYPART_BASE], *m_bodies[BODYPART_RIGHT_LOW], posicionA, posicionB);
		m_joints[JOINT_RIGHT_DOWN] = joint;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_DOWN], true);
	}



}


Drone4::~Drone4()
{
	int i;

	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for (i = 0; i < BODYPART_COUNT; ++i)
	{
		m_ownerWorld->removeRigidBody(m_bodies[i]);

		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
}


btRigidBody* Drone4::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}

void Drone4::subir() {

	for (size_t i = 1; i < BODYPART_COUNT; i++)
	{
		btVector3 relativeForce = btVector3(0, 10, 0);
		btMatrix3x3& boxRot = m_bodies[i]->getWorldTransform().getBasis();
		//	btMatrix3x3& boxRot = m_bodies[0]->getWorldTransform().getBasis();

		btVector3 correctedForce = boxRot * relativeForce;

		m_bodies[i]->activate(true);

		m_bodies[i]->applyForce(correctedForce, btVector3(0.4, 0.0, 0.4));
		m_bodies[i]->applyForce(correctedForce, btVector3(-0.4, 0.0, 0.4));
		m_bodies[i]->applyForce(correctedForce, btVector3(0.4, 0.0, -0.4));
		m_bodies[i]->applyForce(correctedForce, btVector3(-0.4, 0.0, -0.4));
		//m_bodies[i]->applyForce()


	}

	/*{
		btVector3 relativeForce = btVector3(0, 20, 0);
		btMatrix3x3& boxRot = m_bodies[0]->getWorldTransform().getBasis();

		btVector3 correctedForce = boxRot * relativeForce;


		m_bodies[0]->activate(true);
		m_bodies[0]->applyImpulse(correctedForce, btVector3(2.8, 0.0, 2.8));
		m_bodies[0]->applyImpulse(correctedForce, btVector3(-2.8, 0.0, 2.8));
		m_bodies[0]->applyImpulse(correctedForce, btVector3(2.8, 0.0, -2.8));
		m_bodies[0]->applyImpulse(correctedForce, btVector3(-2.8, 0.0, -2.8));

	}*/
}

void Drone4::init()
{
	btMatrix3x3 original = m_bodies[0]->getWorldTransform().getBasis();
	for (size_t i = 1; i < BODYPART_COUNT; i++)
	{
		btTransform aux = m_bodies[i]->getWorldTransform();
		aux.setBasis(original);
		m_bodies[i]->setWorldTransform(aux);
	}
}
