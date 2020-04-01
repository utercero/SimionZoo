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

#include "Drone6DOF.h"
#include "BulletPhysics.h"


//#define RIGID 1

Drone6DOF::Drone6DOF (BulletPhysics* physics, const btVector3& positionOffset) :BulletBody(2.,btVector3(),NULL)
{
	fisicas = physics;
	
	// Setup the geometry
	m_shapes[BODYPART_BASE] = new btBoxShape(btVector3(btScalar(3.0),btScalar( 0.1),btScalar(3.0)));
	m_shapes[BODYPART_LEFT_UPP] = new btBoxShape(btVector3(btScalar(0.5 ), btScalar(0.02 ), btScalar( 0.5)));
	m_shapes[BODYPART_RIGHT_UPP] = new btBoxShape(btVector3(btScalar(0.5 ), btScalar(0.02 ), btScalar( 0.5)));
	m_shapes[BODYPART_LEFT_LOW] = new btBoxShape(btVector3(btScalar(0.5 ), btScalar(0.02 ), btScalar( 0.5)));
	m_shapes[BODYPART_RIGHT_LOW] = new btBoxShape(btVector3(btScalar(0.5 ), btScalar(0.02 ), btScalar( 0.5)));
	m_shapes[UNION_LEFT_UPP] = new btConeShape(0.1, 0.5);
	m_shapes[UNION_LEFT_LOW] = new btConeShape(0.1, 0.5);
	m_shapes[UNION_RIGHT_UPP] = new btConeShape(0.1, 0.5);
	m_shapes[UNION_RIGHT_LOW] = new btConeShape(0.1, 0.5);

	btCompoundShape* base = new btCompoundShape();
	btTransform transformada;
	transformada.setIdentity();
	transformada.setOrigin(btVector3(0, 0.05, 0));
	base->addChildShape(transformada, m_shapes[BODYPART_BASE]);
	transformada.setIdentity();
	transformada.setOrigin(btVector3(2.8, 0.35, 2.8));
	base->addChildShape(transformada, m_shapes[UNION_RIGHT_UPP]);
	transformada.setIdentity();
	transformada.setOrigin(btVector3(-2.8, 0.35, 2.8));
	base->addChildShape(transformada, m_shapes[UNION_LEFT_UPP]);
	transformada.setIdentity();
	transformada.setOrigin(btVector3(-2.8, 0.35, -2.8));
	base->addChildShape(transformada, m_shapes[UNION_LEFT_LOW]);
	transformada.setIdentity();
	transformada.setOrigin(btVector3(2.8, 0.35, -2.8));
	base->addChildShape(transformada, m_shapes[UNION_RIGHT_LOW]);
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(positionOffset+btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	btRigidBody* rigidBase = localCreateRigidBody(btScalar(10.), transform, base);
	m_shapes[BODYPART_COMP] = base;
	m_bodies[BODYPART_BASE] = rigidBase;


	transform.setIdentity();
	transform.setOrigin(positionOffset + btVector3(-2.8, 0.61, 2.8));
	m_bodies[BODYPART_LEFT_UPP] = localCreateRigidBody(btScalar(0.5), transform, m_shapes[BODYPART_LEFT_UPP]);
	
	transform.setIdentity();
	transform.setOrigin(positionOffset + btVector3(-2.8, 0.61, -2.8));
	m_bodies[BODYPART_LEFT_LOW] = localCreateRigidBody(btScalar(0.5), transform, m_shapes[BODYPART_LEFT_LOW]);

	transform.setIdentity();
	transform.setOrigin(positionOffset + btVector3(2.8, 0.61, 2.8));
	m_bodies[BODYPART_RIGHT_UPP] = localCreateRigidBody(btScalar(0.5), transform, m_shapes[BODYPART_RIGHT_UPP]);

	transform.setIdentity();
	transform.setOrigin(positionOffset + btVector3(2.8, 0.61, -2.8));
	m_bodies[BODYPART_RIGHT_LOW] = localCreateRigidBody(btScalar(0.5), transform, m_shapes[BODYPART_RIGHT_LOW]);


///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
	
	btGeneric6DofSpringConstraint* joint;
	btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
/// ******* SPINE HEAD ******** ///
	
	{
		localA.setIdentity(); localB.setIdentity();
		localA.setOrigin(btVector3(btScalar(2.8), btScalar(0.6), btScalar(2.8)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.02), btScalar(0.)));
		joint = new btGeneric6DofSpringConstraint(*rigidBase, *m_bodies[BODYPART_RIGHT_UPP], localA, localB,useLinearReferenceFrameA);
		joint->setAngularLowerLimit(btVector3(-SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25));
		joint->setAngularUpperLimit(btVector3(SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25));
		joint->setLinearLowerLimit(btVector3(0., 0., 0.));
		joint->setLinearUpperLimit(btVector3(0., 0., 0.)); 
		m_joints[JOINT_RIGHT_UP] = joint;
		fisicas->getDynamicsWorld()->addConstraint(m_joints[JOINT_RIGHT_UP], true);


		localA.setIdentity(); localB.setIdentity();
		localA.setOrigin(btVector3(btScalar(-2.8), btScalar(0.6), btScalar(2.8)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.02), btScalar(0.)));
		joint = new btGeneric6DofSpringConstraint(*rigidBase, *m_bodies[BODYPART_LEFT_UPP], localA, localB, useLinearReferenceFrameA);
		joint->setAngularLowerLimit(btVector3(-SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25));
		joint->setAngularUpperLimit(btVector3(SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25));
		joint->setLinearLowerLimit(btVector3(0., 0., 0.));
		joint->setLinearUpperLimit(btVector3(0., 0., 0.));
		m_joints[JOINT_LEFT_UP] = joint;
		fisicas->getDynamicsWorld()->addConstraint(m_joints[JOINT_LEFT_UP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.setOrigin(btVector3(btScalar(-2.8), btScalar(0.6), btScalar(-2.8)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.02), btScalar(0.)));
		joint = new btGeneric6DofSpringConstraint(*rigidBase, *m_bodies[BODYPART_LEFT_LOW], localA, localB, useLinearReferenceFrameA);
		joint->setAngularLowerLimit(btVector3(-SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25));
		joint->setAngularUpperLimit(btVector3(SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25));
		joint->setLinearLowerLimit(btVector3(0., 0., 0.));
		joint->setLinearUpperLimit(btVector3(0., 0., 0.));
		m_joints[JOINT_LEFT_DOWN] = joint;
		fisicas->getDynamicsWorld()->addConstraint(m_joints[JOINT_LEFT_DOWN], true);

		localA.setIdentity(); localB.setIdentity();
		localA.setOrigin(btVector3(btScalar(2.8), btScalar(0.6), btScalar(-2.8)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.02), btScalar(0.)));
		joint = new btGeneric6DofSpringConstraint(*rigidBase, *m_bodies[BODYPART_RIGHT_LOW], localA, localB, useLinearReferenceFrameA);
		joint->setAngularLowerLimit(btVector3(-SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25, -SIMD_HALF_PI * 0.25));
		joint->setAngularUpperLimit(btVector3(SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25, SIMD_HALF_PI * 0.25));
		joint->setLinearLowerLimit(btVector3(0., 0., 0.));
		joint->setLinearUpperLimit(btVector3(0., 0., 0.));
		m_joints[JOINT_RIGHT_DOWN] = joint;
		fisicas->getDynamicsWorld()->addConstraint(m_joints[JOINT_RIGHT_DOWN], true);

	}
	
}

Drone6DOF::~Drone6DOF()
{
	int i;

	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		fisicas->getDynamicsWorld()->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for (i = 0; i < BODYPART_COUNT; ++i)
	{
		fisicas->getDynamicsWorld()->removeRigidBody(m_bodies[i]);

		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
	delete fuerzas;
}


btRigidBody* Drone6DOF::localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setActivationState(DISABLE_DEACTIVATION);
	fisicas->add(shape, body);
	return body;
}

void Drone6DOF::subir() {
	
	for (size_t i = 6; i < BODYPART_COUNT; i++)
	{
		btVector3 relativeForce = btVector3(0., 11., 0.);
		btMatrix3x3& boxRot = m_bodies[i]->getWorldTransform().getBasis();

		btVector3 correctedForce = boxRot * relativeForce;
		
		m_bodies[i]->activate(true);

		m_bodies[i]->applyForce(correctedForce, btVector3(0.3, 0.0, 0.3));
	
		m_bodies[i]->applyForce(correctedForce, btVector3(-0.3, 0.0, 0.3));
	
		m_bodies[i]->applyForce(correctedForce, btVector3(0.3, 0.0, -0.3));
		
		m_bodies[i]->applyForce(correctedForce, btVector3(-0.3, 0.0, -0.3));
	

		
	}

}

void Drone6DOF::init()
{
	btMatrix3x3 original = m_bodies[0]->getWorldTransform().getBasis();
	for (size_t i = 6; i < BODYPART_COUNT; i++)
	{
		btTransform aux = m_bodies[i]->getWorldTransform();
		aux.setBasis(original);
		m_bodies[i]->setWorldTransform(aux);
	}	
}

void Drone6DOF::updateBulletState(State * s, const Action * a, double dt)
{

	*fuerzas[F1_1] = a->get(m_f1_1Id);
	*fuerzas[F1_2] = a->get(m_f1_2Id);
	*fuerzas[F1_3] = a->get(m_f1_3Id);
	*fuerzas[F1_4] = a->get(m_f1_4Id);

	*fuerzas[F2_1] = a->get(m_f2_1Id);
	*fuerzas[F2_2] = a->get(m_f2_2Id);
	*fuerzas[F2_3] = a->get(m_f2_3Id);
	*fuerzas[F2_4] = a->get(m_f2_4Id);

	*fuerzas[F3_1] = a->get(m_f3_1Id);
	*fuerzas[F3_2] = a->get(m_f3_2Id);
	*fuerzas[F3_3] = a->get(m_f3_3Id);
	*fuerzas[F3_4] = a->get(m_f3_4Id);

	*fuerzas[F4_1] = a->get(m_f4_1Id);
	*fuerzas[F4_2] = a->get(m_f4_2Id);
	*fuerzas[F4_3] = a->get(m_f4_3Id);
	*fuerzas[F4_4] = a->get(m_f4_4Id);
	
	int j = 0;
	for (size_t i = 6; i < BODYPART_COUNT; i++)
	{
		btVector3 relativeForce = btVector3(0., 11., 0.);
		btMatrix3x3& boxRot = m_bodies[i]->getWorldTransform().getBasis();
		
		m_bodies[i]->activate(true);

		m_bodies[i]->applyForce(boxRot *(relativeForce**fuerzas[j]), btVector3(0.3, 0.0, 0.3));
		 
		j++;

		m_bodies[i]->applyForce(boxRot *(relativeForce**fuerzas[j]), btVector3(0.3, 0.0, 0.3));

		j++;

		m_bodies[i]->applyForce(boxRot *(relativeForce**fuerzas[j]), btVector3(0.3, 0.0, 0.3));

		j++;

		m_bodies[i]->applyForce(boxRot *(relativeForce**fuerzas[j]), btVector3(0.3, 0.0, 0.3));

		j++;


	}


}

void Drone6DOF::reset(State * s)
{
	//to-do
}
void Drone6DOF::updateState(State * s)
{
	//to-do
}


