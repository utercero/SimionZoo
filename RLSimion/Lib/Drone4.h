#pragma once
#include ""

class Drone4: public BulletBody
{
	enum
	{
		BODYPART_BASE = 0,
		BODYPART_LEFT_UPP,
		BODYPART_LEFT_LOW,
		BODYPART_RIGHT_UPP,
		BODYPART_RIGHT_LOW,
		BODYPART_COUNT
	};

	enum
	{
		JOINT_LEFT_UP = 0,
		JOINT_LEFT_DOWN,
		JOINT_RIGHT_UP,
		JOINT_RIGHT_DOWN,
		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];


	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

public:
	Drone4(btDynamicsWorld* ownerWorld,
		const btVector3& positionOffset,
		btScalar scale_ragdoll = btScalar(1.0));
	void subir();
	void init();
	~Drone4();
};




