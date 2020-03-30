#pragma once
#include "BulletBody.h"





class Drone6DOF:BulletBody
{
	



	enum
	{
		BODYPART_BASE = 0,
		UNION_LEFT_UPP,
		UNION_LEFT_LOW,
		UNION_RIGHT_UPP,
		UNION_RIGHT_LOW,
		BODYPART_COMP,
		BODYPART_LEFT_UPP,
		BODYPART_LEFT_LOW,
		BODYPART_RIGHT_UPP,
		BODYPART_RIGHT_LOW,
		BODYPART_COUNT
	};

	enum
	{
		F1_1 = 0,
		F1_2,
		F1_3,
		F1_4,
		F2_1,
		F2_2,
		F2_3,
		F2_4,
		F3_1,
		F3_2,
		F3_3,
		F3_4,
		F4_1,
		F4_2,
		F4_3,
		F4_4,
		FORCE_COUNT
	};


	enum
	{
		JOINT_LEFT_UP=0,
		JOINT_LEFT_DOWN,
		JOINT_RIGHT_UP,
		JOINT_RIGHT_DOWN,
		JOINT_COUNT
	};
	
	const char* m_f1_1Id;
	const char* m_f1_2Id;
	const char* m_f1_3Id;
	const char* m_f1_4Id;
	const char* m_f2_1Id;
	const char* m_f2_2Id;
	const char* m_f2_3Id;
	const char* m_f2_4Id;
	const char* m_f3_1Id;
	const char* m_f3_2Id;
	const char* m_f3_3Id;
	const char* m_f3_4Id;
	const char* m_f4_1Id;
	const char* m_f4_2Id;
	const char* m_f4_3Id;
	const char* m_f4_4Id;
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btGeneric6DofSpringConstraint* m_joints[JOINT_COUNT];
	double* fuerzas[FORCE_COUNT];


	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

public:
	Drone6DOF (btDynamicsWorld* ownerWorld,
				const btVector3& positionOffset);
	void subir();
	void init();
	void setActionIds(const char* f11, const char* f12, const char* f13, const char* f14,
		const char* f21, const char* f22, const char* f23, const char* f24, 
		const char* f31, const char* f32, const char* f33, const char* f34, 
		const char* f41, const char* f42, const char* f43, const char* f44)
	{ 
		m_f1_1Id = f11;
		m_f1_2Id = f12;
		m_f1_3Id = f13;
		m_f1_4Id = f14;

		m_f2_1Id = f11;
		m_f2_2Id = f12;
		m_f2_3Id = f13;
		m_f2_4Id = f14;

		m_f3_1Id = f11;
		m_f3_2Id = f12;
		m_f3_3Id = f13;
		m_f3_4Id = f14;

		m_f4_1Id = f11;
		m_f4_2Id = f12;
		m_f4_3Id = f13;
		m_f4_4Id = f14;
		
	}
	void updateBulletState(State* s, const Action* a, double dt);
	~Drone6DOF ();
};




