#pragma once

#include "world.h"
#include "../../../3rd-party/bullet3-2.86/src/btBulletDynamicsCommon.h"

class Robot;
class BulletBody;
class BulletPhysics;

//Move box with 2 robots
class Drone6DOFControl : public DynamicModel
{
	/// All-Simulation variables
	double MASS_TARGET;
	double MASS_ROBOT;
	double MASS_GROUND;

	/// State variables
	size_t m_target_X, m_target_Y, m_target_Z;
	size_t m_base_X, m_base_Y,m_base_Z;
	size_t m_rotacion_base_X, m_rotacion_base_Y, m_rotacion_base_Z;

	// Action variables
	size_t m_fuerza_1_1;
	size_t m_fuerza_1_2;
	size_t m_fuerza_1_3;
	size_t m_fuerza_1_4;
	size_t m_fuerza_2_1;
	size_t m_fuerza_2_2;
	size_t m_fuerza_2_3;
	size_t m_fuerza_2_4;
	size_t m_fuerza_3_1;
	size_t m_fuerza_3_2;
	size_t m_fuerza_3_3;
	size_t m_fuerza_3_4;
	size_t m_fuerza_4_1;
	size_t m_fuerza_4_2;
	size_t m_fuerza_4_3;
	size_t m_fuerza_4_4;


	///inicializaci�n
	BulletPhysics* m_pBulletPhysics;

public:
	Drone6DOFControl(ConfigNode* pParameters);
	virtual ~Drone6DOFControl();
	void reset(State *s);
	void executeAction(State *s, const Action *a, double dt);

};

