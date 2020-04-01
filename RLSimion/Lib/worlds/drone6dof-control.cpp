/*
	SimionZoo: A framework for online model-free Reinforcement Learning on continuous
	control problems

	Copyright (c) 2016 SimionSoft. https://github.com/simionsoft

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include "../app.h"
#include "../noise.h"

#include "drone6dof-control.h"
#include "Drone6DOF.h"
#include "BulletPhysics.h"
#include "BulletBody.h"
#include "aux-rewards.h"
#include "Box.h"


Drone6DOFControl::Drone6DOFControl(ConfigNode* pConfigNode)
{
	METADATA("World", "Drone-control");

	m_target_X = addStateVariable("target-x", "m", -20.0, 20.0);
	m_target_Y = addStateVariable("target-y", "m", -20.0, 20.0);
	m_target_Z = addStateVariable("target-z", "m", -20.0, 20.0);

	m_base_X = addStateVariable("drone-x", "m", -20.0, 20.0);
	m_base_Y = addStateVariable("drone-y", "m", -20.0, 20.0);
	m_base_Z = addStateVariable("drone-z", "m", -20.0, 20.0);

	m_rotacion_base_X = addStateVariable("rot-x", "rad", -8.0, 8.0);
	m_rotacion_base_Y = addStateVariable("rot-y", "rad", -8.0, 8.0);
	m_rotacion_base_Z = addStateVariable("rot-z", "rad", -8.0, 8.0);


	addActionVariable("fuerza1-1", "N", 0.0, 2.0);
	addActionVariable("fuerza1-2", "N", 0.0, 2.0);
	addActionVariable("fuerza1-3", "N", 0.0, 2.0);
	addActionVariable("fuerza1-4", "N", 0.0, 2.0);

	addActionVariable("fuerza2-1", "N", 0.0, 2.0);
	addActionVariable("fuerza2-2", "N", 0.0, 2.0);
	addActionVariable("fuerza2-3", "N", 0.0, 2.0);
	addActionVariable("fuerza2-4", "N", 0.0, 2.0);

	addActionVariable("fuerza3-1", "N", 0.0, 2.0);
	addActionVariable("fuerza3-2", "N", 0.0, 2.0);
	addActionVariable("fuerza3-3", "N", 0.0, 2.0);
	addActionVariable("fuerza3-4", "N", 0.0, 2.0);

	addActionVariable("fuerza4-1", "N", 0.0, 2.0);
	addActionVariable("fuerza4-2", "N", 0.0, 2.0);
	addActionVariable("fuerza4-3", "N", 0.0, 2.0);
	addActionVariable("fuerza4-4", "N", 0.0, 2.0);

	MASS_ROBOT = 0.5f;
	MASS_GROUND = 0.f;
	MASS_TARGET = 0.1f;

	m_pBulletPhysics = new BulletPhysics();
	m_pBulletPhysics->initPhysics();
	m_pBulletPhysics->initPlayground();
	
	/// Creating target point, kinematic
	{
		KinematicObject* pTarget = new KinematicObject(MASS_TARGET
			, btVector3(BulletPhysics::TargetX, 0, BulletPhysics::TargetY)
			, new btConeShape(btScalar(0.5), btScalar(0.001)));
		pTarget->setAbsoluteStateVarIds("target-x", "target-y");
		m_pBulletPhysics->add(pTarget);
	}

	///creating a DRONE  
	{
		Drone6DOF* pDrone = new Drone6DOF(m_pBulletPhysics, btVector3(0., 0., 0.));
		pDrone->setActionIds("fuerza1-1", "fuerza1-2", "fuerza1-3", "fuerza1-4",
			"fuerza2-1", "fuerza2-2", "fuerza2-3", "fuerza2-4",
			"fuerza3-1", "fuerza3-2", "fuerza2-3", "fuerza3-4",
			"fuerza4-1", "fuerza4-2", "fuerza4-3", "fuerza4-4");
		pDrone->setAbsoluteStateVarIds("drone-x", "drone-y", "drone-z", "rot-x", "rot-y", "rot-z");
		
	}


	//the reward function
	m_pRewardFunction->addRewardComponent(new DistanceReward2D(getStateDescriptor(),"robot1-x","robot1-y","target-x","target-y"));
	m_pRewardFunction->initialize();
}

void Drone6DOFControl::reset(State *s)
{
	m_pBulletPhysics->reset(s);
}

void Drone6DOFControl::executeAction(State *s, const Action *a, double dt)
{
	btTransform trans;
	m_pBulletPhysics->updateBulletState(s, a, dt);

	//Update Drone
	m_pBulletPhysics->stepSimulation((float)dt,20);

	//Update
	m_pBulletPhysics->updateState(s);
}

Drone6DOFControl::~Drone6DOFControl()
{
	delete m_pBulletPhysics;
}

