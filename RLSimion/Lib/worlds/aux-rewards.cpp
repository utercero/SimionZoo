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

#include "aux-rewards.h"
#include "../../Common/named-var-set.h"
#include "world.h"
#include <algorithm>
#include <math.h>

double static getDistanceBetweenPoints(double x1, double y1, double x2, double y2)
{
	double distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
	return distance;
}

DistanceReward2D::DistanceReward2D(Descriptor& stateDescr, const char* var1xName, const char* var1yName, const char* var2xName, const char* var2yName)
{
	m_var1xId = var1xName;
	m_var1yId = var1yName;
	m_var2xId = var2xName;
	m_var2yId = var2yName;

	//here we assume both variables have the same value range
	m_maxDist = sqrt(stateDescr.getProperties(m_var1xId)->getRangeWidth()
		* stateDescr.getProperties(m_var1xId)->getRangeWidth()
		+ stateDescr.getProperties(m_var1yId)->getRangeWidth()
		* stateDescr.getProperties(m_var1yId)->getRangeWidth());
}

double DistanceReward2D::getReward(const State* s, const Action* a, const State* s_p)
{
	double boxX = s_p->get(m_var1xId);
	double boxY = s_p->get(m_var1yId);
	double targetX = s_p->get(m_var2xId);
	double targetY = s_p->get(m_var2yId);

	double distance = getDistanceBetweenPoints(targetX, targetY, boxX, boxY);

	distance = std::max(distance, 0.0001);

	double reward = 1. - distance / m_maxDist;

	return reward;

}

double DistanceReward2D::getMin()
{
	return 0.0;
}

double DistanceReward2D::getMax()
{
	return 1.0;
}

DistanceReward3D::DistanceReward3D(Descriptor & stateDescr, const char * var1xName, const char * var1yName, const char * var1zName, const char * var1rotxName, const char * var1rotyName, const char* var1vlinearyName, const char * var2xName, const char * var2yName, const char * errorName)
{
	m_error = errorName;

	m_var1xId = var1xName;
	m_var1yId = var1yName;
	m_var1zId = var1zName;
	m_var1rotxId = var1rotxName;
	m_var1rotyId = var1rotyName;
	m_var1vlinearId = var1vlinearyName;
	m_var2xId = var2xName;
	m_var2yId = var2yName;

	//here we assume both variables have the same value range. We do not use z, because before the drone must fly until it starts taking land
	m_maxDist = sqrt(stateDescr.getProperties(m_var1xId)->getRangeWidth()
		* stateDescr.getProperties(m_var1xId)->getRangeWidth()
		+ stateDescr.getProperties(m_var1yId)->getRangeWidth()
		* stateDescr.getProperties(m_var1yId)->getRangeWidth());
}

double DistanceReward3D::getReward(const State * s, const Action * a, const State * s_p)
{
	double error = s_p->get(m_error);
	return 1 - (abs(error) / Drone6DOF::altura);

	/*
	//comprobar si la rotacion en x e y es acceptable
	double rotX = s_p->get(m_var1rotxId);
	double rotY = s_p->get(m_var1rotyId);
	if (abs(rotX) > maxRot || abs(rotY) > maxRot)
		return getMin();
	double descuentoRot = (abs(rotX) + abs(rotY))*factorRot;
	double droneX = s_p->get(m_var1xId);
	double droneY = s_p->get(m_var1yId);
	double droneZ = s_p->get(m_var1zId);
	double targetX = s_p->get(m_var2xId);
	double targetY = s_p->get(m_var2yId);
	double droneVY = s_p->get(m_var1vlinearId);
	double aterriza = 1.0;
	double distance = getDistanceBetweenPoints(targetX, targetY, droneX, droneY);
	if (distance*aterriza <= 0.0001)
	{
		aterriza = 5.0;
		//hay que bajar sin alejarse del punto
		distance += (droneZ / alturaVuelo) + descuentoRot;
	}
	else {
		if (abs(droneZ-alturaVuelo)<0.0001)
		{
			//hay que volar
			//se le añade a la distance un factor que penaliza la rotación del drone
			distance += descuentoRot;
		}
		else 
		{
			if (droneZ<alturaVuelo && droneVY>=0)
			{
				//el drone está despegando
				distance = -droneZ / alturaVuelo + descuentoRot;
			}
			else
			{
				//el drone está volando y ha perdido o ganado altura
				distance += descuentoRot + abs(droneZ - alturaVuelo)*factorAltura;
			}
		}
	}

	distance = std::max(distance, 0.0001);
	double reward = 1. - distance / m_maxDist;
	reward = std::max(reward, getMin());
	return reward;
	*/
}

double DistanceReward3D::getMin()
{
	return 0.0;
}

double DistanceReward3D::getMax()
{
	return 1.0;
}
