#include "stdafx.h"
#include "actor.h"
#include "vfa.h"
#include "vfa-actor.h"
#include "states-and-actions.h"
#include "features.h"
#include "parameter.h"
#include "parameters.h"

CCACLAActor::CCACLAActor(CParameters *pParameters)
: CSingleOutputPolicyLearner(pParameters)
{
	m_e = new CETraces(pParameters->getChild("ETRACES"));
}

CCACLAActor::~CCACLAActor()
{
	delete m_e;
}

void CCACLAActor::updatePolicy(CState *s,CAction *a,CState *s_p,double r,double td)
{
	double lastNoise;
	double alpha;
	const char* actionVar;
	int actionIndex;

	//CACLA (van Hasselt)
	//if delta>0: theta= theta + alpha*(lastNoise)*phi_pi(s)


	alpha = m_pParameters->getParameter("ALPHA")->getDouble();
	actionVar = m_pParameters->getParameter("ACTION")->getStringPtr();

	lastNoise = a->getValue(actionVar) - m_pVFA->getValue(s, a);// m_pOutput->getValue(i);

	m_pVFA->getFeatures(s,a,m_pStateFeatures);

	if (alpha != 0.0)
		m_pVFA->add(m_pStateFeatures,alpha*lastNoise);
}

