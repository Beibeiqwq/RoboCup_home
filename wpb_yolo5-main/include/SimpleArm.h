#pragma once
#include <math.h>
#include <cstdio>

class CSimpleArm
{
public:
    CSimpleArm();
    ~CSimpleArm();
	void SetLinkLen(double inLen_1, double inLen_2);
	bool SetTargetPos(float inX, float inZ);
	double JointAngle[3];

protected:
	double AngleFix(double inAngle);
	double m_fLinkLen[2];
};