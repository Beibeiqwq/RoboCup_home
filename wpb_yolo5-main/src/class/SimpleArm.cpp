#include "SimpleArm.h"

static double kAngDeg = 180/M_PI;

CSimpleArm::CSimpleArm()
{
    m_fLinkLen[0] = 0.2445f;
    m_fLinkLen[1] = 0.274f;
}

CSimpleArm::~CSimpleArm()
{
}

void CSimpleArm::SetLinkLen(double inLen_1, double inLen_2)
{
    m_fLinkLen[0] = inLen_1;
    m_fLinkLen[1] = inLen_2;
}

double CSimpleArm::AngleFix(double inAngle)
{
    double retAngle = inAngle;
    while(retAngle > M_PI)
        retAngle -= 2*M_PI;
    while(retAngle < -1*M_PI)
        retAngle += 2*M_PI;
    return retAngle;
}

bool CSimpleArm::SetTargetPos(float inX, float inZ)
{
    if(inZ != inX)
    {
        // 第一个关节角度
        double a = atan(inZ / inX);
        double d = sqrt(inX*inX + inZ*inZ);
        if(m_fLinkLen[0] *d == 0)
            return false;
        double b = acos((m_fLinkLen[0]*m_fLinkLen[0]  + (inX*inX + inZ*inZ) - m_fLinkLen[1]*m_fLinkLen[1]) / (2*m_fLinkLen[0]*d));
        double angle_0 = M_PI/2 - (a+b);
        if(isnan(angle_0) > 0)
            return false;
        angle_0 = AngleFix(angle_0);
        JointAngle[0] = angle_0 * -1 * kAngDeg;

        // 第二个关节角度
        double c = asin((inZ - m_fLinkLen[0] *sin(a+b)) / (m_fLinkLen[1]));
        double angle_1 = (a+b) - c;
        if(isnan(angle_1) > 0)
            return false;
        angle_1 = AngleFix(angle_1);
        JointAngle[1] = angle_1 * kAngDeg;

        // 第三个关节角度
        if(isnan(angle_1) > 0)
            return false;
        double angle_2 = c;
        angle_2 = AngleFix(angle_2);
        JointAngle[2] = angle_2 * kAngDeg;
    }
    else
    {
        if(inZ == 0)
            return false;
        // 第一个关节角度
        double a = acos((m_fLinkLen[0]*m_fLinkLen[0]  + inZ*inZ - m_fLinkLen[1]*m_fLinkLen[1]) / (2*m_fLinkLen[0]*inZ));
        if(isnan(a) > 0)
            return false;
        double angle_0 = AngleFix(a);
        JointAngle[0] = angle_0 * -1 * kAngDeg;

        // 第二个关节角度
        double b = asin((inZ - m_fLinkLen[0]*cos(a)) / m_fLinkLen[1]);
        if(isnan(b) > 0)
            return false;
        double angle_1 = M_PI - b;
        angle_1 = AngleFix(angle_1);
        JointAngle[1] = angle_1 * kAngDeg;

        // 第三个关节角度
        double angle_2 = b;
        angle_2 = AngleFix(angle_2);
        JointAngle[2] = angle_2 * kAngDeg;
    }
    return true;
}
 