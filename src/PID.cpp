#include "PID.h"

#include <cassert>
#include <vector>


using namespace pid_control;


PID::PID(double kp, double ki, double kd) :
    m_kp(kp), m_ki(ki), m_kd(kd)
{}

double PID::Apply(double cte)
{
    m_totalError += cte;
    double value = - m_kp * cte - m_ki * m_totalError - m_kd * (cte - m_prevError);
    m_prevError = cte;

    // Clamp to [-1.0, 1.0]
    value = value < -1.0 ? -1.0 : (value > 1.0 ? 1.0 : value);

    return value;
}
