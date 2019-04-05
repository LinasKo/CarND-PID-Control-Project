#ifndef PID_H
#define PID_H

#include <vector>


namespace pid_control
{
    class PID
    {
    public:
        PID() = default;
        PID(double kp, double ki, double kd);

        inline void UpdateParams(double kp, double ki, double kd)
        {
            m_kp = kp;
            m_ki = ki;
            m_kd = kd;
        };
        inline void UpdateParams(std::vector<double> pidParams) { UpdateParams(pidParams[0], pidParams[1], pidParams[2]); };

        double Apply(double cte);
        std::vector<double> GetParams() { return {m_kp, m_ki, m_kd}; };

    private:
        /**
        * PID Errors
        */
        double p_error { 0.0 };
        double i_error { 0.0 };
        double d_error { 0.0 };

        /**
        * PID Coefficients
        */
        double m_kp { 0.0 };
        double m_ki { 0.0 };
        double m_kd { 0.0 };

        double m_totalError { 0.0 };
        double m_prevError { 0.0 };
    };
}

#endif  // PID_H
