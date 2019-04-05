#ifndef TWIDDLE_H
#define TWIDDLE_H


#include <limits>
#include <vector>

namespace pid_control
{
	enum State {
		START,
		DECREASE,
		CONCLUDE
	};

	class Twiddle
	{
	public:
		/*
		* A parameter optimizer that attempts to minimize error by fiddling with the parameters and seeing what happens.
		* Prone to finding a local minimum.
		*/
		Twiddle(const double tolerance);

		/*
		* Twiddle with the values of the parameters once, hopefully minimizing the error of the next run.
		* Returns true when completed.
		*/
		bool runOnce(const double prevError, std::vector<double>& params);

		std::vector<double> GetCoefficients() { return m_coeffs; };
		void SetCoefficients(const std::vector<double>& coeffs) { m_coeffs = coeffs; };

	private:
		const double m_tolerance { 0.0 };

		int m_coeffIndex { 0 };
		State m_state { START };

		std::vector<double> m_coeffs;
		double m_bestError {  std::numeric_limits<double>::max() };
		std::vector<double> m_bestParams;
	};
}

#endif  // TWIDDLE_H
