#include <iostream>
#include <numeric>
#include <vector>

#include "Twiddle.h"

using namespace pid_control;


static constexpr double INCREASE_RATE = 1.25;
static constexpr double DECREASE_RATE = 0.75;
static constexpr double INITIAL_COEFF = 1.0;

Twiddle::Twiddle(const double tolerance) :
	m_tolerance(tolerance)
{}

bool Twiddle::runOnce(const double error, std::vector<double>& params)
{
	/* The twiddle algorithm is split into 3 states: START, DECREASE and CONCLUDE.
	 * This is made so that the user can call twiddle, run the car, call twiddle, and so on.
	 * START - starting state, checks tolerance, that increments one parameter. Advances to DECREASE.
	 * DECREASE - if the error is smaller, advances to START, othewise decreases params and goes to CONCLUDE
	 * CONCLUDE - if the error is smaller, advances to START, otherwise resets params and does everything that START does, advancing to DECREASE.
	 * 			  This is so that the user will always have control after the parameters were changed.
	 */
	const double absError = std::abs(error);

	// Initialize
	if (m_bestParams.size() == 0)
	{
		m_bestParams = params;
		m_bestError = absError;
		if (m_coeffs.size() == 0)
		{
			m_coeffs = std::vector<double>(params.size(), INITIAL_COEFF);
		}
	}

	switch (m_state)
	{
		case START:
		{
			// Check for end
			if (std::accumulate(m_coeffs.begin(), m_coeffs.end(), 0.0) < m_tolerance)
			{
				params = m_bestParams;
				return true;  // Completed
			}

			params[m_coeffIndex] += m_coeffs[m_coeffIndex];

			m_state = DECREASE;
			return false;  // Not completed yet
		}

		case DECREASE:
		{
			if (absError < m_bestError)
			{
				m_bestError = absError;
				m_coeffs[m_coeffIndex] *= INCREASE_RATE;
				m_bestParams = params;

				m_coeffIndex = (m_coeffIndex + 1) % m_coeffs.size();
				m_state = START;
				return false;  // Not completed yet
			}

			params[m_coeffIndex] -= 2.0 * m_coeffs[m_coeffIndex];

			m_state = CONCLUDE;
			return false;  // Not completed yet
		}

		case CONCLUDE:
		{
			if (absError < m_bestError)
			{
				m_bestError = absError;
				m_coeffs[m_coeffIndex] *= INCREASE_RATE;
				m_bestParams = params;

				m_coeffIndex = (m_coeffIndex + 1) % m_coeffs.size();
				m_state = START;
				return false;  // Not completed yet
			}

			params[m_coeffIndex] += 1.0 * m_coeffs[m_coeffIndex];
			m_coeffs[m_coeffIndex] *= DECREASE_RATE;

			m_coeffIndex = (m_coeffIndex + 1) % m_coeffs.size();


			// Execute START until it's time to give control to the user

			// Check for end
			if (std::accumulate(m_coeffs.begin(), m_coeffs.end(), 0.0) < m_tolerance)
			{
				params = m_bestParams;
				return true;  // Completed
			}

			params[m_coeffIndex] += m_coeffs[m_coeffIndex];

			m_state = DECREASE;
			return false;  // Not completed yet
		}
	}

	return false;  // Should never happen
}
