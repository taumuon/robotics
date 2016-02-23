#include "stdafx.h"

#include "CartPoleLinear.h"

CartPoleLinear::CartPoleLinear(const SystemParams& system_params)
{
	auto g = system_params.gravity;
	auto m_c = system_params.mass_cart;
	auto m_p = system_params.mass_pole;
	auto length = system_params.length;
	auto I = system_params.moment_inertia;

	auto b = system_params.friction;

	auto lengthSquared = pow(length, 2.0);
	auto p = (I*(m_c + m_p)) + (m_c*m_p*(lengthSquared));

	_a.setZero();
	_a(0, 1) = 1;
	_a(1, 1) = -((I + (m_p*lengthSquared))*b) / p;
	_a(1, 2) = (pow(m_p, 2.0))*g*(lengthSquared) / p;
	_a(2, 3) = 1;
	_a(3, 1) = -(m_p*length*b) / p;
	_a(3, 2) = m_p*g*length*(m_c + m_p) / p;

	_b.setZero();
	_b(1, 0) = ((I + (m_p*lengthSquared))) / p;
	_b(3, 0) = (m_p * length) / p;

	_c.setZero();
	_c(0, 0) = 1.0;
	_c(1, 2) = 1.0;

	_d.setZero();
}
