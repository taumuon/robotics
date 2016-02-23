// CartPoleLQR.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>

#include "Eigen/Dense"

#include "SystemParams.h"
#include "CartPoleLinear.h"

using State = std::array<double, 4>;
using TimeAndState = std::tuple<double, State, double>;

// stackoverflow.com/questions/25999407/initialize-a-constant-eigen-matrix-in-a-header-file
const Eigen::Matrix<double, 1, 4>& GetKMatrix()
{
	static const struct Once
	{
		Eigen::Matrix<double, 1, 4> K;

		Once()
		{
			// This is for friction = 0.1
			// K << -70.711, -37.834, 105.530, 20.924;

			// friction 0.0
			K << -70.711, -37.734, 105.528, 20.923;
		}
	} once;
	return once.K;
}

double ControlLQR(const State& state)
{
	auto K = GetKMatrix();
	auto stateV = Eigen::Vector4d::Map(state.data());
	auto u = (K * stateV).value();
	return u;
}

double ControlSwingUp(const State& state)
{
	// TODO:
	return 0.0;
}

auto GetTimeStepFunctionCartPoleLinear(SystemParams system_params)
{
	// This is nasty, but Eigen has alignment issues if capturing by value

	std::array<double, 16> a_array;
	std::array<double, 4> b_array;
	std::array<double, 8> c_array;
	std::array<double, 2> d_array;

	CartPoleLinear cart_pole_linear(system_params);

	Eigen::Map<Eigen::Matrix<double, 4, 4>>(a_array.data(), 4, 4) = cart_pole_linear.AMatrix();
	Eigen::Map<Eigen::Matrix<double, 4, 1>>(b_array.data(), 4, 1) = cart_pole_linear.BMatrix();
	Eigen::Map<Eigen::Matrix<double, 2, 4>>(c_array.data(), 2, 4) = cart_pole_linear.CMatrix();
	Eigen::Map<Eigen::Matrix<double, 2, 1>>(d_array.data(), 2, 1) = cart_pole_linear.DMatrix();

	return [=](auto state, auto delta_time, auto u)
	{
		Eigen::Matrix<double, 4, 4> A(a_array.data());
		Eigen::Matrix<double, 4, 1> B(b_array.data());
		Eigen::Matrix<double, 2, 4> C(c_array.data());
		Eigen::Matrix<double, 2, 1> D(d_array.data());

		Eigen::Vector4d state_vector(state.data());

		Eigen::Matrix<double, 4, 1> state_dot = (A * state_vector) - (B * u);

		auto new_state = state_vector + (state_dot * delta_time);
		return State{ new_state[0], new_state[1], new_state[2], new_state[3] };
	};
}

auto GetTimeStepFunctionCartPoleNonLinear(SystemParams system_params)
{
	return [=](auto state, auto delta_time, auto u)
	{
		auto x = state[0];
		auto x_dot = state[1];
		auto theta = state[2];
		auto theta_dot = state[3];

		auto g = system_params.gravity;
		auto m_c = system_params.mass_cart;
		auto m_p = system_params.mass_pole;
		auto length = system_params.length;

		auto sin_theta = sin(theta);
		auto cos_theta = cos(theta);

		auto common_factor = (1.0 / (m_c + (m_p * sin_theta * sin_theta)));
		auto length_theta_dot_squared = length * theta_dot * theta_dot;
		auto x_dot_dot = common_factor * (u + (m_p * sin_theta * (length_theta_dot_squared + (g * cos_theta))));

		auto second_term = ((-u * cos_theta) - (m_p * length_theta_dot_squared * cos_theta * sin_theta));
		auto third_term = -((m_c + m_p) * g * sin_theta);

		auto theta_dot_dot = (common_factor / length) * (second_term + third_term);

		x += x_dot * delta_time;
		x_dot += x_dot_dot * delta_time;
		theta += theta_dot * delta_time;
		theta_dot += theta_dot_dot * delta_time;

		return State{ x, x_dot, theta, theta_dot };
	};
}

template <typename FunctorUpdateState, typename FunctorControlInput>
std::vector<TimeAndState> SimulateTrajectory(const State& initial_state,
										 	 int N,
									 		 double delta_time,
											 FunctorUpdateState update_state,
											 FunctorControlInput control_input)
{
	std::vector<TimeAndState> result;

	result.push_back(TimeAndState(0.0, initial_state, 0.0));
	
	auto state = initial_state;
	for (int i = 1; i < N; ++i)
	{
		auto u = control_input(state);
		state = update_state(state, delta_time, u);

		// TODO: boost.odeint
		auto time = i * delta_time;

		result.push_back(TimeAndState(time, state, u));
	}

	return result;
}

int main()
{
	SystemParams system_params;

	auto initial_theta = 0.1;
	auto initial_state = State{ 0.0, 0.0, initial_theta, 0.0 };

	auto time_stepper = GetTimeStepFunctionCartPoleLinear(system_params);
	// auto time_stepper = GetTimeStepFunctionCartPoleNonLinear(system_params);
	auto trajectory = SimulateTrajectory(initial_state, 100000, 0.0001,	time_stepper, ControlLQR);

	auto count = 0;
	std::cout << "# t x theta u" << std::endl;
	for (auto item : trajectory)
	{
		// if (count++ % 10 != 0) { continue; }
		// if (count++ > 1000) { break; }

		auto time = std::get<0>(item);
		auto x = std::get<1>(item)[0];
		auto theta = std::get<1>(item)[2];
		auto u = std::get<2>(item);
		std::cout << time << " " << x << " " << theta << " " << u << std::endl;
	}

	// std::cin.get();
}
