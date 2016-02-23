#pragma once
struct SystemParams
{
public:
	double mass_cart = 0.5;
	double mass_pole = 0.2;
	double length = 0.3;

	double moment_inertia = 0.006;
	double friction = 0.0;

	double gravity = 9.8;
};

