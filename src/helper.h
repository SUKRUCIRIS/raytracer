#pragma once
#include <cstdio>
#include <cstdarg>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void my_printf(const char *fmt, ...);

static float get_random_float()
{
	static thread_local std::mt19937 generator(std::random_device{}());
	std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
	return distribution(generator);
}