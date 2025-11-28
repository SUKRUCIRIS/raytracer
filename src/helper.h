#pragma once
#include <cstdio>
#include <cstdarg>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void my_printf(const char *fmt, ...);

float get_random_float();