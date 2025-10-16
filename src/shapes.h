#pragma once
#include "algebra.h"

struct triangle
{
	vec3 c1, c2, c3;
};

struct sphere
{
	vec3 center;
	float radius;
};

struct plane
{
	vec3 point;
	vec3 normal;
};
