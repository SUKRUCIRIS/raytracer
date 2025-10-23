#pragma once
#include "algebra.h"
#include <vector>
#include "shapes.h"

void ray_trace(simd_vec3 &calculator, const std::vector<shape *> *shapes, const vec3 &ray_origin, const vec3 &ray_dir, const float &intersectionepsilon,
			   const float &shadowrayepsilon, const vec3 &ambientlight, const std::vector<point_light> *point_lights, const vec3 &backgroundcolor,
			   unsigned char *output, const int &index);