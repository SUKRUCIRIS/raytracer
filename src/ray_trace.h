#pragma once
#include "algebra.h"
#include <vector>
#include "shapes.h"

void ray_trace(simd_vec3 &calculator, std::vector<shape *> *shapes, vec3 ray_origin, vec3 ray_dir, float intersectionepsilon,
			   float shadowrayepsilon, vec3 ambientlight, std::vector<point_light> *point_lights, vec3 backgroundcolor,
			   unsigned char *output, const int index);