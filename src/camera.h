#pragma once
#include "algebra.h"
#include <vector>
#include <string>

class camera
{
public:
	simd_vec3 &calculator;
	vec3 position;
	int resx, resy;
	std::string output;
	std::vector<vec3> ray_dirs;
	camera(simd_vec3 &calculator, float position_x, float position_y, float position_z,
		   float gaze_x, float gaze_y, float gaze_z,
		   float up_x, float up_y, float up_z, float neardistance,
		   float nearp_left, float nearp_right, float nearp_bottom, float nearp_top,
		   int resx, int resy, std::string output);
};