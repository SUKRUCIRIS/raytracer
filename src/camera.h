#pragma once
#include "algebra.h"
#include <vector>
#include <string>

enum tmotype
{
	Photographic,
	Filmic,
	ACES
};

struct Tonemap
{
	tmotype type;
	float TMOOptions1;
	float TMOOptions2;
	float Saturation;
	float Gamma;
	std::string Extension;
};

class camera
{
public:
	vec3 position;
	int resx, resy;
	int num_samples;
	float focus_distance;
	float aperture_size;
	vec3 u, v, w;
	float near_left, near_right, near_bottom, near_top;
	float neardistance;
	std::string output;
	struct sample
	{
		vec3 direction;
		vec3 position;
		float time;
	};
	vec3 center;
	bool is_hdr = false;
	std::vector<Tonemap> Tonemaps;
	camera(simd_vec3 &calculator, simd_mat4 &calculator_m, float position_x, float position_y, float position_z,
		   float gaze_x, float gaze_y, float gaze_z,
		   float up_x, float up_y, float up_z, float neardistance,
		   float nearp_left, float nearp_right, float nearp_bottom, float nearp_top,
		   int resx, int resy, int num_samples,
		   float focus_distance, float aperture_size, std::string output, mat4 cameraModel);
	camera(simd_vec3 &calculator, simd_mat4 &calculator_m,
		   float position_x, float position_y, float position_z,
		   float gaze_x, float gaze_y, float gaze_z,
		   float up_x, float up_y, float up_z,
		   float neardistance, float fovY,
		   int resx, int resy, int num_samples,
		   float focus_distance, float aperture_size, std::string output, mat4 cameraModel);
	std::vector<sample> get_samples(simd_vec3 &calculator, int i, int j);
};