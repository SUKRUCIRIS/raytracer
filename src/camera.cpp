#include "camera.h"
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

camera::camera(simd_vec3 &calculator, float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z, float neardistance,
			   float nearp_left, float nearp_right, float nearp_bottom, float nearp_top,
			   int resx, int resy, std::string output)
	: position(position_x, position_y, position_z), calculator(calculator),
	  resx(resx), resy(resy), output(output)
{
	vec3 gaze(gaze_x, gaze_y, gaze_z);
	vec3 up(up_x, up_y, up_z);
	vec3 w;
	vec3 u;
	vec3 v;
	calculator.normalize(gaze, w);
	calculator.mult_scalar(w, -1, w);
	calculator.cross(up, w, u);
	calculator.normalize(u, u);
	calculator.cross(w, u, v);

	vec3 center;

	calculator.mult_scalar(w, neardistance, w);
	calculator.subs(position, w, center);

	vec3 pixelPos;

	for (int j = 0; j < resy; ++j)
	{
		for (int i = 0; i < resx; ++i)
		{
			float su = nearp_left + (nearp_right - nearp_left) * (i + 0.5f) / resx;
			float sv = nearp_bottom + (nearp_top - nearp_bottom) * (j + 0.5f) / resy;
			calculator.mult_scalar(u, su, gaze);
			calculator.mult_scalar(v, sv, up);
			calculator.add(gaze, up, pixelPos);
			calculator.add(pixelPos, center, pixelPos);
			calculator.subs(pixelPos, position, pixelPos);
			calculator.normalize(pixelPos, pixelPos);
			ray_dirs.push_back(pixelPos);
		}
	}
}

camera::camera(simd_vec3 &calculator,
			   float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z,
			   float neardistance, float fovY,
			   int resx, int resy, std::string output)
	: position(position_x, position_y, position_z),
	  calculator(calculator),
	  resx(resx), resy(resy), output(output)
{
	vec3 gazePoint(gaze_x, gaze_y, gaze_z);
	vec3 up(up_x, up_y, up_z);

	vec3 gazeDir;
	calculator.subs(gazePoint, position, gazeDir);
	calculator.normalize(gazeDir, gazeDir);

	vec3 w, u, v;
	calculator.mult_scalar(gazeDir, -1, w);
	calculator.cross(up, w, u);
	calculator.normalize(u, u);
	calculator.cross(w, u, v);

	float aspect = static_cast<float>(resx) / static_cast<float>(resy);
	float near_height = 2.0f * neardistance * tanf((fovY * 0.5f) * M_PI / 180.0f);
	float near_width = near_height * aspect;

	float nearp_left = -near_width / 2.0f;
	float nearp_right = near_width / 2.0f;
	float nearp_bottom = -near_height / 2.0f;
	float nearp_top = near_height / 2.0f;

	vec3 center;
	calculator.mult_scalar(w, neardistance, w);
	calculator.subs(position, w, center);

	vec3 pixelPos;
	for (int j = 0; j < resy; ++j)
	{
		for (int i = 0; i < resx; ++i)
		{
			float su = nearp_left + (nearp_right - nearp_left) * (i + 0.5f) / resx;
			float sv = nearp_bottom + (nearp_top - nearp_bottom) * (j + 0.5f) / resy;

			vec3 du, dv;
			calculator.mult_scalar(u, su, du);
			calculator.mult_scalar(v, sv, dv);

			calculator.add(du, dv, pixelPos);
			calculator.add(pixelPos, center, pixelPos);
			calculator.subs(pixelPos, position, pixelPos);
			calculator.normalize(pixelPos, pixelPos);

			ray_dirs.push_back(pixelPos);
		}
	}
}
