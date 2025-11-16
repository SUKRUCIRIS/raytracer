#include "camera.h"
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

camera::camera(simd_vec3 &calculator, simd_mat4 &calculator_m, float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z, float neardistance,
			   float nearp_left, float nearp_right, float nearp_bottom, float nearp_top,
			   int resx, int resy, std::string output, mat4 cameraModel)
	: position(position_x, position_y, position_z), calculator(calculator),
	  resx(resx), resy(resy), output(output)
{
	vec3 local_gaze(gaze_x, gaze_y, gaze_z);
	vec3 local_up(up_x, up_y, up_z);

	vec3 w_local, u_local, v_local;
	vec3 w_world, u_world, v_world;

	calculator.normalize(local_gaze, w_local);
	calculator.mult_scalar(w_local, -1, w_local);

	calculator.cross(local_up, w_local, u_local);
	calculator.normalize(u_local, u_local);

	calculator.cross(w_local, u_local, v_local);
	w_local.store();
	calculator_m.mult_vec(cameraModel, w_local, w_world, true);

	u_local.store();
	calculator_m.mult_vec(cameraModel, u_local, u_world, true);

	v_local.store();
	calculator_m.mult_vec(cameraModel, v_local, v_world, true);

	vec3 world_pos;
	position.store();
	calculator_m.mult_vec(cameraModel, position, world_pos, false);
	position = world_pos;
	vec3 center;

	vec3 temp_w;
	calculator.mult_scalar(w_world, neardistance, temp_w);
	calculator.subs(position, temp_w, center);
	vec3 pixelPos;

	for (int j = 0; j < resy; ++j)
	{
		for (int i = 0; i < resx; ++i)
		{
			float su = nearp_left + (nearp_right - nearp_left) * (i + 0.5f) / resx;
			float sv = nearp_bottom + (nearp_top - nearp_bottom) * (j + 0.5f) / resy;
			calculator.mult_scalar(u_world, su, local_gaze);

			calculator.mult_scalar(v_world, sv, local_up);

			calculator.add(local_gaze, local_up, pixelPos);

			calculator.add(pixelPos, center, pixelPos);

			calculator.subs(pixelPos, position, pixelPos);

			calculator.normalize(pixelPos, pixelPos);

			pixelPos.store();
			ray_dirs.push_back(pixelPos);
		}
	}
}

camera::camera(simd_vec3 &calculator, simd_mat4 &calculator_m,
			   float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z,
			   float neardistance, float fovY,
			   int resx, int resy, std::string output, mat4 cameraModel)
	: position(position_x, position_y, position_z),
	  calculator(calculator),
	  resx(resx), resy(resy), output(output)
{
	vec3 position_local = position;

	vec3 gazePoint_local(gaze_x, gaze_y, gaze_z);
	vec3 up_local(up_x, up_y, up_z);

	vec3 gazeDir_local;
	calculator.subs(gazePoint_local, position_local, gazeDir_local);
	calculator.normalize(gazeDir_local, gazeDir_local);

	vec3 w_local, u_local, v_local;
	calculator.mult_scalar(gazeDir_local, -1, w_local);
	calculator.cross(up_local, w_local, u_local);
	calculator.normalize(u_local, u_local);
	calculator.cross(w_local, u_local, v_local);
	vec3 w_world, u_world, v_world;

	position_local.store();
	calculator_m.mult_vec(cameraModel, position_local, position, false);
	position.store();

	w_local.store();
	calculator_m.mult_vec(cameraModel, w_local, w_world, true);
	w_world.store();
	calculator.normalize(w_world, w_world);

	u_local.store();
	calculator_m.mult_vec(cameraModel, u_local, u_world, true);
	u_world.store();
	calculator.normalize(u_world, u_world);

	v_local.store();
	calculator_m.mult_vec(cameraModel, v_local, v_world, true);
	v_world.store();
	calculator.normalize(v_world, v_world);

	float aspect = static_cast<float>(resx) / static_cast<float>(resy);
	float near_height = 2.0f * neardistance * tanf((fovY * 0.5f) * M_PI / 180.0f);
	float near_width = near_height * aspect;

	float nearp_left = -near_width / 2.0f;
	float nearp_right = near_width / 2.0f;
	float nearp_bottom = -near_height / 2.0f;
	float nearp_top = near_height / 2.0f;

	vec3 center;
	vec3 temp_w_offset;
	calculator.mult_scalar(w_world, neardistance, temp_w_offset);
	calculator.subs(position, temp_w_offset, center);
	center.store();

	vec3 pixelPos;
	for (int j = 0; j < resy; ++j)
	{
		for (int i = 0; i < resx; ++i)
		{
			float su = nearp_left + (nearp_right - nearp_left) * (i + 0.5f) / resx;
			float sv = nearp_bottom + (nearp_top - nearp_bottom) * (j + 0.5f) / resy;

			vec3 u_scaled, v_scaled;

			calculator.mult_scalar(u_world, su, u_scaled);
			calculator.mult_scalar(v_world, sv, v_scaled);

			calculator.add(u_scaled, v_scaled, pixelPos);
			calculator.add(pixelPos, center, pixelPos);

			calculator.subs(pixelPos, position, pixelPos);

			calculator.normalize(pixelPos, pixelPos);

			pixelPos.store();
			ray_dirs.push_back(pixelPos);
		}
	}
}
