#include "camera.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <random>

camera::camera(simd_vec3 &calculator, simd_mat4 &calculator_m, float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z, float neardistance,
			   float nearp_left, float nearp_right, float nearp_bottom, float nearp_top,
			   int resx, int resy, int num_samples,
			   float focus_distance, float aperture_size, std::string output, mat4 cameraModel)
	: position(position_x, position_y, position_z),
	  resx(resx), resy(resy), output(output), num_samples(num_samples),
	  focus_distance(focus_distance), aperture_size(aperture_size),
	  neardistance(neardistance),
	  near_left(nearp_left), near_right(nearp_right), near_bottom(nearp_bottom), near_top(nearp_top)
{
	vec3 local_gaze(gaze_x, gaze_y, gaze_z);
	vec3 local_up(up_x, up_y, up_z);

	vec3 w_local, u_local, v_local;

	calculator.normalize(local_gaze, w_local);
	calculator.mult_scalar(w_local, -1, w_local);

	calculator.cross(local_up, w_local, u_local);
	calculator.normalize(u_local, u_local);

	calculator.cross(w_local, u_local, v_local);

	vec3 w_world, u_world, v_world;

	w_local.store();
	calculator_m.mult_vec(cameraModel, w_local, w_world, true);

	u_local.store();
	calculator_m.mult_vec(cameraModel, u_local, u_world, true);

	v_local.store();
	calculator_m.mult_vec(cameraModel, v_local, v_world, true);

	calculator.normalize(w_world, this->w);
	calculator.normalize(u_world, this->u);
	calculator.normalize(v_world, this->v);

	vec3 world_pos;
	position.store();
	calculator_m.mult_vec(cameraModel, position, world_pos, false);
	position = world_pos;

	vec3 temp_w;
	calculator.mult_scalar(this->w, neardistance, temp_w);
	calculator.subs(position, temp_w, center);
	center.store();
}

camera::camera(simd_vec3 &calculator, simd_mat4 &calculator_m,
			   float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z,
			   float neardistance, float fovY,
			   int resx, int resy, int num_samples,
			   float focus_distance, float aperture_size, std::string output, mat4 cameraModel)
	: position(position_x, position_y, position_z),
	  resx(resx), resy(resy), output(output), num_samples(num_samples),
	  focus_distance(focus_distance), aperture_size(aperture_size),
	  neardistance(neardistance)
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

	u_local.store();
	calculator_m.mult_vec(cameraModel, u_local, u_world, true);

	v_local.store();
	calculator_m.mult_vec(cameraModel, v_local, v_world, true);

	calculator.normalize(w_world, this->w);
	calculator.normalize(u_world, this->u);
	calculator.normalize(v_world, this->v);

	float aspect = static_cast<float>(resx) / static_cast<float>(resy);
	float near_height = 2.0f * neardistance * tanf((fovY * 0.5f) * M_PI / 180.0f);
	float near_width = near_height * aspect;

	near_left = -near_width / 2.0f;
	near_right = near_width / 2.0f;
	near_bottom = -near_height / 2.0f;
	near_top = near_height / 2.0f;

	vec3 temp_w_offset;
	calculator.mult_scalar(this->w, neardistance, temp_w_offset);
	calculator.subs(position, temp_w_offset, center);
	center.store();
}

std::vector<camera::sample> camera::get_samples(simd_vec3 &calculator, int i, int j)
{
	std::vector<camera::sample> samples;
	samples.reserve(num_samples);

	if (num_samples == 1)
	{
		float su = near_left + (near_right - near_left) * (i + 0.5f) / resx;
		float sv = near_bottom + (near_top - near_bottom) * (j + 0.5f) / resy;

		vec3 u_comp, v_comp, pixel_target;
		calculator.mult_scalar(u, su, u_comp);
		calculator.mult_scalar(v, sv, v_comp);

		calculator.add(center, u_comp, pixel_target);
		calculator.add(pixel_target, v_comp, pixel_target);

		vec3 ray_dir;
		calculator.subs(pixel_target, position, ray_dir);
		calculator.normalize(ray_dir, ray_dir);

		sample s;
		s.position = position;
		s.direction = ray_dir;
		s.position.store();
		s.direction.store();
		s.time = get_random_float();

		samples.push_back(s);
		return samples;
	}

	int grid_n = (int)sqrt(num_samples);

	for (int sy = 0; sy < grid_n; ++sy)
	{
		for (int sx = 0; sx < grid_n; ++sx)
		{
			float r1 = get_random_float();
			float r2 = get_random_float();

			float sub_u = (sx + r1) / grid_n;
			float sub_v = (sy + r2) / grid_n;

			float su = near_left + (near_right - near_left) * (i + sub_u) / resx;
			float sv = near_bottom + (near_top - near_bottom) * (j + sub_v) / resy;

			vec3 u_comp, v_comp, pixel_target;
			calculator.mult_scalar(u, su, u_comp);
			calculator.mult_scalar(v, sv, v_comp);

			calculator.add(center, u_comp, pixel_target);
			calculator.add(pixel_target, v_comp, pixel_target);

			vec3 ray_dir;
			calculator.subs(pixel_target, position, ray_dir);
			calculator.normalize(ray_dir, ray_dir);

			vec3 ray_origin = position;

			if (aperture_size > 0.0f && focus_distance > 0.0f)
			{
				float dot_val;
				calculator.dot(ray_dir, w, dot_val);
				float cos_theta = -dot_val;

				if (std::abs(cos_theta) < 1e-6)
					cos_theta = 1e-6;

				float t = focus_distance / cos_theta;

				vec3 focal_point;
				vec3 travel_vec;
				calculator.mult_scalar(ray_dir, t, travel_vec);
				calculator.add(position, travel_vec, focal_point);

				float r3 = get_random_float();
				float r4 = get_random_float();

				float lens_x = (r3 - 0.5f) * aperture_size;
				float lens_y = (r4 - 0.5f) * aperture_size;

				vec3 lens_offset, lens_u, lens_v;
				calculator.mult_scalar(u, lens_x, lens_u);
				calculator.mult_scalar(v, lens_y, lens_v);
				calculator.add(lens_u, lens_v, lens_offset);

				calculator.add(position, lens_offset, ray_origin);

				calculator.subs(focal_point, ray_origin, ray_dir);
				calculator.normalize(ray_dir, ray_dir);
			}

			sample s;
			s.position = ray_origin;
			s.direction = ray_dir;

			s.position.store();
			s.direction.store();

			s.time = get_random_float();

			samples.push_back(s);
		}
	}

	return samples;
}
