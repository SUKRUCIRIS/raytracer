#pragma once
#include "algebra.h"
#include <vector>
#include "shapes.h"
#include "bvh.h"
#include "lights.h"

class ray_tracer
{
private:
	const bvh *bvhx = 0;
	const std::vector<shape *> *shapes = 0;
	const float intersectionepsilon;
	const float shadowrayepsilon;
	const vec3 ambientlight;
	std::vector<Light *> *lights = 0;
	const vec3 backgroundcolor;
	const float max_depth;

	void trace_rec(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
				   vec3 &color, const float &raytime, const bool culling, int depth) const;

	void calculate_color(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &normal, const material *mat,
						 const std::vector<texture *> *textures, vec3 &hit_point, const vec3 &ray_origin,
						 const shape *min_shape, const float &raytime, vec3 &color) const;

	static void calculate_reflected_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I, vec3 &R, float roughness);

	static bool calculate_refracted_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I,
										float n1, float n2, vec3 &T, float roughness);

	void apply_normal_map(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, const std::vector<texture *> *textures,
						  const shape *min_shape, vec3 &normal) const;

	void apply_bump_map(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, const std::vector<texture *> *textures,
						const shape *min_shape, vec3 &normal) const;

public:
	ray_tracer() = delete;
	ray_tracer(const std::vector<shape *> *shapes,
			   const float &intersectionepsilon,
			   const float &shadowrayepsilon,
			   const vec3 &ambientlight,
			   std::vector<Light *> *lights,
			   const vec3 &backgroundcolor,
			   const float &max_depth) : shapes(shapes), intersectionepsilon(intersectionepsilon),
										 shadowrayepsilon(shadowrayepsilon), ambientlight(ambientlight),
										 lights(lights), backgroundcolor(backgroundcolor),
										 max_depth(max_depth)
	{
		bvhx = new bvh(shapes);
	};
	~ray_tracer()
	{
		delete bvhx;
	}
	void trace(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
			   const int &index, const float &raytime, const bool culling, unsigned char *output) const;
};