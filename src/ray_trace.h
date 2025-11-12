#pragma once
#include "algebra.h"
#include <vector>
#include "shapes.h"
#include "grid.h"

class ray_tracer
{
private:
	const grid *gridx = 0;
	const std::vector<shape *> *shapes = 0;
	const float intersectionepsilon;
	const float shadowrayepsilon;
	const vec3 ambientlight;
	const std::vector<point_light> *point_lights = 0;
	const vec3 backgroundcolor;
	const float max_depth;
	bool use_grid;

	void trace_rec(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
				   vec3 &color, const bool culling, int depth) const;

	void calculate_color(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &normal, const material *mat,
						 const vec3 &hit_point, const vec3 &ray_origin, const shape *min_shape,
						 vec3 &color) const;

	static void calculate_reflected_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I, vec3 &R);

	static bool calculate_refracted_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I,
										float n1, float n2, vec3 &T);

public:
	ray_tracer() = delete;
	ray_tracer(const std::vector<shape *> *shapes,
			   const float &intersectionepsilon,
			   const float &shadowrayepsilon,
			   const vec3 &ambientlight,
			   const std::vector<point_light> *point_lights,
			   const vec3 &backgroundcolor,
			   const float &max_depth, bool use_grid) : shapes(shapes), intersectionepsilon(intersectionepsilon),
														shadowrayepsilon(shadowrayepsilon), ambientlight(ambientlight),
														point_lights(point_lights), backgroundcolor(backgroundcolor),
														max_depth(max_depth), use_grid(use_grid)
	{
		if (use_grid)
		{
			gridx = new grid(shapes);
		}
	};
	~ray_tracer()
	{
		delete gridx;
	}
	void trace(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
			   const int &index, const bool culling, unsigned char *output) const;
};