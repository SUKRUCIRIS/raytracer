#pragma once

#include "shapes.h"
#include "algebra.h"
#include <vector>

class grid
{
private:
	struct gridcell
	{
		std::vector<shape *> shapes;
		std::vector<int> ids; // shape ids for instancing
	};
	std::vector<shape *> plane_shapes;
	const std::vector<shape *> *shapes;
	gridcell *cells;
	aabb bounds;
	vec3 grid_dim;
	vec3 world_size;
	vec3 inv_cell_size;

	void calculate_aabb(simd_vec3 &calculator, simd_mat4 &calculator_m);

	int get_cell_index(float pos, float grid_min, float inv_cell_sz, float grid_dim_sz) const;

	float get_t_max(float start, int cell_idx, int step, float cell_sz, float grid_min, float inv_dir) const;

	bool intersect_ray_aabb(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t_min, float &t_max, const float EPS = 1e-6f) const;

public:
	grid() = delete;

	grid(const std::vector<shape *> *shape_list);

	~grid();

	bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir, float raytime,
				   float &t_hit, shape **hit_shape, int &hit_id, bool culling = true, const float EPSILON = 1e-6f,
				   bool any_hit = false, float stop_t = 1e30f) const;
};