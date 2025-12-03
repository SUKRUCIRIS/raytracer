#pragma once

#include "shapes.h"
#include "algebra.h"
#include <vector>
#include <algorithm>
#include <cstdint>

class bvh
{
private:
	struct BVHNode
	{
		aabb bounds;
		union
		{
			int primitivesOffset;
			int secondChildOffset;
		};
		uint16_t nPrimitives;
		uint8_t axis;
		uint8_t pad;
	};
	struct BVHPrimitive
	{
		shape *s;
		int id;
	};
	std::vector<shape *> plane_shapes;
	std::vector<BVHNode> nodes;
	std::vector<BVHPrimitive> ordered_primitives;

	static float get_axis_value(const vec3 &v, int axis);

	inline bool intersect_aabb_fast(const aabb &box, const vec3 &ray_origin, const vec3 &inv_dir,
									const int dir_is_neg[3], float t_max_curr) const;

public:
	bvh() = delete;

	bvh(const std::vector<shape *> *shape_list);

	~bvh();

	bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir, float raytime,
				   float &t_hit, shape **hit_shape, int &hit_id, bool culling = true, const float EPSILON = 1e-6f,
				   bool any_hit = false, float stop_t = 1e30f) const;
};