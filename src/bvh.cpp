#include "bvh.h"
#include <limits>
#include <algorithm>
#include <cmath>
#include <functional>
#include <new>

float bvh::get_axis_value(const vec3 &v, int axis)
{
	if (axis == 0)
		return v.get_x();
	if (axis == 1)
		return v.get_y();
	return v.get_z();
}

struct PrimitiveInfo
{
	int primitiveNumber;
	aabb bounds;
	vec3 centroid;

	PrimitiveInfo(int pn, const aabb &b) : primitiveNumber(pn), bounds(b)
	{
		float cx = (b.min.get_x() + b.max.get_x()) * 0.5f;
		float cy = (b.min.get_y() + b.max.get_y()) * 0.5f;
		float cz = (b.min.get_z() + b.max.get_z()) * 0.5f;
		centroid.load(cx, cy, cz);
	}
};

struct BuildNode
{
	aabb bounds;
	BuildNode *children[2] = {nullptr, nullptr};
	int splitAxis = 0;
	int firstPrimOffset = 0;
	int nPrimitives = 0;
};

bvh::bvh(const std::vector<shape *> *shape_list)
{
	if (!shape_list || shape_list->empty())
		return;

	simd_vec3 calculator;
	simd_mat4 calculator_m(calculator);

	std::vector<PrimitiveInfo> build_data;
	std::vector<BVHPrimitive> all_primitives;

	int prim_cnt = 0;
	aabb temp_aabb;

	for (auto s : *shape_list)
	{
		if (s->get_shapetype() == Plane)
		{
			plane_shapes.push_back(s);
			continue;
		}
		std::vector<int> ids = s->get_ids();
		for (int id : ids)
		{
			s->getBoundingBox(id, calculator, calculator_m, temp_aabb);

			temp_aabb.min.store();
			temp_aabb.max.store();

			build_data.emplace_back(prim_cnt++, temp_aabb);
			all_primitives.push_back({s, id});
		}
	}

	if (build_data.empty())
		return;

	int totalNodes = 0;
	ordered_primitives.reserve(build_data.size());

	std::function<BuildNode *(int, int)> build_recursive =
		[&](int start, int end) -> BuildNode *
	{
		BuildNode *node = new BuildNode();
		totalNodes++;

		vec3 min_v, max_v;
		float min_arr[3] = {1e30f, 1e30f, 1e30f};
		float max_arr[3] = {-1e30f, -1e30f, -1e30f};

		for (int i = start; i < end; ++i)
		{
			vec3 p_min = build_data[i].bounds.min;
			vec3 p_max = build_data[i].bounds.max;
			min_arr[0] = std::min(min_arr[0], p_min.get_x());
			min_arr[1] = std::min(min_arr[1], p_min.get_y());
			min_arr[2] = std::min(min_arr[2], p_min.get_z());
			max_arr[0] = std::max(max_arr[0], p_max.get_x());
			max_arr[1] = std::max(max_arr[1], p_max.get_y());
			max_arr[2] = std::max(max_arr[2], p_max.get_z());
		}
		node->bounds.min.load(min_arr[0], min_arr[1], min_arr[2]);
		node->bounds.max.load(max_arr[0], max_arr[1], max_arr[2]);
		node->bounds.min.store();
		node->bounds.max.store();

		int nPrims = end - start;
		if (nPrims == 1)
		{
			node->firstPrimOffset = (int)ordered_primitives.size();
			node->nPrimitives = nPrims;
			for (int i = start; i < end; ++i)
				ordered_primitives.push_back(all_primitives[build_data[i].primitiveNumber]);
			return node;
		}

		float c_min[3] = {1e30f, 1e30f, 1e30f}, c_max[3] = {-1e30f, -1e30f, -1e30f};
		for (int i = start; i < end; ++i)
		{
			vec3 c = build_data[i].centroid;
			c_min[0] = std::min(c_min[0], c.get_x());
			c_min[1] = std::min(c_min[1], c.get_y());
			c_min[2] = std::min(c_min[2], c.get_z());
			c_max[0] = std::max(c_max[0], c.get_x());
			c_max[1] = std::max(c_max[1], c.get_y());
			c_max[2] = std::max(c_max[2], c.get_z());
		}

		int dim = 0;
		float max_extent = c_max[0] - c_min[0];
		if ((c_max[1] - c_min[1]) > max_extent)
		{
			dim = 1;
			max_extent = c_max[1] - c_min[1];
		}
		if ((c_max[2] - c_min[2]) > max_extent)
		{
			dim = 2;
		}

		if (max_extent < 1e-5f)
		{
			node->firstPrimOffset = (int)ordered_primitives.size();
			node->nPrimitives = nPrims;
			for (int i = start; i < end; ++i)
				ordered_primitives.push_back(all_primitives[build_data[i].primitiveNumber]);
			return node;
		}

		constexpr int nBuckets = 12;
		struct Bucket
		{
			int count = 0;
			aabb b;
			bool init = false;
		};
		Bucket buckets[nBuckets];

		float k1 = nBuckets * (1.0f - 1e-4f) / max_extent;
		float c_min_dim = c_min[dim];

		for (int i = start; i < end; ++i)
		{
			int b = (int)((get_axis_value(build_data[i].centroid, dim) - c_min_dim) * k1);
			if (b < 0)
				b = 0;
			if (b >= nBuckets)
				b = nBuckets - 1;
			buckets[b].count++;

			vec3 b_min = build_data[i].bounds.min;
			vec3 b_max = build_data[i].bounds.max;

			if (!buckets[b].init)
			{
				buckets[b].b = build_data[i].bounds;
				buckets[b].init = true;
			}
			else
			{
				float nx = std::min(buckets[b].b.min.get_x(), b_min.get_x());
				float ny = std::min(buckets[b].b.min.get_y(), b_min.get_y());
				float nz = std::min(buckets[b].b.min.get_z(), b_min.get_z());
				buckets[b].b.min.load(nx, ny, nz);

				float mx = std::max(buckets[b].b.max.get_x(), b_max.get_x());
				float my = std::max(buckets[b].b.max.get_y(), b_max.get_y());
				float mz = std::max(buckets[b].b.max.get_z(), b_max.get_z());
				buckets[b].b.max.load(mx, my, mz);

				buckets[b].b.min.store();
				buckets[b].b.max.store();
			}
		}

		float cost[nBuckets - 1];
		for (int i = 0; i < nBuckets - 1; ++i)
		{
			aabb b0, b1;
			int count0 = 0, count1 = 0;
			bool b0_init = false, b1_init = false;

			for (int j = 0; j <= i; ++j)
			{
				if (buckets[j].count == 0)
					continue;
				count0 += buckets[j].count;
				if (!b0_init)
				{
					b0 = buckets[j].b;
					b0_init = true;
				}
				else
				{
					b0.min.load(std::min(b0.min.get_x(), buckets[j].b.min.get_x()),
								std::min(b0.min.get_y(), buckets[j].b.min.get_y()),
								std::min(b0.min.get_z(), buckets[j].b.min.get_z()));
					b0.max.load(std::max(b0.max.get_x(), buckets[j].b.max.get_x()),
								std::max(b0.max.get_y(), buckets[j].b.max.get_y()),
								std::max(b0.max.get_z(), buckets[j].b.max.get_z()));
					b0.min.store();
					b0.max.store();
				}
			}

			for (int j = i + 1; j < nBuckets; ++j)
			{
				if (buckets[j].count == 0)
					continue;
				count1 += buckets[j].count;
				if (!b1_init)
				{
					b1 = buckets[j].b;
					b1_init = true;
				}
				else
				{
					b1.min.load(std::min(b1.min.get_x(), buckets[j].b.min.get_x()),
								std::min(b1.min.get_y(), buckets[j].b.min.get_y()),
								std::min(b1.min.get_z(), buckets[j].b.min.get_z()));
					b1.max.load(std::max(b1.max.get_x(), buckets[j].b.max.get_x()),
								std::max(b1.max.get_y(), buckets[j].b.max.get_y()),
								std::max(b1.max.get_z(), buckets[j].b.max.get_z()));
					b1.min.store();
					b1.max.store();
				}
			}

			auto sa = [](const aabb &box)
			{
				float x = box.max.get_x() - box.min.get_x();
				float y = box.max.get_y() - box.min.get_y();
				float z = box.max.get_z() - box.min.get_z();
				if (x < 0 || y < 0 || z < 0)
					return 0.0f;
				return 2.f * (x * y + x * z + y * z);
			};

			cost[i] = 0.125f + (count0 * sa(b0) + count1 * sa(b1)) / sa(node->bounds);
		}

		float minCost = cost[0];
		int minCostSplitBucket = 0;
		for (int i = 1; i < nBuckets - 1; ++i)
		{
			if (cost[i] < minCost)
			{
				minCost = cost[i];
				minCostSplitBucket = i;
			}
		}

		float leafCost = (float)nPrims;
		if (nPrims > 4 || minCost < leafCost)
		{
			BuildNode *child0;
			BuildNode *child1;

			auto pmid = std::partition(build_data.begin() + start, build_data.begin() + end,
									   [=](const PrimitiveInfo &pi)
									   {
										   int b = (int)((get_axis_value(pi.centroid, dim) - c_min_dim) * k1);
										   if (b < 0)
											   b = 0;
										   if (b >= nBuckets)
											   b = nBuckets - 1;
										   return b <= minCostSplitBucket;
									   });

			int mid = (int)(pmid - build_data.begin());

			if (mid == start || mid == end)
			{
				mid = start + (end - start) / 2;
				std::nth_element(build_data.begin() + start, build_data.begin() + mid, build_data.begin() + end,
								 [dim](const PrimitiveInfo &a, const PrimitiveInfo &b)
								 {
									 return get_axis_value(a.centroid, dim) < get_axis_value(b.centroid, dim);
								 });
			}

			node->splitAxis = dim;
			node->children[0] = build_recursive(start, mid);
			node->children[1] = build_recursive(mid, end);
		}
		else
		{
			node->firstPrimOffset = (int)ordered_primitives.size();
			node->nPrimitives = nPrims;
			for (int i = start; i < end; ++i)
				ordered_primitives.push_back(all_primitives[build_data[i].primitiveNumber]);
		}
		return node;
	};

	BuildNode *root = build_recursive(0, build_data.size());

	nodes.resize(totalNodes);
	int offset = 0;

	std::function<int(BuildNode *)> flatten = [&](BuildNode *node) -> int
	{
		int myOffset = offset++;
		BVHNode &linearNode = nodes[myOffset];
		linearNode.bounds = node->bounds;

		if (node->nPrimitives > 0)
		{
			linearNode.primitivesOffset = node->firstPrimOffset;
			linearNode.nPrimitives = (uint16_t)node->nPrimitives;
		}
		else
		{
			linearNode.axis = (uint8_t)node->splitAxis;
			linearNode.nPrimitives = 0;
			flatten(node->children[0]);
			linearNode.secondChildOffset = flatten(node->children[1]);
		}
		delete node;
		return myOffset;
	};

	flatten(root);
}

bvh::~bvh() {}

inline bool bvh::intersect_aabb_fast(const aabb &box, const vec3 &ray_origin, const vec3 &inv_dir,
									 const int dir_is_neg[3], float t_hit) const
{
	float b_min_x = (dir_is_neg[0] ? box.max.get_x() : box.min.get_x());
	float b_max_x = (dir_is_neg[0] ? box.min.get_x() : box.max.get_x());
	float t_min = (b_min_x - ray_origin.get_x()) * inv_dir.get_x();
	float t_max = (b_max_x - ray_origin.get_x()) * inv_dir.get_x();

	float b_min_y = (dir_is_neg[1] ? box.max.get_y() : box.min.get_y());
	float b_max_y = (dir_is_neg[1] ? box.min.get_y() : box.max.get_y());
	float ty_min = (b_min_y - ray_origin.get_y()) * inv_dir.get_y();
	float ty_max = (b_max_y - ray_origin.get_y()) * inv_dir.get_y();

	if ((t_min > ty_max) || (ty_min > t_max))
		return false;
	if (ty_min > t_min)
		t_min = ty_min;
	if (ty_max < t_max)
		t_max = ty_max;

	float b_min_z = (dir_is_neg[2] ? box.max.get_z() : box.min.get_z());
	float b_max_z = (dir_is_neg[2] ? box.min.get_z() : box.max.get_z());
	float tz_min = (b_min_z - ray_origin.get_z()) * inv_dir.get_z();
	float tz_max = (b_max_z - ray_origin.get_z()) * inv_dir.get_z();

	if ((t_min > tz_max) || (tz_min > t_max))
		return false;
	if (tz_min > t_min)
		t_min = tz_min;
	if (tz_max < t_max)
		t_max = tz_max;

	return (t_min < t_hit) && (t_max > 0.0f);
}

bool bvh::intersect(simd_vec3 &calculator, simd_mat4 &calculator_m,
					const vec3 &rayOrigin, const vec3 &rayDir,
					float raytime, float &t_hit, shape **hit_shape, int &hit_id,
					bool culling, const float EPSILON, bool any_hit, float stop_t) const
{
	t_hit = stop_t;
	*hit_shape = nullptr;
	hit_id = -1;

	for (auto s : plane_shapes)
	{
		float t_candidate;
		if (s->intersect(calculator, calculator_m, rayOrigin, rayDir,
						 t_candidate, -1, raytime, culling, EPSILON))
		{
			if (t_candidate > EPSILON && t_candidate < t_hit)
			{
				t_hit = t_candidate;
				*hit_shape = s;
				hit_id = -1;
				if (any_hit)
					return true;
			}
		}
	}

	if (nodes.empty())
		return (*hit_shape != nullptr);

	vec3 ro = rayOrigin;
	ro.store();
	vec3 rd = rayDir;
	rd.store();

	vec3 invDir;
	invDir.load(1.0f / rd.get_x(), 1.0f / rd.get_y(), 1.0f / rd.get_z());
	invDir.store();

	int dirIsNeg[3] = {invDir.get_x() < 0, invDir.get_y() < 0, invDir.get_z() < 0};

	int nodesToVisit[64];
	int toVisitOffset = 0;
	nodesToVisit[0] = 0;

	while (toVisitOffset >= 0)
	{
		int currentNodeIndex = nodesToVisit[toVisitOffset--];
		const BVHNode &node = nodes[currentNodeIndex];

		if (intersect_aabb_fast(node.bounds, ro, invDir, dirIsNeg, t_hit))
		{

			if (node.nPrimitives > 0)
			{
				for (int i = 0; i < node.nPrimitives; ++i)
				{
					const BVHPrimitive &prim = ordered_primitives[node.primitivesOffset + i];
					float t_candidate;

					if (prim.s->intersect(calculator, calculator_m, rayOrigin, rayDir,
										  t_candidate, prim.id, raytime, culling, EPSILON))
					{
						if (t_candidate > EPSILON && t_candidate < t_hit)
						{
							t_hit = t_candidate;
							*hit_shape = prim.s;
							hit_id = prim.id;
							if (any_hit)
								return true;
						}
					}
				}
			}
			else
			{

				int axis = node.axis;
				int nearIndex, farIndex;

				if (dirIsNeg[axis])
				{
					nearIndex = node.secondChildOffset;
					farIndex = currentNodeIndex + 1;
				}
				else
				{
					nearIndex = currentNodeIndex + 1;
					farIndex = node.secondChildOffset;
				}

				nodesToVisit[++toVisitOffset] = farIndex;
				nodesToVisit[++toVisitOffset] = nearIndex;
			}
		}
	}

	return (*hit_shape != nullptr);
}