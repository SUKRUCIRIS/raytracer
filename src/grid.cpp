#include "grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>

constexpr int clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

grid::grid(const std::vector<shape *> *shape_list)
	: shapes(shape_list), cells(nullptr)
{
	if (!shapes || shapes->empty())
		return;

	simd_vec3 calculator;
	simd_mat4 calculator_m(calculator);
	calculate_aabb(calculator, calculator_m);

	if (!shapes || shapes->empty())
		return;

	int n_shapes = 0;

	for (auto &&i : *shapes)
	{
		n_shapes += i->get_ids().size();
	}

	world_size.store();
	const float wx = world_size.get_x();
	const float wy = world_size.get_y();
	const float wz = world_size.get_z();

	const float alpha = 10.0f;

	const float n_total_target = std::max(1.0f, alpha * n_shapes);
	const float wbb_volume = wx * wy * wz;
	const float cbrt_n_total = std::cbrt(n_total_target);
	const float cbrt_wbb_volume = std::cbrt(wbb_volume);

	const int N_min = 2;
	int nx = std::max(N_min, (int)std::floor((wx / cbrt_wbb_volume) * cbrt_n_total));
	int ny = std::max(N_min, (int)std::floor((wy / cbrt_wbb_volume) * cbrt_n_total));
	int nz = std::max(N_min, (int)std::floor((wz / cbrt_wbb_volume) * cbrt_n_total));

	my_printf("Grid dimensions: %d %d %d\n", nx, ny, nz);

	grid_dim = vec3((float)nx, (float)ny, (float)nz);
	grid_dim.store();

	const float cellSizeX = wx / (float)nx;
	const float cellSizeY = wy / (float)ny;
	const float cellSizeZ = wz / (float)nz;

	inv_cell_size = vec3(1.0f / cellSizeX, 1.0f / cellSizeY, 1.0f / cellSizeZ);
	inv_cell_size.store();

	const size_t cell_count = (size_t)nx * (size_t)ny * (size_t)nz;
	cells = new (std::nothrow) gridcell[cell_count];
	if (!cells)
		return;

	aabb sa;
	for (auto s : *shapes)
	{
		std::vector<int> ids = s->get_ids();

		for (int id : ids)
		{
			s->getBoundingBox(id, calculator, calculator_m, sa);
			sa.min.store();
			sa.max.store();

			int ix0 = get_cell_index(sa.min.get_x(), bounds.min.get_x(), inv_cell_size.get_x(), grid_dim.get_x());
			int iy0 = get_cell_index(sa.min.get_y(), bounds.min.get_y(), inv_cell_size.get_y(), grid_dim.get_y());
			int iz0 = get_cell_index(sa.min.get_z(), bounds.min.get_z(), inv_cell_size.get_z(), grid_dim.get_z());

			int ix1 = get_cell_index(sa.max.get_x(), bounds.min.get_x(), inv_cell_size.get_x(), grid_dim.get_x());
			int iy1 = get_cell_index(sa.max.get_y(), bounds.min.get_y(), inv_cell_size.get_y(), grid_dim.get_y());
			int iz1 = get_cell_index(sa.max.get_z(), bounds.min.get_z(), inv_cell_size.get_z(), grid_dim.get_z());

			ix0 = clamp_int(ix0, 0, nx - 1);
			iy0 = clamp_int(iy0, 0, ny - 1);
			iz0 = clamp_int(iz0, 0, nz - 1);
			ix1 = clamp_int(ix1, 0, nx - 1);
			iy1 = clamp_int(iy1, 0, ny - 1);
			iz1 = clamp_int(iz1, 0, nz - 1);

			for (int iz = iz0; iz <= iz1; ++iz)
			{
				for (int iy = iy0; iy <= iy1; ++iy)
				{
					for (int ix = ix0; ix <= ix1; ++ix)
					{
						const size_t index =
							(size_t)ix + (size_t)nx * ((size_t)iy + (size_t)ny * (size_t)iz);
						cells[index].shapes.push_back(s);
						cells[index].ids.push_back(id);
					}
				}
			}
		}
	}
}

grid::~grid()
{
	if (cells)
	{
		delete[] cells;
		cells = nullptr;
	}
}

void grid::calculate_aabb(simd_vec3 &calculator, simd_mat4 &calculator_m)
{
	const float INF = std::numeric_limits<float>::infinity();
	const float aabb_pad = 1e-4f;

	bounds.min.load(INF, INF, INF);
	bounds.max.load(-INF, -INF, -INF);

	aabb sa;
	for (auto s : *shapes)
	{
		std::vector<int> ids = s->get_ids();
		if (s->get_shapetype() != Plane)
		{
			for (int id : ids)
			{
				s->getBoundingBox(id, calculator, calculator_m, sa);

				calculator.min(bounds.min, sa.min, bounds.min);
				calculator.max(bounds.max, sa.max, bounds.max);
			}
		}
		else
		{
			plane_shapes.push_back(s);
		}
	}

	vec3 pad(aabb_pad);
	calculator.subs(bounds.min, pad, bounds.min);
	calculator.add(bounds.max, pad, bounds.max);
	bounds.min.store();
	bounds.max.store();

	calculator.subs(bounds.max, bounds.min, world_size);
	world_size.store();
}

int grid::get_cell_index(float pos, float grid_min, float inv_cell_sz, float grid_dim_sz) const
{
	float f = (pos - grid_min) * inv_cell_sz;
	int idx = (int)std::floor(f);
	if (idx < 0)
		idx = 0;
	int maxIdx = (int)grid_dim_sz - 1;
	if (idx > maxIdx)
		idx = maxIdx;
	return idx;
}

float grid::get_t_max(float start, int cell_idx, int step, float cell_sz, float grid_min, float inv_dir) const
{
	if (step > 0)
	{
		float boundary = grid_min + (cell_idx + 1) * cell_sz;
		return (boundary - start) * inv_dir;
	}
	else
	{
		float boundary = grid_min + (cell_idx)*cell_sz;
		return (boundary - start) * inv_dir;
	}
}

bool grid::intersect_ray_aabb(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir,
							  float &t_min, float &t_max, const float EPS) const
{
	vec3 bounds_min_scaled, bounds_max_scaled, ro_scaled;
	calculator.mult_scalar(bounds.min, GEOMETRY_SCALE_FACTOR, bounds_min_scaled);
	calculator.mult_scalar(bounds.max, GEOMETRY_SCALE_FACTOR, bounds_max_scaled);
	calculator.mult_scalar(rayOrigin, GEOMETRY_SCALE_FACTOR, ro_scaled);

	vec3 invDir;
	vec3 one(1.0f);
	calculator.div(one, rayDir, invDir);

	vec3 t_min_vec, t_max_vec;
	vec3 t1, t2;

	calculator.subs(bounds_min_scaled, ro_scaled, t1);
	calculator.mult(t1, invDir, t1);

	calculator.subs(bounds_max_scaled, ro_scaled, t2);
	calculator.mult(t2, invDir, t2);

	calculator.min(t1, t2, t_min_vec);
	calculator.max(t1, t2, t_max_vec);

	t_min_vec.store();
	float tmin = std::max(std::max(t_min_vec.get_x(), t_min_vec.get_y()), t_min_vec.get_z());

	t_max_vec.store();
	float tmax = std::min(std::min(t_max_vec.get_x(), t_max_vec.get_y()), t_max_vec.get_z());

	if (tmin > tmax + EPS)
	{
		return false;
	}

	t_min = tmin * INV_GEOMETRY_SCALE_FACTOR;
	t_max = tmax * INV_GEOMETRY_SCALE_FACTOR;

	return true;
}

bool grid::intersect(simd_vec3 &calculator,
					 simd_mat4 &calculator_m,
					 const vec3 &rayOrigin,
					 const vec3 &rayDir,
					 float raytime,
					 float &t_hit,
					 shape **hit_shape,
					 int &hit_id,
					 bool culling,
					 const float EPSILON,
					 bool any_hit,
					 float stop_t) const
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

	if (!shapes || shapes->empty() || !cells)
	{
		return (*hit_shape != nullptr);
	}

	vec3 ro = rayOrigin;
	vec3 rd = rayDir;
	ro.store();
	rd.store();

	float tmin_box = 0;
	float tmax_box = 0;
	const bool aabb_hit = intersect_ray_aabb(calculator, ro, rd, tmin_box, tmax_box, EPSILON);

	if (!aabb_hit)
	{
		return (*hit_shape != nullptr);
	}

	if (tmin_box > t_hit)
	{
		return (*hit_shape != nullptr);
	}

	const int nx = (int)grid_dim.get_x();
	const int ny = (int)grid_dim.get_y();
	const int nz = (int)grid_dim.get_z();

	if (nx == 1 && ny == 1 && nz == 1)
	{
		const auto &cell = cells[0];
		for (size_t i = 0; i < cell.shapes.size(); ++i)
		{
			auto s = cell.shapes[i];
			int id = cell.ids[i];

			float t_candidate;
			if (s->intersect(calculator, calculator_m, rayOrigin, rayDir,
							 t_candidate, id, raytime, culling, EPSILON))
			{
				if (t_candidate > EPSILON && t_candidate < t_hit)
				{
					t_hit = t_candidate;
					*hit_shape = s;
					hit_id = id;
					if (any_hit)
						return true;
				}
			}
		}
		return (*hit_shape != nullptr);
	}

	float t = std::max(tmin_box, 0.0f);

	vec3 pos;
	calculator.mult_scalar(rd, t, pos);
	calculator.add(ro, pos, pos);
	pos.store();

	int ix = get_cell_index(pos.get_x(), bounds.min.get_x(), inv_cell_size.get_x(), grid_dim.get_x());
	int iy = get_cell_index(pos.get_y(), bounds.min.get_y(), inv_cell_size.get_y(), grid_dim.get_y());
	int iz = get_cell_index(pos.get_z(), bounds.min.get_z(), inv_cell_size.get_z(), grid_dim.get_z());

	const int stepX = (rd.get_x() > 0.0f) ? 1 : ((rd.get_x() < 0.0f) ? -1 : 0);
	const int stepY = (rd.get_y() > 0.0f) ? 1 : ((rd.get_y() < 0.0f) ? -1 : 0);
	const int stepZ = (rd.get_z() > 0.0f) ? 1 : ((rd.get_z() < 0.0f) ? -1 : 0);

	const float cellSizeX = world_size.get_x() / (float)nx;
	const float cellSizeY = world_size.get_y() / (float)ny;
	const float cellSizeZ = world_size.get_z() / (float)nz;

	const float INF = std::numeric_limits<float>::infinity();

	const float invDirX = 1.0f / rd.get_x();
	const float invDirY = 1.0f / rd.get_y();
	const float invDirZ = 1.0f / rd.get_z();

	float tMaxX = (stepX == 0) ? INF : get_t_max(ro.get_x(), ix, stepX, cellSizeX, bounds.min.get_x(), invDirX);
	float tMaxY = (stepY == 0) ? INF : get_t_max(ro.get_y(), iy, stepY, cellSizeY, bounds.min.get_y(), invDirY);
	float tMaxZ = (stepZ == 0) ? INF : get_t_max(ro.get_z(), iz, stepZ, cellSizeZ, bounds.min.get_z(), invDirZ);

	const float tDeltaX = (stepX == 0) ? INF : (cellSizeX * fabs(invDirX));
	const float tDeltaY = (stepY == 0) ? INF : (cellSizeY * fabs(invDirY));
	const float tDeltaZ = (stepZ == 0) ? INF : (cellSizeZ * fabs(invDirZ));

	while (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz)
	{
		const size_t cellIndex = (size_t)ix + (size_t)nx * ((size_t)iy + (size_t)ny * (size_t)iz);
		const auto &cell = cells[cellIndex];

		for (size_t i = 0; i < cell.shapes.size(); ++i)
		{
			auto s = cell.shapes[i];
			int id = cell.ids[i];

			float t_candidate;
			if (s->intersect(calculator, calculator_m, rayOrigin, rayDir,
							 t_candidate, id, raytime, culling, EPSILON))
			{
				if (t_candidate > EPSILON && t_candidate < t_hit)
				{
					t_hit = t_candidate;
					*hit_shape = s;
					hit_id = id;
					if (any_hit)
						return true;
				}
			}
		}

		const float earliestNextT = std::min({tMaxX, tMaxY, tMaxZ});

		if (*hit_shape && t_hit < earliestNextT)
			break;

		if (earliestNextT > tmax_box)
			break;

		if (earliestNextT > t_hit)
			break;

		if (tMaxX <= tMaxY && tMaxX <= tMaxZ)
		{
			ix += stepX;
			tMaxX += tDeltaX;
		}
		else if (tMaxY <= tMaxZ)
		{
			iy += stepY;
			tMaxY += tDeltaY;
		}
		else
		{
			iz += stepZ;
			tMaxZ += tDeltaZ;
		}
	}

	return (*hit_shape != nullptr);
}