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

	calculate_aabb(calculator);

	if (!shapes || shapes->empty())
		return;

	const int n_shapes = (int)shapes->size();
	const float s = std::cbrt(std::max(1, n_shapes));
	const float voxels_per_axis = std::max(1.0f, s * 2.0f);

	world_size.store();
	const float wx = world_size.get_x();
	const float wy = world_size.get_y();
	const float wz = world_size.get_z();
	const float maxW = std::max(wx, std::max(wy, wz));

	int nx = (int)std::max(1.0f, std::floor(voxels_per_axis * (wx / maxW)));
	int ny = (int)std::max(1.0f, std::floor(voxels_per_axis * (wy / maxW)));
	int nz = (int)std::max(1.0f, std::floor(voxels_per_axis * (wz / maxW)));

	const int MAX_DIM = 1024;
	nx = clamp_int(nx, 1, MAX_DIM);
	ny = clamp_int(ny, 1, MAX_DIM);
	nz = clamp_int(nz, 1, MAX_DIM);

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
		s->getBoundingBox(sa);
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
					const size_t index = (size_t)ix + (size_t)nx * ((size_t)iy + (size_t)ny * (size_t)iz);
					cells[index].shapes.push_back(s);
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

void grid::calculate_aabb(simd_vec3 &calculator)
{
	const float INF = std::numeric_limits<float>::infinity();
	const float aabb_pad = 1e-4f;

	bounds.min.load(INF, INF, INF);
	bounds.max.load(-INF, -INF, -INF);

	aabb sa;
	for (auto s : *shapes)
	{
		s->getBoundingBox(sa);

		calculator.min(bounds.min, sa.min, bounds.min);
		calculator.max(bounds.max, sa.max, bounds.max);
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
					 const vec3 &rayOrigin,
					 const vec3 &rayDir,
					 float &t_hit,
					 shape **hit_shape,
					 bool culling,
					 const float EPSILON) const
{
	t_hit = std::numeric_limits<float>::infinity();
	*hit_shape = nullptr;

	if (!cells || !shapes || shapes->empty())
		return false;

	vec3 ro = rayOrigin;
	ro.store();
	vec3 rd = rayDir;
	rd.store();

	float tmin_box = 0;
	float tmax_box = 0;

	if (!intersect_ray_aabb(calculator, ro, rd, tmin_box, tmax_box, EPSILON))
	{
		return false;
	}

	float t = std::max(tmin_box, 0.0f);

	vec3 pos;
	calculator.mult_scalar(rd, t, pos);
	pos.store();
	calculator.add(ro, pos, pos);
	pos.store();

	const int nx = (int)grid_dim.get_x();
	const int ny = (int)grid_dim.get_y();
	const int nz = (int)grid_dim.get_z();

	int ix = get_cell_index(pos.get_x(), bounds.min.get_x(), inv_cell_size.get_x(), grid_dim.get_x());
	int iy = get_cell_index(pos.get_y(), bounds.min.get_y(), inv_cell_size.get_y(), grid_dim.get_y());
	int iz = get_cell_index(pos.get_z(), bounds.min.get_z(), inv_cell_size.get_z(), grid_dim.get_z());

	int stepX = (rd.get_x() > 0.0f) ? 1 : ((rd.get_x() < 0.0f) ? -1 : 0);
	int stepY = (rd.get_y() > 0.0f) ? 1 : ((rd.get_y() < 0.0f) ? -1 : 0);
	int stepZ = (rd.get_z() > 0.0f) ? 1 : ((rd.get_z() < 0.0f) ? -1 : 0);

	const float cellSizeX = world_size.get_x() / (float)nx;
	const float cellSizeY = world_size.get_y() / (float)ny;
	const float cellSizeZ = world_size.get_z() / (float)nz;

	const float INF = std::numeric_limits<float>::infinity();
	const float invDirX = (std::fabs(rd.get_x()) < EPSILON) ? INF : 1.0f / rd.get_x();
	const float invDirY = (std::fabs(rd.get_y()) < EPSILON) ? INF : 1.0f / rd.get_y();
	const float invDirZ = (std::fabs(rd.get_z()) < EPSILON) ? INF : 1.0f / rd.get_z();

	float tMaxX = (stepX == 0) ? INF : get_t_max(ro.get_x(), ix, stepX, cellSizeX, bounds.min.get_x(), invDirX);
	float tMaxY = (stepY == 0) ? INF : get_t_max(ro.get_y(), iy, stepY, cellSizeY, bounds.min.get_y(), invDirY);
	float tMaxZ = (stepZ == 0) ? INF : get_t_max(ro.get_z(), iz, stepZ, cellSizeZ, bounds.min.get_z(), invDirZ);

	const float tDeltaX = (stepX == 0) ? INF : (cellSizeX * std::fabs(invDirX));
	const float tDeltaY = (stepY == 0) ? INF : (cellSizeY * std::fabs(invDirY));
	const float tDeltaZ = (stepZ == 0) ? INF : (cellSizeZ * std::fabs(invDirZ));

	while (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz)
	{
		const size_t cellIndex = (size_t)ix + (size_t)nx * ((size_t)iy + (size_t)ny * (size_t)iz);
		const auto &cell = cells[cellIndex];

		float local_best_t = t_hit;
		shape *local_best_shape = *hit_shape;

		for (auto s : cell.shapes)
		{
			float t_candidate;
			if (s->intersect(calculator, rayOrigin, rayDir, t_candidate, culling, EPSILON))
			{
				if (t_candidate <= EPSILON)
					continue;

				if (t_candidate >= tmin_box - EPSILON && t_candidate <= tmax_box + EPSILON)
				{
					if (t_candidate < local_best_t)
					{
						local_best_t = t_candidate;
						local_best_shape = s;
					}
				}
			}
		}

		if (local_best_shape != *hit_shape)
		{
			t_hit = local_best_t;
			*hit_shape = local_best_shape;
		}

		const float earliestNextT = std::min(std::min(tMaxX, tMaxY), tMaxZ);

		if (*hit_shape && t_hit < earliestNextT)
		{
			return true;
		}

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

		if (earliestNextT > tmax_box)
			break;
	}

	return (*hit_shape != nullptr);
}
