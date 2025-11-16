#pragma once
#include "algebra.h"
#include <cfloat>
#include <vector>
#include <unordered_map>

struct aabb
{
	vec3 min;
	vec3 max;
};

struct point_light
{
	vec3 position;
	vec3 intensity;
};

enum material_type
{
	Regular,
	Mirror,
	Conductor,
	Dielectric
};

struct transformations
{
	std::vector<mat4> translations;
	std::vector<mat4> rotations;
	std::vector<mat4> scales;
	aabb box;
};

struct material
{
	material_type mt = Regular;
	vec3 AmbientReflectance{0, 0, 0};
	vec3 DiffuseReflectance{0, 0, 0};
	vec3 SpecularReflectance{0, 0, 0};
	float PhongExponent = 1;
	vec3 MirrorReflectance{0, 0, 0};
	vec3 AbsorptionCoefficient{0, 0, 0};
	float RefractionIndex = 0;
	float AbsorptionIndex = 1;
};

enum shape_type
{
	Triangle,
	Sphere,
	Plane
};

struct mesh_info
{
	mat4 model;
	mat4 inv_model;
	mat4 normal;
	material *mat;
};

struct all_mesh_infos
{
	std::unordered_map<int, mesh_info> mesh_infos;
};

class shape
{
protected:
	material *mat;
	shape_type t;
	aabb box;
	shape() = delete;
	shape(material *mat, shape_type t) : mat(mat), t(t) {};

public:
	virtual material *getMaterial(int id) const { return mat; };
	shape_type get_shapetype() const { return t; };
	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, bool culling = true, const float EPSILON = 1e-6f) const = 0;
	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const = 0;
	virtual void getBoundingBox(int id, simd_vec3 &calculator, simd_mat4 &calculator_m, aabb &box) const { box = this->box; };
	virtual std::vector<int> get_ids()
		const
	{
		return {123456};
	};
};

#ifndef GEOMETRY_SCALE_FACTOR
#define GEOMETRY_SCALE_FACTOR 1000.0f
#define INV_GEOMETRY_SCALE_FACTOR (1.0f / GEOMETRY_SCALE_FACTOR)
#endif

class triangle : public shape
{
private:
	vec3 c1, c2, c3;
	vec3 normal;
	vec3 c1_scaled, c2_scaled, c3_scaled;

	bool smooth = false;
	vec3 n1, n2, n3;
	all_mesh_infos *m = 0;

public:
	triangle() = delete;
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, material *mat, all_mesh_infos *m)
		: c1(*c1), c2(*c2), c3(*c3), shape(mat, shape_type::Triangle), smooth(false), m(m)
	{
		calculator.mult_scalar(*c1, GEOMETRY_SCALE_FACTOR, c1_scaled);
		calculator.mult_scalar(*c2, GEOMETRY_SCALE_FACTOR, c2_scaled);
		calculator.mult_scalar(*c3, GEOMETRY_SCALE_FACTOR, c3_scaled);

		vec3 edge1_scaled, edge2_scaled;

		calculator.subs(c2_scaled, c1_scaled, edge1_scaled);
		calculator.subs(c3_scaled, c1_scaled, edge2_scaled);

		calculator.cross(edge1_scaled, edge2_scaled, normal);

		calculator.normalize(normal, normal);

		calculator.min(*c1, *c2, box.min);
		calculator.min(box.min, *c3, box.min);

		calculator.max(*c1, *c2, box.max);
		calculator.max(box.max, *c3, box.max);

		vec3 pad = vec3(1e-4f);
		calculator.subs(box.min, pad, box.min);
		calculator.add(box.max, pad, box.max);

		normal.store();
		box.min.store();
		box.max.store();
	};
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3,
			 const vec3 &n1, const vec3 &n2, const vec3 &n3,
			 material *mat, all_mesh_infos *m)
		: c1(*c1), c2(*c2), c3(*c3), n1(n1), n2(n2), n3(n3),
		  shape(mat, shape_type::Triangle), smooth(true), m(m)
	{
		calculator.mult_scalar(*c1, GEOMETRY_SCALE_FACTOR, c1_scaled);
		calculator.mult_scalar(*c2, GEOMETRY_SCALE_FACTOR, c2_scaled);
		calculator.mult_scalar(*c3, GEOMETRY_SCALE_FACTOR, c3_scaled);

		calculator.min(*c1, *c2, box.min);
		calculator.min(box.min, *c3, box.min);

		calculator.max(*c1, *c2, box.max);
		calculator.max(box.max, *c3, box.max);

		vec3 pad = vec3(1e-4f);
		calculator.subs(box.min, pad, box.min);
		calculator.add(box.max, pad, box.max);

		this->n1.store();
		this->n2.store();
		this->n3.store();
		box.min.store();
		box.max.store();
	}
	virtual material *getMaterial(int id)
		const override
	{
		return m->mesh_infos[id].mat;
	};
	virtual void getBoundingBox(int id, simd_vec3 &calculator, simd_mat4 &calculator_m, aabb &box)
		const override
	{
		const mesh_info &mi = m->mesh_infos[id];
		vec3 v0 = c1, v1 = c2, v2 = c3;
		calculator_m.mult_vec(mi.model, v0, v0, false);
		calculator_m.mult_vec(mi.model, v1, v1, false);
		calculator_m.mult_vec(mi.model, v2, v2, false);

		calculator.min(v0, v1, box.min);
		calculator.min(box.min, v2, box.min);

		calculator.max(v0, v1, box.max);
		calculator.max(box.max, v2, box.max);

		vec3 pad = vec3(1e-4f);
		calculator.subs(box.min, pad, box.min);
		calculator.add(box.max, pad, box.max);
		box.min.store();
		box.max.store();
	}
	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal)
		const override
	{
		const mesh_info &mi = m->mesh_infos[id];
		if (!smooth)
		{
			vec3 tmp = this->normal;
			calculator_m.mult_vec(mi.normal, tmp, normal, true);
			calculator.normalize(normal, normal);
			normal.store();
			return;
		}

		vec3 hit_point_obj;
		hit_point.store();
		calculator_m.mult_vec(mi.inv_model, hit_point, hit_point_obj, false);

		vec3 v0, v1, v2;
		calculator.subs(c2, c1, v0);
		calculator.subs(c3, c1, v1);
		calculator.subs(hit_point_obj, c1, v2);

		float d00, d01, d11, d20, d21;
		calculator.dot(v0, v0, d00);
		calculator.dot(v0, v1, d01);
		calculator.dot(v1, v1, d11);
		calculator.dot(v2, v0, d20);
		calculator.dot(v2, v1, d21);

		const float denom = d00 * d11 - d01 * d01;
		float v = (d11 * d20 - d01 * d21) / denom;
		float w = (d00 * d21 - d01 * d20) / denom;
		float u = 1.0f - v - w;

		vec3 n_interp;
		vec3 t1, t2;
		calculator.mult_scalar(n1, u, n_interp);
		calculator.mult_scalar(n2, v, t1);
		calculator.add(n_interp, t1, n_interp);
		calculator.mult_scalar(n3, w, t2);
		calculator.add(n_interp, t2, n_interp);
		calculator.normalize(n_interp, normal);
		normal.store();
		calculator_m.mult_vec(mi.normal, normal, normal, true);
		calculator.normalize(normal, normal);
		normal.store();
	};
	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual std::vector<int> get_ids()
		const override
	{
		std::vector<int> keys;
		keys.reserve(m->mesh_infos.size());

		for (const auto &[key, value] : m->mesh_infos)
		{
			keys.push_back(key);
		}

		return keys;
	};
};

class sphere : public shape
{
private:
	vec3 center;
	float radius;
	mat4 model;
	mat4 inv_model;
	mat4 normal_m;

public:
	sphere() = delete;
	sphere(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 *center, float radius, material *mat, mat4 model,
		   mat4 inv_model, mat4 normal_m)
		: center(*center), radius(radius), shape(mat, shape_type::Sphere), model(model), inv_model(inv_model), normal_m(normal_m)
	{
		vec3 rvec(radius);
		calculator.subs(*center, rvec, box.min);
		calculator.add(*center, rvec, box.max);

		box.min.store();
		box.max.store();

		vec3 corners[8];
		corners[0].load(box.min.get_x(), box.min.get_y(), box.min.get_z(), 1.0f);
		corners[1].load(box.max.get_x(), box.min.get_y(), box.min.get_z(), 1.0f);
		corners[2].load(box.min.get_x(), box.max.get_y(), box.min.get_z(), 1.0f);
		corners[3].load(box.max.get_x(), box.max.get_y(), box.min.get_z(), 1.0f);
		corners[4].load(box.min.get_x(), box.min.get_y(), box.max.get_z(), 1.0f);
		corners[5].load(box.max.get_x(), box.min.get_y(), box.max.get_z(), 1.0f);
		corners[6].load(box.min.get_x(), box.max.get_y(), box.max.get_z(), 1.0f);
		corners[7].load(box.max.get_x(), box.max.get_y(), box.max.get_z(), 1.0f);

		vec3 world_min(FLT_MAX, FLT_MAX, FLT_MAX, 1.0f);
		vec3 world_max(-FLT_MAX, -FLT_MAX, -FLT_MAX, 1.0f);

		for (int i = 0; i < 8; ++i)
		{
			vec3 world_corner;
			calculator_m.mult_vec(model, corners[i], world_corner, false);

			calculator.min(world_min, world_corner, world_min);
			calculator.max(world_max, world_corner, world_max);
		}

		box.min = world_min;
		box.max = world_max;

		vec3 pad(1e-4f);
		calculator.subs(box.min, pad, box.min);
		calculator.add(box.max, pad, box.max);

		box.min.store();
		box.max.store();
	};
	vec3 get_center() const { return center; };
	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal)
		const override
	{
		vec3 hit_point_obj;
		vec3 tmp_hit = hit_point;
		tmp_hit.store();
		calculator_m.mult_vec(inv_model, tmp_hit, hit_point_obj, false);

		vec3 normal_obj;
		calculator.subs(hit_point_obj, center, normal_obj);

		normal_obj.store();
		calculator_m.mult_vec(normal_m, normal_obj, normal, true);

		calculator.normalize(normal, normal);
		normal.store();
	};
};

class plane : public shape
{
private:
	vec3 point;
	vec3 normal;

public:
	plane() = delete;
	plane(vec3 *point, vec3 *normal, material *mat)
		: point(*point), normal(*normal), shape(mat, shape_type::Plane)
	{
		const float M = FLT_MAX;
		box.min = vec3(-M, -M, -M);
		box.max = vec3(M, M, M);
	};
	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal)
		const override { normal = this->normal; };
	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, bool culling = true, const float EPSILON = 1e-6f) const override;
};
