#pragma once
#include "algebra.h"
#include <cfloat>

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
};

// if not mesh id shall be -100 and only use model and normal
struct mesh_info
{
	int id;
	std::vector<mat4> models;
	std::vector<mat4> normals;
	std::vector<material *> mats;
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

class shape
{
protected:
	material *mat;
	shape_type t;
	aabb box;
	shape() = delete;
	shape(material *mat, shape_type t) : mat(mat), t(t) {};

public:
	material *getMaterial() const { return mat; };
	shape_type get_shapetype() const { return t; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const = 0;
	virtual void get_normal(simd_vec3 &calculator, const vec3 &hit_point, vec3 &normal) const = 0;
	void getBoundingBox(aabb &box) const { box = this->box; };
};

#ifndef GEOMETRY_SCALE_FACTOR
#define GEOMETRY_SCALE_FACTOR 1000.0f
#define INV_GEOMETRY_SCALE_FACTOR (1.0f / GEOMETRY_SCALE_FACTOR)
#endif

class triangle : public shape
{
private:
	vec3 *c1, *c2, *c3;
	vec3 normal;
	vec3 c1_scaled, c2_scaled, c3_scaled;

	bool smooth = false;
	vec3 n1, n2, n3;
	mesh_info *m = 0;

public:
	triangle() = delete;
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, material *mat, mesh_info *m)
		: c1(c1), c2(c2), c3(c3), shape(mat, shape_type::Triangle), smooth(false), m(m)
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

		const float aabb_pad = 1e-4f;
		calculator.subs(box.min, vec3(aabb_pad), box.min);
		calculator.add(box.max, vec3(aabb_pad), box.max);

		normal.store();
		box.min.store();
		box.max.store();
	};
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3,
			 const vec3 &n1, const vec3 &n2, const vec3 &n3,
			 material *mat, mesh_info *m)
		: c1(c1), c2(c2), c3(c3), n1(n1), n2(n2), n3(n3),
		  shape(mat, shape_type::Triangle), smooth(true), m(m)
	{
		calculator.mult_scalar(*c1, GEOMETRY_SCALE_FACTOR, c1_scaled);
		calculator.mult_scalar(*c2, GEOMETRY_SCALE_FACTOR, c2_scaled);
		calculator.mult_scalar(*c3, GEOMETRY_SCALE_FACTOR, c3_scaled);

		calculator.min(*c1, *c2, box.min);
		calculator.min(box.min, *c3, box.min);
		calculator.max(*c1, *c2, box.max);
		calculator.max(box.max, *c3, box.max);
		const float pad = 1e-4f;
		calculator.subs(box.min, vec3(pad), box.min);
		calculator.add(box.max, vec3(pad), box.max);

		this->n1.store();
		this->n2.store();
		this->n3.store();
		box.min.store();
		box.max.store();
	}
	virtual void get_normal(simd_vec3 &calculator, const vec3 &hit_point, vec3 &normal)
		const override
	{
		if (!smooth)
		{
			normal = this->normal;
			return;
		}

		vec3 v0, v1, v2;
		calculator.subs(*c2, *c1, v0);
		calculator.subs(*c3, *c1, v1);
		calculator.subs(hit_point, *c1, v2);

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
	};
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
};

class sphere : public shape
{
private:
	vec3 *center;
	float radius;

	mat4 model;
	mat4 normal;

public:
	sphere() = delete;
	sphere(simd_vec3 &calculator, vec3 *center, float radius, material *mat)
		: center(center), radius(radius), shape(mat, shape_type::Sphere)
	{
		vec3 rvec(radius, radius, radius);

		calculator.subs(*center, rvec, box.min);
		calculator.add(*center, rvec, box.max);

		const float aabb_pad = 1e-4f;
		calculator.subs(box.min, vec3(aabb_pad), box.min);
		calculator.add(box.max, vec3(aabb_pad), box.max);

		box.min.store();
		box.max.store();
	};
	vec3 *get_center() const { return center; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual void get_normal(simd_vec3 &calculator, const vec3 &hit_point, vec3 &normal) const override
	{
		calculator.subs(hit_point, *center, normal);
		calculator.normalize(normal, normal);
		normal.store();
	};
};

class plane : public shape
{
private:
	vec3 *point;
	vec3 *normal;

	mat4 model;
	mat4 normal;

public:
	plane() = delete;
	plane(vec3 *point, vec3 *normal, material *mat)
		: point(point), normal(normal), shape(mat, shape_type::Plane)
	{
		const float M = FLT_MAX;
		box.min = vec3(-M, -M, -M);
		box.max = vec3(M, M, M);
	};
	virtual void get_normal(simd_vec3 &calculator, const vec3 &hit_point, vec3 &normal) const override { normal = *this->normal; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
};
