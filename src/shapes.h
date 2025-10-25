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

class triangle : public shape
{
private:
	vec3 *c1, *c2, *c3;
	vec3 normal;

public:
	triangle() = delete;
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, material *mat)
		: c1(c1), c2(c2), c3(c3), shape(mat, shape_type::Triangle)
	{
		vec3 edge1, edge2;
		calculator.subs(*c2, *c1, edge1);
		calculator.subs(*c3, *c1, edge2);
		calculator.cross(edge1, edge2, normal);
		calculator.normalize(normal, normal);

		calculator.min(*c1, *c2, edge1);
		calculator.min(edge1, *c3, box.min);

		calculator.max(*c1, *c2, edge1);
		calculator.max(edge1, *c3, box.max);
	};
	virtual void get_normal(simd_vec3 &calculator, const vec3 &hit_point, vec3 &normal) const override { normal = this->normal; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
};

class sphere : public shape
{
private:
	vec3 *center;
	float radius;

public:
	sphere() = delete;
	sphere(simd_vec3 &calculator, vec3 *center, float radius, material *mat)
		: center(center), radius(radius), shape(mat, shape_type::Sphere)
	{
		vec3 rvec(radius, radius, radius);

		calculator.subs(*center, rvec, box.min);
		calculator.add(*center, rvec, box.max);
	};
	vec3 *get_center() const { return center; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual void get_normal(simd_vec3 &calculator, const vec3 &hit_point, vec3 &normal) const override
	{
		calculator.subs(hit_point, *center, normal);
		calculator.normalize(normal, normal);
	};
};

class plane : public shape
{
private:
	vec3 *point;
	vec3 *normal;

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
