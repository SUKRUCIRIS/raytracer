#pragma once
#include "algebra.h"

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
private:
	material *mat;
	shape_type t;

public:
	shape() = delete;
	shape(material *mat, shape_type t) : mat(mat), t(t) {};
	material *getMaterial() const { return mat; };
	shape_type get_shapetype() const { return t; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const = 0;
	virtual aabb getBoundingBox(simd_vec3 &calculator) const = 0;
};

class triangle : public shape
{
private:
	vec3 *c1, *c2, *c3;
	vec3 normal;

public:
	triangle() = delete;
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, material *mat) : c1(c1), c2(c2), c3(c3), shape(mat, shape_type::Triangle)
	{
		vec3 edge1, edge2;
		calculator.subs(*c2, *c1, edge1);
		calculator.subs(*c3, *c1, edge2);
		calculator.cross(edge1, edge2, normal);
		calculator.normalize(normal, normal);
	};
	vec3 *get_normal() { return &normal; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual aabb getBoundingBox(simd_vec3 &calculator) const override;
};

class sphere : public shape
{
private:
	vec3 *center;
	float radius;

public:
	sphere() = delete;
	sphere(vec3 *center, float radius, material *mat) : center(center), radius(radius), shape(mat, shape_type::Sphere) {};
	vec3 *get_center() const { return center; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual aabb getBoundingBox(simd_vec3 &calculator) const override;
};

class plane : public shape
{
private:
	vec3 *point;
	vec3 *normal;

public:
	plane() = delete;
	plane(vec3 *point, vec3 *normal, material *mat) : point(point), normal(normal), shape(mat, shape_type::Plane) {};
	vec3 *get_normal() const { return normal; };
	virtual bool intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling = true, const float EPSILON = 1e-6f) const override;
	virtual aabb getBoundingBox(simd_vec3 &calculator) const override;
};
