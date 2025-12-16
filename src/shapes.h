#pragma once
#include "algebra.h"
#include <cfloat>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "texture.h"

#ifndef GEOMETRY_SCALE_FACTOR
#define GEOMETRY_SCALE_FACTOR 1000.0f
#define INV_GEOMETRY_SCALE_FACTOR (1.0f / GEOMETRY_SCALE_FACTOR)
#endif

struct aabb
{
	vec3 min;
	vec3 max;
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
	float roughness = 0;
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
	vec3 motionblur;
	std::vector<texture *> textures;
};

struct all_mesh_infos
{
	std::unordered_map<int, mesh_info> mesh_infos;
};

class shape
{
protected:
	material *mat;
	std::vector<texture *> textures;
	shape_type t;
	aabb box;
	shape() = delete;
	shape(material *mat, shape_type t, std::vector<texture *> textures);

public:
	virtual material *getMaterial(int id) const;
	virtual std::vector<texture *> *getTextures(int id);
	virtual void calculate_uv(simd_vec3 &calculator, vec3 hit_point, float &u, float &v) const = 0;
	virtual void get_tangent(simd_vec3 &calculator, const vec3 &hit_point, vec3 &tangent) const = 0;
	shape_type get_shapetype() const;
	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, float time, bool culling = true, const float EPSILON = 1e-6f) const = 0;
	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const = 0;
	virtual void getBoundingBox(int id, simd_vec3 &calculator, simd_mat4 &calculator_m, aabb &box) const;
	virtual std::vector<int> get_ids() const;
};

class triangle : public shape
{
private:
	vec3 c1, c2, c3;
	vec3 normal;
	vec3 c1_scaled, c2_scaled, c3_scaled;

	bool smooth = false;
	vec3 n1, n2, n3;
	all_mesh_infos *m = 0;
	vec3 u, v;

public:
	triangle() = delete;
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, vec3 u, vec3 v, material *mat, all_mesh_infos *m);
	triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, vec3 u, vec3 v,
			 const vec3 &n1, const vec3 &n2, const vec3 &n3,
			 material *mat, all_mesh_infos *m);

	virtual material *getMaterial(int id) const override;

	virtual std::vector<texture *> *getTextures(int id) override;

	virtual void calculate_uv(simd_vec3 &calculator, vec3 hit_point, float &u, float &v) const override;

	virtual void get_tangent(simd_vec3 &calculator, const vec3 &hit_point, vec3 &tangent) const override;

	virtual void getBoundingBox(int id, simd_vec3 &calculator, simd_mat4 &calculator_m, aabb &box) const override;

	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const override;

	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, float time, bool culling = true, const float EPSILON = 1e-6f) const override;

	virtual std::vector<int> get_ids() const override;
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
	sphere(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 *center, float radius, material *mat, std::vector<texture *> textures, mat4 model,
		   mat4 inv_model, mat4 normal_m);

	vec3 get_center() const;

	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, float time, bool culling = true, const float EPSILON = 1e-6f) const override;

	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const override;

	virtual void calculate_uv(simd_vec3 &calculator, vec3 hit_point, float &u, float &v) const override;

	virtual void get_tangent(simd_vec3 &calculator, const vec3 &hit_point, vec3 &tangent) const override;
};

class plane : public shape
{
private:
	vec3 point;
	vec3 normal;

public:
	plane() = delete;
	plane(vec3 *point, vec3 *normal, material *mat, std::vector<texture *> textures);

	virtual void get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const override;

	virtual bool intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						   float &t, int id, float time, bool culling = true, const float EPSILON = 1e-6f) const override;

	virtual void calculate_uv(simd_vec3 &calculator, vec3 hit_point, float &u, float &v) const override;

	virtual void get_tangent(simd_vec3 &calculator, const vec3 &hit_point, vec3 &tangent) const override;
};