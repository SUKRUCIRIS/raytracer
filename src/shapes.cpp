#include "shapes.h"
#include <math.h>

shape::shape(material *mat, shape_type t) : mat(mat), t(t) {};

material *shape::getMaterial(int id) const { return mat; };

shape_type shape::get_shapetype() const { return t; };

void shape::getBoundingBox(int id, simd_vec3 &calculator, simd_mat4 &calculator_m, aabb &box) const { box = this->box; };

std::vector<int> shape::get_ids() const
{
	return {123456};
};

triangle::triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3, material *mat, all_mesh_infos *m)
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

triangle::triangle(simd_vec3 &calculator, vec3 *c1, vec3 *c2, vec3 *c3,
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

material *triangle::getMaterial(int id) const
{
	return m->mesh_infos[id].mat;
};

void triangle::getBoundingBox(int id, simd_vec3 &calculator, simd_mat4 &calculator_m, aabb &box) const
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

	vec3 min_moving, max_moving;
	calculator.add(box.min, mi.motionblur, min_moving);
	calculator.add(box.max, mi.motionblur, max_moving);

	calculator.min(box.min, min_moving, box.min);
	calculator.max(box.max, max_moving, box.max);

	vec3 pad = vec3(1e-4f);
	calculator.subs(box.min, pad, box.min);
	calculator.add(box.max, pad, box.max);
	box.min.store();
	box.max.store();
}

void triangle::get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const
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

std::vector<int> triangle::get_ids() const
{
	std::vector<int> keys;
	keys.reserve(m->mesh_infos.size());

	for (const auto &[key, value] : m->mesh_infos)
	{
		keys.push_back(key);
	}

	return keys;
};

bool triangle::intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
						 float &t, int id, float time, bool culling, const float EPSILON) const
{

	const mesh_info &mi = m->mesh_infos[id];

	vec3 offset;
	calculator.mult_scalar(mi.motionblur, time, offset);

	vec3 moved_origin;
	calculator.subs(rayOrigin, offset, moved_origin);

	vec3 rayOrigin_obj, rayDir_obj;
	vec3 tmp = moved_origin;
	tmp.store();
	calculator_m.mult_vec(mi.inv_model, tmp, rayOrigin_obj, false);
	tmp = rayDir;
	tmp.store();
	calculator_m.mult_vec(mi.inv_model, tmp, rayDir_obj, true);

	vec3 rayOrigin_scaled;
	calculator.mult_scalar(rayOrigin_obj, GEOMETRY_SCALE_FACTOR, rayOrigin_scaled);

	vec3 edge1, edge2, h, s, q;
	calculator.subs(c2_scaled, c1_scaled, edge1);
	calculator.subs(c3_scaled, c1_scaled, edge2);

	calculator.cross(rayDir_obj, edge2, h);

	float a;
	calculator.dot(edge1, h, a);

	if (culling)
	{
		if (a < EPSILON)
			return false;
	}
	else
	{
		if (fabsf(a) < EPSILON)
			return false;
	}

	float f = 1.0f / a;

	calculator.subs(rayOrigin_scaled, c1_scaled, s);

	float u;
	calculator.dot(s, h, u);
	u *= f;
	if (u < (0.0f - EPSILON) || u > (1.0f + EPSILON))
		return false;

	calculator.cross(s, edge1, q);

	float v;
	calculator.dot(rayDir_obj, q, v);
	v *= f;
	if (v < (0.0f - EPSILON) || (u + v) > (1.0f + EPSILON))
		return false;

	calculator.dot(edge2, q, t);
	t *= f;

	t *= INV_GEOMETRY_SCALE_FACTOR;

	return t > EPSILON;
}

sphere::sphere(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 *center, float radius, material *mat, mat4 model,
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

vec3 sphere::get_center() const { return center; };

void sphere::get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const
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

bool sphere::intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
					   float &t, int id, float time, bool culling, const float EPSILON) const
{
	vec3 rayOrigin_obj, rayDir_obj;
	vec3 tmp = rayOrigin;
	tmp.store();
	calculator_m.mult_vec(inv_model, tmp, rayOrigin_obj, false);
	tmp = rayDir;
	tmp.store();
	calculator_m.mult_vec(inv_model, tmp, rayDir_obj, true);

	vec3 oc;
	calculator.subs(rayOrigin_obj, center, oc);

	float a, b, c;
	calculator.dot(rayDir_obj, rayDir_obj, a);
	calculator.dot(oc, rayDir_obj, b);
	b *= 2.0f;
	calculator.dot(oc, oc, c);
	c -= (radius * radius);

	float discriminant = b * b - 4 * a * c;

	if (discriminant < 0.0f)
		return false;

	float sqrtDisc = sqrtf(discriminant);
	float inv2a = 1.0f / (2.0f * a);

	float t1 = (-b - sqrtDisc) * inv2a;
	float t2 = (-b + sqrtDisc) * inv2a;

	if (t1 > EPSILON)
		t = t1;
	else if (t2 > EPSILON)
		t = t2;
	else
		return false;

	return true;
}

plane::plane(vec3 *point, vec3 *normal, material *mat)
	: point(*point), normal(*normal), shape(mat, shape_type::Plane)
{
	const float M = FLT_MAX;
	box.min = vec3(-M, -M, -M);
	box.max = vec3(M, M, M);
};

void plane::get_normal(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, int id, vec3 &normal) const { normal = this->normal; };

bool plane::intersect(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &rayOrigin, const vec3 &rayDir,
					  float &t, int id, float time, bool culling, const float EPSILON) const
{
	float denom;
	calculator.dot(normal, rayDir, denom);

	if (fabs(denom) < EPSILON)
		return false;

	vec3 p0l0;
	calculator.subs(point, rayOrigin, p0l0);

	float num;
	calculator.dot(p0l0, normal, num);

	t = num / denom;

	if (t < EPSILON)
		return false;

	return true;
}