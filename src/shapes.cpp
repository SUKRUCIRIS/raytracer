#include "shapes.h"
#include <math.h>

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