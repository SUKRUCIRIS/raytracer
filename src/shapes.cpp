#include "shapes.h"
#include <math.h>

bool triangle::intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling, const float EPSILON) const
{
	vec3 rayOrigin_scaled;
	calculator.mult_scalar(rayOrigin, GEOMETRY_SCALE_FACTOR, rayOrigin_scaled);

	const vec3 &rayDir_scaled = rayDir;

	vec3 edge1, edge2, h, s, q;
	calculator.subs(c2_scaled, c1_scaled, edge1);
	calculator.subs(c3_scaled, c1_scaled, edge2);

	calculator.cross(rayDir_scaled, edge2, h);

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
	calculator.dot(rayDir_scaled, q, v);
	v *= f;
	if (v < (0.0f - EPSILON) || (u + v) > (1.0f + EPSILON))
		return false;

	calculator.dot(edge2, q, t);
	t *= f;

	t *= INV_GEOMETRY_SCALE_FACTOR;

	return t > EPSILON;
}

bool sphere::intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling, const float EPSILON) const
{
	vec3 oc;
	calculator.subs(rayOrigin, *center, oc);

	float b;
	calculator.dot(oc, rayDir, b);

	float c;
	calculator.dot(oc, oc, c);

	c -= radius * radius;

	float discriminant = b * b - c;

	if (discriminant < 0.0f)
		return false;

	float sqrtDisc = sqrtf(discriminant);

	float t1 = -b - sqrtDisc;
	float t2 = -b + sqrtDisc;

	if (t1 > EPSILON)
		t = t1;
	else if (t2 > EPSILON)
		t = t2;
	else
		return false;

	return true;
}

bool plane::intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling, const float EPSILON) const
{
	float denom;
	calculator.dot(*normal, rayDir, denom);

	if (fabs(denom) < EPSILON)
		return false;

	vec3 p0l0;
	calculator.subs(*point, rayOrigin, p0l0);

	float num;
	calculator.dot(p0l0, *normal, num);

	t = num / denom;

	if (t < EPSILON)
		return false;

	return true;
}