#include "shapes.h"
#include <math.h>
#include <cfloat>

bool triangle::intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling, const float EPSILON) const
{
	vec3 edge1, edge2, h, s, q;
	calculator.subs(*c2, *c1, edge1);
	calculator.subs(*c3, *c1, edge2);

	// h = cross(rayDir, edge2)
	calculator.cross(rayDir, edge2, h);

	// a = dot(edge1, h)
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

	// s = rayOrigin - c1
	calculator.subs(rayOrigin, *c1, s);

	// u = f * dot(s, h)
	float u;
	calculator.dot(s, h, u);
	u *= f;
	if (u < 0.0f || u > 1.0f)
		return false;

	// q = cross(s, edge1)
	calculator.cross(s, edge1, q);

	// v = f * dot(rayDir, q)
	float v;
	calculator.dot(rayDir, q, v);
	v *= f;
	if (v < 0.0f || u + v > 1.0f)
		return false;

	// t = f * dot(edge2, q)
	calculator.dot(edge2, q, t);
	t *= f;

	return t > EPSILON;
}

bool sphere::intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling, const float EPSILON) const
{
	vec3 oc;
	calculator.subs(rayOrigin, *center, oc); // oc = rayOrigin - center

	float b;
	calculator.dot(oc, rayDir, b); // b = dot(oc, rayDir)

	float c;
	calculator.dot(oc, oc, c); // c = dot(oc, oc)

	c -= radius * radius;

	// Quadratic discriminant: discriminant = b^2 - c
	float discriminant = b * b - c;

	if (discriminant < 0.0f)
		return false; // No intersection

	float sqrtDisc = sqrtf(discriminant);

	// Compute the nearest positive t
	float t1 = -b - sqrtDisc;
	float t2 = -b + sqrtDisc;

	if (t1 > EPSILON)
		t = t1;
	else if (t2 > EPSILON)
		t = t2;
	else
		return false; // Both intersections are behind the ray

	return true;
}

bool plane::intersect(simd_vec3 &calculator, const vec3 &rayOrigin, const vec3 &rayDir, float &t, bool culling, const float EPSILON) const
{
	float denom;
	calculator.dot(*normal, rayDir, denom); // denom = dot(N, D)

	// If denom is near zero, the ray is parallel to the plane
	if (fabs(denom) < EPSILON)
		return false;

	vec3 p0l0;
	calculator.subs(*point, rayOrigin, p0l0); // (P0 - O)

	float num;
	calculator.dot(p0l0, *normal, num); // numerator = (P0 - O)·N

	t = num / denom;

	// Intersection must be in front of the ray
	if (t < EPSILON)
		return false;

	return true;
}

aabb sphere::getBoundingBox(simd_vec3 &calculator) const
{
	aabb box;
	vec3 rvec(radius, radius, radius);

	calculator.subs(*center, rvec, box.min);
	calculator.add(*center, rvec, box.max);

	return box;
}

aabb triangle::getBoundingBox(simd_vec3 &calculator) const
{
	aabb box;
	vec3 temp;

	calculator.min(*c1, *c2, temp);
	calculator.min(temp, *c3, box.min);

	calculator.max(*c1, *c2, temp);
	calculator.max(temp, *c3, box.max);

	return box;
}

aabb plane::getBoundingBox(simd_vec3 &calculator) const
{
	const float M = FLT_MAX;
	return {vec3(-M, -M, -M), vec3(M, M, M)};
}