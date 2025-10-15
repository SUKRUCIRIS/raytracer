#pragma once
#include <immintrin.h> //sse2

struct vec3
{
	float x, y, z;
};

class simd_calculator
{
private:
	__m128 s1;
	__m128 s2;
	__m128 d;

public:
	void vec3_mult(const vec3 &a, const vec3 &b, vec3 &c);
	void vec3_add(const vec3 &a, const vec3 &b, vec3 &c);
	void vec3_subs(const vec3 &a, const vec3 &b, vec3 &c);
	void vec3_div(const vec3 &a, const vec3 &b, vec3 &c);
	void vec3_scale(const vec3 &a, const float scale, vec3 &c);
	void vec3_dot(const vec3 &a, const vec3 &b, vec3 &c);
	void vec3_cross(const vec3 &a, const vec3 &b, vec3 &c);
};