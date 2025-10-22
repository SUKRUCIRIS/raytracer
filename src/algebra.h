#pragma once
#include <smmintrin.h> // SSE4.1
#include <stdio.h>

class simd_vec3;

struct alignas(16) vec3
{
public:
	friend class simd_vec3;
	vec3() : vec(_mm_setzero_ps()) {};
	vec3(float a, float b, float c) : x(a), y(b), z(c), vec(_mm_load_ps(&this->x)) {};
	void load(float a, float b, float c, float d = 0)
	{
		x = a;
		y = b;
		z = c;
		padding = d;
		vec = _mm_load_ps(&this->x);
	}
	void store()
	{
		_mm_store_ps(&this->x, vec);
	}
	float get_x() const { return x; };
	float get_y() const { return y; };
	float get_z() const { return z; };
	void print() const
	{
		printf("\nmem: %p x: %f y: %f z: %f\n", this, x, y, z);
	}

private:
	float x = 0, y = 0, z = 0;
	float padding = 0;
	__m128 vec;
};

class simd_vec3
{
private:
	__m128 d, d1, d2;

public:
	void mult(const vec3 &a, const vec3 &b, vec3 &c);
	void add(const vec3 &a, const vec3 &b, vec3 &c);
	void subs(const vec3 &a, const vec3 &b, vec3 &c);
	void div(const vec3 &a, const vec3 &b, vec3 &c);
	void mult_scalar(const vec3 &a, const float s, vec3 &c);
	void add_scalar(const vec3 &a, const float s, vec3 &c);
	void dot(const vec3 &a, const vec3 &b, float &c);
	void cross(const vec3 &a, const vec3 &b, vec3 &c);
	void length_squared(const vec3 &a, float &c);
	void normalize(const vec3 &a, vec3 &c);
	void max(const vec3 &a, const vec3 &b, vec3 &c);
	void min(const vec3 &a, const vec3 &b, vec3 &c);
	void pow(const vec3 &a, const float b, vec3 &c);
};