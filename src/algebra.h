#pragma once
#include <smmintrin.h> // SSE4.1
#include <stdio.h>
#include "helper.h"

class simd_vec3;

struct alignas(16) vec3
{
public:
	friend class simd_vec3;
	vec3() : vec(_mm_setzero_ps()), x(0), y(0), z(0), padding(0) {};
	vec3(float a) : x(a), y(a), z(a), vec(_mm_load_ps(&this->x)) {};
	vec3(float a, float b, float c) : x(a), y(b), z(c), vec(_mm_load_ps(&this->x)) {};
	vec3(float a, float b, float c, float d) : x(a), y(b), z(c), padding(d), vec(_mm_load_ps(&this->x)) {};
	void load(float a)
	{
		x = a;
		y = a;
		z = a;
		vec = _mm_load_ps(&this->x);
	}
	void load(float a, float b, float c, float d = 1)
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
	void set(__m128 *x)
	{
		vec = *x;
	}
	void set(__m128 x)
	{
		vec = x;
	}
	void set_padding(float d = 1.0f)
	{
		padding = d;
		vec = _mm_load_ps(&this->x);
	}
	float get_x() const { return x; };
	float get_y() const { return y; };
	float get_z() const { return z; };
	__m128 get_vec() const { return vec; };
	void print()
	{
		store();
		my_printf("\nx: %f y: %f z: %f p:%f\n", x, y, z, padding);
	}

private:
	float x = 0, y = 0, z = 0;
	float padding = 1;
	__m128 vec;
};

class simd_mat4;

struct mat4
{
private:
	friend class simd_mat4;
	vec3 row0, row1, row2, row3;

public:
	mat4() : row0(1.0f, 0.0f, 0.0f, 0.0f),
			 row1(0.0f, 1.0f, 0.0f, 0.0f),
			 row2(0.0f, 0.0f, 1.0f, 0.0f),
			 row3(0.0f, 0.0f, 0.0f, 1.0f) {};
	mat4(float a[16]) : row0(a[0], a[1], a[2], a[3]), row1(a[4], a[5], a[6], a[7]),
						row2(a[8], a[9], a[10], a[11]), row3(a[12], a[13], a[14], a[15]) {};
	mat4(vec3 &row0, vec3 &row1, vec3 &row2, vec3 &row3) : row0(row0), row1(row1), row2(row2), row3(row3) {};
	void print()
	{
		row0.store();
		row1.store();
		row2.store();
		row3.store();
		my_printf("Mat4:\n");
		row0.print();
		row1.print();
		row2.print();
		row3.print();
	}
};

class simd_mat4
{
private:
	simd_vec3 &calculator;

public:
	simd_mat4(simd_vec3 &calculator) : calculator(calculator) {};
	void translate(vec3 &t, mat4 &d);
	void rotate(vec3 &r, const float angle, mat4 &d);
	void scale(vec3 &s, mat4 &d);
	void mult(const mat4 &a, const mat4 &b, mat4 &d);
	void transpose(const mat4 &a, mat4 &d);
	void inverse(const mat4 &a, mat4 &d);
	void mult_vec(const mat4 &m, vec3 &v, vec3 &d, bool is_dir);
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
	void exp(const vec3 &a, vec3 &c);
};