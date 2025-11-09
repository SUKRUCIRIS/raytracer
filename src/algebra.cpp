#include "algebra.h"
#include "../third_party/sse_mathfun/sse_mathfun.h"

void simd_vec3::mult(const vec3 &a, const vec3 &b, vec3 &c)
{
	c.vec = _mm_mul_ps(a.vec, b.vec);
}
void simd_vec3::add(const vec3 &a, const vec3 &b, vec3 &c)
{
	c.vec = _mm_add_ps(a.vec, b.vec);
}
void simd_vec3::subs(const vec3 &a, const vec3 &b, vec3 &c)
{
	c.vec = _mm_sub_ps(a.vec, b.vec);
}
void simd_vec3::div(const vec3 &a, const vec3 &b, vec3 &c)
{
	c.vec = _mm_div_ps(a.vec, b.vec);
}
void simd_vec3::mult_scalar(const vec3 &a, const float s, vec3 &c)
{
	d = _mm_set_ps1(s);
	c.vec = _mm_mul_ps(a.vec, d);
}
void simd_vec3::add_scalar(const vec3 &a, const float s, vec3 &c)
{
	d = _mm_set_ps1(s);
	c.vec = _mm_add_ps(a.vec, d);
}
void simd_vec3::dot(const vec3 &a, const vec3 &b, float &c)
{
	d = _mm_dp_ps(a.vec, b.vec, 0x71);
	c = _mm_cvtss_f32(d);
}
void simd_vec3::cross(const vec3 &a, const vec3 &b, vec3 &c)
{
	d = _mm_mul_ps(_mm_shuffle_ps(a.vec, a.vec, _MM_SHUFFLE(3, 0, 2, 1)),
				   _mm_shuffle_ps(b.vec, b.vec, _MM_SHUFFLE(3, 1, 0, 2)));
	d1 = _mm_mul_ps(_mm_shuffle_ps(a.vec, a.vec, _MM_SHUFFLE(3, 1, 0, 2)),
					_mm_shuffle_ps(b.vec, b.vec, _MM_SHUFFLE(3, 0, 2, 1)));

	c.vec = _mm_sub_ps(d, d1);
}
void simd_vec3::length_squared(const vec3 &a, float &c)
{
	d = _mm_dp_ps(a.vec, a.vec, 0x71);
	c = _mm_cvtss_f32(d);
}
void simd_vec3::normalize(const vec3 &a, vec3 &c)
{
	d = _mm_dp_ps(a.vec, a.vec, 0x77);
	d2 = _mm_set_ps1(1e-12f);
	d1 = _mm_cmpgt_ps(d, d2);
	d = _mm_rsqrt_ps(d);
	d = _mm_and_ps(d, d1);
	c.vec = _mm_mul_ps(a.vec, d);
}

void simd_vec3::max(const vec3 &a, const vec3 &b, vec3 &c)
{
	c.vec = _mm_max_ps(a.vec, b.vec);
}

void simd_vec3::min(const vec3 &a, const vec3 &b, vec3 &c)
{
	c.vec = _mm_min_ps(a.vec, b.vec);
}

void simd_vec3::pow(const vec3 &a, const float b, vec3 &c)
{
	d = _mm_set1_ps(b);
	d1 = log_ps(a.vec);
	d2 = _mm_mul_ps(d, d1);
	c.vec = exp_ps(d2);
}

void simd_vec3::exp(const vec3 &a, vec3 &c)
{
	c.vec = exp_ps(a.vec);
}