#include "algebra.h"

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
	d = _mm_rcp_ps(b.vec);
	c.vec = _mm_mul_ps(a.vec, d);
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
	d = _mm_shuffle_ps(a.vec, a.vec, _MM_SHUFFLE(3, 0, 2, 1));
	d1 = _mm_shuffle_ps(b.vec, b.vec, _MM_SHUFFLE(3, 0, 2, 1));

	d2 = _mm_mul_ps(d, b.vec);
	d = _mm_mul_ps(a.vec, d1);
	d1 = _mm_sub_ps(d, d2);

	c.vec = _mm_shuffle_ps(d1, d1, _MM_SHUFFLE(3, 0, 2, 1));
}
void simd_vec3::length_squared(const vec3 &a, float &c)
{
	d = _mm_dp_ps(a.vec, a.vec, 0x71);
	c = _mm_cvtss_f32(d);
}
void simd_vec3::normalize(const vec3 &a, vec3 &c)
{
	d = _mm_dp_ps(a.vec, a.vec, 0x7F);
	d1 = _mm_rsqrt_ps(d);
	c.vec = _mm_mul_ps(a.vec, d1);
}