#include "algebra.h"
#include "../third_party/sse_mathfun/sse_mathfun.h"
#include <cmath>

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

void simd_mat4::translate(vec3 &t, mat4 &d)
{
	t.store();
	d.row0.load(1.0f, 0.0f, 0.0f, t.get_x());
	d.row1.load(0.0f, 1.0f, 0.0f, t.get_y());
	d.row2.load(0.0f, 0.0f, 1.0f, t.get_z());
	d.row3.load(0.0f, 0.0f, 0.0f, 1.0f);
}

void simd_mat4::rotate(vec3 &r, const float angle, mat4 &d)
{
	vec3 axis;
	calculator.normalize(r, axis);
	axis.store();

	float x = axis.get_x();
	float y = axis.get_y();
	float z = axis.get_z();

	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - c;

	d.row0.load(t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0f);
	d.row1.load(t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0f);
	d.row2.load(t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0f);
	d.row3.load(0.0f, 0.0f, 0.0f, 1.0f);
}

void simd_mat4::scale(vec3 &s, mat4 &d)
{
	s.store();
	d.row0.load(s.get_x(), 0.0f, 0.0f, 0.0f);
	d.row1.load(0.0f, s.get_y(), 0.0f, 0.0f);
	d.row2.load(0.0f, 0.0f, s.get_z(), 0.0f);
	d.row3.load(0.0f, 0.0f, 0.0f, 1.0f);
}

void simd_mat4::transpose(const mat4 &a, mat4 &d)
{
	__m128 tmp0, tmp1, tmp2, tmp3;

	tmp0 = _mm_unpacklo_ps(a.row0.get_vec(), a.row1.get_vec());
	tmp1 = _mm_unpacklo_ps(a.row2.get_vec(), a.row3.get_vec());
	tmp2 = _mm_unpackhi_ps(a.row0.get_vec(), a.row1.get_vec());
	tmp3 = _mm_unpackhi_ps(a.row2.get_vec(), a.row3.get_vec());

	d.row0.set(_mm_movelh_ps(tmp0, tmp1));
	d.row1.set(_mm_movehl_ps(tmp1, tmp0));
	d.row2.set(_mm_movelh_ps(tmp2, tmp3));
	d.row3.set(_mm_movehl_ps(tmp3, tmp2));
}

void simd_mat4::mult(const mat4 &a, const mat4 &b, mat4 &d)
{
	const __m128 b_r0 = b.row0.get_vec();
	const __m128 b_r1 = b.row1.get_vec();
	const __m128 b_r2 = b.row2.get_vec();
	const __m128 b_r3 = b.row3.get_vec();

	__m128 v1, v2, v3, v4;

	// --- Calculate d.row0 ---
	__m128 a_r0 = a.row0.get_vec();

	v1 = _mm_shuffle_ps(a_r0, a_r0, _MM_SHUFFLE(0, 0, 0, 0));
	v2 = _mm_shuffle_ps(a_r0, a_r0, _MM_SHUFFLE(1, 1, 1, 1));
	v3 = _mm_shuffle_ps(a_r0, a_r0, _MM_SHUFFLE(2, 2, 2, 2));
	v4 = _mm_shuffle_ps(a_r0, a_r0, _MM_SHUFFLE(3, 3, 3, 3));
	v1 = _mm_mul_ps(v1, b_r0);
	v2 = _mm_mul_ps(v2, b_r1);
	v3 = _mm_mul_ps(v3, b_r2);
	v4 = _mm_mul_ps(v4, b_r3);

	v1 = _mm_add_ps(v1, v2);
	v3 = _mm_add_ps(v3, v4);
	d.row0.set(_mm_add_ps(v1, v3));

	// --- Calculate d.row1 ---
	__m128 a_r1 = a.row1.get_vec();
	v1 = _mm_shuffle_ps(a_r1, a_r1, _MM_SHUFFLE(0, 0, 0, 0));
	v2 = _mm_shuffle_ps(a_r1, a_r1, _MM_SHUFFLE(1, 1, 1, 1));
	v3 = _mm_shuffle_ps(a_r1, a_r1, _MM_SHUFFLE(2, 2, 2, 2));
	v4 = _mm_shuffle_ps(a_r1, a_r1, _MM_SHUFFLE(3, 3, 3, 3));

	v1 = _mm_mul_ps(v1, b_r0);
	v2 = _mm_mul_ps(v2, b_r1);
	v3 = _mm_mul_ps(v3, b_r2);
	v4 = _mm_mul_ps(v4, b_r3);

	v1 = _mm_add_ps(v1, v2);
	v3 = _mm_add_ps(v3, v4);
	d.row1.set(_mm_add_ps(v1, v3));

	// --- Calculate d.row2 ---
	__m128 a_r2 = a.row2.get_vec();
	v1 = _mm_shuffle_ps(a_r2, a_r2, _MM_SHUFFLE(0, 0, 0, 0));
	v2 = _mm_shuffle_ps(a_r2, a_r2, _MM_SHUFFLE(1, 1, 1, 1));
	v3 = _mm_shuffle_ps(a_r2, a_r2, _MM_SHUFFLE(2, 2, 2, 2));
	v4 = _mm_shuffle_ps(a_r2, a_r2, _MM_SHUFFLE(3, 3, 3, 3));

	v1 = _mm_mul_ps(v1, b_r0);
	v2 = _mm_mul_ps(v2, b_r1);
	v3 = _mm_mul_ps(v3, b_r2);
	v4 = _mm_mul_ps(v4, b_r3);

	v1 = _mm_add_ps(v1, v2);
	v3 = _mm_add_ps(v3, v4);
	d.row2.set(_mm_add_ps(v1, v3));

	// --- Calculate d.row3 ---
	__m128 a_r3 = a.row3.get_vec();
	v1 = _mm_shuffle_ps(a_r3, a_r3, _MM_SHUFFLE(0, 0, 0, 0));
	v2 = _mm_shuffle_ps(a_r3, a_r3, _MM_SHUFFLE(1, 1, 1, 1));
	v3 = _mm_shuffle_ps(a_r3, a_r3, _MM_SHUFFLE(2, 2, 2, 2));
	v4 = _mm_shuffle_ps(a_r3, a_r3, _MM_SHUFFLE(3, 3, 3, 3));

	v1 = _mm_mul_ps(v1, b_r0);
	v2 = _mm_mul_ps(v2, b_r1);
	v3 = _mm_mul_ps(v3, b_r2);
	v4 = _mm_mul_ps(v4, b_r3);

	v1 = _mm_add_ps(v1, v2);
	v3 = _mm_add_ps(v3, v4);
	d.row3.set(_mm_add_ps(v1, v3));
}

void simd_mat4::mult_vec(const mat4 &m, vec3 &v, vec3 &d)
{
	v.set_padding();
	const __m128 v_vec = v.get_vec();

	__m128 d_x_vec = _mm_dp_ps(m.row0.get_vec(), v_vec, 0xF1);

	__m128 d_y_vec = _mm_dp_ps(m.row1.get_vec(), v_vec, 0xF1);

	__m128 d_z_vec = _mm_dp_ps(m.row2.get_vec(), v_vec, 0xF1);

	__m128 d_w_vec = _mm_dp_ps(m.row3.get_vec(), v_vec, 0xF1);

	__m128 res = _mm_shuffle_ps(d_x_vec, d_y_vec, _MM_SHUFFLE(0, 0, 0, 0));

	__m128 temp_zw = _mm_shuffle_ps(d_z_vec, d_w_vec, _MM_SHUFFLE(0, 0, 0, 0));

	res = _mm_shuffle_ps(res, temp_zw, _MM_SHUFFLE(2, 0, 2, 0));

	d.set(&res);
}