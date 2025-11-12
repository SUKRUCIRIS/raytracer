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
	const __m128 half = _mm_set_ps1(0.5f);
	const __m128 three = _mm_set_ps1(3.0f);
	d = _mm_dp_ps(a.vec, a.vec, 0x77);
	d1 = _mm_set_ps1(1e-12f);
	d2 = _mm_cmpgt_ps(d, d1);
	d1 = _mm_rsqrt_ps(d);
	__m128 d3 = _mm_mul_ps(d1, d1);
	d3 = _mm_mul_ps(d, d3);
	d3 = _mm_sub_ps(three, d3);
	__m128 d4 = _mm_mul_ps(d1, half);
	d1 = _mm_mul_ps(d4, d3);
	d1 = _mm_and_ps(d1, d2);
	c.vec = _mm_mul_ps(a.vec, d1);
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
	__m128 r0 = a.row0.get_vec();
	__m128 r1 = a.row1.get_vec();
	__m128 r2 = a.row2.get_vec();
	__m128 r3 = a.row3.get_vec();

	_MM_TRANSPOSE4_PS(r0, r1, r2, r3);

	d.row0.set(r0);
	d.row1.set(r1);
	d.row2.set(r2);
	d.row3.set(r3);
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

void simd_mat4::mult_vec(const mat4 &m, vec3 &v, vec3 &d, bool is_dir)
{
	if (is_dir)
	{
		v.set_padding(0);
	}
	else
	{
		v.set_padding();
	}
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

void simd_mat4::inverse(const mat4 &a, mat4 &d)
{
	float src[16];
	float dst[16];
	_mm_store_ps(&src[0], a.row0.get_vec());
	_mm_store_ps(&src[4], a.row1.get_vec());
	_mm_store_ps(&src[8], a.row2.get_vec());
	_mm_store_ps(&src[12], a.row3.get_vec());

	__m128 minor0, minor1, minor2, minor3;
	__m128 row0, row1, row2, row3;
	__m128 det, tmp1;

	tmp1 = _mm_loadh_pi(_mm_loadl_pi(tmp1, (__m64 *)(src)), (__m64 *)(src + 4));
	row1 = _mm_loadh_pi(_mm_loadl_pi(row1, (__m64 *)(src + 8)), (__m64 *)(src + 12));

	row0 = _mm_shuffle_ps(tmp1, row1, 0x88);
	row1 = _mm_shuffle_ps(row1, tmp1, 0xDD);

	tmp1 = _mm_loadh_pi(_mm_loadl_pi(tmp1, (__m64 *)(src + 2)), (__m64 *)(src + 6));
	row3 = _mm_loadh_pi(_mm_loadl_pi(row3, (__m64 *)(src + 10)), (__m64 *)(src + 14));

	row2 = _mm_shuffle_ps(tmp1, row3, 0x88);
	row3 = _mm_shuffle_ps(row3, tmp1, 0xDD);

	tmp1 = _mm_mul_ps(row2, row3);
	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0xB1);

	minor0 = _mm_mul_ps(row1, tmp1);
	minor1 = _mm_mul_ps(row0, tmp1);

	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0x4E);

	minor0 = _mm_sub_ps(_mm_mul_ps(row1, tmp1), minor0);
	minor1 = _mm_sub_ps(_mm_mul_ps(row0, tmp1), minor1);
	minor1 = _mm_shuffle_ps(minor1, minor1, 0x4E);

	tmp1 = _mm_mul_ps(row1, row2);
	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0xB1);

	minor0 = _mm_add_ps(_mm_mul_ps(row3, tmp1), minor0);
	minor3 = _mm_mul_ps(row0, tmp1);

	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0x4E);

	minor0 = _mm_sub_ps(minor0, _mm_mul_ps(row3, tmp1));
	minor3 = _mm_sub_ps(_mm_mul_ps(row0, tmp1), minor3);
	minor3 = _mm_shuffle_ps(minor3, minor3, 0x4E);

	tmp1 = _mm_mul_ps(_mm_shuffle_ps(row1, row1, 0x4E), row3);
	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0xB1);
	row2 = _mm_shuffle_ps(row2, row2, 0x4E);

	minor0 = _mm_add_ps(_mm_mul_ps(row2, tmp1), minor0);
	minor2 = _mm_mul_ps(row0, tmp1);

	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0x4E);

	minor0 = _mm_sub_ps(minor0, _mm_mul_ps(row2, tmp1));
	minor2 = _mm_sub_ps(_mm_mul_ps(row0, tmp1), minor2);
	minor2 = _mm_shuffle_ps(minor2, minor2, 0x4E);

	tmp1 = _mm_mul_ps(row0, row1);
	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0xB1);

	minor2 = _mm_add_ps(_mm_mul_ps(row3, tmp1), minor2);
	minor3 = _mm_sub_ps(_mm_mul_ps(row2, tmp1), minor3);

	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0x4E);

	minor2 = _mm_sub_ps(_mm_mul_ps(row3, tmp1), minor2);
	minor3 = _mm_sub_ps(minor3, _mm_mul_ps(row2, tmp1));

	tmp1 = _mm_mul_ps(row0, row3);
	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0xB1);

	minor1 = _mm_sub_ps(minor1, _mm_mul_ps(row2, tmp1));
	minor2 = _mm_add_ps(_mm_mul_ps(row1, tmp1), minor2);

	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0x4E);

	minor1 = _mm_add_ps(_mm_mul_ps(row2, tmp1), minor1);
	minor2 = _mm_sub_ps(minor2, _mm_mul_ps(row1, tmp1));

	tmp1 = _mm_mul_ps(row0, row2);
	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0xB1);

	minor1 = _mm_add_ps(_mm_mul_ps(row3, tmp1), minor1);
	minor3 = _mm_sub_ps(minor3, _mm_mul_ps(row1, tmp1));

	tmp1 = _mm_shuffle_ps(tmp1, tmp1, 0x4E);

	minor1 = _mm_sub_ps(minor1, _mm_mul_ps(row3, tmp1));
	minor3 = _mm_add_ps(_mm_mul_ps(row1, tmp1), minor3);

	det = _mm_mul_ps(row0, minor0);
	det = _mm_add_ps(_mm_shuffle_ps(det, det, 0x4E), det);
	det = _mm_add_ss(_mm_shuffle_ps(det, det, 0xB1), det);

	tmp1 = _mm_rcp_ss(det);

	det = _mm_sub_ss(_mm_add_ss(tmp1, tmp1), _mm_mul_ss(det, _mm_mul_ss(tmp1, tmp1)));
	det = _mm_shuffle_ps(det, det, 0x00);

	minor0 = _mm_mul_ps(det, minor0);
	_mm_storel_pi((__m64 *)(dst), minor0);
	_mm_storeh_pi((__m64 *)(dst + 2), minor0);

	minor1 = _mm_mul_ps(det, minor1);
	_mm_storel_pi((__m64 *)(dst + 4), minor1);
	_mm_storeh_pi((__m64 *)(dst + 6), minor1);

	minor2 = _mm_mul_ps(det, minor2);
	_mm_storel_pi((__m64 *)(dst + 8), minor2);
	_mm_storeh_pi((__m64 *)(dst + 10), minor2);

	minor3 = _mm_mul_ps(det, minor3);
	_mm_storel_pi((__m64 *)(dst + 12), minor3);
	_mm_storeh_pi((__m64 *)(dst + 14), minor3);

	d.row0.set(_mm_load_ps(&dst[0]));
	d.row1.set(_mm_load_ps(&dst[4]));
	d.row2.set(_mm_load_ps(&dst[8]));
	d.row3.set(_mm_load_ps(&dst[12]));
}