#include "ray_trace.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <random>

void get_latlong_uv(const vec3 &dir, float &u, float &v)
{
	float pi = 3.14159265359f;
	float dx = dir.get_x();
	float dy = dir.get_y();
	float dz = dir.get_z();

	u = (1.0f + std::atan2(dx, -dz) / pi) * 0.5f;
	v = std::acos(dy) / pi;
}

void get_probe_uv(const vec3 &dir, float &u, float &v)
{
	float pi = 3.14159265359f;
	float dx = dir.get_x();
	float dy = dir.get_y();
	float dz = dir.get_z();

	float len_sq = dx * dx + dy * dy;
	float r = 0.0f;

	if (len_sq > 1e-6f)
	{
		r = (1.0f / pi) * std::acos(-dz) / std::sqrt(len_sq);
	}
	else if (dz < 0.0f)
	{
		r = 0.0f;
	}

	u = (r * dx + 1.0f) * 0.5f;
	v = (-r * dy + 1.0f) * 0.5f;
}

void ray_tracer::get_orthonormal_basis(const vec3 &normal, vec3 &u, vec3 &v, vec3 &w) const
{
	w = normal;
	vec3 helper;
	if (std::abs(w.get_x()) > 0.9f)
		helper.load(0, 1, 0);
	else
		helper.load(1, 0, 0);

	simd_vec3 calc;
	calc.cross(helper, w, u);
	calc.normalize(u, u);
	calc.cross(w, u, v);
	calc.normalize(v, v);
}

void ray_tracer::sample_cosine_hemisphere(float r1, float r2, const vec3 &normal, vec3 &dir, float &pdf) const
{
	float phi = 2.0f * 3.14159265359f * r1;
	float sin_theta = std::sqrt(r2);
	float cos_theta = std::sqrt(1.0f - r2);

	float x = sin_theta * std::cos(phi);
	float y = sin_theta * std::sin(phi);
	float z = cos_theta;

	vec3 u, v, w;
	get_orthonormal_basis(normal, u, v, w);

	simd_vec3 calc;
	vec3 term1, term2, term3;
	calc.mult_scalar(u, x, term1);
	calc.mult_scalar(v, y, term2);
	calc.mult_scalar(w, z, term3);
	calc.add(term1, term2, dir);
	calc.add(dir, term3, dir);
	calc.normalize(dir, dir);

	pdf = cos_theta / 3.14159265359f;
}

void ray_tracer::sample_uniform_hemisphere(float r1, float r2, const vec3 &normal, vec3 &dir, float &pdf) const
{
	float phi = 2.0f * 3.14159265359f * r1;
	float sin_theta = std::sqrt(1.0f - r2 * r2);
	float cos_theta = r2;

	float x = sin_theta * std::cos(phi);
	float y = sin_theta * std::sin(phi);
	float z = cos_theta;

	vec3 u, v, w;
	get_orthonormal_basis(normal, u, v, w);

	simd_vec3 calc;
	vec3 term1, term2, term3;
	calc.mult_scalar(u, x, term1);
	calc.mult_scalar(v, y, term2);
	calc.mult_scalar(w, z, term3);
	calc.add(term1, term2, dir);
	calc.add(dir, term3, dir);
	calc.normalize(dir, dir);

	pdf = 1.0f / (2.0f * 3.14159265359f);
}

void ray_tracer::calculate_color(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &normal, const material *mat,
								 const std::vector<texture *> *textures, vec3 &hit_point, const vec3 &ray_origin,
								 const shape *min_shape, const float &raytime, int id, bool is_hdr, vec3 &color) const
{
	static thread_local shape *last_shadow_blocker = nullptr;
	static thread_local int last_blocker_id = -1;

	calculator.mult(mat->AmbientReflectance, ambientlight, color);

	bool replacekd = false;
	bool blendkd = false;
	vec3 replacekd_color;
	bool replaceks = false;
	vec3 replaceks_color;

	for (auto &&i : *textures)
	{
		if (i->dmode == replace_kd)
		{
			replacekd = true;
			float u, v;
			min_shape->calculate_uv(calculator, calculator_m, hit_point, id, u, v);
			i->sample(u, v, hit_point, replacekd_color);
		}
		else if (i->dmode == blend_kd)
		{
			blendkd = true;
			float u, v;
			min_shape->calculate_uv(calculator, calculator_m, hit_point, id, u, v);
			i->sample(u, v, hit_point, replacekd_color);
			calculator.add(mat->DiffuseReflectance, replacekd_color, replacekd_color);
			calculator.mult_scalar(replacekd_color, 0.5f, replacekd_color);
		}
		else if (i->dmode == replace_ks)
		{
			replaceks = true;
			float u, v;
			min_shape->calculate_uv(calculator, calculator_m, hit_point, id, u, v);
			i->sample(u, v, hit_point, replaceks_color);
		}
		else if (i->dmode == replace_all)
		{
			float u, v;
			min_shape->calculate_uv(calculator, calculator_m, hit_point, id, u, v);
			i->sample(u, v, hit_point, color);
			return;
		}
	}

	vec3 view_dir;
	calculator.subs(ray_origin, hit_point, view_dir);
	calculator.normalize(view_dir, view_dir);

	for (Light *light : *lights)
	{
		int num_samples = light->get_sample_count();
		float weight = 1.0f / num_samples;

		for (int s = 0; s < num_samples; ++s)
		{
			float r1 = get_random_float();
			float r2 = get_random_float();

			vec3 sample_pos, incident_radiance, light_dir;
			float dist;

			light->get_sample(calculator, calculator_m, hit_point, normal, r1, r2,
							  sample_pos, incident_radiance, light_dir, dist);

			if (incident_radiance.get_x() == 0 && incident_radiance.get_y() == 0 && incident_radiance.get_z() == 0)
				continue;

			bool in_shadow = false;
			float t_shadow;
			vec3 shadow_origin;
			calculator.mult_scalar(light_dir, shadowrayepsilon, shadow_origin);
			calculator.add(hit_point, shadow_origin, shadow_origin);

			if (last_shadow_blocker)
			{
				float t_temp;
				if (last_shadow_blocker->intersect(calculator, calculator_m, shadow_origin, light_dir,
												   t_temp, last_blocker_id, raytime, false, shadowrayepsilon))
				{
					if (t_temp < dist - shadowrayepsilon)
					{
						in_shadow = true;
					}
					else
					{
						last_shadow_blocker = nullptr;
					}
				}
				else
				{
					last_shadow_blocker = nullptr;
				}
			}

			if (!in_shadow)
			{
				shape *hit_shape = 0;
				int hit_id;
				if (bvhx->intersect(calculator, calculator_m, shadow_origin, light_dir, raytime, t_shadow,
									&hit_shape, hit_id, false, shadowrayepsilon, true, dist))
				{
					if (t_shadow < dist - shadowrayepsilon)
					{
						in_shadow = true;
						last_shadow_blocker = hit_shape;
						last_blocker_id = hit_id;
					}
				}
			}
			if (in_shadow)
				continue;

			float ndotl;
			calculator.dot(normal, light_dir, ndotl);
			ndotl = std::max(ndotl, 0.0f);

			vec3 h;
			calculator.add(light_dir, view_dir, h);
			calculator.normalize(h, h);

			vec3 final_diffuse = {0, 0, 0}, final_specular = {0, 0, 0};

			vec3 kd = mat->DiffuseReflectance;
			if (replacekd || blendkd)
				kd = replacekd_color;

			vec3 ks = mat->SpecularReflectance;
			if (replaceks)
				ks = replaceks_color;

			float p = mat->PhongExponent;
			float PI = 3.14159265359f;

			switch (mat->brdfType)
			{
			case OriginalBlinnPhong:
			{
				calculator.mult_scalar(kd, ndotl, final_diffuse);

				float ndoth;
				calculator.dot(normal, h, ndoth);
				ndoth = std::max(ndoth, 0.0f);
				float spec_val = powf(ndoth, p);
				calculator.mult_scalar(ks, spec_val, final_specular);
				break;
			}
			case OriginalPhong:
			{
				calculator.mult_scalar(kd, ndotl, final_diffuse);

				vec3 R;
				vec3 negL;
				calculator.mult_scalar(light_dir, -1.0f, negL);
				calculate_reflected_dir(calculator, normal, negL, R, 0);
				float rdoth;
				calculator.dot(R, view_dir, rdoth);
				rdoth = std::max(rdoth, 0.0f);
				float spec_val = powf(rdoth, p);
				calculator.mult_scalar(ks, spec_val, final_specular);
				break;
			}
			case ModifiedBlinnPhong:
			{

				float ndoth;
				calculator.dot(normal, h, ndoth);
				ndoth = std::max(ndoth, 0.0f);

				if (mat->normalized)
				{
					calculator.mult_scalar(kd, 1.0f / PI * ndotl, final_diffuse);

					float norm = (p + 8.0f) / (8.0f * PI);
					float spec_val = norm * powf(ndoth, p) * ndotl;
					calculator.mult_scalar(ks, spec_val, final_specular);
				}
				else
				{
					calculator.mult_scalar(kd, ndotl, final_diffuse);

					float spec_val = powf(ndoth, p) * ndotl;
					calculator.mult_scalar(ks, spec_val, final_specular);
				}
				break;
			}
			case ModifiedPhong:
			{
				vec3 R;
				vec3 negL;
				calculator.mult_scalar(light_dir, -1.0f, negL);
				calculate_reflected_dir(calculator, normal, negL, R, 0);
				float rdoth;
				calculator.dot(R, view_dir, rdoth);
				rdoth = std::max(rdoth, 0.0f);

				if (mat->normalized)
				{
					calculator.mult_scalar(kd, 1.0f / PI * ndotl, final_diffuse);

					float norm = (p + 2.0f) / (2.0f * PI);
					float spec_val = norm * powf(rdoth, p) * ndotl;
					calculator.mult_scalar(ks, spec_val, final_specular);
				}
				else
				{
					calculator.mult_scalar(kd, ndotl, final_diffuse);

					float spec_val = powf(rdoth, p) * ndotl;
					calculator.mult_scalar(ks, spec_val, final_specular);
				}
				break;
			}
			case TorranceSparrow:
			{

				float ndoth, ndotv, vdoth;
				calculator.dot(normal, h, ndoth);
				calculator.dot(normal, view_dir, ndotv);
				calculator.dot(view_dir, h, vdoth);

				ndoth = std::max(ndoth, 0.0f);
				ndotv = std::max(ndotv, 0.0f);
				vdoth = std::max(vdoth, 0.0f);

				float D = (p + 2.0f) / (2.0f * PI) * powf(ndoth, p);

				vec3 F;
				float f_term = powf(1.0f - vdoth, 5.0f);
				vec3 one(1, 1, 1);
				vec3 inv_ks;
				calculator.subs(one, ks, inv_ks);
				calculator.mult_scalar(inv_ks, f_term, inv_ks);
				calculator.add(ks, inv_ks, F);

				float G = 1.0f;
				if (vdoth > 1e-6f)
				{
					float g1 = (2.0f * ndoth * ndotv) / vdoth;
					float g2 = (2.0f * ndoth * ndotl) / vdoth;
					G = std::min(1.0f, std::min(g1, g2));
				}

				if (ndotv > 1e-6f)
				{
					float denom = 4.0f * ndotv;
					float factor = (D * G) / denom;
					calculator.mult_scalar(F, factor, final_specular);
				}

				if (mat->kdfresnel)
				{
					vec3 diff_factor;
					calculator.subs(one, F, diff_factor);
					vec3 kd_ndotl;
					calculator.mult_scalar(kd, ndotl, kd_ndotl);
					calculator.mult(kd_ndotl, diff_factor, final_diffuse);
				}
				else
				{
					calculator.mult_scalar(kd, 1.0f / PI * ndotl, final_diffuse);
				}
				break;
			}
			}

			calculator.mult(final_diffuse, incident_radiance, final_diffuse);
			calculator.mult_scalar(final_diffuse, weight, final_diffuse);

			calculator.mult(final_specular, incident_radiance, final_specular);
			calculator.mult_scalar(final_specular, weight, final_specular);

			calculator.add(color, final_diffuse, color);
			calculator.add(color, final_specular, color);
		}
	}

	if (!is_hdr)
	{
		calculator.mult_scalar(color, 0.004f, color);
	}
}

void add_roughness(simd_vec3 &calculator, vec3 &R, float roughness)
{
	R.store();

	vec3 r_prime = R;
	float abs_x = std::abs(R.get_x());
	float abs_y = std::abs(R.get_y());
	float abs_z = std::abs(R.get_z());

	if (abs_x <= abs_y && abs_x <= abs_z)
	{
		r_prime.load(1.0f, R.get_y(), R.get_z());
	}
	else if (abs_y <= abs_z)
	{
		r_prime.load(R.get_x(), 1.0f, R.get_z());
	}
	else
	{
		r_prime.load(R.get_x(), R.get_y(), 1.0f);
	}

	vec3 u, v;
	calculator.cross(R, r_prime, u);
	calculator.normalize(u, u);

	calculator.cross(R, u, v);
	float xi1 = get_random_float() - 0.5f;
	float xi2 = get_random_float() - 0.5f;

	vec3 u_comp, v_comp;
	calculator.mult_scalar(u, xi1, u_comp);
	calculator.mult_scalar(v, xi2, v_comp);

	vec3 offset;
	calculator.add(u_comp, v_comp, offset);
	calculator.mult_scalar(offset, roughness, offset);

	calculator.add(R, offset, R);
}

void ray_tracer::calculate_reflected_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I, vec3 &R, float roughness)
{
	float dot;
	calculator.dot(I, N, dot);

	calculator.mult_scalar(N, dot * -2.0f, R);
	calculator.add(I, R, R);
	if (roughness > 0.0f)
	{
		add_roughness(calculator, R, roughness);
	}
	calculator.normalize(R, R);
	R.store();
}

bool ray_tracer::calculate_refracted_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I,
										 float n1, float n2, vec3 &T, float roughness)
{
	float cosi;
	calculator.dot(I, N, cosi);
	cosi = -cosi;
	float eta = n1 / n2;

	const vec3 &normal = N;
	float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
	if (k < 0.0f)
	{
		return false;
	}
	vec3 term1, term2;
	calculator.mult_scalar(I, eta, term1);
	calculator.mult_scalar(normal, (eta * cosi - sqrtf(k)), term2);
	calculator.add(term1, term2, T);

	if (roughness > 0.0f)
	{
		add_roughness(calculator, T, roughness);
	}

	calculator.normalize(T, T);
	T.store();

	return true;
}

void ray_tracer::apply_normal_map(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, const std::vector<texture *> *textures,
								  const shape *min_shape, int id, vec3 &normal) const
{
	for (const auto *tex : *textures)
	{
		if (tex->dmode == replace_normal)
		{
			float u, v;
			min_shape->calculate_uv(calculator, calculator_m, hit_point, id, u, v);

			vec3 raw_normal_color(0.5f, 0.5f, 1.0f);
			tex->sample(u, v, hit_point, raw_normal_color);

			vec3 tangent_space_normal;
			calculator.mult_scalar(raw_normal_color, 2.0f, tangent_space_normal);
			vec3 ones(1.0f, 1.0f, 1.0f);
			calculator.subs(tangent_space_normal, ones, tangent_space_normal);

			vec3 T, B;
			min_shape->get_tangent(calculator, calculator_m, hit_point, id, T);

			float ndott;
			calculator.dot(normal, T, ndott);
			vec3 temp_vec;
			calculator.mult_scalar(normal, ndott, temp_vec);
			calculator.subs(T, temp_vec, T);
			calculator.normalize(T, T);

			calculator.cross(normal, T, B);

			vec3 term1, term2, term3;
			tangent_space_normal.store();
			calculator.mult_scalar(T, tangent_space_normal.get_x(), term1);
			calculator.mult_scalar(B, tangent_space_normal.get_y(), term2);
			calculator.mult_scalar(normal, tangent_space_normal.get_z(), term3);

			vec3 new_normal;
			calculator.add(term1, term2, new_normal);
			calculator.add(new_normal, term3, new_normal);
			calculator.normalize(new_normal, normal);

			return;
		}
	}
}

void ray_tracer::apply_bump_map(simd_vec3 &calculator, simd_mat4 &calculator_m, vec3 &hit_point, const std::vector<texture *> *textures,
								const shape *min_shape, int id, vec3 &normal) const
{
	for (const auto *tex : *textures)
	{
		if (tex->dmode == bump_normal)
		{
			if (tex->im)
			{
				float u, v;
				min_shape->calculate_uv(calculator, calculator_m, hit_point, id, u, v);

				float du, dv;
				vec3 hit_u, hit_v;
				du = 1.0f / tex->im->width;
				dv = 1.0f / tex->im->height;

				vec3 h_center, h_u, h_v;

				tex->sample(u, v, hit_point, h_center);
				tex->sample(u + du, v, hit_point, h_u);
				tex->sample(u, v + dv, hit_point, h_v);

				float intensity_c = h_center.get_x();
				float intensity_u = h_u.get_x();
				float intensity_v = h_v.get_x();

				float dh_du = (intensity_u - intensity_c) * tex->BumpFactor;
				float dh_dv = (intensity_v - intensity_c) * tex->BumpFactor;

				vec3 T, B;
				min_shape->get_tangent(calculator, calculator_m, hit_point, id, T);

				float ndott;
				calculator.dot(normal, T, ndott);
				vec3 temp_vec;
				calculator.mult_scalar(normal, ndott, temp_vec);
				calculator.subs(T, temp_vec, T);
				calculator.normalize(T, T);

				calculator.cross(normal, T, B);

				vec3 p_u, p_v;
				calculator.mult_scalar(T, dh_du, p_u);
				calculator.mult_scalar(B, dh_dv, p_v);

				vec3 new_normal;
				calculator.subs(normal, p_u, new_normal);
				calculator.subs(new_normal, p_v, new_normal);

				calculator.normalize(new_normal, normal);
			}
			else
			{
				float eps = 0.01f;
				float inv_eps = 1.0f / eps;

				vec3 h_x, h_y, h_z, h_c;
				vec3 off_x, off_y, off_z;

				calculator.add(hit_point, vec3(eps, 0, 0, 0), off_x);
				calculator.add(hit_point, vec3(0, eps, 0, 0), off_y);
				calculator.add(hit_point, vec3(0, 0, eps, 0), off_z);

				hit_point.store();
				off_x.store();
				off_y.store();
				off_z.store();

				float d_u = 0, d_v = 0;
				tex->sample(d_u, d_v, hit_point, h_c);
				tex->sample(d_u, d_v, off_x, h_x);
				tex->sample(d_u, d_v, off_y, h_y);
				tex->sample(d_u, d_v, off_z, h_z);

				h_c.store();
				h_x.store();
				h_y.store();
				h_z.store();

				float noise_c = h_c.get_x();

				vec3 gradN(
					(h_x.get_x() - noise_c) * inv_eps,
					(h_y.get_x() - noise_c) * inv_eps,
					(h_z.get_x() - noise_c) * inv_eps);

				float g_dot_n;
				calculator.dot(gradN, normal, g_dot_n);

				vec3 n_component, surf_grad;
				calculator.mult_scalar(normal, g_dot_n, n_component);
				calculator.subs(gradN, n_component, surf_grad);

				vec3 perturbation;
				calculator.mult_scalar(surf_grad, tex->BumpFactor, perturbation);

				vec3 new_normal;
				calculator.subs(normal, perturbation, new_normal);
				calculator.normalize(new_normal, normal);
			}
			return;
		}
	}
}

void ray_tracer::trace_rec(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
						   vec3 &color, const float &raytime, const bool culling, bool is_hdr, texture *bg, bool is_probe, float pixelu, float pixelv, int depth) const
{
	if (depth >= max_depth)
	{
		if (bg && bg->im)
		{
			if (is_hdr)
			{
				float u, v;

				if (is_probe)
				{
					get_probe_uv(ray_dir, u, v);
				}
				else
				{
					get_latlong_uv(ray_dir, u, v);
				}

				bg->sample(u, v, vec3(0), color);
			}
			else
			{
				vec3 zort(pixelu, pixelv, 0);
				bg->sample(pixelu, pixelv, zort, color);
			}
		}
		else
		{
			color = backgroundcolor;
		}
		return;
	}
	float t;
	float min_t = FLT_MAX;
	shape *min_shape = 0;
	int min_id = 0;

	shape *hit_shape = 0;
	if (bvhx->intersect(calculator, calculator_m, ray_origin, ray_dir, raytime, min_t, &hit_shape, min_id, culling, intersectionepsilon))
	{
		min_shape = hit_shape;
	}

	if (!min_shape)
	{
		if (bg && bg->im)
		{
			if (is_hdr)
			{
				float u, v;

				if (is_probe)
				{
					get_probe_uv(ray_dir, u, v);
				}
				else
				{
					get_latlong_uv(ray_dir, u, v);
				}

				bg->sample(u, v, vec3(0), color);
			}
			else
			{
				vec3 zort(pixelu, pixelv, 0);
				bg->sample(pixelu, pixelv, zort, color);
			}
		}
		else
		{
			color = backgroundcolor;
		}
		return;
	}
	material *mat = min_shape->getMaterial(min_id);
	auto textures = min_shape->getTextures(min_id);
	vec3 hit_point;
	calculator.mult_scalar(ray_dir, min_t, hit_point);
	calculator.add(ray_origin, hit_point, hit_point);
	hit_point.store();
	vec3 normal;
	min_shape->get_normal(calculator, calculator_m, hit_point, min_id, normal);
	apply_normal_map(calculator, calculator_m, hit_point, textures, min_shape, min_id, normal);
	apply_bump_map(calculator, calculator_m, hit_point, textures, min_shape, min_id, normal);
	if (mat->mt == Mirror)
	{
		vec3 R;
		calculate_reflected_dir(calculator, normal, ray_dir, R, mat->roughness);
		vec3 offset;
		calculator.mult_scalar(normal, shadowrayepsilon, offset);
		calculator.add(hit_point, offset, hit_point);
		trace_rec(calculator, calculator_m, hit_point, R, color, raytime, culling, is_hdr, bg, is_probe, pixelu, pixelv, depth + 1);
		calculator.mult(color, mat->MirrorReflectance, color);

		vec3 own_color;
		calculate_color(calculator, calculator_m, normal, mat, textures, hit_point, ray_origin, min_shape, raytime, min_id, is_hdr, own_color);
		calculator.add(color, own_color, color);
	}
	else if (mat->mt == Conductor)
	{
		vec3 R;
		calculate_reflected_dir(calculator, normal, ray_dir, R, mat->roughness);

		float cosTheta;
		calculator.dot(ray_dir, normal, cosTheta);
		cosTheta = fabsf(cosTheta);

		float n = mat->RefractionIndex;
		float k = mat->AbsorptionIndex;

		float c = cosTheta;
		float c2 = c * c;
		float s2 = 1.0f - c2;

		float n2 = n * n;
		float k2 = k * k;
		float n2_k2 = n2 - k2;
		float n2k2_4 = 4.0f * n2 * k2;

		float a2_b2 = sqrtf((n2_k2 - s2) * (n2_k2 - s2) + n2k2_4);
		float a2 = 0.5f * (a2_b2 + n2_k2 - s2);
		float a = sqrtf(a2);
		float b2 = a2_b2 - a2;
		float b = sqrtf(b2);

		float Rs_num = a2_b2 + c2 - (2.0f * a * c);
		float Rs_den = a2_b2 + c2 + (2.0f * a * c);
		float Rs = (Rs_den == 0.0f) ? 1.0f : Rs_num / Rs_den;

		float ac = a * c;
		float bc = b * c;
		float Rp_num_term1 = ac - s2;
		float Rp_den_term1 = ac + s2;
		float Rp_num = (Rp_num_term1 * Rp_num_term1) + (bc * bc);
		float Rp_den = (Rp_den_term1 * Rp_den_term1) + (bc * bc);
		float Rp = (Rp_den == 0.0f) ? 1.0f : Rs * (Rp_num / Rp_den);

		float F_scalar = 0.5f * (Rs + Rp);
		F_scalar = std::clamp(F_scalar, 0.0f, 1.0f);

		vec3 F_vec(F_scalar, F_scalar, F_scalar);

		vec3 F;
		calculator.mult(F_vec, mat->MirrorReflectance, F);

		vec3 reflectedColor;
		vec3 offset;
		calculator.mult_scalar(normal, shadowrayepsilon, offset);
		calculator.add(hit_point, offset, hit_point);

		trace_rec(calculator, calculator_m, hit_point, R, reflectedColor, raytime, culling, is_hdr, bg, is_probe, pixelu, pixelv, depth + 1);
		calculator.mult(reflectedColor, F, color);

		vec3 own_color;
		calculate_color(calculator, calculator_m, normal, mat, textures, hit_point, ray_origin, min_shape, raytime, min_id, is_hdr, own_color);
		calculator.add(color, own_color, color);
	}
	else if (mat->mt == Dielectric)
	{
		float n1 = 1.0f;
		float n2 = mat->RefractionIndex;

		float cosTheta;
		calculator.dot(ray_dir, normal, cosTheta);

		bool entering = cosTheta < 0.0f;
		if (!entering)
		{
			std::swap(n1, n2);
			calculator.mult_scalar(normal, -1.0f, normal);
			cosTheta = -cosTheta;
		}

		float cosI = -cosTheta;
		cosI = std::clamp(cosI, 0.0f, 1.0f);

		float r0 = (n1 - n2) / (n1 + n2);
		r0 = r0 * r0;
		float F = r0 + (1.0f - r0) * powf(1.0f - cosI, 5.0f);

		vec3 reflectDir, reflectColor;
		calculate_reflected_dir(calculator, normal, ray_dir, reflectDir, mat->roughness);
		vec3 offsetR;
		calculator.mult_scalar(normal, shadowrayepsilon, offsetR);
		calculator.add(hit_point, offsetR, offsetR);
		trace_rec(calculator, calculator_m, offsetR, reflectDir, reflectColor, raytime, culling, is_hdr, bg, is_probe, pixelu, pixelv, depth + 1);

		calculator.mult(reflectColor, mat->MirrorReflectance, reflectColor);

		vec3 refractDir, refractColor;
		bool refracts = calculate_refracted_dir(calculator, normal, ray_dir, n1, n2, refractDir, mat->roughness);

		if (refracts)
		{
			vec3 offsetT;
			calculator.mult_scalar(normal, -shadowrayepsilon, offsetT);
			calculator.add(hit_point, offsetT, offsetT);
			trace_rec(calculator, calculator_m, offsetT, refractDir, refractColor, raytime, false, is_hdr, bg, is_probe, pixelu, pixelv, depth + 1);

			if (!entering)
			{
				vec3 attenuation;
				calculator.mult_scalar(mat->AbsorptionCoefficient, -min_t, attenuation);
				calculator.exp(attenuation, attenuation);
				calculator.mult(refractColor, attenuation, refractColor);
			}
		}

		vec3 reflectScaled, refractScaled;
		calculator.mult_scalar(reflectColor, F, reflectScaled);
		calculator.mult_scalar(refractColor, 1.0f - F, refractScaled);
		calculator.add(reflectScaled, refractScaled, color);

		vec3 own_color;
		calculate_color(calculator, calculator_m, normal, mat, textures, hit_point, ray_origin, min_shape, raytime, min_id, is_hdr, own_color);
		calculator.add(color, own_color, color);
	}
	else
	{
		calculate_color(calculator, calculator_m, normal, mat, textures, hit_point, ray_origin, min_shape, raytime, min_id, is_hdr, color);
	}
}

void ray_tracer::path_trace(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
							vec3 &color, const float &raytime, const bool culling, texture *bg, bool is_probe, int depth, const RenderSettings &settings) const
{
	color.load(0, 0, 0);

	float t;
	float min_t = FLT_MAX;
	shape *min_shape = 0;
	int min_id = 0;
	shape *hit_shape = 0;

	if (bvhx->intersect(calculator, calculator_m, ray_origin, ray_dir, raytime, min_t, &hit_shape, min_id, culling, intersectionepsilon))
	{
		min_shape = hit_shape;
	}

	if (!min_shape)
	{
		if (bg && bg->im)
		{
			float u, v;
			if (is_probe)
				get_probe_uv(ray_dir, u, v);
			else
				get_latlong_uv(ray_dir, u, v);
			bg->sample(u, v, vec3(0), color);
		}
		else
		{
			color = backgroundcolor;
		}
		return;
	}

	material *mat = min_shape->getMaterial(min_id);

	vec3 emission = min_shape->getEmission(min_id);
	calculator.add(color, emission, color);

	vec3 hit_point;
	calculator.mult_scalar(ray_dir, min_t, hit_point);
	calculator.add(ray_origin, hit_point, hit_point);

	vec3 normal;
	min_shape->get_normal(calculator, calculator_m, hit_point, min_id, normal);
	auto textures = min_shape->getTextures(min_id);
	apply_normal_map(calculator, calculator_m, hit_point, textures, min_shape, min_id, normal);
	apply_bump_map(calculator, calculator_m, hit_point, textures, min_shape, min_id, normal);

	if (mat->mt == Mirror || mat->mt == Dielectric || mat->mt == Conductor)
	{
		vec3 recursive_color;
		if (depth < max_depth)
		{
			if (mat->mt == Mirror)
			{
				vec3 R;
				calculate_reflected_dir(calculator, normal, ray_dir, R, mat->roughness);
				vec3 offset;
				calculator.mult_scalar(normal, shadowrayepsilon, offset);
				vec3 next_o;
				calculator.add(hit_point, offset, next_o);
				path_trace(calculator, calculator_m, next_o, R, recursive_color, raytime, culling, bg, is_probe, depth + 1, settings);
				calculator.mult(recursive_color, mat->MirrorReflectance, color);
			}
			else if (mat->mt == Conductor)
			{
				vec3 R;
				calculate_reflected_dir(calculator, normal, ray_dir, R, mat->roughness);

				float cosTheta;
				calculator.dot(ray_dir, normal, cosTheta);
				cosTheta = fabsf(cosTheta);

				float n = mat->RefractionIndex;
				float k = mat->AbsorptionIndex;

				float c = cosTheta;
				float c2 = c * c;
				float s2 = 1.0f - c2;

				float n2 = n * n;
				float k2 = k * k;
				float n2_k2 = n2 - k2;
				float n2k2_4 = 4.0f * n2 * k2;

				float a2_b2 = sqrtf((n2_k2 - s2) * (n2_k2 - s2) + n2k2_4);
				float a2 = 0.5f * (a2_b2 + n2_k2 - s2);
				float a = sqrtf(a2);
				float b2 = a2_b2 - a2;
				float b = sqrtf(b2);

				float Rs_num = a2_b2 + c2 - (2.0f * a * c);
				float Rs_den = a2_b2 + c2 + (2.0f * a * c);
				float Rs = (Rs_den == 0.0f) ? 1.0f : Rs_num / Rs_den;

				float ac = a * c;
				float bc = b * c;
				float Rp_num_term1 = ac - s2;
				float Rp_den_term1 = ac + s2;
				float Rp_num = (Rp_num_term1 * Rp_num_term1) + (bc * bc);
				float Rp_den = (Rp_den_term1 * Rp_den_term1) + (bc * bc);
				float Rp = (Rp_den == 0.0f) ? 1.0f : Rs * (Rp_num / Rp_den);

				float F_scalar = 0.5f * (Rs + Rp);
				F_scalar = std::clamp(F_scalar, 0.0f, 1.0f);

				vec3 F_vec(F_scalar, F_scalar, F_scalar);
				vec3 F;
				calculator.mult(F_vec, mat->MirrorReflectance, F);

				vec3 offset;
				calculator.mult_scalar(normal, shadowrayepsilon, offset);
				vec3 next_o;
				calculator.add(hit_point, offset, next_o);

				path_trace(calculator, calculator_m, next_o, R, recursive_color, raytime, culling, bg, is_probe, depth + 1, settings);
				calculator.mult(recursive_color, F, color);
			}
			else if (mat->mt == Dielectric)
			{
				float n1 = 1.0f;
				float n2 = mat->RefractionIndex;
				float cosTheta;
				calculator.dot(ray_dir, normal, cosTheta);

				bool entering = cosTheta < 0.0f;
				vec3 real_normal = normal;
				if (!entering)
				{
					std::swap(n1, n2);
					calculator.mult_scalar(normal, -1.0f, real_normal);
					cosTheta = -cosTheta;
				}
				else
				{
					cosTheta = -cosTheta;
				}

				float r0 = (n1 - n2) / (n1 + n2);
				r0 = r0 * r0;
				float F = r0 + (1.0f - r0) * std::pow(1.0f - cosTheta, 5.0f);

				vec3 R;
				calculate_reflected_dir(calculator, real_normal, ray_dir, R, mat->roughness);
				vec3 offset;
				calculator.mult_scalar(real_normal, shadowrayepsilon, offset);
				vec3 refl_orig;
				calculator.add(hit_point, offset, refl_orig);

				vec3 refl_col;
				path_trace(calculator, calculator_m, refl_orig, R, refl_col, raytime, culling, bg, is_probe, depth + 1, settings);

				vec3 T;
				if (calculate_refracted_dir(calculator, real_normal, ray_dir, n1, n2, T, mat->roughness))
				{
					vec3 refr_col;
					vec3 neg_offset;
					calculator.mult_scalar(offset, -1.0f, neg_offset);
					vec3 refr_orig;
					calculator.add(hit_point, neg_offset, refr_orig);

					path_trace(calculator, calculator_m, refr_orig, T, refr_col, raytime, culling, bg, is_probe, depth + 1, settings);

					if (!entering)
					{
						vec3 absorb;
						calculator.mult_scalar(mat->AbsorptionCoefficient, -min_t, absorb);
						calculator.exp(absorb, absorb);
						calculator.mult(refr_col, absorb, refr_col);
					}

					vec3 term1, term2;
					calculator.mult_scalar(refl_col, F, term1);
					calculator.mult_scalar(refr_col, 1.0f - F, term2);
					calculator.add(term1, term2, color);
				}
				else
				{
					color = refl_col;
				}
				calculator.mult(color, mat->MirrorReflectance, color);
			}
		}
		return;
	}

	float rr_prob = 1.0f;
	if (settings.russianRoulette && depth >= settings.minRecursionDepth)
	{
		float max_refl = std::max({mat->DiffuseReflectance.get_x(), mat->DiffuseReflectance.get_y(), mat->DiffuseReflectance.get_z()});
		rr_prob = std::min(max_refl, 0.99f);
		if (get_random_float() > rr_prob)
			return;
	}

	if (settings.nextEventEstimation)
	{
		for (Light *light : *lights)
		{
			int samples = light->get_sample_count();
			for (int s = 0; s < samples; ++s)
			{
				vec3 L_i, light_dir, light_pos;
				float dist;
				light->get_sample(calculator, calculator_m, hit_point, normal, get_random_float(), get_random_float(), light_pos, L_i, light_dir, dist);

				vec3 shadow_orig;
				calculator.mult_scalar(light_dir, shadowrayepsilon, shadow_orig);
				calculator.add(hit_point, shadow_orig, shadow_orig);

				float t_shadow;
				shape *sh_hit = 0;
				int sh_id;
				bool in_shadow = false;
				if (bvhx->intersect(calculator, calculator_m, shadow_orig, light_dir, raytime, t_shadow, &sh_hit, sh_id, false, shadowrayepsilon, true, dist))
				{
					if (t_shadow < dist - shadowrayepsilon)
						in_shadow = true;
				}

				if (!in_shadow)
				{
					float ndotl;
					calculator.dot(normal, light_dir, ndotl);
					ndotl = std::max(ndotl, 0.0f);
					if (ndotl > 0.0f)
					{
						vec3 f_r;
						calculator.mult_scalar(mat->DiffuseReflectance, 1.0f / 3.14159265359f, f_r);
						vec3 contrib;
						calculator.mult(f_r, L_i, contrib);
						calculator.mult_scalar(contrib, ndotl, contrib);
						calculator.add(color, contrib, color);
					}
				}
			}
		}
	}

	if (depth < max_depth)
	{
		vec3 wi;
		float pdf;

		if (settings.importanceSampling)
			sample_cosine_hemisphere(get_random_float(), get_random_float(), normal, wi, pdf);
		else
			sample_uniform_hemisphere(get_random_float(), get_random_float(), normal, wi, pdf);

		int split = (depth == 0 && settings.splittingFactor > 1) ? settings.splittingFactor : 1;
		vec3 indirect_accum(0, 0, 0);

		for (int i = 0; i < split; ++i)
		{
			if (split > 1)
			{
				if (settings.importanceSampling)
					sample_cosine_hemisphere(get_random_float(), get_random_float(), normal, wi, pdf);
				else
					sample_uniform_hemisphere(get_random_float(), get_random_float(), normal, wi, pdf);
			}

			vec3 bounce_color;
			vec3 offset;
			calculator.mult_scalar(normal, shadowrayepsilon, offset);
			vec3 next_o;
			calculator.add(hit_point, offset, next_o);

			path_trace(calculator, calculator_m, next_o, wi, bounce_color, raytime, culling, bg, is_probe, depth + 1, settings);

			float ndotl;
			calculator.dot(normal, wi, ndotl);
			ndotl = std::max(ndotl, 0.0f);
			if (pdf > 1e-6f)
			{
				vec3 f_r;
				calculator.mult_scalar(mat->DiffuseReflectance, 1.0f / 3.14159265359f, f_r);
				vec3 val;
				calculator.mult(bounce_color, f_r, val);
				calculator.mult_scalar(val, ndotl / pdf, val);
				calculator.add(indirect_accum, val, indirect_accum);
			}
		}

		calculator.mult_scalar(indirect_accum, 1.0f / split, indirect_accum);

		if (rr_prob < 1.0f && rr_prob > 0.0f)
			calculator.mult_scalar(indirect_accum, 1.0f / rr_prob, indirect_accum);

		calculator.add(color, indirect_accum, color);
	}
}

void ray_tracer::trace(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
					   const float &raytime, const bool culling, bool is_hdr, texture *bg, bool is_probe, float pixelu,
					   float pixelv, float *output, const RenderSettings &settings) const
{
	vec3 color;

	if (settings.isPathTracing)
	{
		path_trace(calculator, calculator_m, ray_origin, ray_dir, color, raytime, culling, bg, is_probe, 0, settings);
	}
	else
	{
		trace_rec(calculator, calculator_m, ray_origin, ray_dir, color, raytime, culling, is_hdr, bg, is_probe, pixelu, pixelv, 0);
	}

	color.store();

	if (settings.sampleMaxVal > 0.0f)
	{
		float r = color.get_x();
		float g = color.get_y();
		float b = color.get_z();
		r = std::min(r, settings.sampleMaxVal);
		g = std::min(g, settings.sampleMaxVal);
		b = std::min(b, settings.sampleMaxVal);
		color.load(r, g, b);
	}

	output[0] = color.get_x();
	output[1] = color.get_y();
	output[2] = color.get_z();
}