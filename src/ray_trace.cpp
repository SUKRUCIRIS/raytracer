#include "ray_trace.h"
#include <algorithm>
#include <cfloat>
#include <cmath>

void ray_tracer::calculate_color(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &normal, const material *mat,
								 const vec3 &hit_point, const vec3 &ray_origin, const shape *min_shape,
								 vec3 &color) const
{
	calculator.mult(mat->AmbientReflectance, ambientlight, color);

	for (auto &&pl : *point_lights)
	{
		vec3 light_dir_unnormalized;
		calculator.subs(pl.position, hit_point, light_dir_unnormalized);

		float d_squared;
		calculator.dot(light_dir_unnormalized, light_dir_unnormalized, d_squared);
		float distance_to_light = std::sqrt(d_squared);

		if (d_squared < intersectionepsilon)
			continue;

		vec3 attenuated_intensity;
		calculator.mult_scalar(pl.intensity, 1.0f / d_squared, attenuated_intensity);

		vec3 light_dir;
		calculator.mult_scalar(light_dir_unnormalized, 1.0f / distance_to_light, light_dir);

		bool in_shadow = false;
		float t_shadow;
		vec3 shadow_origin;
		calculator.mult_scalar(light_dir, shadowrayepsilon, shadow_origin);
		calculator.add(hit_point, shadow_origin, shadow_origin);

		if (use_grid)
		{
			shape *hit_shape = 0;
			if (gridx->intersect(calculator, calculator_m, shadow_origin, light_dir, t_shadow, &hit_shape, false, intersectionepsilon))
			{
				if (t_shadow < distance_to_light && hit_shape != min_shape)
				{
					in_shadow = true;
				}
			}
		}
		else
		{
			for (auto &&s : *shapes)
			{
				if (s == min_shape)
					continue;

				if (s->intersect(calculator, calculator_m, shadow_origin, light_dir, t_shadow, false, intersectionepsilon))
				{
					if (t_shadow < distance_to_light)
					{
						in_shadow = true;
						break;
					}
				}
			}
		}
		if (in_shadow)
			continue;

		float ndotl;
		calculator.dot(normal, light_dir, ndotl);
		ndotl = std::max(ndotl, 0.0f);

		vec3 diffuse;
		calculator.mult_scalar(mat->DiffuseReflectance, ndotl, diffuse);
		calculator.mult(diffuse, attenuated_intensity, diffuse);
		calculator.add(color, diffuse, color);

		vec3 view_dir;
		calculator.subs(ray_origin, hit_point, view_dir);
		calculator.normalize(view_dir, view_dir);

		vec3 h;
		calculator.add(light_dir, view_dir, h);
		calculator.normalize(h, h);

		float ndoth;
		calculator.dot(normal, h, ndoth);
		ndoth = std::max(ndoth, 0.0f);

		float spec_factor = powf(ndoth, mat->PhongExponent);
		vec3 specular;
		calculator.mult_scalar(mat->SpecularReflectance, spec_factor, specular);
		calculator.mult(specular, attenuated_intensity, specular);
		calculator.add(color, specular, color);
	}

	calculator.mult_scalar(color, 0.004f, color);
}

void ray_tracer::calculate_reflected_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I, vec3 &R)
{
	float dot;
	calculator.dot(I, N, dot);

	calculator.mult_scalar(N, dot * -2.0f, R);
	calculator.add(I, R, R);
	calculator.normalize(R, R);
}

bool ray_tracer::calculate_refracted_dir(simd_vec3 &calculator, const vec3 &N, const vec3 &I,
										 float n1, float n2, vec3 &T)
{
	float cosi;
	calculator.dot(I, N, cosi);
	cosi = -cosi;
	vec3 normal = N;
	float eta = n1 / n2;
	if (cosi < 0.0f)
	{
		cosi = -cosi;
		eta = n2 / n1;
		calculator.mult_scalar(N, -1.0f, normal);
	}

	float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
	if (k < 0.0f)
	{
		return false;
	}

	vec3 term1, term2;
	calculator.mult_scalar(I, eta, term1);
	calculator.mult_scalar(normal, (eta * cosi - sqrtf(k)), term2);
	calculator.add(term1, term2, T);
	calculator.normalize(T, T);

	return true;
}

void ray_tracer::trace_rec(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
						   vec3 &color, const bool culling, int depth) const
{
	if (depth >= max_depth)
	{
		color = backgroundcolor;
		return;
	}
	float t;
	float min_t = FLT_MAX;
	shape *min_shape = 0;

	if (use_grid)
	{
		shape *hit_shape = 0;
		if (gridx->intersect(calculator, calculator_m, ray_origin, ray_dir, t, &hit_shape, culling, intersectionepsilon))
		{
			if (t < min_t)
			{
				min_t = t;
				min_shape = hit_shape;
			}
		}
	}
	else
	{
		for (auto &&s : *shapes)
		{
			if (s->intersect(calculator, calculator_m, ray_origin, ray_dir, t, culling, intersectionepsilon))
			{
				if (t < min_t)
				{
					min_t = t;
					min_shape = s;
				}
			}
		}
	}

	if (!min_shape)
	{
		color = backgroundcolor;
		return;
	}
	material *mat = min_shape->getMaterial();
	vec3 hit_point;
	calculator.mult_scalar(ray_dir, min_t, hit_point);
	calculator.add(ray_origin, hit_point, hit_point);
	vec3 normal;
	min_shape->get_normal(calculator, hit_point, normal);
	if (mat->mt == Mirror)
	{
		vec3 R;
		calculate_reflected_dir(calculator, normal, ray_dir, R);
		vec3 offset;
		calculator.mult_scalar(normal, shadowrayepsilon, offset);
		calculator.add(hit_point, offset, hit_point);
		trace_rec(calculator, calculator_m, hit_point, R, color, culling, depth + 1);
		calculator.mult(color, mat->MirrorReflectance, color);

		vec3 own_color;
		calculate_color(calculator, calculator_m, normal, mat, hit_point, ray_origin, min_shape, own_color);
		calculator.add(color, own_color, color);
	}
	else if (mat->mt == Conductor)
	{
		vec3 R;
		calculate_reflected_dir(calculator, normal, ray_dir, R);

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

		trace_rec(calculator, calculator_m, hit_point, R, reflectedColor, culling, depth + 1);
		calculator.mult(reflectedColor, F, color);

		vec3 own_color;
		calculate_color(calculator, calculator_m, normal, mat, hit_point, ray_origin, min_shape, own_color);
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
		calculate_reflected_dir(calculator, normal, ray_dir, reflectDir);
		vec3 offsetR;
		calculator.mult_scalar(normal, shadowrayepsilon, offsetR);
		calculator.add(hit_point, offsetR, offsetR);
		trace_rec(calculator, calculator_m, offsetR, reflectDir, reflectColor, culling, depth + 1);

		calculator.mult(reflectColor, mat->MirrorReflectance, reflectColor);

		vec3 refractDir, refractColor;
		bool refracts = calculate_refracted_dir(calculator, normal, ray_dir, n1, n2, refractDir);

		if (refracts)
		{
			vec3 offsetT;
			calculator.mult_scalar(normal, -shadowrayepsilon, offsetT);
			calculator.add(hit_point, offsetT, offsetT);
			trace_rec(calculator, calculator_m, offsetT, refractDir, refractColor, culling, depth + 1);

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
		calculate_color(calculator, calculator_m, normal, mat, hit_point, ray_origin, min_shape, own_color);
		calculator.add(color, own_color, color);
	}
	else
	{
		calculate_color(calculator, calculator_m, normal, mat, hit_point, ray_origin, min_shape, color);
	}
}

void ray_tracer::trace(simd_vec3 &calculator, simd_mat4 &calculator_m, const vec3 &ray_origin, const vec3 &ray_dir,
					   const int &index, const bool culling, unsigned char *output) const
{
	vec3 color;

	trace_rec(calculator, calculator_m, ray_origin, ray_dir, color, culling, 0);

	color.store();
	output[index * 3] = static_cast<unsigned char>(std::clamp(color.get_x() * 255, 0.0f, 255.0f));
	output[index * 3 + 1] = static_cast<unsigned char>(std::clamp(color.get_y() * 255, 0.0f, 255.0f));
	output[index * 3 + 2] = static_cast<unsigned char>(std::clamp(color.get_z() * 255, 0.0f, 255.0f));
}