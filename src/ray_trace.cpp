#include "ray_trace.h"
#include <algorithm>
#include <cfloat>
#include <cmath>

void ray_tracer::calculate_color(simd_vec3 &calculator, const vec3 &normal, const material *mat,
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

		if (d_squared < 1e-6)
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

		for (auto &&s : *shapes)
		{
			if (s == min_shape)
				continue;

			if (s->intersect(calculator, shadow_origin, light_dir, t_shadow, false, intersectionepsilon))
			{
				if (t_shadow < distance_to_light)
				{
					in_shadow = true;
					break;
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

void ray_tracer::trace_rec(simd_vec3 &calculator, const vec3 &ray_origin, const vec3 &ray_dir,
						   vec3 &color, int depth) const
{
	if (depth == max_depth)
	{
		color = backgroundcolor;
		return;
	}
	float t;
	float min_t = FLT_MAX;
	shape *min_shape = 0;

	for (auto &&s : *shapes)
	{
		if (s->intersect(calculator, ray_origin, ray_dir, t, true, intersectionepsilon))
		{
			if (t < min_t)
			{
				min_t = t;
				min_shape = s;
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
		trace_rec(calculator, hit_point, R, color, depth + 1);
		calculator.mult(color, mat->MirrorReflectance, color);
		vec3 own_color;
		calculate_color(calculator, normal, mat, hit_point, ray_origin, min_shape, own_color);
		calculator.add(color, own_color, color);
	}
	else
	{
		calculate_color(calculator, normal, mat, hit_point, ray_origin, min_shape, color);
	}
}

void ray_tracer::trace(simd_vec3 &calculator, const vec3 &ray_origin, const vec3 &ray_dir,
					   const int &index, unsigned char *output) const
{
	vec3 color;

	trace_rec(calculator, ray_origin, ray_dir, color, 0);

	color.store();
	output[index * 3] = static_cast<unsigned char>(std::clamp(color.get_x() * 255, 0.0f, 255.0f));
	output[index * 3 + 1] = static_cast<unsigned char>(std::clamp(color.get_y() * 255, 0.0f, 255.0f));
	output[index * 3 + 2] = static_cast<unsigned char>(std::clamp(color.get_z() * 255, 0.0f, 255.0f));
}