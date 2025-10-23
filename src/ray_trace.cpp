#include "ray_trace.h"
#include <algorithm>
#include <cfloat>
#include <cmath>

void ray_trace(simd_vec3 &calculator, const std::vector<shape *> *shapes, const vec3 &ray_origin, const vec3 &ray_dir, const float &intersectionepsilon,
			   const float &shadowrayepsilon, const vec3 &ambientlight, const std::vector<point_light> *point_lights, const vec3 &backgroundcolor,
			   unsigned char *output, const int &index)
{
	float t;
	float min_t = FLT_MAX;
	shape *min_shape = 0;

	// 1. Find closest intersection
	for (auto &&s : *shapes)
	{
		// Use intersectionepsilon for finding the closest hit
		if (s->intersect(calculator, ray_origin, ray_dir, t, true, intersectionepsilon))
		{
			if (t < min_t)
			{
				min_t = t;
				min_shape = s;
			}
		}
	}

	// 2. No intersection -> return background
	if (!min_shape)
	{
		output[index * 3] = static_cast<unsigned char>(std::clamp(backgroundcolor.get_x() * 255, 0.0f, 255.0f));
		output[index * 3 + 1] = static_cast<unsigned char>(std::clamp(backgroundcolor.get_y() * 255, 0.0f, 255.0f));
		output[index * 3 + 2] = static_cast<unsigned char>(std::clamp(backgroundcolor.get_z() * 255, 0.0f, 255.0f));
		return;
	}

	// 3. Compute hit point
	vec3 hit_point;
	calculator.mult_scalar(ray_dir, min_t, hit_point);
	calculator.add(ray_origin, hit_point, hit_point);

	// 4. Compute normal based on shape type
	vec3 normal;
	switch (min_shape->get_shapetype())
	{
	case Sphere:
	{
		sphere *s = static_cast<sphere *>(min_shape);
		calculator.subs(hit_point, *(s->get_center()), normal);
		calculator.normalize(normal, normal);
		break;
	}
	case Triangle:
	{
		triangle *tri = static_cast<triangle *>(min_shape);
		normal = *tri->get_normal(); // assuming triangle stores precomputed *normalized* normal
		break;
	}
	case Plane:
	{
		plane *p = static_cast<plane *>(min_shape);
		normal = *p->get_normal(); // assuming plane stores its *normalized* normal
		break;
	}
	}

	// 5. Start with ambient component
	material *mat = min_shape->getMaterial();
	vec3 color;
	calculator.mult(mat->AmbientReflectance, ambientlight, color);

	// 6. Add diffuse + specular for each point light
	for (auto &&pl : *point_lights)
	{
		vec3 light_dir_unnormalized;
		calculator.subs(pl.position, hit_point, light_dir_unnormalized);

		// --- ATTENUATION (Req 8) ---
		float d_squared;
		calculator.dot(light_dir_unnormalized, light_dir_unnormalized, d_squared);
		float distance_to_light = std::sqrt(d_squared);

		// Avoid division by zero if light is at the hit point
		if (d_squared < 1e-6) // Use a small epsilon
			continue;

		vec3 attenuated_intensity;
		calculator.mult_scalar(pl.intensity, 1.0f / d_squared, attenuated_intensity);

		vec3 light_dir; // Normalized version
		calculator.mult_scalar(light_dir_unnormalized, 1.0f / distance_to_light, light_dir);

		// --- Shadow ray check ---
		bool in_shadow = false;
		float t_shadow;
		vec3 shadow_origin;
		// Offset origin along the *light direction* to avoid self-intersection
		calculator.mult_scalar(light_dir, shadowrayepsilon, shadow_origin);
		calculator.add(hit_point, shadow_origin, shadow_origin);

		for (auto &&s : *shapes)
		{
			// Don't check for intersection with the object itself
			if (s == min_shape)
				continue;

			// Use the *intersection* epsilon here, not shadow epsilon.
			// We just need to find *an* intersection.
			if (s->intersect(calculator, shadow_origin, light_dir, t_shadow, false, intersectionepsilon))
			{
				// Check if the intersection is *between* the hit point and the light
				if (t_shadow < distance_to_light)
				{
					in_shadow = true;
					break;
				}
			}
		}
		if (in_shadow)
			continue;

		// --- Diffuse ---
		float ndotl;
		calculator.dot(normal, light_dir, ndotl);
		ndotl = std::max(ndotl, 0.0f);

		vec3 diffuse;
		calculator.mult_scalar(mat->DiffuseReflectance, ndotl, diffuse);
		// Use attenuated intensity
		calculator.mult(diffuse, attenuated_intensity, diffuse);
		calculator.add(color, diffuse, color);

		// --- SPECULAR (Blinn-Phong) (Req 6) ---
		vec3 view_dir;
		calculator.subs(ray_origin, hit_point, view_dir);
		calculator.normalize(view_dir, view_dir);

		// Halfway vector h = normalize(L + V)
		vec3 h;
		calculator.add(light_dir, view_dir, h);
		calculator.normalize(h, h);

		// Specular factor = (N . H)^phong
		float ndoth;
		calculator.dot(normal, h, ndoth);
		ndoth = std::max(ndoth, 0.0f);

		float spec_factor = powf(ndoth, mat->PhongExponent);
		vec3 specular;
		calculator.mult_scalar(mat->SpecularReflectance, spec_factor, specular);
		// Use attenuated intensity
		calculator.mult(specular, attenuated_intensity, specular);
		calculator.add(color, specular, color);
	}

	calculator.mult_scalar(color, 0.004f, color);

	color.store();
	output[index * 3] = static_cast<unsigned char>(std::clamp(color.get_x() * 255, 0.0f, 255.0f));
	output[index * 3 + 1] = static_cast<unsigned char>(std::clamp(color.get_y() * 255, 0.0f, 255.0f));
	output[index * 3 + 2] = static_cast<unsigned char>(std::clamp(color.get_z() * 255, 0.0f, 255.0f));
}