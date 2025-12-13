#include "lights.h"

PointLight::PointLight(vec3 pos, vec3 inten) : position(pos), intensity(inten) {}

int PointLight::get_sample_count() const { return 1; }

void PointLight::get_sample(simd_vec3 &calculator,
							const vec3 &hit_point,
							float rand_u, float rand_v,
							vec3 &sample_pos,
							vec3 &incident_radiance,
							vec3 &light_dir,
							float &dist) const
{
	vec3 dir_unnormalized;
	calculator.subs(position, hit_point, dir_unnormalized);

	float d2;
	calculator.dot(dir_unnormalized, dir_unnormalized, d2);
	dist = std::sqrt(d2);

	if (d2 < 1e-6f)
	{
		light_dir = dir_unnormalized;
		incident_radiance.load(0, 0, 0);
		return;
	}

	calculator.mult_scalar(dir_unnormalized, 1.0f / dist, light_dir);

	light_dir.store();

	calculator.mult_scalar(intensity, 1.0f / d2, incident_radiance);

	incident_radiance.store();
}

AreaLight::AreaLight(simd_vec3 &calculator, vec3 pos, vec3 norm, float sz, vec3 rad, int sample_count)
	: position(pos), normal(norm), size(sz), radiance(rad), samples(sample_count)
{
	vec3 helper;
	if (std::abs(normal.get_x()) > 0.1f)
	{
		helper.load(0.0f, 1.0f, 0.0f);
	}
	else
	{
		helper.load(1.0f, 0.0f, 0.0f);
	}

	calculator.cross(helper, normal, u_vec);
	calculator.normalize(u_vec, u_vec);

	calculator.cross(normal, u_vec, v_vec);
	calculator.normalize(v_vec, v_vec);
}

int AreaLight::get_sample_count() const { return samples; }

void AreaLight::get_sample(simd_vec3 &calculator,
						   const vec3 &hit_point,
						   float rand_u, float rand_v,
						   vec3 &sample_pos,
						   vec3 &incident_radiance,
						   vec3 &light_dir,
						   float &dist) const
{
	float u_offset_val = (rand_u - 0.5f) * size;
	float v_offset_val = (rand_v - 0.5f) * size;

	vec3 u_offset, v_offset;
	calculator.mult_scalar(u_vec, u_offset_val, u_offset);
	calculator.mult_scalar(v_vec, v_offset_val, v_offset);

	calculator.add(position, u_offset, sample_pos);
	calculator.add(sample_pos, v_offset, sample_pos);

	vec3 dir_unnormalized;
	calculator.subs(sample_pos, hit_point, dir_unnormalized);

	float d2;
	calculator.dot(dir_unnormalized, dir_unnormalized, d2);
	dist = std::sqrt(d2);

	calculator.mult_scalar(dir_unnormalized, 1.0f / dist, light_dir);

	vec3 neg_light_dir;
	calculator.mult_scalar(light_dir, -1.0f, neg_light_dir);

	float cos_theta_light;
	calculator.dot(normal, neg_light_dir, cos_theta_light);

	if (cos_theta_light <= 0.0f)
	{
		cos_theta_light = -cos_theta_light;
	}

	float area = size * size;
	float factor = (area * cos_theta_light) / d2;

	calculator.mult_scalar(radiance, factor, incident_radiance);

	light_dir.store();
	incident_radiance.store();
}