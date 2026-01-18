#include "lights.h"
#include "texture.h"

PointLight::PointLight(vec3 pos, vec3 inten) : position(pos), intensity(inten) {}

int PointLight::get_sample_count() const { return 1; }

void PointLight::get_sample(simd_vec3 &calculator,
							const vec3 &hit_point,
							const vec3 &normal,
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
		light_dir.store();
		return;
	}

	calculator.mult_scalar(dir_unnormalized, 1.0f / dist, light_dir);
	light_dir.store();

	sample_pos = position;

	calculator.mult_scalar(intensity, 1.0f / d2, incident_radiance);
	incident_radiance.store();
}

AreaLight::AreaLight(simd_vec3 &calculator, vec3 pos, vec3 norm, float sz, vec3 rad, int sample_count)
	: position(pos), normal(norm), size(sz), radiance(rad), samples(sample_count)
{
	vec3 helper;
	if (std::abs(normal.get_x()) > 0.1f)
		helper.load(0.0f, 1.0f, 0.0f);
	else
		helper.load(1.0f, 0.0f, 0.0f);

	calculator.cross(helper, normal, u_vec);
	calculator.normalize(u_vec, u_vec);

	calculator.cross(normal, u_vec, v_vec);
	calculator.normalize(v_vec, v_vec);
}

int AreaLight::get_sample_count() const { return samples; }

void AreaLight::get_sample(simd_vec3 &calculator,
						   const vec3 &hit_point,
						   const vec3 &normal_at_hit,
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
		incident_radiance.load(0, 0, 0);
		light_dir.store();
		return;
	}

	float area = size * size;
	float factor = (area * cos_theta_light) / d2;

	calculator.mult_scalar(radiance, factor, incident_radiance);

	light_dir.store();
	incident_radiance.store();
}

DirectionalLight::DirectionalLight(simd_vec3 &calculator, vec3 dir, vec3 rad)
	: direction(dir), radiance(rad)
{
	calculator.normalize(direction, direction);
	direction.store();
}

int DirectionalLight::get_sample_count() const { return 1; }

void DirectionalLight::get_sample(simd_vec3 &calculator,
								  const vec3 &hit_point,
								  const vec3 &normal,
								  float rand_u, float rand_v,
								  vec3 &sample_pos,
								  vec3 &incident_radiance,
								  vec3 &light_dir,
								  float &dist) const
{
	dist = std::numeric_limits<float>::max();

	calculator.mult_scalar(direction, -1.0f, light_dir);
	light_dir.store();

	vec3 far_point;
	calculator.mult_scalar(light_dir, 1e9f, far_point);
	calculator.add(hit_point, far_point, sample_pos);

	incident_radiance = radiance;
	incident_radiance.store();
}

SpotLight::SpotLight(simd_vec3 &calculator, vec3 pos, vec3 dir, vec3 inten, float coverage_deg, float falloff_deg)
	: position(pos), direction(dir), intensity(inten)
{
	float pi = 3.14159265359f;
	float cov_rad = (coverage_deg * pi / 180.0f) / 2.0f;
	float fall_rad = (falloff_deg * pi / 180.0f) / 2.0f;

	coverage_angle_cos = std::cos(cov_rad);
	falloff_angle_cos = std::cos(fall_rad);
	calculator.normalize(direction, direction);
	direction.store();
}

int SpotLight::get_sample_count() const { return 1; }

void SpotLight::get_sample(simd_vec3 &calculator,
						   const vec3 &hit_point,
						   const vec3 &normal,
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
		light_dir.store();
		incident_radiance.load(0, 0, 0);
		return;
	}

	calculator.mult_scalar(dir_unnormalized, 1.0f / dist, light_dir);
	light_dir.store();

	sample_pos = position;

	vec3 light_to_point;
	calculator.mult_scalar(light_dir, -1.0f, light_to_point);

	float cos_alpha;
	calculator.dot(light_to_point, direction, cos_alpha);

	float spot_factor = 0.0f;

	if (cos_alpha > falloff_angle_cos)
	{
		spot_factor = 1.0f;
	}
	else if (cos_alpha > coverage_angle_cos)
	{
		float num = cos_alpha - coverage_angle_cos;
		float den = falloff_angle_cos - coverage_angle_cos;
		float ratio = num / den;
		spot_factor = ratio * ratio * ratio * ratio;
	}

	float combined_factor = spot_factor / d2;
	calculator.mult_scalar(intensity, combined_factor, incident_radiance);

	incident_radiance.store();
}

SphericalDirectionalLight::SphericalDirectionalLight(const image *img, bool cosine_sample, bool is_probe)
	: env_map(img), use_cosine_sampling(cosine_sample), is_probe_map(is_probe) {}

int SphericalDirectionalLight::get_sample_count() const { return 1; }

void SphericalDirectionalLight::get_sample(simd_vec3 &calculator,
										   const vec3 &hit_point,
										   const vec3 &normal,
										   float rand_u, float rand_v,
										   vec3 &sample_pos,
										   vec3 &incident_radiance,
										   vec3 &light_dir,
										   float &dist) const
{
	dist = std::numeric_limits<float>::max();

	float pi = 3.14159265359f;
	float cos_theta, sin_theta, phi;
	float pdf;

	if (use_cosine_sampling)
	{
		phi = 2.0f * pi * rand_u;
		cos_theta = std::sqrt(rand_v);
		sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
		pdf = cos_theta / pi;
	}
	else
	{
		phi = 2.0f * pi * rand_u;
		cos_theta = rand_v;
		sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
		pdf = 1.0f / (2.0f * pi);
	}

	float lx = sin_theta * std::cos(phi);
	float ly = sin_theta * std::sin(phi);
	float lz = cos_theta;

	vec3 w = normal;
	w.store();

	vec3 u, v_basis;
	vec3 up;

	if (std::abs(w.get_y()) < 0.9f)
		up.load(0, 1, 0);
	else
		up.load(0, 0, 1);

	calculator.cross(up, w, u);
	calculator.normalize(u, u);
	calculator.cross(w, u, v_basis);
	calculator.normalize(v_basis, v_basis);

	vec3 u_comp, v_comp, w_comp;
	calculator.mult_scalar(u, lx, u_comp);
	calculator.mult_scalar(v_basis, ly, v_comp);
	calculator.mult_scalar(w, lz, w_comp);

	calculator.add(u_comp, v_comp, light_dir);
	calculator.add(light_dir, w_comp, light_dir);
	calculator.normalize(light_dir, light_dir);

	vec3 far_point;
	calculator.mult_scalar(light_dir, 1e9f, far_point);
	calculator.add(hit_point, far_point, sample_pos);
	light_dir.store();
	float u_uv, v_uv;
	float dx = light_dir.get_x();
	float dy = light_dir.get_y();
	float dz = light_dir.get_z();

	if (is_probe_map)
	{
		float len_sq = dx * dx + dy * dy;
		float r = 0.0f;
		if (len_sq > 1e-6f)
			r = (1.0f / pi) * std::acos(-dz) / std::sqrt(len_sq);
		else if (dz < 0.0f)
			r = 0.0f;

		u_uv = (r * dx + 1.0f) * 0.5f;
		v_uv = (-r * dy + 1.0f) * 0.5f;
	}
	else
	{
		u_uv = (1.0f + std::atan2(dx, -dz) / pi) * 0.5f;
		v_uv = std::acos(dy) / pi;
	}

	if (env_map)
		env_map->sample_bilinear(u_uv, v_uv, incident_radiance);
	else
		incident_radiance.load(0, 0, 0);

	if (pdf < 1e-6f)
		pdf = 1e-6f;
	calculator.mult_scalar(incident_radiance, 1.0f / pdf, incident_radiance);
	light_dir.store();
	incident_radiance.store();
}