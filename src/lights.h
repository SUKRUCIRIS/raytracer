#pragma once
#include "algebra.h"
#include "texture.h"

class Light
{
protected:
	Light() {};

public:
	virtual ~Light() = default;

	virtual int get_sample_count() const = 0;

	virtual void get_sample(simd_vec3 &calculator,
							const vec3 &hit_point,
							const vec3 &normal,
							float rand_u, float rand_v,
							vec3 &sample_pos,
							vec3 &incident_radiance,
							vec3 &light_dir,
							float &dist) const = 0;
};

class PointLight : public Light
{
public:
	vec3 position;
	vec3 intensity;

	PointLight(vec3 pos, vec3 inten);

	int get_sample_count() const override;

	void get_sample(simd_vec3 &calculator,
					const vec3 &hit_point,
					const vec3 &normal,
					float rand_u, float rand_v,
					vec3 &sample_pos,
					vec3 &incident_radiance,
					vec3 &light_dir,
					float &dist) const override;
};

class AreaLight : public Light
{
public:
	vec3 position;
	vec3 normal;
	vec3 radiance;
	float size;
	vec3 u_vec;
	vec3 v_vec;

	int samples;

	AreaLight(simd_vec3 &calculator, vec3 pos, vec3 norm, float sz, vec3 rad, int sample_count = 16);

	int get_sample_count() const override;

	void get_sample(simd_vec3 &calculator,
					const vec3 &hit_point,
					const vec3 &normal,
					float rand_u, float rand_v,
					vec3 &sample_pos,
					vec3 &incident_radiance,
					vec3 &light_dir,
					float &dist) const override;
};

class DirectionalLight : public Light
{
public:
	vec3 direction;
	vec3 radiance;

	DirectionalLight(simd_vec3 &calculator, vec3 dir, vec3 rad);

	int get_sample_count() const override;

	void get_sample(simd_vec3 &calculator,
					const vec3 &hit_point,
					const vec3 &normal,
					float rand_u, float rand_v,
					vec3 &sample_pos,
					vec3 &incident_radiance,
					vec3 &light_dir,
					float &dist) const override;
};

class SpotLight : public Light
{
public:
	vec3 position;
	vec3 direction;
	vec3 intensity;
	float coverage_angle_cos;
	float falloff_angle_cos;

	SpotLight(simd_vec3 &calculator, vec3 pos, vec3 dir, vec3 inten, float coverage_deg, float falloff_deg);

	int get_sample_count() const override;

	void get_sample(simd_vec3 &calculator,
					const vec3 &hit_point,
					const vec3 &normal,
					float rand_u, float rand_v,
					vec3 &sample_pos,
					vec3 &incident_radiance,
					vec3 &light_dir,
					float &dist) const override;
};

class SphericalDirectionalLight : public Light
{
public:
	const image *env_map;
	bool use_cosine_sampling;
	bool is_probe_map;

	SphericalDirectionalLight(const image *img, bool cosine_sample, bool is_probe);

	int get_sample_count() const override;

	void get_sample(simd_vec3 &calculator,
					const vec3 &hit_point,
					const vec3 &normal,
					float rand_u, float rand_v,
					vec3 &sample_pos,
					vec3 &incident_radiance,
					vec3 &light_dir,
					float &dist) const override;
};