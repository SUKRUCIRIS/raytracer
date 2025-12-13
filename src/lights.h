#pragma once
#include "algebra.h"

class Light
{
protected:
	Light() {};

public:
	virtual ~Light() = default;

	virtual int get_sample_count() const = 0;

	virtual void get_sample(simd_vec3 &calculator,
							const vec3 &hit_point,
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
					float rand_u, float rand_v,
					vec3 &sample_pos,
					vec3 &incident_radiance,
					vec3 &light_dir,
					float &dist) const override;
};