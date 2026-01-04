#pragma once
#include "algebra.h"

class image
{
public:
	unsigned char *data;
	float *hdr_data;
	bool is_hdr;
	int id;
	int width, height, channels;
	image() = delete;
	image(const char *filename, int id);
	~image();
	void sample_nearest(float u, float v, vec3 &color) const;
	void sample_bilinear(float u, float v, vec3 &color) const;
};

enum DecalMode
{
	replace_kd,
	blend_kd,
	replace_ks,
	replace_background,
	replace_normal,
	bump_normal,
	replace_all
};

enum Interpolation
{
	nearest,
	bilinear
};

enum NoiseConversion
{
	absval,
	linear
};

class texture
{
public:
	image *im;
	int id;
	DecalMode dmode;
	Interpolation interp;
	float BumpFactor = 10;
	float NoiseScale = 1;
	NoiseConversion nc = NoiseConversion::linear;
	float NumOctaves = 1;
	bool ischess = false;
	vec3 BlackColor;
	vec3 WhiteColor;
	float Scale;
	float Offset;
	texture() = delete;
	texture(image *im, int id, DecalMode dmode, Interpolation interp)
		: im(im), id(id), dmode(dmode), interp(interp) {}
	void sample(float u, float v, vec3 h, vec3 &color) const;
};

float perlin(float u, float v, float m, float NoiseScale, NoiseConversion nc, float NumOctaves);

bool is_black_chess(float u, float v, float Scale, float Offset);