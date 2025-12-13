#pragma once
#include "algebra.h"

class image
{
public:
	unsigned char *data;
	int id;
	int width, height, channels;
	image() = delete;
	image(const char *filename, int id);
	~image();
	void sample_nearest(float u, float v, vec3 &color);
	void sample_bilinear(float u, float v, vec3 &color);
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

class texture
{
public:
	image *im;
	int id;
	DecalMode dmode;
	Interpolation interp;
	texture() = delete;
	texture(image *im, int id, DecalMode dmode, Interpolation interp)
		: im(im), id(id), dmode(dmode), interp(interp) {}
};