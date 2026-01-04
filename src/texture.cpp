#include "texture.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../third_party/stb/stb_image.h"
#ifndef TINYEXR_IMPLEMENTATION
#define TINYEXR_IMPLEMENTATION
#endif
#include "../third_party/tinyexr/tinyexr.h"

bool is_exr(const char *filename)
{
	std::string fn(filename);
	if (fn.length() < 4)
		return false;
	std::string ext = fn.substr(fn.length() - 4);
	return ext == ".exr" || ext == ".EXR";
}

image::image(const char *filename, int id) : id(id)
{
	data = nullptr;
	hdr_data = nullptr;
	is_hdr = is_exr(filename);

	if (is_hdr)
	{
		const char *err = nullptr;
		int ret = LoadEXR(&hdr_data, &width, &height, filename, &err);

		if (ret != TINYEXR_SUCCESS)
		{
			if (err)
			{
				printf("TinyEXR Error: %s\n", err);
				FreeEXRErrorMessage(err);
			}
			width = 0;
			height = 0;
		}
		else
		{
			channels = 4;
		}
	}
	else
	{
		data = stbi_load(filename, &width, &height, &channels, 3);
		if (!data)
		{
			printf("Error: Could not load texture %s\n", filename);
			width = 0;
			height = 0;
		}
	}
}

image::~image()
{
	if (data)
		stbi_image_free(data);
	if (hdr_data)
		free(hdr_data);
}

void image::sample_nearest(float u, float v, vec3 &color) const
{
	if (!data && !hdr_data)
		return;

	u -= std::floor(u);
	v -= std::floor(v);

	int x = static_cast<int>(u * width);
	int y = static_cast<int>(v * height);

	if (x >= width)
		x = width - 1;
	if (y >= height)
		y = height - 1;

	if (is_hdr)
	{
		int index = (y * width + x) * 4;
		color.load(hdr_data[index + 0], hdr_data[index + 1], hdr_data[index + 2]);
	}
	else
	{
		int index = (y * width + x) * 3;
		float r = data[index + 0] * (1.0f / 255.0f);
		float g = data[index + 1] * (1.0f / 255.0f);
		float b = data[index + 2] * (1.0f / 255.0f);
		color.load(r, g, b);
	}
}

void image::sample_bilinear(float u, float v, vec3 &color) const
{
	if (!data && !hdr_data)
		return;

	u -= std::floor(u);
	v -= std::floor(v);

	float px = u * width - 0.5f;
	float py = v * height - 0.5f;

	int x = static_cast<int>(std::floor(px));
	int y = static_cast<int>(std::floor(py));

	float u_ratio = px - x;
	float v_ratio = py - y;
	float u_opp = 1.0f - u_ratio;
	float v_opp = 1.0f - v_ratio;

	int x0 = (x % width + width) % width;
	int x1 = ((x + 1) % width + width) % width;
	int y0 = (y % height + height) % height;
	int y1 = ((y + 1) % height + height) % height;

	int stride = is_hdr ? 4 : 3;

	int row0_idx = y0 * width * stride;
	int row1_idx = y1 * width * stride;

	int idx00 = row0_idx + x0 * stride;
	int idx10 = row0_idx + x1 * stride;
	int idx01 = row1_idx + x0 * stride;
	int idx11 = row1_idx + x1 * stride;

	float final_color[3];

	for (int c = 0; c < 3; ++c)
	{
		float val00, val10, val01, val11;

		if (is_hdr)
		{
			val00 = hdr_data[idx00 + c];
			val10 = hdr_data[idx10 + c];
			val01 = hdr_data[idx01 + c];
			val11 = hdr_data[idx11 + c];
		}
		else
		{
			val00 = data[idx00 + c];
			val10 = data[idx10 + c];
			val01 = data[idx01 + c];
			val11 = data[idx11 + c];
		}

		float top = val00 * u_opp + val10 * u_ratio;
		float bot = val01 * u_opp + val11 * u_ratio;
		final_color[c] = top * v_opp + bot * v_ratio;
	}

	if (is_hdr)
	{
		color.load(final_color[0], final_color[1], final_color[2]);
	}
	else
	{
		color.load(
			final_color[0] * (1.0f / 255.0f),
			final_color[1] * (1.0f / 255.0f),
			final_color[2] * (1.0f / 255.0f));
	}
}

void texture::sample(float u, float v, vec3 h, vec3 &color) const
{
	if (this->im)
	{
		if (this->interp == nearest)
		{
			this->im->sample_nearest(u, v, color);
		}
		else
		{
			this->im->sample_bilinear(u, v, color);
		}
	}
	else if (ischess)
	{
		if (is_black_chess(u, v, Scale, Offset))
		{
			color = BlackColor;
		}
		else
		{
			color = WhiteColor;
		}
	}
	else
	{
		float x = perlin(h.get_x(), h.get_y(), h.get_z(), NoiseScale, nc, NumOctaves);
		color.load(x);
	}
}

class Perlin
{
public:
	static const int p[512];

	static float noise(float x, float y, float z)
	{
		int X = static_cast<int>(std::floor(x)) & 255;
		int Y = static_cast<int>(std::floor(y)) & 255;
		int Z = static_cast<int>(std::floor(z)) & 255;

		x -= std::floor(x);
		y -= std::floor(y);
		z -= std::floor(z);

		float u = fade(x), v = fade(y), w = fade(z);

		int A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z;
		int B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;

		return lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)), lerp(u, grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z))),
					lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1), grad(p[BA + 1], x - 1, y, z - 1)),
						 lerp(u, grad(p[AB + 1], x, y - 1, z - 1),
							  grad(p[BB + 1], x - 1, y - 1, z - 1))));
	}

private:
	static inline float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
	static inline float lerp(float t, float a, float b) { return a + t * (b - a); }
	static inline float grad(int hash, float x, float y, float z)
	{
		int h = hash & 15;
		float u = h < 8 ? x : y;
		float v = h < 4 ? y : h == 12 || h == 14 ? x
												 : z;
		return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
	}
};

const int Perlin::p[512] = {151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180,
							151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180};

float perlin(float u, float v, float m, float NoiseScale, NoiseConversion nc, float NumOctaves)
{
	float s = 0.0f;
	float x = u * NoiseScale;
	float y = v * NoiseScale;
	float z = m * NoiseScale;

	float weight = 1.0f;
	float maxAmplitude = 0.0f;
	int K = static_cast<int>(NumOctaves);
	if (K < 1)
		K = 1;

	for (int k = 0; k < K; k++)
	{
		s += weight * Perlin::noise(x, y, z);
		maxAmplitude += weight;

		x *= 2.0f;
		y *= 2.0f;
		weight *= 0.5f;
	}

	float s_norm = s / maxAmplitude;

	if (nc == absval)
	{
		return std::fabs(s_norm);
	}
	else
	{
		return (s_norm + 1.0f) * 0.5f;
	}
}

bool is_black_chess(float u, float v, float Scale, float Offset)
{
	int x = static_cast<int>(std::floor((u + Offset) * Scale));
	int y = static_cast<int>(std::floor((v + Offset) * Scale));

	return (x + y) % 2 != 0;
}