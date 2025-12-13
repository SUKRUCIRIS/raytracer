#include "texture.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../third_party/stb/stb_image.h"

image::image(const char *filename, int id) : id(id)
{
	data = stbi_load(filename, &width, &height, &channels, 3);

	if (!data)
	{
		printf("Error: Could not load texture %s\n", filename);
		width = 0;
		height = 0;
	}
}

image::~image()
{
	if (data)
	{
		stbi_image_free(data);
	}
}

void image::sample_nearest(float u, float v, vec3 &color)
{
	if (!data)
		return;

	u -= std::floor(u);
	v -= std::floor(v);

	int x = static_cast<int>(u * width);
	int y = static_cast<int>(v * height);

	if (x >= width)
		x = width - 1;
	if (y >= height)
		y = height - 1;

	int index = (y * width + x) * 3;

	float r = data[index + 0] * (1.0f / 255.0f);
	float g = data[index + 1] * (1.0f / 255.0f);
	float b = data[index + 2] * (1.0f / 255.0f);

	color.load(r, g, b);
}

void image::sample_bilinear(float u, float v, vec3 &color)
{
	if (!data)
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

	int row0_idx = y0 * width * 3;
	int row1_idx = y1 * width * 3;

	int idx00 = row0_idx + x0 * 3;
	int idx10 = row0_idx + x1 * 3;
	int idx01 = row1_idx + x0 * 3;
	int idx11 = row1_idx + x1 * 3;

	float final_color[3];

	for (int c = 0; c < 3; ++c)
	{
		float top = data[idx00 + c] * u_opp + data[idx10 + c] * u_ratio;
		float bot = data[idx01 + c] * u_opp + data[idx11 + c] * u_ratio;
		final_color[c] = top * v_opp + bot * v_ratio;
	}

	color.load(
		final_color[0] * (1.0f / 255.0f),
		final_color[1] * (1.0f / 255.0f),
		final_color[2] * (1.0f / 255.0f));
}