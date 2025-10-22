#include "parser.h"
#include "algebra.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third_party/stb/stb_image_write.h"
#include <algorithm>
#include <cfloat>

vec3 ray_trace(simd_vec3 &calculator, std::vector<shape *> *shapes, vec3 ray_origin, vec3 ray_dir, float intersectionepsilon,
			   float shadowrayepsilon, vec3 ambientlight, std::vector<point_light> *point_lights, vec3 backgroundcolor)
{
	float t;
	float min_t = FLT_MAX;
	shape *min_shape = 0;

	// 1. Find closest intersection
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

	// 2. No intersection -> return background
	if (!min_shape)
		return backgroundcolor;

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
		normal = *tri->get_normal(); // assuming triangle stores precomputed normal
		break;
	}
	case Plane:
	{
		plane *p = static_cast<plane *>(min_shape);
		normal = *p->get_normal(); // assuming plane stores its normal
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
		vec3 light_dir;
		calculator.subs(pl.position, hit_point, light_dir);
		calculator.normalize(light_dir, light_dir);

		// Shadow ray check
		bool in_shadow = false;
		float t_shadow;
		vec3 shadow_origin;
		calculator.add_scalar(hit_point, shadowrayepsilon, shadow_origin); // offset to avoid self-intersection
		for (auto &&s : *shapes)
		{
			if (s != min_shape && s->intersect(calculator, shadow_origin, light_dir, t_shadow, false, intersectionepsilon))
			{
				in_shadow = true;
				break;
			}
		}
		if (in_shadow)
			continue;

		// Diffuse
		float ndotl;
		calculator.dot(normal, light_dir, ndotl);
		ndotl = std::max(ndotl, 0.0f);

		vec3 diffuse;
		calculator.mult_scalar(mat->DiffuseReflectance, ndotl, diffuse);
		calculator.mult(diffuse, pl.intensity, diffuse);
		calculator.add(color, diffuse, color);

		// Specular
		vec3 view_dir;
		calculator.subs(ray_origin, hit_point, view_dir);
		calculator.normalize(view_dir, view_dir);

		vec3 reflect_dir;
		vec3 n_times_ndotl;
		calculator.mult_scalar(normal, 2 * ndotl, n_times_ndotl);
		calculator.subs(n_times_ndotl, light_dir, reflect_dir);
		calculator.normalize(reflect_dir, reflect_dir);

		float rdotv;
		calculator.dot(reflect_dir, view_dir, rdotv);
		rdotv = std::max(rdotv, 0.0f);

		float spec_factor = powf(rdotv, mat->PhongExponent);
		vec3 specular;
		calculator.mult_scalar(mat->SpecularReflectance, spec_factor, specular);
		calculator.mult(specular, pl.intensity, specular);
		calculator.add(color, specular, color);
	}

	vec3 denominator_vec;
	calculator.add_scalar(color, 1.0f, denominator_vec);
	calculator.div(color, denominator_vec, color);

	const float gamma = 2000.0f;
	calculator.pow(color, gamma, color);

	return color;
}

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		printf("Just give input json name...\n");
		return 0;
	}
	parser p(argv[1]);

	simd_vec3 calculator;

	auto cameras = p.get_camera(calculator);

	auto vertices = p.get_vertices();

	auto materials = p.get_materials();

	auto shapes = p.get_shapes(calculator, vertices, materials);

	float intersectionepsilon = p.get_intersectionepsilon();

	float shadowrayepsilon = p.get_shadowrayepsilon();

	vec3 backgroundcolor = p.get_backgroundcolor();

	vec3 ambientlight = p.get_ambientlight();

	auto point_lights = p.get_pointlights();

	stbi_flip_vertically_on_write(1);

	for (auto &&camera : *cameras)
	{
		unsigned char *output = new unsigned char[camera.resx * camera.resy * 3];
		memset(output, 0, camera.resx * camera.resy);
		int index = 0;
		vec3 color(0, 0, 0);
		for (auto &&raydir : camera.ray_dirs)
		{
			color = ray_trace(calculator, shapes, camera.position, raydir, intersectionepsilon, shadowrayepsilon, ambientlight, point_lights, backgroundcolor);
			color.store();
			output[index * 3] = static_cast<unsigned char>(std::clamp(color.get_x() * 255, 0.0f, 255.0f));
			output[index * 3 + 1] = static_cast<unsigned char>(std::clamp(color.get_y() * 255, 0.0f, 255.0f));
			output[index * 3 + 2] = static_cast<unsigned char>(std::clamp(color.get_z() * 255, 0.0f, 255.0f));
			index++;
		}
		stbi_write_png(camera.output.c_str(), camera.resx, camera.resy, 3, output, camera.resx * 3);
		delete output;
	}

	delete cameras;
	for (auto &&shape : *shapes)
	{
		delete shape;
	}
	delete shapes;
	delete vertices;
	delete materials;
	delete point_lights;

	return 0;
}