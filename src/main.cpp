#include "parser.h"
#include "algebra.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third_party/stb/stb_image_write.h"
#include "ray_trace.h"
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
	int thread_count = 1;
	if (argc == 2)
	{
		thread_count = std::thread::hardware_concurrency();
	}
	else if (argc == 3)
	{
		thread_count = atoi(argv[2]);
	}
	else
	{
		printf("Just give input json name and thread count:\n./raytracer json_file thread_count\n");
		return 0;
	}

	printf("SIMD On / Thread count: %d\n", thread_count);
	printf("JSON parsing started\n");
	auto start = std::chrono::high_resolution_clock::now();

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

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	printf("JSON parsing done: %d ms\n", duration.count());

	for (auto &&camera : *cameras)
	{
		printf("%s started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();
		unsigned char *output = new unsigned char[camera.resx * camera.resy * 3];
		auto ray_thread = [shapes, camera, intersectionepsilon, shadowrayepsilon, ambientlight, point_lights, backgroundcolor, output](int index_start, int index_end)
		{
			simd_vec3 calculatorp;
			for (size_t index = index_start; index < camera.ray_dirs.size() && index < index_end; index++)
			{
				ray_trace(calculatorp, shapes, camera.position, camera.ray_dirs.at(index), intersectionepsilon, shadowrayepsilon,
						  ambientlight, point_lights, backgroundcolor, output, index);
			}
		};
		int index_count_per_thread = camera.ray_dirs.size() / thread_count;
		index_count_per_thread++;
		int index_start = 0;
		if (thread_count <= 0)
		{
			printf("Invalid thread count\n");
			return -1;
		}
		std::thread *ths = new std::thread[thread_count];
		for (int i = 0; i < thread_count; i++)
		{
			ths[i] = std::thread(ray_thread, index_start, index_start + index_count_per_thread);
			index_start += index_count_per_thread;
		}
		for (int i = 0; i < thread_count; i++)
		{
			ths[i].join();
		}
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		printf("%s done: %d ms\n", camera.output.c_str(), duration.count());

		printf("%s write png started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();
		stbi_write_png(camera.output.c_str(), camera.resx, camera.resy, 3, output, camera.resx * 3);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		printf("%s write png done: %d ms\n", camera.output.c_str(), duration.count());

		delete[] output;
		delete[] ths;
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