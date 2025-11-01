#include "parser.h"
#include "algebra.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third_party/stb/stb_image_write.h"
#include "ray_trace.h"
#include <chrono>
#include <thread>
#include <filesystem>
#include <cstdio>
#include "helper.h"

void process_file(const char *filename, int thread_count)
{
	auto start = std::chrono::high_resolution_clock::now();

	my_printf("JSON parsing started\n");

	parser p(filename);

	simd_vec3 calculator;

	auto cameras = p.get_camera(calculator);

	auto vertices = p.get_vertices();

	auto materials = p.get_materials();

	auto shapes = p.get_shapes(calculator, vertices, materials);

	my_printf("Vertex count: %d\n", vertices->size());
	my_printf("Shape count: %d\n", shapes->size());
	my_printf("Camera count: %d\n", cameras->size());

	float intersectionepsilon = p.get_intersectionepsilon();

	float shadowrayepsilon = p.get_shadowrayepsilon();

	float maxdepth = p.get_maxrecursiondepth();

	vec3 backgroundcolor = p.get_backgroundcolor();

	vec3 ambientlight = p.get_ambientlight();

	auto point_lights = p.get_pointlights();

	stbi_flip_vertically_on_write(1);

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	my_printf("JSON parsing done: %d ms\n", duration.count());
	auto rt = new ray_tracer(shapes, intersectionepsilon, shadowrayepsilon, ambientlight,
							 point_lights, backgroundcolor, maxdepth, true);

	for (auto &&camera : *cameras)
	{
		my_printf("%s started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();
		unsigned char *output = new unsigned char[camera.resx * camera.resy * 3];
		auto ray_thread = [rt, camera, output](int index_start, int index_end, bool print)
		{
			simd_vec3 calculatorp;
			for (size_t index = index_start; index < camera.ray_dirs.size() && index < index_end; index++)
			{
				rt->trace(calculatorp, camera.position, camera.ray_dirs.at(index), index, true, output);
				if (print && index % 5000 == 0)
				{
					my_printf("Thread 0: %d/%d\n", index, index_end);
				}
			}
		};
		int index_count_per_thread = camera.ray_dirs.size() / thread_count;
		index_count_per_thread++;
		int index_start = 0;
		std::thread *ths = new std::thread[thread_count];
		for (int i = 0; i < thread_count; i++)
		{
			ths[i] = std::thread(ray_thread, index_start, index_start + index_count_per_thread, i == 0);
			index_start += index_count_per_thread;
		}
		for (int i = 0; i < thread_count; i++)
		{
			ths[i].join();
		}
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		my_printf("%s done: %d ms\n", camera.output.c_str(), duration.count());

		my_printf("%s write png started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();
		stbi_write_png(camera.output.c_str(), camera.resx, camera.resy, 3, output, camera.resx * 3);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		my_printf("%s write png done: %d ms\n", camera.output.c_str(), duration.count());

		my_printf("\n");

		delete[] output;
		delete[] ths;
	}

	delete rt;
	delete cameras;
	for (auto &&shape : *shapes)
	{
		delete shape;
	}
	delete shapes;
	delete vertices;
	delete materials;
	delete point_lights;
}

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
		my_printf("Just give input json name and thread count:\n./raytracer json_file thread_count\n");
		return 0;
	}
	if (thread_count <= 0)
	{
		my_printf("Invalid thread count\n");
		return -1;
	}

	my_printf("SIMD On / Thread count: %d\n", thread_count);

	std::vector<std::string> files;

	std::string input_path = argv[1];

	if (input_path.ends_with(".json"))
	{
		files.push_back(input_path);
	}
	else if (std::filesystem::is_directory(input_path))
	{
		for (const auto &entry : std::filesystem::recursive_directory_iterator(input_path))
		{
			if (entry.is_regular_file() && entry.path().extension() == ".json")
			{
				files.push_back(entry.path().string());
			}
		}
	}
	else
	{
		my_printf("Error: %s is neither a .json file nor a directory.\n", input_path);
		return 1;
	}

	for (auto &&i : files)
	{
		process_file(i.c_str(), thread_count);
	}
	return 0;
}