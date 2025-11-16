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
#include <regex>
#include <map>
#include <algorithm>
#include <iostream>
#include <vector>

namespace fs = std::filesystem;

std::string getBundledFFmpeg()
{
#ifdef _WIN32
	return ".\\win_ffmpeg\\ffmpeg.exe";
#else
	return "./linux_ffmpeg/ffmpeg";
#endif
}

void createVideosFromPngSequences(const std::string &folderPath)
{
	std::regex filePattern(R"(^(.+)_([0-9]+)\.png$)", std::regex::icase);

	std::map<std::string, std::vector<std::pair<int, fs::path>>> sequences;

	try
	{
		std::cout << "Current Working Directory (CWD): " << fs::current_path() << "\n";
	}
	catch (const fs::filesystem_error &e)
	{
		std::cerr << "Warning: Could not determine CWD: " << e.what() << "\n";
	}
	try
	{
		for (const auto &entry : fs::directory_iterator(folderPath))
		{
			if (!entry.is_regular_file())
				continue;

			std::string filename = entry.path().filename().string();
			std::smatch matches;

			if (std::regex_match(filename, matches, filePattern))
			{
				std::string prefix = matches[1].str();
				int frameIndex = std::stoi(matches[2].str());
				sequences[prefix].push_back({frameIndex, entry.path()});
			}
		}
	}
	catch (const fs::filesystem_error &e)
	{
		std::cerr << "Filesystem Error during iteration: " << e.what() << "\n";
		return;
	}

	std::string ffmpegPath = getBundledFFmpeg();

	if (!fs::exists(ffmpegPath))
	{
		try
		{
			std::cerr << "ERROR: FFmpeg executable not found. Looked for it at (based on CWD): " << fs::absolute(ffmpegPath) << "\n";
		}
		catch (...)
		{
			std::cerr << "ERROR: FFmpeg executable not found. Looked for it at: " << ffmpegPath << "\n";
		}
		std::cerr << "Please ensure the executable is in the correct relative path based on your Current Working Directory.\n";
		return;
	}

	for (auto &[prefix, frames] : sequences)
	{
		if (frames.empty())
			continue;

		std::sort(frames.begin(), frames.end(),
				  [](auto &a, auto &b)
				  { return a.first < b.first; });

		std::smatch m;
		std::string filename = frames[0].second.filename().string();

		if (!std::regex_match(filename, m, filePattern))
		{
			std::cerr << "Internal Error: Could not re-match filename to determine padding.\n";
			continue;
		}
		int pad = (int)m[2].str().length();

		std::string pattern = prefix + "_%0" + std::to_string(pad) + "d.png";

		std::string inputPattern = (fs::path(folderPath) / pattern).string();
		std::string outputFile = (fs::path(folderPath) / (prefix + ".mp4")).string();

		std::string cmd =
#ifdef _WIN32
			ffmpegPath + " -y -framerate 30 -i \"" + inputPattern +
			"\" -pix_fmt yuv420p \"" + outputFile + "\"";
#else
			ffmpegPath + " -y -framerate 30 -i \"" + inputPattern +
			"\" -pix_fmt yuv420p \"" + outputFile + "\"";
#endif

		std::cout << "Running: " << cmd << "\n";
		int result = system(cmd.c_str());
		if (result != 0)
		{
			std::cerr << "FFmpeg execution failed with code: " << result << "\n";
		}
		else
		{
			std::cout << "Successfully created: " << outputFile << "\n";
			std::cout << "Erasing " << frames.size() << " source PNG files for sequence '" << prefix << "'...\n";
			for (const auto &frame : frames)
			{
				try
				{
					if (fs::remove(frame.second))
					{
						// std::cout << "Deleted: " << frame.second.filename().string() << "\n";
					}
					else
					{
						std::cerr << "Warning: Could not delete file: " << frame.second.string() << "\n";
					}
				}
				catch (const fs::filesystem_error &e)
				{
					std::cerr << "Filesystem Error during deletion of " << frame.second.filename().string() << ": " << e.what() << "\n";
				}
			}
			std::cout << "Cleanup complete.\n";
		}
	}
}

void process_file(const char *filename, int thread_count)
{
	auto start = std::chrono::high_resolution_clock::now();

	my_printf("JSON parsing started\n");

	parser p(filename);

	simd_vec3 calculator;
	simd_mat4 mat_calc(calculator);

	auto transformations = p.get_transformations(mat_calc);

	auto cameras = p.get_camera(calculator, mat_calc, transformations);

	auto vertices = p.get_vertices();

	auto materials = p.get_materials();

	std::vector<all_mesh_infos *> m;

	auto shapes = p.get_shapes(calculator, mat_calc, vertices, materials, transformations, &m);

	my_printf("Vertex count: %d\n", vertices->size());
	my_printf("Shape count: %d\n", shapes->size());
	my_printf("Camera count: %d\n", cameras->size());

	float intersectionepsilon = p.get_intersectionepsilon();

	float shadowrayepsilon = p.get_shadowrayepsilon();

	float maxdepth = p.get_maxrecursiondepth();

	vec3 backgroundcolor = p.get_backgroundcolor();

	vec3 ambientlight = p.get_ambientlight();

	auto point_lights = p.get_pointlights(mat_calc, transformations);

	stbi_flip_vertically_on_write(1);

	auto rt = new ray_tracer(shapes, intersectionepsilon, shadowrayepsilon, ambientlight,
							 point_lights, backgroundcolor, maxdepth, true);

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	my_printf("JSON parsing and grid preparation is done: %d ms\n", duration.count());

	for (auto &&camera : *cameras)
	{
		my_printf("%s started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();
		unsigned char *output = new unsigned char[camera.resx * camera.resy * 3];
		auto ray_thread = [rt, camera, output](int index_start, int index_end, bool print, int t)
		{
			simd_vec3 calculatorp;
			simd_mat4 calculator_m(calculatorp);
			for (size_t index = index_start; index < camera.ray_dirs.size() && index < index_end; index++)
			{
				rt->trace(calculatorp, calculator_m, camera.position, camera.ray_dirs.at(index), index, false, output);
				if (print && index % 5000 == 0)
				{
					my_printf("Thread 0: %d/%d\n", index, index_end);
				}
			}
			my_printf("Thread %d is done\n", t);
		};
		int index_count_per_thread = camera.ray_dirs.size() / thread_count;
		index_count_per_thread++;
		int index_start = 0;
		std::thread *ths = new std::thread[thread_count];
		for (int i = 0; i < thread_count; i++)
		{
			ths[i] = std::thread(ray_thread, index_start, index_start + index_count_per_thread, i == 0, i);
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
		stbi_write_png(("outputs/" + camera.output).c_str(), camera.resx, camera.resy, 3, output, camera.resx * 3);
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
	std::filesystem::create_directories("outputs");
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
		my_printf("Error: %s is neither a .json file nor a directory.\n", input_path.c_str());
		return 1;
	}

	for (auto &&i : files)
	{
		process_file(i.c_str(), thread_count);
	}

	createVideosFromPngSequences("outputs");

	return 0;
}