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
	std::regex filePattern;
	try
	{
		filePattern = std::regex(R"(^(.+)_(\d+)\.png$)", std::regex::ECMAScript | std::regex::icase);
	}
	catch (const std::regex_error &e)
	{
		std::cerr << "Regex construction failed: " << e.what() << " (Code: " << e.code() << ")" << std::endl;
		return;
	}

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
			ffmpegPath + " -y -framerate 30 -i \"" + inputPattern +
			"\" -c:v libopenh264 -b:v 8M -maxrate 8M -bufsize 16M -preset slow "
			"-profile:v high -pix_fmt yuv420p \"" +
			outputFile + "\"";

		std::cout << "Running: " << cmd << "\n";
		int result = system(cmd.c_str());
		if (result != 0)
		{
			std::cerr << "FFmpeg execution failed with code: " << result << "\n";
		}
		else
		{
			std::cout << "Successfully created: " << outputFile << "\n";

			fs::path usedDir = fs::path(folderPath) / "used";

			try
			{
				if (!fs::exists(usedDir))
				{
					fs::create_directory(usedDir);
				}

				std::cout << "Moving " << frames.size() << " source PNG files to '" << usedDir.string() << "'...\n";

				for (const auto &frame : frames)
				{
					fs::path sourcePath = frame.second;
					fs::path destPath = usedDir / sourcePath.filename();

					fs::rename(sourcePath, destPath);
				}
				std::cout << "Move complete.\n";
			}
			catch (const fs::filesystem_error &e)
			{
				std::cerr << "Filesystem Error during file cleanup: " << e.what() << "\n";
			}
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

	my_printf("transformations parsed\n");

	auto cameras = p.get_camera(calculator, mat_calc, transformations);

	my_printf("cameras parsed\n");

	auto vertices = p.get_vertices();

	my_printf("vertices parsed\n");

	auto materials = p.get_materials();

	my_printf("materials parsed\n");

	auto images = p.get_images();

	my_printf("images parsed\n");

	auto textures = p.get_textures(images);

	my_printf("textures parsed\n");

	texture *bg = 0;

	for (auto &&i : *textures)
	{
		if (i->dmode == replace_background)
		{
			bg = i;
			break;
		}
	}

	auto uvs = p.get_uvs();

	my_printf("uvs parsed\n");

	std::vector<all_mesh_infos *> m;

	auto shapes = p.get_shapes(calculator, mat_calc, vertices, materials, transformations, textures, uvs, &m);

	my_printf("shapes parsed\n");

	float intersectionepsilon = p.get_intersectionepsilon();

	float shadowrayepsilon = p.get_shadowrayepsilon();

	float maxdepth = p.get_maxrecursiondepth();

	vec3 backgroundcolor = p.get_backgroundcolor();

	vec3 ambientlight = p.get_ambientlight();

	auto point_lights = p.get_pointlights(calculator, mat_calc, transformations);

	stbi_flip_vertically_on_write(1);

	auto rt = new ray_tracer(shapes, intersectionepsilon, shadowrayepsilon, ambientlight,
							 point_lights, backgroundcolor, maxdepth);

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	my_printf("JSON parsing and grid preparation is done: %d ms\n", duration.count());

	for (auto &&camera : *cameras)
	{
		my_printf("%s started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();

		int total_pixels = camera.resx * camera.resy;
		unsigned char *output = new unsigned char[total_pixels * 3];

		std::atomic<int> next_scanline(0);

		auto ray_thread = [rt, &camera, output, total_pixels, &next_scanline, bg](int t) mutable
		{
			simd_vec3 calculatorp;
			simd_mat4 calculator_m(calculatorp);

			while (true)
			{
				int j = next_scanline++;
				if (j % 100 == 0)
				{
					my_printf("Row %d\n", j);
				}

				if (j >= camera.resy)
					break;

				float pixelv = 1.0f - ((float)j / (float)(camera.resy - 1));
				for (int i = 0; i < camera.resx; ++i)
				{
					int index = j * camera.resx + i;

					std::vector<camera::sample> samples = camera.get_samples(calculatorp, i, j);

					float r_acc = 0.0f;
					float g_acc = 0.0f;
					float b_acc = 0.0f;

					float pixelu = (float)i / (float)(camera.resx - 1);

					for (const auto &sample : samples)
					{
						unsigned char temp_color[3];

						rt->trace(calculatorp, calculator_m, sample.position, sample.direction, 0, sample.time, true, bg, pixelu, pixelv, temp_color);

						r_acc += temp_color[0];
						g_acc += temp_color[1];
						b_acc += temp_color[2];
					}

					float num_samples = static_cast<float>(samples.size());
					r_acc /= num_samples;
					g_acc /= num_samples;
					b_acc /= num_samples;

					int pixel_index = index * 3;
					output[pixel_index + 0] = (unsigned char)(r_acc > 255 ? 255 : r_acc);
					output[pixel_index + 1] = (unsigned char)(g_acc > 255 ? 255 : g_acc);
					output[pixel_index + 2] = (unsigned char)(b_acc > 255 ? 255 : b_acc);
				}
			}
		};

		std::thread *ths = new std::thread[thread_count];
		for (int i = 0; i < thread_count; i++)
		{
			ths[i] = std::thread(ray_thread, i);
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

	delete uvs;

	for (auto &&i : *images)
	{
		delete i;
	}
	delete images;

	for (auto &&i : *textures)
	{
		delete i;
	}
	delete textures;

	for (auto &&i : m)
	{
		delete i;
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
	my_printf("\n>>> NEW RUN <<<\n");

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