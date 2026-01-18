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
#include "../third_party/tinyexr/tinyexr.h"

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

enum threading_type
{
	row_dynamic,
	row_static,
	square_dynamic,
	pixel_dynamic,
	MAXIMUM
};

static float clamp01(float v)
{
	return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f
										  : v;
}

static float get_luminance(float r, float g, float b)
{
	return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

static float compute_log_average_luminance(const std::vector<float> &luminances)
{
	if (luminances.empty())
		return 1.0f;

	const float delta = 1e-4f;
	double sumLog = 0.0;

	for (float lum : luminances)
	{
		float val = (lum < 0.0f) ? 0.0f : lum;
		sumLog += std::log(delta + val);
	}

	return (float)std::exp(sumLog / (double)luminances.size());
}

static float compute_white_point_exposed(const std::vector<float> &Yin, float exposure, float burnPct)
{
	if (Yin.empty())
		return 1e9f;
	if (burnPct <= 1e-6f)
		return 1e9f;

	std::vector<float> L = Yin;
	for (float &v : L)
	{
		v = std::max(0.0f, v) * exposure;
	}

	float percentile = (100.0f - burnPct) / 100.0f;
	int k = (int)std::floor(percentile * (L.size() - 1));
	k = std::max(0, std::min((int)L.size() - 1, k));

	std::nth_element(L.begin(), L.begin() + k, L.end());
	return std::max(L[k], 1e-6f);
}

static void reconstruct_color(float rIn, float gIn, float bIn, float Yin, float Yout, float saturation, float &rOut, float &gOut, float &bOut)
{
	if (Yin < 1e-6f)
	{
		rOut = 0.0f;
		gOut = 0.0f;
		bOut = 0.0f;
		return;
	}

	float ratioR = rIn / Yin;
	float ratioG = gIn / Yin;
	float ratioB = bIn / Yin;

	if (std::abs(saturation - 1.0f) > 1e-4f)
	{
		ratioR = std::pow(std::max(0.0f, ratioR), saturation);
		ratioG = std::pow(std::max(0.0f, ratioG), saturation);
		ratioB = std::pow(std::max(0.0f, ratioB), saturation);
	}

	rOut = Yout * ratioR;
	gOut = Yout * ratioG;
	bOut = Yout * ratioB;
}

static float aces_map(float x)
{
	const float A = 2.51f;
	const float B = 0.03f;
	const float C = 2.43f;
	const float D = 0.59f;
	const float E = 0.14f;
	return (x * (A * x + B)) / (x * (C * x + D) + E);
}

static float filmic_map(float x)
{
	const float A = 0.22f;
	const float B = 0.30f;
	const float C = 0.10f;
	const float D = 0.20f;
	const float E = 0.01f;
	const float F = 0.30f;
	float num = x * (A * x + C * B) + D * E;
	float den = x * (A * x + B) + D * F;
	return (num / den) - (E / F);
}

void apply_tonemap(
	const Tonemap &tm,
	int width,
	int height,
	const std::vector<float> &hdr_data,
	unsigned char *output_png)
{
	int num_pixels = width * height;
	std::vector<float> Yin(num_pixels);

	for (int i = 0; i < num_pixels; ++i)
	{
		float r = hdr_data[i * 3 + 0];
		float g = hdr_data[i * 3 + 1];
		float b = hdr_data[i * 3 + 2];
		Yin[i] = get_luminance(r, g, b);
	}

	float Yavg = compute_log_average_luminance(Yin);
	float key = tm.TMOOptions1;
	float exposure = (Yavg > 1e-6f) ? (key / Yavg) : 1.0f;

	float L_white = compute_white_point_exposed(Yin, exposure, tm.TMOOptions2);
	float L_white_sq = L_white * L_white;

	float saturation = tm.Saturation + 0.2f;
	float gamma = (tm.Gamma > 1e-6f) ? tm.Gamma : 2.2f;
	float inv_gamma = 1.0f / gamma;

	float denom_aces = 1.0f;
	float denom_filmic = 1.0f;

	if (tm.type == ACES)
	{
		denom_aces = std::max(aces_map(L_white), 1e-6f);
	}
	else if (tm.type == Filmic)
	{
		denom_filmic = std::max(filmic_map(L_white), 1e-6f);
	}

	for (int i = 0; i < num_pixels; ++i)
	{
		int idx = i * 3;
		float r_in = hdr_data[idx + 0];
		float g_in = hdr_data[idx + 1];
		float b_in = hdr_data[idx + 2];
		float Y_i = Yin[i];

		float L = std::max(0.0f, Y_i) * exposure;
		float Y_out = 0.0f;

		if (tm.type == Photographic)
		{
			Y_out = (L * (1.0f + (L / L_white_sq))) / (1.0f + L);
		}
		else if (tm.type == ACES)
		{
			Y_out = aces_map(L) / denom_aces;
		}
		else if (tm.type == Filmic)
		{
			Y_out = filmic_map(L) / denom_filmic;
		}

		Y_out = clamp01(Y_out);

		float r_final, g_final, b_final;
		reconstruct_color(r_in, g_in, b_in, Y_i, Y_out, saturation, r_final, g_final, b_final);

		r_final = std::pow(clamp01(r_final), inv_gamma);
		g_final = std::pow(clamp01(g_final), inv_gamma);
		b_final = std::pow(clamp01(b_final), inv_gamma);

		output_png[idx + 0] = (unsigned char)std::lround(255.0f * r_final);
		output_png[idx + 1] = (unsigned char)std::lround(255.0f * g_final);
		output_png[idx + 2] = (unsigned char)std::lround(255.0f * b_final);
	}
}

void process_file(const char *filename, int thread_count, threading_type TT)
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
	auto lights = p.get_lights(calculator, mat_calc, transformations, images);
	std::vector<all_mesh_infos *> m;
	auto shapes = p.get_shapes(calculator, mat_calc, vertices, materials, transformations, textures, uvs, &m, lights);
	my_printf("shapes parsed\n");

	float intersectionepsilon = p.get_intersectionepsilon();
	float shadowrayepsilon = p.get_shadowrayepsilon();
	float maxdepth = p.get_maxrecursiondepth();
	vec3 backgroundcolor = p.get_backgroundcolor();
	vec3 ambientlight = p.get_ambientlight();

	bool is_probe = false;
	for (Light *light : *lights)
	{
		SphericalDirectionalLight *env_light = dynamic_cast<SphericalDirectionalLight *>(light);

		if (env_light)
		{
			if (env_light->env_map)
			{
				bg = new texture(const_cast<image *>(env_light->env_map), -1, replace_background, bilinear);
				is_probe = env_light->is_probe_map;
			}
			break;
		}
	}

	stbi_flip_vertically_on_write(1);

	auto rt = new ray_tracer(shapes, intersectionepsilon, shadowrayepsilon, ambientlight,
							 lights, backgroundcolor, maxdepth);

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	my_printf("JSON parsing and grid preparation is done: %d ms\n", duration.count());

	for (auto &&camera : *cameras)
	{
		my_printf("%s started\n", camera.output.c_str());
		start = std::chrono::high_resolution_clock::now();

		int total_pixels = camera.resx * camera.resy;

		std::vector<float> hdr_output(total_pixels * 3, 0.0f);

		std::atomic<int> next_scanline(0);
		std::atomic<int> next_tile(0);
		std::atomic<int> next_pixel(0);

		int tile_size = 32;
		int tiles_x = (camera.resx + tile_size - 1) / tile_size;
		int tiles_y = (camera.resy + tile_size - 1) / tile_size;
		int total_tiles = tiles_x * tiles_y;

		auto ray_thread = [&](int t) mutable
		{
			simd_vec3 calculatorp;
			simd_mat4 calculator_m(calculatorp);

			auto process_pixel = [&](int i, int j)
			{
				int index = j * camera.resx + i;

				float pixelv = 1.0f - ((float)j / (float)(camera.resy - 1));
				float pixelu = (float)i / (float)(camera.resx - 1);

				std::vector<camera::sample> samples = camera.get_samples(calculatorp, i, j);

				float r_acc = 0.0f;
				float g_acc = 0.0f;
				float b_acc = 0.0f;

				for (const auto &sample : samples)
				{
					float temp_color[3];
					rt->trace(calculatorp, calculator_m, sample.position, sample.direction, sample.time, false,
							  camera.is_hdr, bg, is_probe, pixelu, pixelv, temp_color, camera.renderSettings);

					r_acc += temp_color[0];
					g_acc += temp_color[1];
					b_acc += temp_color[2];
				}

				float num_samples = static_cast<float>(samples.size());
				r_acc /= num_samples;
				g_acc /= num_samples;
				b_acc /= num_samples;

				int pixel_index = index * 3;
				hdr_output[pixel_index + 0] = r_acc;
				hdr_output[pixel_index + 1] = g_acc;
				hdr_output[pixel_index + 2] = b_acc;
			};

			switch (TT)
			{
			case row_dynamic:
				while (true)
				{
					int j = next_scanline++;
					if (j >= camera.resy)
						break;
					for (int i = 0; i < camera.resx; ++i)
						process_pixel(i, j);
				}
				break;
			case row_static:
			{
				int rows_per_thread = camera.resy / thread_count;
				int start_y = t * rows_per_thread;
				int end_y = (t == thread_count - 1) ? camera.resy : start_y + rows_per_thread;
				for (int j = start_y; j < end_y; ++j)
					for (int i = 0; i < camera.resx; ++i)
						process_pixel(i, j);
				break;
			}
			case square_dynamic:
				while (true)
				{
					int tile_idx = next_tile++;
					if (tile_idx >= total_tiles)
						break;
					int ty = tile_idx / tiles_x;
					int tx = tile_idx % tiles_x;
					int start_y = ty * tile_size;
					int end_y = std::min(start_y + tile_size, camera.resy);
					int start_x = tx * tile_size;
					int end_x = std::min(start_x + tile_size, camera.resx);

					for (int j = start_y; j < end_y; ++j)
						for (int i = start_x; i < end_x; ++i)
							process_pixel(i, j);
				}
				break;
			case pixel_dynamic:
				while (true)
				{
					int k = next_pixel++;
					if (k >= total_pixels)
						break;
					int j = k / camera.resx;
					int i = k % camera.resx;
					process_pixel(i, j);
				}
				break;
			default:
				break;
			}
		};

		if (thread_count > 4096 or thread_count < 1)
		{
			exit(-2);
		}
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
		my_printf("%s rendering done: %d ms\n", camera.output.c_str(), duration.count());

		if (camera.is_hdr || camera.output.ends_with(".exr"))
		{
			my_printf("%s write EXR started\n", camera.output.c_str());
			const char *err = nullptr;
			int ret = SaveEXR(hdr_output.data(), camera.resx, camera.resy, 3, 0, ("outputs/" + camera.output).c_str(), &err);
			if (ret != TINYEXR_SUCCESS)
			{
				if (err)
				{
					fprintf(stderr, "Save EXR Error: %s\n", err);
					FreeEXRErrorMessage(err);
				}
			}
		}
		else
		{
			if (camera.Tonemaps.empty())
			{
				std::vector<unsigned char> ldr_out(total_pixels * 3);
				for (int i = 0; i < total_pixels * 3; ++i)
				{
					ldr_out[i] = (unsigned char)(std::clamp(hdr_output[i] * 255.0f, 0.0f, 255.0f));
				}
				stbi_write_png(("outputs/" + camera.output).c_str(), camera.resx, camera.resy, 3, ldr_out.data(), camera.resx * 3);
			}
		}

		if (!camera.Tonemaps.empty())
		{
			std::vector<unsigned char> tm_output(total_pixels * 3);

			for (const auto &tm : camera.Tonemaps)
			{
				apply_tonemap(tm, camera.resx, camera.resy, hdr_output, tm_output.data());

				std::string base = camera.output;
				size_t dot_pos = base.find_last_of('.');
				if (dot_pos != std::string::npos)
				{
					base = base.substr(0, dot_pos);
				}
				std::string tm_filename = base + tm.Extension;

				my_printf("Saving tone mapped image: %s\n", tm_filename.c_str());
				stbi_write_png(("outputs/" + tm_filename).c_str(), camera.resx, camera.resy, 3, tm_output.data(), camera.resx * 3);
			}
		}

		delete[] ths;
	}

	delete uvs;

	for (auto &&i : *images)
		delete i;
	delete images;

	for (auto &&i : *textures)
		delete i;
	delete textures;

	for (auto &&i : m)
		delete i;

	delete rt;
	delete cameras;
	for (auto &&shape : *shapes)
		delete shape;
	delete shapes;
	delete vertices;
	delete materials;
	delete lights;
}

int main(int argc, char **argv)
{
	my_printf("\n>>> NEW RUN <<<\n");

	std::filesystem::create_directories("outputs");
	int thread_count = 1;
	threading_type TT = row_dynamic;
	if (argc == 2)
	{
		thread_count = std::thread::hardware_concurrency();
		TT = row_dynamic;
	}
	else if (argc == 3)
	{
		thread_count = std::thread::hardware_concurrency();
		int tmp = atoi(argv[2]);
		if (tmp >= threading_type::MAXIMUM || tmp < 0)
		{
			my_printf("Invalid threading type\n");
			return -3;
		}
		TT = (threading_type)tmp;
	}
	else if (argc == 4)
	{
		thread_count = atoi(argv[3]);
		int tmp = atoi(argv[2]);
		if (tmp >= threading_type::MAXIMUM || tmp < 0)
		{
			my_printf("Invalid threading type\n");
			return -3;
		}
		TT = (threading_type)tmp;
	}
	else
	{
		my_printf("Just give input json name, threading type and thread count:\n./raytracer json_file threading_type thread_count\n");
		return -2;
	}
	if (thread_count <= 0)
	{
		my_printf("Invalid thread count\n");
		return -1;
	}

	const char *threading_types[] = {"row_dynamic", "row_static", "square_dynamic", "pixel_dynamic"};

	my_printf("SIMD On / Thread count: %d / Threading type: %s\n", thread_count, threading_types[(int)TT]);

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
		process_file(i.c_str(), thread_count, TT);
	}

	createVideosFromPngSequences("outputs");

	return 0;
}