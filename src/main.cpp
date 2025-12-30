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

enum threading_type
{
	row_dynamic,
	row_static,
	square_dynamic,
	pixel_dynamic,
	MAXIMUM
};

float get_luminance(float r, float g, float b)
{
	return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

// Photographic Tone Mapping Operator (Reinhard)
float tmo_photographic(float L_scaled, float L_white_sq)
{
	if (L_white_sq > 0.0f)
	{
		// Equation 4: Burn-out
		return (L_scaled * (1.0f + (L_scaled / L_white_sq))) / (1.0f + L_scaled);
	}
	// Equation 3: Simple
	return L_scaled / (1.0f + L_scaled);
}

// ACES approximation (Narkowicz)
float tmo_aces(float x)
{
	float a = 2.51f;
	float b = 0.03f;
	float c = 2.43f;
	float d = 0.59f;
	float e = 0.14f;
	return std::clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0f, 1.0f);
}

// Filmic approximation (Uncharted 2)
float tmo_filmic_curve(float x)
{
	float A = 0.15f;
	float B = 0.50f;
	float C = 0.10f;
	float D = 0.20f;
	float E = 0.02f;
	float F = 0.30f;
	return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
}

float tmo_filmic(float x)
{
	float exposure_bias = 2.0f;
	float curr = tmo_filmic_curve(x * exposure_bias);
	float white_scale = 1.0f / tmo_filmic_curve(11.2f);
	return curr * white_scale;
}

void apply_tonemap(const Tonemap &tm, int width, int height, const std::vector<float> &hdr_data, unsigned char *output_png)
{
	int num_pixels = width * height;
	float key = tm.TMOOptions1;			 // Key value
	float burn_percent = tm.TMOOptions2; // Burn-out percentage
	float gamma = tm.Gamma;
	float saturation = tm.Saturation;

	// Photographic TMO Pre-computation
	float log_avg_luminance = 1.0f;
	float L_white_sq = 0.0f;

	if (tm.type == Photographic)
	{
		float sum_log = 0.0f;
		float delta = 0.000001f;
		std::vector<float> luminances;

		// Collect luminances for log-avg and potential sorting
		if (burn_percent > 0.0f)
		{
			luminances.reserve(num_pixels);
		}

		for (int i = 0; i < num_pixels; ++i)
		{
			float lum = get_luminance(hdr_data[i * 3], hdr_data[i * 3 + 1], hdr_data[i * 3 + 2]);
			sum_log += std::log(delta + lum);
			if (burn_percent > 0.0f)
			{
				luminances.push_back(lum);
			}
		}

		log_avg_luminance = std::exp(sum_log / num_pixels);

		// Handle Burn-out logic if parameter is non-zero
		if (burn_percent > 0.0f)
		{
			// First scale luminances by key/L_avg
			float scale_factor = key / log_avg_luminance;
			for (auto &l : luminances)
			{
				l *= scale_factor;
			}

			// Sort to find percentile
			std::sort(luminances.begin(), luminances.end());

			// Example: if value is 1, select 99th percentile.
			float percentile = 100.0f - burn_percent;
			int idx = (int)((percentile / 100.0f) * (luminances.size() - 1));
			idx = std::clamp(idx, 0, (int)luminances.size() - 1);

			float L_white = luminances[idx];
			L_white_sq = L_white * L_white;
		}
	}

	// Apply Tone Mapping per pixel
	for (int i = 0; i < num_pixels; ++i)
	{
		float r = hdr_data[i * 3];
		float g = hdr_data[i * 3 + 1];
		float b = hdr_data[i * 3 + 2];

		// 1. Compute Luminance Yi
		float Yi = get_luminance(r, g, b);
		float Yo = 0.0f;

		// 2. Apply Tone Mapping Algorithm
		if (tm.type == Photographic)
		{
			float L_scaled = (key / log_avg_luminance) * Yi;
			Yo = tmo_photographic(L_scaled, L_white_sq);
		}
		else if (tm.type == Filmic)
		{
			Yo = tmo_filmic(Yi);
		}
		else if (tm.type == ACES)
		{
			Yo = tmo_aces(Yi);
		}

		// 3. Reconstruct Color (apply saturation)
		// Ro = Yo * (R / Yi)^s
		float ratio_r = (Yi > 1e-6f) ? (r / Yi) : 0.0f;
		float ratio_g = (Yi > 1e-6f) ? (g / Yi) : 0.0f;
		float ratio_b = (Yi > 1e-6f) ? (b / Yi) : 0.0f;

		float Ro = Yo * std::pow(ratio_r, saturation);
		float Go = Yo * std::pow(ratio_g, saturation);
		float Bo = Yo * std::pow(ratio_b, saturation);

		// 4. Gamma Correction
		Ro = std::clamp(Ro, 0.0f, 1.0f);
		Go = std::clamp(Go, 0.0f, 1.0f);
		Bo = std::clamp(Bo, 0.0f, 1.0f);

		float inv_gamma = 1.0f / gamma;
		unsigned char Rf = (unsigned char)(std::pow(Ro, inv_gamma) * 255.0f);
		unsigned char Gf = (unsigned char)(std::pow(Go, inv_gamma) * 255.0f);
		unsigned char Bf = (unsigned char)(std::pow(Bo, inv_gamma) * 255.0f);

		output_png[i * 3 + 0] = Rf;
		output_png[i * 3 + 1] = Gf;
		output_png[i * 3 + 2] = Bf;
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

		// Use a float vector to store high-precision HDR radiance
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
					// Using the new trace signature that accepts float*
					rt->trace(calculatorp, calculator_m, sample.position, sample.direction, sample.time, true, camera.is_hdr, bg, pixelu, pixelv, temp_color);

					r_acc += temp_color[0];
					g_acc += temp_color[1];
					b_acc += temp_color[2];
				}

				float num_samples = static_cast<float>(samples.size());
				r_acc /= num_samples;
				g_acc /= num_samples;
				b_acc /= num_samples;

				// Store raw radiance in the float buffer
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

		// --- Post-Processing: Save Files ---

		// 1. Save EXR if requested
		if (camera.is_hdr || camera.output.ends_with(".exr"))
		{
			/*todoooo
			my_printf("%s write EXR started\n", camera.output.c_str());
			const char *err = nullptr;
			// Save as EXR (3 channels, float precision)
			int ret = SaveEXR(hdr_output.data(), camera.resx, camera.resy, 3, 0, ("outputs/" + camera.output).c_str(), &err);
			if (ret != TINYEXR_SUCCESS)
			{
				if (err)
				{
					fprintf(stderr, "Save EXR Error: %s\n", err);
					FreeEXRErrorMessage(err);
				}
			}
			*/
		}
		else
		{
			// If no tonemap and not EXR, save as LDR PNG (clamp manually)
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

		// 2. Process and Save Tone Mapped images
		if (!camera.Tonemaps.empty())
		{
			std::vector<unsigned char> tm_output(total_pixels * 3);

			for (const auto &tm : camera.Tonemaps)
			{
				apply_tonemap(tm, camera.resx, camera.resy, hdr_output, tm_output.data());

				// Generate filename suffix: "image.exr" + "_phot.png" -> "image_phot.png"
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
	delete point_lights;
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