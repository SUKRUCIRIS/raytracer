#include "parser.h"
#include <algorithm>
#include <sstream>
#include <fstream>
#include <array>

static float read_float(std::ifstream &f)
{
	float v;
	f.read(reinterpret_cast<char *>(&v), sizeof(float));
	return v;
}

static uint32_t read_uint32(std::ifstream &f)
{
	uint32_t v;
	f.read(reinterpret_cast<char *>(&v), sizeof(uint32_t));
	return v;
}

static uint8_t read_uint8(std::ifstream &f)
{
	uint8_t v;
	f.read(reinterpret_cast<char *>(&v), sizeof(uint8_t));
	return v;
}

static int32_t read_int32(std::ifstream &f)
{
	int32_t v;
	f.read(reinterpret_cast<char *>(&v), sizeof(int32_t));
	return v;
}

static uint16_t read_uint16(std::ifstream &f)
{
	uint16_t v;
	f.read(reinterpret_cast<char *>(&v), sizeof(uint16_t));
	return v;
}

static int16_t read_int16(std::ifstream &f)
{
	int16_t v;
	f.read(reinterpret_cast<char *>(&v), sizeof(int16_t));
	return v;
}

static int8_t read_int8(std::ifstream &f)
{
	int8_t v;
	f.read(reinterpret_cast<char *>(&v), sizeof(int8_t));
	return v;
}

static double read_double(std::ifstream &f)
{
	double v;
	f.read(reinterpret_cast<char *>(&v), sizeof(double));
	return v;
}

void parser::load_ply(simd_vec3 &calculator,
					  const std::string &filename,
					  std::vector<vec3> *vertices,
					  std::vector<shape *> *shapes,
					  material *mat, all_mesh_infos *ami,
					  bool smooth_shading)
{
	std::ifstream file(filename, std::ios::binary);
	if (!file.is_open())
	{
		my_printf("Error: could not open PLY file %s\n", filename.c_str());
		exit(-1);
	}

	std::string line;
	size_t vertexCount = 0, faceCount = 0;
	std::string vertexIndexType = "int";
	std::string vertexListCountType = "uchar";
	std::vector<std::pair<std::string, std::string>> vertexProperties;
	std::string format = "ascii";
	bool has_normals = false;
	bool has_uvs = false;
	std::string current_element = "";

	while (std::getline(file, line))
	{
		auto trim = [](std::string &s)
		{
			size_t a = s.find_first_not_of(" \t\r\n");
			size_t b = s.find_last_not_of(" \t\r\n");
			if (a == std::string::npos)
			{
				s = "";
				return;
			}
			s = s.substr(a, b - a + 1);
		};
		trim(line);
		if (line.empty())
			continue;

		std::istringstream iss(line);
		std::string token;
		if (!(iss >> token))
			continue;

		if (token == "format")
		{
			iss >> format;
		}
		else if (token == "element")
		{
			iss >> current_element;
			if (current_element == "vertex")
			{
				iss >> vertexCount;
			}
			else if (current_element == "face")
			{
				iss >> faceCount;
			}
		}
		else if (token == "property")
		{
			if (current_element == "vertex")
			{
				std::string type, name;
				iss >> type >> name;

				if (type == "list")
				{
					std::string index_type;
					iss >> index_type >> name;
					vertexProperties.emplace_back("list", name);
				}
				else
				{
					vertexProperties.emplace_back(type, name);
				}

				if (name == "nx" || name == "ny" || name == "nz")
					has_normals = true;
				if (name == "u" || name == "v" || name == "s" || name == "t")
					has_uvs = true;
			}
			else if (current_element == "face")
			{
				std::string next;
				iss >> next;
				if (next == "list")
				{
					std::string c_type, i_type, name;
					iss >> c_type >> i_type >> name;
					if (name == "vertex_indices" || name == "vertex_index")
					{
						vertexListCountType = c_type;
						vertexIndexType = i_type;
					}
				}
			}
		}
		else if (token == "end_header")
		{
			break;
		}
	}

	size_t startIndex = vertices->size();
	std::vector<vec3> loaded_normals;
	std::vector<std::pair<float, float>> loaded_uvs;

	if (has_normals || smooth_shading)
	{
		loaded_normals.reserve(vertexCount);
	}
	if (has_uvs)
	{
		loaded_uvs.reserve(vertexCount);
	}

	bool is_ascii = (format.find("ascii") != std::string::npos);

	for (size_t vi = 0; vi < vertexCount; ++vi)
	{
		float vx = 0.0f, vy = 0.0f, vz = 0.0f;
		float vnx = 0.0f, vny = 0.0f, vnz = 0.0f;
		float vu = 0.0f, vv = 0.0f;
		bool gotX = false, gotY = false, gotZ = false;

		auto assign_prop = [&](const std::string &name, float val)
		{
			if (name == "x")
			{
				vx = val;
				gotX = true;
			}
			else if (name == "y")
			{
				vy = val;
				gotY = true;
			}
			else if (name == "z")
			{
				vz = val;
				gotZ = true;
			}
			else if (name == "nx")
				vnx = val;
			else if (name == "ny")
				vny = val;
			else if (name == "nz")
				vnz = val;
			else if (name == "u" || name == "s")
				vu = val;
			else if (name == "v" || name == "t")
				vv = val;
		};

		if (is_ascii)
		{
			std::string vline;
			if (!std::getline(file, vline))
			{
				my_printf("Unexpected EOF\n");
				exit(-1);
			}
			std::istringstream viss(vline);
			for (auto &prop : vertexProperties)
			{
				std::string token_val;
				if (!(viss >> token_val))
					break;
				if (prop.first == "list")
					continue;
				assign_prop(prop.second, std::stof(token_val));
			}
		}
		else
		{
			for (auto &prop : vertexProperties)
			{
				std::string type = prop.first;
				std::string name = prop.second;
				std::transform(type.begin(), type.end(), type.begin(), ::tolower);

				float val = 0.0f;
				bool read_success = true;

				if (type == "float" || type == "float32")
					val = read_float(file);
				else if (type == "double" || type == "float64")
					val = static_cast<float>(read_double(file));
				else if (type == "uchar" || type == "uint8")
					val = static_cast<float>(read_uint8(file));
				else if (type == "char" || type == "int8")
					val = static_cast<float>(read_int8(file));
				else if (type == "ushort" || type == "uint16")
					val = static_cast<float>(read_uint16(file));
				else if (type == "short" || type == "int16")
					val = static_cast<float>(read_int16(file));
				else if (type == "int" || type == "int32")
					val = static_cast<float>(read_int32(file));
				else if (type == "uint" || type == "uint32")
					val = static_cast<float>(read_uint32(file));
				else if (type == "list")
				{
					uint8_t count = read_uint8(file);
					for (int k = 0; k < count; ++k)
						read_uint8(file);
					read_success = false;
				}
				else
				{
					file.seekg(4, std::ios::cur);
					read_success = false;
				}

				if (read_success)
					assign_prop(name, val);
			}
		}

		vertices->emplace_back(vx, vy, vz);
		if (has_normals || smooth_shading)
		{
			loaded_normals.emplace_back(vnx, vny, vnz);
		}
		if (has_uvs)
		{
			loaded_uvs.emplace_back(vu, vv);
		}
	}

	auto read_index_value = [&](std::ifstream &f) -> int32_t
	{
		std::string t = vertexIndexType;
		if (t == "int" || t == "int32" || t == "int32_t")
			return read_int32(f);
		if (t == "uint" || t == "uint32" || t == "uint32_t")
			return static_cast<int32_t>(read_uint32(f));
		if (t == "ushort" || t == "uint16")
			return static_cast<int32_t>(read_uint16(f));
		if (t == "short" || t == "int16")
			return read_int16(f);
		if (t == "uchar" || t == "uint8")
			return static_cast<int32_t>(read_uint8(f));
		if (t == "char" || t == "int8")
			return read_int8(f);
		exit(-1);
	};

	auto read_list_count = [&](std::ifstream &f) -> uint32_t
	{
		std::string t = vertexListCountType;
		if (t == "uchar" || t == "uint8")
			return read_uint8(f);
		if (t == "ushort" || t == "uint16")
			return read_uint16(f);
		if (t == "uint" || t == "uint32")
			return read_uint32(f);
		exit(-1);
	};

	std::vector<std::array<int32_t, 3>> pending_tris;

	for (size_t fi = 0; fi < faceCount; ++fi)
	{
		uint32_t n = 0;
		std::vector<int32_t> face_indices;

		if (is_ascii)
		{
			std::string fline;
			if (!std::getline(file, fline))
				break;
			std::istringstream fiss(fline);
			fiss >> n;
			face_indices.reserve(n);
			int32_t idx;
			for (uint32_t j = 0; j < n; ++j)
			{
				if (fiss >> idx)
					face_indices.push_back(idx);
			}
		}
		else
		{
			n = read_list_count(file);
			face_indices.reserve(n);
			for (uint32_t j = 0; j < n; ++j)
			{
				face_indices.push_back(read_index_value(file));
			}
		}

		if (n < 3)
			continue;

		for (size_t i = 1; i < n - 1; ++i)
		{
			pending_tris.push_back(std::array<int32_t, 3>{face_indices[0], face_indices[i], face_indices[i + 1]});
		}
	}

	if (smooth_shading && !has_normals)
	{
		loaded_normals.assign(vertexCount, vec3(0.0f, 0.0f, 0.0f));

		for (auto &tri : pending_tris)
		{
			vec3 &v0 = vertices->at(startIndex + tri[0]);
			vec3 &v1 = vertices->at(startIndex + tri[1]);
			vec3 &v2 = vertices->at(startIndex + tri[2]);

			vec3 e1, e2, n;
			calculator.subs(v1, v0, e1);
			calculator.subs(v2, v0, e2);
			calculator.cross(e1, e2, n);

			calculator.add(loaded_normals[tri[0]], n, loaded_normals[tri[0]]);
			calculator.add(loaded_normals[tri[1]], n, loaded_normals[tri[1]]);
			calculator.add(loaded_normals[tri[2]], n, loaded_normals[tri[2]]);
		}

		for (auto &vn : loaded_normals)
		{
			calculator.normalize(vn, vn);
		}

		has_normals = true;
	}

	for (auto &tri : pending_tris)
	{
		vec3 u_coords(0, 0, 0);
		vec3 v_coords(0, 0, 0);

		if (has_uvs)
		{
			u_coords = vec3(loaded_uvs[tri[0]].first, loaded_uvs[tri[1]].first, loaded_uvs[tri[2]].first);
			v_coords = vec3(loaded_uvs[tri[0]].second, loaded_uvs[tri[1]].second, loaded_uvs[tri[2]].second);
		}

		if (has_normals)
		{
			shapes->push_back(new triangle(
				calculator,
				&vertices->at(startIndex + tri[0]),
				&vertices->at(startIndex + tri[1]),
				&vertices->at(startIndex + tri[2]),
				u_coords, v_coords,
				loaded_normals[tri[0]],
				loaded_normals[tri[1]],
				loaded_normals[tri[2]],
				mat, ami));
		}
		else
		{
			shapes->push_back(new triangle(
				calculator,
				&vertices->at(startIndex + tri[0]),
				&vertices->at(startIndex + tri[1]),
				&vertices->at(startIndex + tri[2]),
				u_coords, v_coords,
				mat, ami));
		}
	}

	my_printf("Loaded PLY: %zu vertices, %zu tris. Smooth Shading: %s, UVs: %s\n",
			  vertexCount, pending_tris.size(), has_normals ? "ON" : "OFF", has_uvs ? "ON" : "OFF");
}