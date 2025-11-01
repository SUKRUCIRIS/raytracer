#include "parser.h"
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include "helper.h"
#include <algorithm>

char *parser::get_json_content(const char *fileName)
{
	FILE *fp = fopen(fileName, "rb");
	if (!fp)
		return NULL;

	fseek(fp, 0, SEEK_END);
	long size_l = ftell(fp);
	rewind(fp);

	if (size_l < 0)
	{
		fclose(fp);
		return NULL;
	}
	size_t size = (size_t)size_l;

	char *file = (char *)malloc(size + 1);
	if (!file)
	{
		fclose(fp);
		return NULL;
	}

	size_t read = fread(file, 1, size, fp);
	fclose(fp);

	file[read] = '\0';

	return file;
}

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

void parser::load_ply(simd_vec3 &calculator,
					  const std::string &filename,
					  std::vector<vec3> *vertices,
					  std::vector<shape *> *shapes,
					  material *mat)
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
		if (line.size() == 0)
			continue;

		if (line.rfind("format ", 0) == 0)
		{
			std::istringstream iss(line);
			std::string tmp;
			iss >> tmp >> format;
		}
		else if (line.rfind("element vertex", 0) == 0)
		{
			std::istringstream iss(line);
			std::string tmp;
			iss >> tmp >> tmp >> vertexCount;
		}
		else if (line.rfind("element face", 0) == 0)
		{
			std::istringstream iss(line);
			std::string tmp;
			iss >> tmp >> tmp >> faceCount;
		}
		else if (line.rfind("property list", 0) == 0 && line.find("vertex_indices") != std::string::npos)
		{
			std::istringstream iss(line);
			std::string tmp;
			iss >> tmp >> tmp >> vertexListCountType >> vertexIndexType >> tmp;
		}
		else if (line.rfind("property ", 0) == 0)
		{
			static std::string current_element = "";
			{
				std::istringstream iss(line);
				std::string tmp, type, name;
				iss >> tmp >> type >> name;
			}
		}

		if (line == "end_header")
			break;
	}

	file.clear();
	file.seekg(0, std::ios::beg);

	std::string current_element = "";
	vertexProperties.clear();
	format = "ascii";
	vertexCount = 0;
	faceCount = 0;
	vertexIndexType = "int";
	vertexListCountType = "uchar";

	while (std::getline(file, line))
	{
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
			std::string elemName;
			iss >> elemName;
			current_element = elemName;
			if (elemName == "vertex")
			{
				iss >> vertexCount;
			}
			else if (elemName == "face")
			{
				iss >> faceCount;
			}
		}
		else if (token == "property")
		{
			if (current_element == "vertex")
			{
				std::string next;
				iss >> next;
				if (next == "list")
				{
					std::string count_type, index_type, name;
					iss >> count_type >> index_type >> name;
					vertexProperties.emplace_back("list", name);
				}
				else
				{
					std::string name;
					iss >> name;
					vertexProperties.emplace_back(next, name);
				}
			}
			else if (current_element == "face")
			{
				std::string next;
				iss >> next;
				if (next == "list")
				{
					std::string count_type, index_type, name;
					iss >> count_type >> index_type >> name;
					if (name == "vertex_indices" || name == "vertex_index")
					{
						vertexListCountType = count_type;
						vertexIndexType = index_type;
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

	auto read_double_bin = [&](std::ifstream &f) -> double
	{
		double v = 0.0;
		f.read(reinterpret_cast<char *>(&v), sizeof(v));
		return v;
	};

	bool is_ascii = (format.find("ascii") != std::string::npos);
	bool is_binary_little = (format.find("binary_little_endian") != std::string::npos);
	bool is_binary_big = (format.find("binary_big_endian") != std::string::npos);

	for (size_t vi = 0; vi < vertexCount; ++vi)
	{
		float vx = 0.0f, vy = 0.0f, vz = 0.0f;
		bool gotX = false, gotY = false, gotZ = false;

		if (is_ascii)
		{
			std::string vline;
			if (!std::getline(file, vline))
			{
				my_printf("Unexpected EOF while reading ascii vertex %zu\n", vi);
				exit(-1);
			}
			std::istringstream viss(vline);
			for (auto &prop : vertexProperties)
			{
				std::string ptype = prop.first;
				std::string pname = prop.second;
				std::string token;
				if (!(viss >> token))
				{
					my_printf("Malformed vertex line %zu\n", vi);
					exit(-1);
				}
				if (pname == "x")
				{
					vx = std::stof(token);
					gotX = true;
				}
				else if (pname == "y")
				{
					vy = std::stof(token);
					gotY = true;
				}
				else if (pname == "z")
				{
					vz = std::stof(token);
					gotZ = true;
				}
			}
		}
		else
		{
			for (auto &prop : vertexProperties)
			{
				std::string ptype = prop.first;
				std::string pname = prop.second;

				std::string t = ptype;
				std::transform(t.begin(), t.end(), t.begin(), ::tolower);

				if (t == "float" || t == "float32")
				{
					float v = read_float(file);
					if (pname == "x")
					{
						vx = v;
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = v;
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = v;
						gotZ = true;
					}
				}
				else if (t == "double" || t == "float64")
				{
					double d = read_double_bin(file);
					if (pname == "x")
					{
						vx = static_cast<float>(d);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(d);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(d);
						gotZ = true;
					}
				}
				else if (t == "uchar" || t == "uint8")
				{
					uint8_t u = read_uint8(file);
					if (pname == "x")
					{
						vx = static_cast<float>(u);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(u);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(u);
						gotZ = true;
					}
				}
				else if (t == "char" || t == "int8")
				{
					int8_t s = read_int8(file);
					if (pname == "x")
					{
						vx = static_cast<float>(s);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(s);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(s);
						gotZ = true;
					}
				}
				else if (t == "ushort" || t == "uint16")
				{
					uint16_t u = read_uint16(file);
					if (pname == "x")
					{
						vx = static_cast<float>(u);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(u);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(u);
						gotZ = true;
					}
				}
				else if (t == "short" || t == "int16")
				{
					int16_t s = read_int16(file);
					if (pname == "x")
					{
						vx = static_cast<float>(s);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(s);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(s);
						gotZ = true;
					}
				}
				else if (t == "int" || t == "int32")
				{
					int32_t s = read_int32(file);
					if (pname == "x")
					{
						vx = static_cast<float>(s);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(s);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(s);
						gotZ = true;
					}
				}
				else if (t == "uint" || t == "uint32")
				{
					uint32_t u = read_uint32(file);
					if (pname == "x")
					{
						vx = static_cast<float>(u);
						gotX = true;
					}
					else if (pname == "y")
					{
						vy = static_cast<float>(u);
						gotY = true;
					}
					else if (pname == "z")
					{
						vz = static_cast<float>(u);
						gotZ = true;
					}
				}
				else if (t == "list")
				{
					uint32_t count = read_uint8(file);
					for (uint32_t k = 0; k < count; ++k)
						(void)read_uint8(file);
				}
				else
				{
					my_printf("Warning: unsupported vertex property type '%s' — skipping. You may need to extend parser.\n", ptype.c_str());
					file.seekg(4, std::ios::cur);
				}
			}
		}

		static bool warned_missing_xyz = false;
		if ((!gotX || !gotY || !gotZ) && !warned_missing_xyz)
		{
			my_printf("Warning: PLY vertex properties did not include all x,y,z. Missing values will be zeroed.\n");
			warned_missing_xyz = true;
		}

		vertices->emplace_back(vx, vy, vz);
	}

	auto read_index_value = [&](std::ifstream &f) -> int32_t
	{
		if (vertexIndexType == "uint" || vertexIndexType == "uint32" || vertexIndexType == "uint32_t")
			return static_cast<int32_t>(read_uint32(f));
		else if (vertexIndexType == "int" || vertexIndexType == "int32" || vertexIndexType == "int32_t")
			return read_int32(f);
		else if (vertexIndexType == "ushort" || vertexIndexType == "uint16")
			return static_cast<int32_t>(read_uint16(f));
		else if (vertexIndexType == "short" || vertexIndexType == "int16")
			return read_int16(f);
		else if (vertexIndexType == "uchar" || vertexIndexType == "uint8")
			return static_cast<int32_t>(read_uint8(f));
		else if (vertexIndexType == "char" || vertexIndexType == "int8")
			return read_int8(f);
		else
		{
			my_printf("Unsupported vertex index type '%s' in PLY file.\n", vertexIndexType.c_str());
			exit(-1);
		}
	};

	auto read_list_count = [&](std::ifstream &f) -> uint32_t
	{
		if (vertexListCountType == "uchar" || vertexListCountType == "uint8")
			return read_uint8(f);
		else if (vertexListCountType == "ushort" || vertexListCountType == "uint16")
			return read_uint16(f);
		else if (vertexListCountType == "uint" || vertexListCountType == "uint32")
			return read_uint32(f);
		else
		{
			my_printf("Unsupported list count type '%s' in PLY file.\n", vertexListCountType.c_str());
			exit(-1);
		}
	};

	int triCount = 0;
	for (size_t fi = 0; fi < faceCount; ++fi)
	{
		uint32_t n = 0;
		if (is_ascii)
		{
			std::string fline;
			if (!std::getline(file, fline))
			{
				my_printf("Unexpected EOF while reading ascii face %zu\n", fi);
				exit(-1);
			}
			std::istringstream fiss(fline);
			fiss >> n;
			if (n == 3)
			{
				int32_t a, b, c;
				fiss >> a >> b >> c;
				if (a < 0 || b < 0 || c < 0 || a >= (int)vertexCount || b >= (int)vertexCount || c >= (int)vertexCount)
				{
					my_printf("Warning: skipping invalid face %zu (indices: %d, %d, %d)\n", (int)fi, a, b, c);
					continue;
				}
				shapes->push_back(new triangle(
					calculator,
					&vertices->at(a + startIndex),
					&vertices->at(b + startIndex),
					&vertices->at(c + startIndex),
					mat));
				triCount++;
			}
			else
			{
				for (uint32_t j = 0; j < n; ++j)
				{
					int32_t idx;
					fiss >> idx;
				}
			}
		}
		else
		{
			n = read_list_count(file);
			if (n == 3)
			{
				int32_t a = read_index_value(file);
				int32_t b = read_index_value(file);
				int32_t c = read_index_value(file);

				if (a < 0 || b < 0 || c < 0 ||
					a >= (int)vertexCount || b >= (int)vertexCount || c >= (int)vertexCount)
				{
					my_printf("Warning: skipping invalid face %zu (indices: %d, %d, %d)\n",
							  (int)fi, a, b, c);
					continue;
				}

				shapes->push_back(new triangle(
					calculator,
					&vertices->at(a + startIndex),
					&vertices->at(b + startIndex),
					&vertices->at(c + startIndex),
					mat));
				triCount++;
			}
			else
			{
				for (uint32_t j = 0; j < n; ++j)
					(void)read_index_value(file);
			}
		}
	}

	my_printf("Loaded PLY: %zu vertices (%d tris), startIndex=%zu\n", vertexCount, triCount, startIndex);
}

parser::parser(const char *file_name)
{
	json_content = get_json_content(file_name);
	d.Parse(json_content);
	if (d.HasParseError())
	{
		my_printf("RapidJSON parse error at offset %zu: %u\n",
				  d.GetErrorOffset(), d.GetParseError());
	}
	this->file_name = file_name;
}

std::vector<camera> *parser::get_camera(simd_vec3 &calculator)
{
	std::vector<camera> *res = new std::vector<camera>;

	const rapidjson::Value &cameras = d["Scene"]["Cameras"];

	if (!cameras.HasMember("Camera"))
		return res;

	const rapidjson::Value &camNode = cameras["Camera"];

	auto parseCamera = [&](const rapidjson::Value &cam)
	{
		std::string tmp;

		std::string type = "pinhole";
		if (cam.HasMember("_type"))
			type = cam["_type"].GetString();

		if (type == "lookAt")
		{
			float position_x, position_y, position_z;
			float gaze_x, gaze_y, gaze_z;
			float up_x, up_y, up_z;
			float neardistance;
			float fovY;
			int resx, resy;

			tmp = cam["Position"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &position_x, &position_y, &position_z);

			tmp = cam["GazePoint"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &gaze_x, &gaze_y, &gaze_z);

			tmp = cam["Up"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &up_x, &up_y, &up_z);

			tmp = cam["FovY"].GetString();
			sscanf(tmp.c_str(), "%f", &fovY);

			tmp = cam["NearDistance"].GetString();
			sscanf(tmp.c_str(), "%f", &neardistance);

			tmp = cam["ImageResolution"].GetString();
			sscanf(tmp.c_str(), "%d %d", &resx, &resy);

			tmp = cam["ImageName"].GetString();

			res->emplace_back(
				calculator, position_x, position_y, position_z,
				gaze_x, gaze_y, gaze_z, up_x, up_y, up_z,
				neardistance, fovY, resx, resy, tmp);
		}
		else
		{
			float position_x, position_y, position_z;
			float gaze_x, gaze_y, gaze_z;
			float up_x, up_y, up_z;
			float nearp_left, nearp_right, nearp_bottom, nearp_top;
			float neardistance;
			int resx, resy;

			tmp = cam["Position"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &position_x, &position_y, &position_z);

			tmp = cam["Gaze"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &gaze_x, &gaze_y, &gaze_z);

			tmp = cam["Up"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &up_x, &up_y, &up_z);

			tmp = cam["NearPlane"].GetString();
			sscanf(tmp.c_str(), "%f %f %f %f", &nearp_left, &nearp_right, &nearp_bottom, &nearp_top);

			tmp = cam["NearDistance"].GetString();
			sscanf(tmp.c_str(), "%f", &neardistance);

			tmp = cam["ImageResolution"].GetString();
			sscanf(tmp.c_str(), "%d %d", &resx, &resy);

			tmp = cam["ImageName"].GetString();

			res->emplace_back(calculator, position_x, position_y, position_z,
							  gaze_x, gaze_y, gaze_z, up_x, up_y, up_z,
							  neardistance, nearp_left, nearp_right, nearp_bottom, nearp_top,
							  resx, resy, tmp);
		}
	};

	if (camNode.IsObject())
		parseCamera(camNode);
	else if (camNode.IsArray())
		for (auto &c : camNode.GetArray())
			parseCamera(c);

	return res;
}

std::vector<vec3> *parser::get_vertices()
{
	std::vector<vec3> *res = new std::vector<vec3>;

	std::istringstream iss(d["Scene"]["VertexData"]["_data"].GetString());
	float x, y, z;

	while (iss >> x >> y >> z)
	{
		res->emplace_back(x, y, z);
	}

	return res;
}

std::vector<material> *parser::get_materials()
{
	auto *materials = new std::vector<material>;

	if (!d["Scene"].IsObject() || !d["Scene"]["Materials"].IsObject())
	{
		my_printf("Warning: No 'Materials' object found in scene.\n");
		return materials;
	}

	const auto &materialsNode = d["Scene"]["Materials"];
	if (!materialsNode.HasMember("Material"))
	{
		return materials;
	}

	auto processMaterial = [&](const rapidjson::Value &mat_json)
	{
		auto parseVec3 = [](const char *str_val) -> vec3
		{
			std::istringstream iss(str_val);
			float r, g, b;
			iss >> r >> g >> b;
			return vec3(r, g, b);
		};

		material new_material;

		if (mat_json.HasMember("_type"))
		{
			std::string type_str = mat_json["_type"].GetString();
			if (type_str == "mirror")
			{
				new_material.mt = Mirror;
			}
			else if (type_str == "dielectric")
			{
				new_material.mt = Dielectric;
			}
			else if (type_str == "conductor")
			{
				new_material.mt = Conductor;
			}
			else
			{
				new_material.mt = Regular;
			}
		}

		if (mat_json.HasMember("AmbientReflectance"))
		{
			new_material.AmbientReflectance = parseVec3(mat_json["AmbientReflectance"].GetString());
		}
		if (mat_json.HasMember("DiffuseReflectance"))
		{
			new_material.DiffuseReflectance = parseVec3(mat_json["DiffuseReflectance"].GetString());
		}
		if (mat_json.HasMember("SpecularReflectance"))
		{
			new_material.SpecularReflectance = parseVec3(mat_json["SpecularReflectance"].GetString());
		}
		if (mat_json.HasMember("PhongExponent"))
		{
			new_material.PhongExponent = std::stof(mat_json["PhongExponent"].GetString());
		}

		if (mat_json.HasMember("MirrorReflectance"))
		{
			new_material.MirrorReflectance = parseVec3(mat_json["MirrorReflectance"].GetString());
		}
		if (mat_json.HasMember("AbsorptionCoefficient"))
		{
			new_material.AbsorptionCoefficient = parseVec3(mat_json["AbsorptionCoefficient"].GetString());
		}
		if (mat_json.HasMember("RefractionIndex"))
		{
			new_material.RefractionIndex = std::stof(mat_json["RefractionIndex"].GetString());
		}
		if (mat_json.HasMember("AbsorptionIndex"))
		{
			new_material.AbsorptionIndex = std::stof(mat_json["AbsorptionIndex"].GetString());
		}

		materials->emplace_back(new_material);
	};

	const auto &material_data = materialsNode["Material"];

	if (material_data.IsArray())
	{
		for (const auto &mat_item : material_data.GetArray())
		{
			processMaterial(mat_item);
		}
	}
	else if (material_data.IsObject())
	{
		processMaterial(material_data);
	}

	return materials;
}

std::vector<shape *> *parser::get_shapes(simd_vec3 &calculator, std::vector<vec3> *vertices, std::vector<material> *materials)
{
	auto *shapes = new std::vector<shape *>;

	if (!d["Scene"].IsObject() || !d["Scene"].HasMember("Objects"))
	{
		my_printf("Warning: No 'Objects' found in scene.\n");
		return shapes;
	}
	const auto &objects = d["Scene"]["Objects"];

	auto processMesh = [&](const rapidjson::Value &mesh)
	{
		if (!mesh.HasMember("Faces") || !(mesh["Faces"].HasMember("_data") || mesh["Faces"].HasMember("_plyFile")))
		{
			return;
		}

		if (mesh["Faces"].HasMember("_plyFile"))
		{
			std::string plyName = mesh["Faces"]["_plyFile"].GetString();
			std::filesystem::path baseDir = std::filesystem::path(this->file_name).parent_path();
			std::filesystem::path fullPath = baseDir / plyName;
			load_ply(calculator, fullPath.string(), vertices, shapes,
					 &materials->at(std::stoi(mesh["Material"].GetString()) - 1));
		}
		else
		{
			std::istringstream iss(mesh["Faces"]["_data"].GetString());
			long long i0, i1, i2;

			while (iss >> i0 >> i1 >> i2)
			{
				shapes->push_back(new triangle(
					calculator,
					&vertices->at(static_cast<size_t>(i0 - 1)),
					&vertices->at(static_cast<size_t>(i1 - 1)),
					&vertices->at(static_cast<size_t>(i2 - 1)),
					&materials->at(std::stoi(mesh["Material"].GetString()) - 1)));
			}
		}
	};

	auto processTriangle = [&](const rapidjson::Value &tri)
	{
		if (!tri.HasMember("Indices"))
			return;

		std::istringstream iss(tri["Indices"].GetString());
		long long i0, i1, i2;

		if (iss >> i0 >> i1 >> i2)
		{
			shapes->push_back(new triangle(
				calculator,
				&vertices->at(static_cast<size_t>(i0 - 1)),
				&vertices->at(static_cast<size_t>(i1 - 1)),
				&vertices->at(static_cast<size_t>(i2 - 1)),
				&materials->at(std::stoi(tri["Material"].GetString()) - 1)));
		}
	};

	auto processSphere = [&](const rapidjson::Value &sph)
	{
		if (!sph.HasMember("Center") || !sph.HasMember("Radius"))
			return;

		int center_idx = std::stoi(sph["Center"].GetString());
		float radius = std::stof(sph["Radius"].GetString());

		shapes->push_back(new sphere(
			calculator,
			&vertices->at(center_idx - 1),
			radius,
			&materials->at(std::stoi(sph["Material"].GetString()) - 1)));
	};

	auto processPlane = [&](const rapidjson::Value &pl)
	{
		if (!pl.HasMember("Point") || !pl.HasMember("Normal"))
			return;

		int point_idx = std::stoi(pl["Point"].GetString());

		std::istringstream iss(pl["Normal"].GetString());
		float nx, ny, nz;
		iss >> nx >> ny >> nz;

		vec3 *normal_vec = new vec3(nx, ny, nz);

		shapes->push_back(new plane(
			&vertices->at(point_idx - 1),
			normal_vec,
			&materials->at(std::stoi(pl["Material"].GetString()) - 1)));
	};

	size_t total_extra_vertices = 0;
	if (objects.HasMember("Mesh"))
	{
		const auto &node = objects["Mesh"];
		auto checkNode = [&](const rapidjson::Value &item)
		{
			if (!item.HasMember("Faces") || !item["Faces"].HasMember("_plyFile"))
				return;
			std::string plyName = item["Faces"]["_plyFile"].GetString();
			std::filesystem::path baseDir = std::filesystem::path(this->file_name).parent_path();
			std::filesystem::path fullPath = baseDir / plyName;
			std::ifstream pf(fullPath, std::ios::binary);
			if (!pf.is_open())
			{
				my_printf("Warning: could not open PLY header for reserve: %s\n", fullPath.string().c_str());
				return;
			}
			std::string line;
			while (std::getline(pf, line))
			{
				size_t a = line.find_first_not_of(" \t\r\n");
				if (a == std::string::npos)
					continue;
				if (line.rfind("element vertex", 0) == 0)
				{
					std::istringstream iss(line);
					std::string tmp;
					size_t cnt = 0;
					iss >> tmp >> tmp >> cnt;
					total_extra_vertices += cnt;
					break;
				}
				if (line == "end_header")
					break;
			}
			pf.close();
		};

		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
				checkNode(item);
		}
		else if (node.IsObject())
		{
			checkNode(node);
		}
	}

	if (total_extra_vertices > 0)
	{
		vertices->reserve(vertices->size() + total_extra_vertices);
		my_printf("DEBUG: reserved vertices capacity: %zu (+%zu for meshes)\n", vertices->capacity(), total_extra_vertices);
	}

	if (objects.HasMember("Mesh"))
	{
		const auto &node = objects["Mesh"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processMesh(item);
			}
		}
		else if (node.IsObject())
		{
			processMesh(node);
		}
	}

	if (objects.HasMember("Triangle"))
	{
		const auto &node = objects["Triangle"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processTriangle(item);
			}
		}
		else if (node.IsObject())
		{
			processTriangle(node);
		}
	}

	if (objects.HasMember("Sphere"))
	{
		const auto &node = objects["Sphere"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processSphere(item);
			}
		}
		else if (node.IsObject())
		{
			processSphere(node);
		}
	}

	if (objects.HasMember("Plane"))
	{
		const auto &node = objects["Plane"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processPlane(item);
			}
		}
		else if (node.IsObject())
		{
			processPlane(node);
		}
	}

	return shapes;
}

float parser::get_intersectionepsilon()
{
	if (d["Scene"].HasMember("IntersectionTestEpsilon"))
		return std::stof(d["Scene"]["IntersectionTestEpsilon"].GetString());
	return 1e-6f;
}

float parser::get_shadowrayepsilon()
{
	if (d["Scene"].HasMember("ShadowRayEpsilon"))
		return std::stof(d["Scene"]["ShadowRayEpsilon"].GetString());
	return 1e-3f;
}

float parser::get_maxrecursiondepth()
{
	if (d["Scene"].HasMember("MaxRecursionDepth"))
		return std::stof(d["Scene"]["MaxRecursionDepth"].GetString());
	return 6;
}

vec3 parser::get_backgroundcolor()
{
	vec3 color;
	if (d["Scene"].HasMember("BackgroundColor"))
	{
		std::string tmp = d["Scene"]["BackgroundColor"].GetString();
		std::istringstream iss(tmp);
		float r, g, b;
		iss >> r >> g >> b;
		color.load(r, g, b);
	}
	else
		color.load(0, 0, 0);
	return color;
}

vec3 parser::get_ambientlight()
{
	vec3 amb;
	if (d["Scene"].HasMember("Lights") && d["Scene"]["Lights"].HasMember("AmbientLight"))
	{
		std::string tmp = d["Scene"]["Lights"]["AmbientLight"].GetString();
		std::istringstream iss(tmp);
		float r, g, b;
		iss >> r >> g >> b;
		amb.load(r, g, b);
	}
	else
		amb.load(0, 0, 0);
	return amb;
}

std::vector<point_light> *parser::get_pointlights()
{
	std::vector<point_light> *lights = new std::vector<point_light>();

	if (!d["Scene"].HasMember("Lights") || !d["Scene"]["Lights"].HasMember("PointLight"))
		return lights;

	const auto &plNode = d["Scene"]["Lights"]["PointLight"];

	if (plNode.IsObject())
	{
		point_light light;

		std::string tmp = plNode["Position"].GetString();
		std::istringstream posStream(tmp);
		float px, py, pz;
		posStream >> px >> py >> pz;
		light.position.load(px, py, pz);

		tmp = plNode["Intensity"].GetString();
		std::istringstream intStream(tmp);
		float ix, iy, iz;
		intStream >> ix >> iy >> iz;
		light.intensity.load(ix, iy, iz);

		lights->push_back(light);
	}
	else if (plNode.IsArray())
	{
		for (auto &v : plNode.GetArray())
		{
			point_light light;

			std::string tmp = v["Position"].GetString();
			std::istringstream posStream(tmp);
			float px, py, pz;
			posStream >> px >> py >> pz;
			light.position.load(px, py, pz);

			tmp = v["Intensity"].GetString();
			std::istringstream intStream(tmp);
			float ix, iy, iz;
			intStream >> ix >> iy >> iz;
			light.intensity.load(ix, iy, iz);

			lights->push_back(light);
		}
	}

	return lights;
}