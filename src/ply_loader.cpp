#include "parser.h"
#include <algorithm>
#include <sstream>
#include <fstream>

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
					  material *mat, all_mesh_infos *ami)
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

		if (line.rfind("end_header", 0) == 0)
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
					mat, ami));
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
					mat, ami));
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