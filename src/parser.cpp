#include "parser.h"
#include <string>
#include <sstream>

char *parser::get_json_content(const char *fileName)
{
	FILE *fp = fopen(fileName, "rb");
	if (!fp)
		return NULL;

	fseek(fp, 0, SEEK_END);
	long size = ftell(fp);
	rewind(fp); // Back to start

	char *file = (char *)calloc(size + 1, 1); // +1 for null terminator
	if (!file)
	{
		fclose(fp);
		return NULL;
	}

	size_t read = fread(file, 1, size, fp);
	fclose(fp);

	// Handle partial read
	if (read < (size_t)size)
	{
		file[read] = '\0'; // Ensure termination
	}
	return file;
}

parser::parser(const char *file_name)
{
	d.Parse(get_json_content(file_name));
	if (d.HasParseError())
	{
		printf("%s parse error.\n", file_name);
	}
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
	};

	if (camNode.IsObject())
	{
		parseCamera(camNode);
	}
	else if (camNode.IsArray())
	{
		for (auto &c : camNode.GetArray())
		{
			parseCamera(c);
		}
	}

	return res;
}

std::vector<vec3> *parser::get_vertices()
{
	std::vector<vec3> *res = new std::vector<vec3>;

	const std::string dataStr = d["Scene"]["VertexData"]["_data"].GetString();

	std::istringstream iss(dataStr);
	float x, y, z;

	while (iss >> x >> y >> z)
	{
		res->emplace_back(x, y, z);
	}

	return res;
}

std::vector<material> *parser::get_materials()
{
	// Allocate the vector on the heap, as per the function signature.
	auto *materials = new std::vector<material>;

	// Check for the existence of the "Materials" node in the JSON document.
	if (!d["Scene"].IsObject() || !d["Scene"]["Materials"].IsObject())
	{
		printf("Warning: No 'Materials' object found in scene.\n");
		return materials; // Return the empty vector
	}

	const auto &materialsNode = d["Scene"]["Materials"];
	if (!materialsNode.HasMember("Material"))
	{
		return materials; // No "Material" key found
	}

	// --- Helper lambda to parse a single JSON material object into the C++ struct ---
	auto processMaterial = [&](const rapidjson::Value &mat_json)
	{
		// Internal helper to parse a "r g b" string into a vec3 object
		auto parseVec3 = [](const char *str_val) -> vec3
		{
			std::istringstream iss(str_val);
			float r, g, b;
			iss >> r >> g >> b;
			return vec3(r, g, b);
		};

		material new_material;

		// Safely parse each component of the material
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

		// Note: The material "_id" is ignored as the function returns a simple vector.
		// The order of materials in the JSON file will determine their index in the vector.
		// Material with ID "1" will be at index 0, ID "2" at index 1, and so on.
		materials->emplace_back(new_material);
	};

	// --- Main Logic: Handle both single object and array cases ---
	const auto &material_data = materialsNode["Material"];

	if (material_data.IsArray())
	{
		// If it's an array, iterate and process each material object
		for (const auto &mat_item : material_data.GetArray())
		{
			processMaterial(mat_item);
		}
	}
	else if (material_data.IsObject())
	{
		// If it's just a single object, process it
		processMaterial(material_data);
	}

	return materials;
}

std::vector<shape *> *parser::get_shapes(simd_vec3 &calculator, std::vector<vec3> *vertices, std::vector<material> *materials)
{
	// The vector of shape pointers we will build and return.
	auto *shapes = new std::vector<shape *>;

	// Check for the existence of the "Objects" node.
	if (!d["Scene"].IsObject() || !d["Scene"].HasMember("Objects"))
	{
		printf("Warning: No 'Objects' found in scene.\n");
		return shapes;
	}
	const auto &objects = d["Scene"]["Objects"];

	// --- Helper Lambda to process a single "Mesh" JSON object ---
	auto processMesh = [&](const rapidjson::Value &mesh)
	{
		if (!mesh.HasMember("Faces") || !mesh["Faces"].HasMember("_data"))
			return;

		std::istringstream iss(mesh["Faces"]["_data"].GetString());
		int i0, i1, i2;

		// A mesh is composed of multiple triangles.
		while (iss >> i0 >> i1 >> i2)
		{
			shapes->push_back(new triangle(calculator,
										   &vertices->at(i0 - 1),
										   &vertices->at(i1 - 1),
										   &vertices->at(i2 - 1),
										   &materials->at(std::stoi(mesh["Material"].GetString()) - 1)));
		}
	};

	// --- Helper Lambda to process a single "Triangle" JSON object ---
	auto processTriangle = [&](const rapidjson::Value &tri)
	{
		if (!tri.HasMember("Indices"))
			return;

		std::istringstream iss(tri["Indices"].GetString());
		int i0, i1, i2;

		if (iss >> i0 >> i1 >> i2)
		{
			shapes->push_back(new triangle(calculator,
										   &vertices->at(i0 - 1),
										   &vertices->at(i1 - 1),
										   &vertices->at(i2 - 1),
										   &materials->at(std::stoi(tri["Material"].GetString()) - 1)));
		}
	};

	// --- Helper Lambda to process a single "Sphere" JSON object ---
	auto processSphere = [&](const rapidjson::Value &sph)
	{
		if (!sph.HasMember("Center") || !sph.HasMember("Radius"))
			return;

		int center_idx = std::stoi(sph["Center"].GetString());
		float radius = std::stof(sph["Radius"].GetString());

		shapes->push_back(new sphere(
			&vertices->at(center_idx - 1),
			radius,
			&materials->at(std::stoi(sph["Material"].GetString()) - 1)));
	};

	// --- Helper Lambda to process a single "Plane" JSON object ---
	auto processPlane = [&](const rapidjson::Value &pl)
	{
		if (!pl.HasMember("Point") || !pl.HasMember("Normal"))
			return;

		int point_idx = std::stoi(pl["Point"].GetString());

		std::istringstream iss(pl["Normal"].GetString());
		float nx, ny, nz;
		iss >> nx >> ny >> nz;

		// The plane's normal is defined by value, not by an index.
		// We must allocate it on the heap, as the constructor takes a pointer.
		vec3 *normal_vec = new vec3(nx, ny, nz);

		shapes->push_back(new plane(
			&vertices->at(point_idx - 1),
			normal_vec,
			&materials->at(std::stoi(pl["Material"].GetString()) - 1)));
	};

	// --- Main Parsing Logic ---
	// For each shape type, check if it exists. If it's an array, loop through it.
	// If it's a single object, process just that one.

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
		return lights; // return empty if none

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