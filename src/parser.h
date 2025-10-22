#pragma once
#include "tp.h"
#include "camera.h"
#include "shapes.h"

class parser
{
private:
	rapidjson::Document d;
	static char *get_json_content(const char *fileName);

public:
	parser() = delete;
	parser(const char *file_name);
	std::vector<camera> *get_camera(simd_vec3 &calculator);
	std::vector<vec3> *get_vertices();
	std::vector<material> *get_materials();
	std::vector<shape *> *get_shapes(simd_vec3 &calculator, std::vector<vec3> *vertices, std::vector<material> *materials);
	float get_intersectionepsilon();
	float get_shadowrayepsilon();
	vec3 get_backgroundcolor();
	vec3 get_ambientlight();
	std::vector<point_light> *get_pointlights();
};