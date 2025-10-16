#pragma once
#include "tp.h"
#include "camera.h"

class parser
{
private:
	rapidjson::Document d;
	static char *get_json_content(const char *fileName);

public:
	parser() = delete;
	parser(const char *file_name);
	camera *get_camera();
};