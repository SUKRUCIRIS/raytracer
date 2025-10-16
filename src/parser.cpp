#include "parser.h"

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