#include "helper.h"

static FILE *file = fopen("output.txt", "a");

void my_printf(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vprintf(fmt, args);
	va_end(args);

	va_start(args, fmt);
	vfprintf(file, fmt, args);
	va_end(args);

	fflush(stdout);
	fflush(file);
}

float get_random_float()
{
	static thread_local std::mt19937 generator(std::random_device{}());
	static thread_local std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
	return distribution(generator);
}