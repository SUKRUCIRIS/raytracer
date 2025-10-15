#include "tp.h"
#include "parser.h"
#include "algebra.h"

int main(int argc, char **argv)
{
	vec3 vec1 = {10, 20, 30};
	vec3 vec2 = {5, 2, 3};

	vec3 res;

	simd_vec3 calculator;

	calculator.mult(vec1, vec2, res);
	res.store();
	res.print();

	calculator.add(vec1, vec2, res);
	res.store();
	res.print();

	calculator.subs(vec1, vec2, res);
	res.store();
	res.print();

	calculator.div(vec1, vec2, res);
	res.store();
	res.print();

	calculator.mult_scalar(vec1, 2, res);
	res.store();
	res.print();

	calculator.add_scalar(vec1, 3, res);
	res.store();
	res.print();

	float x;
	calculator.dot(vec1, vec2, x);
	printf("dot: %f\n", x);

	calculator.cross(vec1, vec2, res);
	res.store();
	res.print();

	calculator.length_squared(vec1, x);
	printf("dot: %f\n", x);

	calculator.normalize(vec1, res);
	res.store();
	res.print();

	return 0;
}