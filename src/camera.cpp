#include "camera.h"

camera::camera(simd_vec3 &calculator, float position_x, float position_y, float position_z,
			   float gaze_x, float gaze_y, float gaze_z,
			   float up_x, float up_y, float up_z, float neardistance,
			   float nearp_left, float nearp_right, float nearp_bottom, float nearp_top,
			   int resx, int resy, std::string output) : position(position_x, position_y, position_z), calculator(calculator),
														 resx(resx), resy(resy), output(output)
{
	vec3 gaze(gaze_x, gaze_y, gaze_z);
	vec3 up(up_x, up_y, up_z);
	vec3 w;
	vec3 u;
	vec3 v;
	calculator.normalize(gaze, w);
	calculator.mult_scalar(w, -1, w);
	calculator.cross(up, w, u);
	calculator.normalize(u, u);
	calculator.cross(w, u, v);

	vec3 center;

	calculator.mult_scalar(w, neardistance, w);
	calculator.subs(position, w, center);

	vec3 pixelPos;

	for (int j = 0; j < resy; ++j)
	{
		for (int i = 0; i < resx; ++i)
		{
			float su = nearp_left + (nearp_right - nearp_left) * (i + 0.5f) / resx;
			float sv = nearp_bottom + (nearp_top - nearp_bottom) * (j + 0.5f) / resy;
			calculator.mult_scalar(u, su, gaze);
			calculator.mult_scalar(v, sv, up);
			calculator.add(gaze, up, pixelPos);
			calculator.add(pixelPos, center, pixelPos);
			calculator.subs(pixelPos, position, pixelPos);
			calculator.normalize(pixelPos, pixelPos);
			ray_dirs.push_back(pixelPos);
		}
	}
}
