main:
	g++ -std=c++20 -O3 -march=native -ffast-math -ftree-vectorize -msse2 -flto -pipe -DNDEBUG ./src/*.cpp -o raytracer