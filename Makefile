main:
	g++ -std=c++20 -O3 -march=native -flto -pipe -DNDEBUG ./src/*.cpp -o raytracer