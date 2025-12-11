CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra -I../raytracer -fopenmp
ENZYME_PATH = $(HOME)/Documents/Enzyme/enzyme/build/Enzyme/ClangEnzyme-18.so
LDFLAGS = -lm -lwebp -lwebpmux -lpthread -fopenmp -flto

# Raytracer object files
RAYTRACER_OBJS = ../raytracer/scene.o \
                 ../raytracer/math/mat4.o ../raytracer/math/ray.o ../raytracer/math/vec3.o \
                 ../raytracer/geometry/aabb.o ../raytracer/geometry/mesh.o \
                 ../raytracer/accel/bvh.o \
                 ../raytracer/render/camera.o ../raytracer/render/light.o \
                 ../raytracer/utils/image.o ../raytracer/utils/progress.o

sim.out: sim.o $(RAYTRACER_OBJS)
	$(CC) sim.o $(RAYTRACER_OBJS) $(LDFLAGS) -o $@

sim.o: sim.c quad.h
	$(CC) $(CFLAGS) -fplugin=$(ENZYME_PATH) -c sim.c -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: sim.out
	@time ./sim.out

clean:
	rm -f *.out *.o ../raytracer/*.o ../raytracer/*/*.o *_flight.webp