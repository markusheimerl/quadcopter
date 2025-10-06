CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra -Iraytracer -fopenmp
LDFLAGS = -lm -lwebp -lwebpmux -lpthread -fopenmp -flto

# Raytracer object files
RAYTRACER_OBJS = raytracer/scene.o \
                 raytracer/math/mat4.o raytracer/math/ray.o raytracer/math/vec3.o \
                 raytracer/geometry/aabb.o raytracer/geometry/mesh.o \
                 raytracer/accel/bvh.o \
                 raytracer/render/camera.o raytracer/render/light.o \
                 raytracer/utils/image.o raytracer/utils/progress.o

# Full simulation with rendering
sim.out: sim.o $(RAYTRACER_OBJS)
	$(CC) sim.o $(RAYTRACER_OBJS) $(LDFLAGS) -o $@

# Fast data generation without rendering
sim_data.out: sim.c
	$(CC) $(CFLAGS) -DNO_RENDER sim.c -lm -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: sim.out
	@./sim.out

data: sim_data.out
	@./sim_data.out --data 1000

clean:
	rm -f *.out *.o raytracer/*.o raytracer/*/*.o *_flight.webp *_data.csv