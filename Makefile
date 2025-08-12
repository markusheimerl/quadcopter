CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra -I. -fopenmp
LDFLAGS = -lm -lwebp -lwebpmux -lpthread -fopenmp -flto

OBJS = raytracer.o scene.o \
       math/mat4.o math/ray.o math/vec3.o \
       geometry/aabb.o geometry/mesh.o \
       accel/bvh.o \
       render/camera.o render/light.o \
       utils/image.o utils/progress.o

raytracer.out: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: raytracer.out
	@time ./raytracer.out

clean:
	rm -f *.out *.o */*.o *.webp