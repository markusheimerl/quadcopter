CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra -I.
LDFLAGS = -lm -lwebp -lwebpmux -lpthread -flto

TARGET = raytracer.out
OBJECTS = raytracer.o math/mat4.o math/ray.o math/vec3.o geometry/aabb.o geometry/mesh.o \
          accel/bvh.o render/camera.o render/light.o utils/image.o utils/progress.o scene.o

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

raytracer.o: raytracer.c scene.h
	$(CC) $(CFLAGS) -c raytracer.c -o $@

math/mat4.o: math/mat4.c math/mat4.h
	$(CC) $(CFLAGS) -c math/mat4.c -o $@

math/ray.o: math/ray.c math/ray.h math/vec3.h
	$(CC) $(CFLAGS) -c math/ray.c -o $@

math/vec3.o: math/vec3.c math/vec3.h
	$(CC) $(CFLAGS) -c math/vec3.c -o $@

geometry/aabb.o: geometry/aabb.c geometry/aabb.h math/vec3.h math/ray.h
	$(CC) $(CFLAGS) -c geometry/aabb.c -o $@

geometry/mesh.o: geometry/mesh.c geometry/mesh.h geometry/triangle.h accel/bvh.h math/ray.h
	$(CC) $(CFLAGS) -c geometry/mesh.c -o $@

accel/bvh.o: accel/bvh.c accel/bvh.h geometry/aabb.h geometry/triangle.h math/ray.h
	$(CC) $(CFLAGS) -c accel/bvh.c -o $@

render/camera.o: render/camera.c render/camera.h math/vec3.h math/ray.h
	$(CC) $(CFLAGS) -c render/camera.c -o $@

render/light.o: render/light.c render/light.h math/vec3.h
	$(CC) $(CFLAGS) -c render/light.c -o $@

utils/image.o: utils/image.c utils/image.h
	$(CC) $(CFLAGS) -c utils/image.c -o $@

utils/progress.o: utils/progress.c utils/progress.h
	$(CC) $(CFLAGS) -c utils/progress.c -o $@

scene.o: scene.c scene.h accel/bvh.h geometry/mesh.h render/camera.h render/light.h utils/progress.h utils/image.h
	$(CC) $(CFLAGS) -c scene.c -o $@

run: $(TARGET)
	@time ./$(TARGET)

clean:
	rm -f *.out *.o */*.o *.webp