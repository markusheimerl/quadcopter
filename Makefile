CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra -Isrc
LDFLAGS = -lm -lwebp -lwebpmux -lpthread -flto

TARGET = raytracer.out
SRCS = src/raytracer.c src/math/mat4.c src/math/ray.c src/math/vec3.c \
       src/geometry/aabb.c src/geometry/mesh.c src/accel/bvh.c \
       src/render/camera.c src/render/light.c src/utils/image.c \
       src/utils/progress.c src/scene.c

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -o $(TARGET)

run: $(TARGET)
	@time ./$(TARGET)

clean:
	rm -f *.out *_rendering.webp