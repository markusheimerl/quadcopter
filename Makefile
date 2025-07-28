CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra -Isrc

# Try to link with different library combinations for better compatibility
# Some systems have sharpyuv integrated into libwebp, others need it separately
LDFLAGS_BASE = -static -lwebp -lwebpmux -lm -lpthread -flto
LDFLAGS_SHARP = -static -lwebp -lwebpmux -lsharpyuv -lm -lpthread -flto

TARGET = raytracer.out
MAIN_SRC = raytracer.c

# Source directories
MATH_DIR = src/math
GEOMETRY_DIR = src/geometry
ACCEL_DIR = src/accel
RENDER_DIR = src/render
UTILS_DIR = src/utils

# Source files
MATH_SRCS = $(wildcard $(MATH_DIR)/*.c)
GEOMETRY_SRCS = $(wildcard $(GEOMETRY_DIR)/*.c)
ACCEL_SRCS = $(wildcard $(ACCEL_DIR)/*.c)
RENDER_SRCS = $(wildcard $(RENDER_DIR)/*.c)
UTILS_SRCS = $(wildcard $(UTILS_DIR)/*.c)
SCENE_SRCS = src/scene.c

ALL_SRCS = $(MAIN_SRC) $(MATH_SRCS) $(GEOMETRY_SRCS) $(ACCEL_SRCS) $(RENDER_SRCS) $(UTILS_SRCS) $(SCENE_SRCS)

# Object files
MATH_OBJS = $(MATH_SRCS:.c=.o)
GEOMETRY_OBJS = $(GEOMETRY_SRCS:.c=.o)
ACCEL_OBJS = $(ACCEL_SRCS:.c=.o)
RENDER_OBJS = $(RENDER_SRCS:.c=.o)
UTILS_OBJS = $(UTILS_SRCS:.c=.o)
SCENE_OBJS = $(SCENE_SRCS:.c=.o)
MAIN_OBJ = $(MAIN_SRC:.c=.o)

ALL_OBJS = $(MAIN_OBJ) $(MATH_OBJS) $(GEOMETRY_OBJS) $(ACCEL_OBJS) $(RENDER_OBJS) $(UTILS_OBJS) $(SCENE_OBJS)

$(TARGET): $(ALL_OBJS)
	@echo "Linking raytracer..."
	@if $(CC) $(ALL_OBJS) $(LDFLAGS_BASE) -o $(TARGET) 2>/dev/null; then \
		echo "Successfully linked with base WebP libraries."; \
	elif $(CC) $(ALL_OBJS) $(LDFLAGS_SHARP) -o $(TARGET) 2>/dev/null; then \
		echo "Successfully linked with sharpyuv library included."; \
	else \
		echo "Error: Failed to link. Trying without static linking..."; \
		$(CC) $(ALL_OBJS) -lwebp -lwebpmux -lm -lpthread -flto -o $(TARGET); \
	fi

webp-check:
	@if ! echo '#include <webp/encode.h>' | $(CC) -E - >/dev/null 2>&1; then \
		echo "Error: WebP development headers not found."; \
		echo "Please install libwebp-dev (Debian/Ubuntu) or libwebp-devel (RedHat/CentOS):"; \
		echo "  sudo apt-get install libwebp-dev"; \
		echo "  sudo yum install libwebp-devel"; \
		echo ""; \
		echo "On Jetson/ARM systems, you may need:"; \
		echo "  sudo apt-get update"; \
		echo "  sudo apt-get install libwebp-dev"; \
		exit 1; \
	fi

%.o: %.c webp-check
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	@time ./$(TARGET)

clean:
	rm -f *.out $(ALL_OBJS) *_rendering.webp

.PHONY: clean run webp-check