CC = clang
CFLAGS = -O3 -march=native -Wall -Wextra
LDFLAGS = -static -lm -lwebp -lwebpmux -lsharpyuv -lpthread -flto

TARGET = raytracer.out
SRC = raytracer.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

run: $(TARGET)
	@time ./$(TARGET)

clean:
	rm -f *.out *_rendering.webp