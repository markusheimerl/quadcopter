CC = clang
CFLAGS = -O3 -march=native -ffast-math
LDFLAGS = -flto -lm

TARGET = raytracer.out
SRC = raytracer.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f *.out *.gif