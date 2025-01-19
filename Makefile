CC = clang
CFLAGS = -O3 -march=native -ffast-math -I./rasterizer
LDFLAGS = -lm -flto

TARGET = sim.out
SRC = sim.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) *.gif

.PHONY: run clean