CC = clang
CFLAGS = -O3 -march=native -ffast-math -Wall -Wextra -Iraytracer
LDFLAGS = -static -lm -lwebp -lwebpmux -lpthread -flto

TARGET = sim.out
SRC = sim.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

run: $(TARGET)
	@time ./$(TARGET)

clean:
	rm -f *.out *_flight.webp