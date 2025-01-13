CC = clang
CFLAGS = -O3 -march=native -ffast-math -I./rasterizer
LDFLAGS = -lm -flto

TARGET = sim.out
SRC = sim.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

render: $(SRC)
	$(CC) $(CFLAGS) -DRENDER $(SRC) $(LDFLAGS) -o $(TARGET)

log: $(SRC)
	$(CC) $(CFLAGS) -DLOG $(SRC) $(LDFLAGS) -o $(TARGET)

run: render
	./$(TARGET)

clean:
	rm -f $(TARGET) *_state_data.csv *_simulation.gif