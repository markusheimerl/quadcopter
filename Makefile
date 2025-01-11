# Compiler and flags
CC = clang
CFLAGS = -O3 -march=native -ffast-math -I./rasterizer
LDFLAGS = -lm -flto

# Target and source
TARGET = sim.out
SRC = sim.c

# Build target
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

# Build target with RENDER flag
render: CFLAGS += -DRENDER
render: $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

# Build target with LOG flag
log: CFLAGS += -DLOG
log: $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

# Run the program
run: $(TARGET)
	./$(TARGET)

# Clean up
clean:
	rm -f $(TARGET) *_state_data.csv *_simulation.gif