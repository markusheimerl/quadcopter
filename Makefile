# Compiler and flags
CC = gcc
CFLAGS = -O3 -march=native -ffast-math -funroll-loops -I./rasterizer
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
	rm -f $(TARGET) *_control_data.csv *_simulation.gif