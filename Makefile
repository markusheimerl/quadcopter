USE_ONEAPI = true

CC = clang
BASE_CFLAGS = -O3 -march=native -ffast-math -I./rasterizer
BASE_LDFLAGS = -lm -flto

ifeq ($(USE_ONEAPI),true)
CFLAGS = $(BASE_CFLAGS) -DONEAPI -I${MKLROOT}/include -DMKL_ILP64 -m64
LDFLAGS = -L${MKLROOT}/lib/intel64 -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lpthread -ldl $(BASE_LDFLAGS)
PREREQ = check-env
else
CFLAGS = $(BASE_CFLAGS)
LDFLAGS = $(BASE_LDFLAGS)
PREREQ =
endif

TARGET = sim.out
SRC = sim.c

check-env:
	@if [ -z "$$MKLROOT" ]; then \
		echo "MKLROOT is not set. Please run: source /opt/intel/oneapi/setvars.sh"; \
		exit 1; \
	fi

$(TARGET): $(PREREQ) $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) *.gif

.PHONY: check-env run clean