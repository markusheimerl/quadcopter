CC = clang
CFLAGS = -O3 -march=native -ffast-math -Wall -Wextra -Iraytracer
LDFLAGS = -static -lm -lwebp -lwebpmux -lpthread -flto

sim.out: sim.c
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

run: sim.out
	@time ./sim.out

clean:
	rm -f *.out *_flight.webp
