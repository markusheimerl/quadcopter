# raytracer
A minimal raytracer

![animation](https://raw.githubusercontent.com/markusheimerl/raytracer/4ee7c04c67b42cb8211b24771bf415078c1187b0/20250127_141008_rendering.webp)

## How to run
```
sudo apt update
sudo apt install clang libwebp-dev time
make run
```

## Parallelization
The raytracer now supports parallel rendering using multiple threads. By default, it uses 2 threads for optimal performance. You can control the number of threads using the `RAYTRACER_THREADS` environment variable:

```bash
# Use single thread
RAYTRACER_THREADS=1 ./raytracer.out

# Use 4 threads  
RAYTRACER_THREADS=4 ./raytracer.out

# Use default (2 threads)
./raytracer.out
```