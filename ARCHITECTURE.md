# Raytracer - Modular Architecture

This raytracer has been refactored to improve separation of concerns, logic flow, and project structure while maintaining all original functionality and performance characteristics.

## Project Structure

The code is now organized into clear, modular components:

```
src/
├── math/           # Mathematical operations and primitives
│   ├── vec3.h/c    # 3D vector operations
│   ├── mat4.h/c    # 4x4 matrix operations 
│   └── ray.h/c     # Ray operations and transformations
│
├── geometry/       # Geometric primitives and operations
│   ├── triangle.h  # Triangle data structure
│   ├── aabb.h/c    # Axis-aligned bounding box operations
│   └── mesh.h/c    # 3D mesh loading and management
│
├── accel/          # Acceleration structures
│   └── bvh.h/c     # Bounding Volume Hierarchy for fast ray tracing
│
├── render/         # Rendering components
│   ├── camera.h/c  # Camera operations and ray generation
│   └── light.h/c   # Lighting calculations
│
├── utils/          # Utility functions
│   ├── progress.h/c # Progress bar reporting
│   └── image.h/c   # Image processing and interpolation
│
└── scene.h/c       # High-level scene management and rendering
```

## Key Improvements

### 1. Separation of Concerns
- **Math operations** are isolated in their own module
- **Geometric primitives** are separated from acceleration structures
- **Rendering logic** is cleanly separated from scene management
- **Utility functions** are modularized for reusability

### 2. Better Logic Flow
- Clear dependency hierarchy (math → geometry → acceleration → rendering)
- Single responsibility for each module
- Minimal coupling between components
- Clean interfaces between modules

### 3. Improved Project Structure
- Header/implementation separation for better compilation
- Modular organization allows for easier testing and maintenance
- Clear module boundaries make the codebase more navigable
- Extensible architecture for future enhancements

### 4. Maintained Functionality
- All original features preserved: mesh loading, BVH acceleration, lighting, animation
- Performance characteristics maintained through proper optimization flags
- Same rendering quality and output format
- Identical user interface and behavior

## Building and Running

```bash
make clean      # Clean build artifacts
make           # Build the raytracer
make run       # Build and run with timing
```

## Dependencies

- clang compiler
- libwebp-dev (for WebP image format support)
- Standard C library and math library

This refactored version maintains backward compatibility while providing a much cleaner, more maintainable codebase that follows software engineering best practices.