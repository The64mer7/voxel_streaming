# Voxel Streaming

An experimental voxel streaming and rendering application inspired by
[*GigaVoxels: Ray-Guided Streaming Analysis of Large Octree Structures*](https://doi.org/10.1145/1507149.1507152).
It is a native C++20 application using OpenGL, GLFW, and Dear ImGui.

## Build and run

From the repository root in the UCRT64 terminal:

```sh
cmake --preset ucrt64-gcc-ninja-debug
cmake --build --preset build-debug
./build/debug/opengl_app.exe
```

For an optimized build, use the release presets instead:

```sh
cmake --preset ucrt64-gcc-ninja-release
cmake --build --preset build-release
./build/release/opengl_app.exe
```