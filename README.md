# Voxel Streaming

An experimental voxel streaming and rendering application inspired by
[*GigaVoxels: Ray-Guided Streaming Analysis of Large Octree Structures*](https://doi.org/10.1145/1507149.1507152).
It is a native C++20 application using OpenGL, GLFW, and Dear ImGui.

## Requirements

- Windows with an OpenGL 4.6-capable GPU and up-to-date graphics drivers.
- CMake 3.25 or newer, Ninja, GCC/G++, and Git.
- An internet connection during the first CMake configure, to fetch
  `tinyobjloader`, GLM, FastNoise2, stb, and xsimd.

The checked-in CMake presets select GCC/G++ and Ninja, and are intended to be
used from an MSYS2 **UCRT64** terminal with those tools available on `PATH`.
They are not Visual Studio project presets.

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

The build copies the shaders, resources, and meshes into the build output
directory. CMake downloads the external dependencies during configuration, so
the first configure can take longer than later builds.

## Important: shader path

Shader filenames in `src/application.hpp` currently use the absolute path
`C:/Users/jaros/Desktop/voxel_streaming/shaders`. As written, the application
will only find its shaders if the repository is located at that exact path.
If you cloned it somewhere else, either move the checkout there or update those
paths in the source before building. This restriction applies even though CMake
also copies shaders beside the executable.

## Troubleshooting

- If CMake cannot find `gcc`, `g++`, or Ninja, open the MSYS2 UCRT64 terminal
  and confirm the tools are on `PATH`.
- If dependency fetching fails, check the network connection and rerun the
  configure command.
- If the window fails to open or shader compilation fails, check that the GPU
  driver supports OpenGL 4.6 and that the shader path above is valid.
