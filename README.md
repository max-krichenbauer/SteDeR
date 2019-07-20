# SteDeR
Stereo Depth Reconstruction from a pair of stereo color images,
implemented as a single C++ header file.
Both CPU and GPU implementation.
The CPU implementation uses SIMD / SSE instructions for maximum performance.
The GPU implementations use either OpenGL (implemented as a shader for maximum compatibility) or OpenCL.
