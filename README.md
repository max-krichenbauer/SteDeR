# SteDeR
Stereo Depth Reconstruction from a pair of stereo color images,
implemented as a single C++ header file.
Both CPU and GPU implementation.
The CPU implementation uses SIMD / SSE instructions for maximum performance.
The GPU implementations use either OpenGL (implemented as a shader for maximum compatibility) or OpenCL.

## [steder.h](steder.h)
Stereo color image depth reconstruction on the CPU, using Intel SSE SIMD instruction and an optimized memory layout for high performance.
The whole code is included in the single header file.
Just create a ```SteDeR``` object, initialize it with ```init(image_width, image_height, disparity_limit)``` where disparity_limit is the maximum pixel disparity (relative to image width) that you wish to allow, and use the ```()``` operator on the SteDeR object to receive floating point depth buffers.

Usage example:
```
// Load images
uchar *left_image, *right_image;
left_image = load_image("C:/path/to/image/left.png");
right_image = load_image("C:/path/to/image/right.png");

// Create and initialize SteDeR
SteDeR s;
int error = s.init(left_width,left_height, 0.2f);
if(error) {
    printf("Failed to initialize SteDeR (%i).\n",error);
    return -1;
}

// Allocate depth buffers
float* zL = (float*) malloc(left_width * left_height * sizeof(float));
float* zR = (float*) malloc(left_width * left_height * sizeof(float));

// Perform depth reconstruction
s(left_image, right_image, zL, zR);

```

