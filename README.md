# SteDeR
Stereo Depth Reconstruction from a pair of stereo color images, implemented as a single C++ header file.

Both CPU and GPU implementation.

The CPU implementation uses SIMD / SSE instructions for maximum performance.

The GPU implementations use either OpenGL (implemented as a shader for maximum compatibility) or OpenCL.

## [steder.h](steder.h)
Stereo color image depth reconstruction on the CPU, using Intel SSE SIMD instruction and an optimized memory layout for high performance.

The whole code is included in the single header file.

Just create a ```SteDeR``` object, initialize it with ```init(image_width, image_height, disparity_limit)``` where disparity_limit is the maximum pixel disparity (relative to image width) that you wish to allow, and use the ```()``` operator on the SteDeR object to receive floating point depth buffers.

Usage example:
```
// Load color stereo images
unsigned char *left_image, *right_image;
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

## [steder_gl.h](steder_gl.h)
Stereo color image depth reconstruction on the GPU by using OpenGL shaders.

The whole code is included in the single header file. ```steder.h``` is not required.

SteDeR_GL uses OpenGL shaders (GLSL) to calculate the depth from the stereo image pair on the GPU by rendering it to a OpenGL floating point render target.

This is very handy if you wish to use the depth buffer for later rendering. For example to insert virtual 3DCG objects into a stereo video stream such as a video see-through AR HMD.

Note that you need have an intialized OpenGL rendering contect for running SteDeR_GL.

Usage example:
```
// Load color stereo images
unsigned char *left_image, *right_image;
left_image = load_image("C:/path/to/image/left.png");
right_image = load_image("C:/path/to/image/right.png");

// Initialize OpenGL context:
int argc = 0;
glutInit(&argc, 0);
glutInitWindowPosition(0, 0);
glutInitWindowSize(left_width,left_height);
glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
int window = glutCreateWindow("StDeR - Test");
glewInit();

// Prepare textures and render targets:
GLuint framebuffer_left, framebuffer_right;
GLuint framebuffer_left_tex, framebuffer_right_tex;
GLuint depthbuffer_left, depthbuffer_right;
GLuint depthbuffer_left_tex, depthbuffer_right_tex;
glActiveTexture(GL_TEXTURE0);
genFramebuffer(framebuffer_left, framebuffer_left_tex, depthbuffer_left, depthbuffer_left_tex, left_width, left_height);
genFramebuffer(framebuffer_right, framebuffer_right_tex, depthbuffer_right, depthbuffer_right_tex, left_width, left_height);    
glBindTexture(GL_TEXTURE_2D, framebuffer_left_tex);
glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, left_width, left_height, 0, GL_RGB, GL_UNSIGNED_BYTE, left_img);
glBindTexture(GL_TEXTURE_2D, framebuffer_right_tex);
glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, left_width, left_height, 0, GL_RGB, GL_UNSIGNED_BYTE, right_img);

// Create SteDeR_GL object and set the textures and render targets:
SteDeRGL s;
s.imgL.fb = framebuffer_left;
s.imgR.fb = framebuffer_right;
s.imgL.tx = framebuffer_left_tex;
s.imgR.tx = framebuffer_right_tex;
s.imgL.vp.x = s.imgR.vp.x = 0;
s.imgL.vp.y = s.imgR.vp.y = 0;
s.imgL.vp.w = s.imgR.vp.w = left_width;
s.imgL.vp.h = s.imgR.vp.h = left_height;

int error = s.init(image_width, image_height, 0.2f);
if (error) {
    printf("Failed to initialize SteDeRGL (%i).\n",error);
    return -1;
}

// Execute stereo depth reconstruction:
error = s();

// Optional: download the depth buffers from the GPU:
glBindTexture(GL_TEXTURE_2D, depthbuffer_left_tex);
glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, zL);
glBindTexture(GL_TEXTURE_2D, depthbuffer_right_tex);
glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, zR);
```
