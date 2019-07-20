/***********************************************************************************************//**
 * \file      steder_cl.h
 *            Stereo depth reconstruction: OpenCL version.
 *            Contains code for estimating the depth map of a stereo image pair.
 *            This file contains all the code necessary to use SteDeR.
 *            It can be included into any project without requiring further libraries.
 *            However, it does require OpenCL capabilities on the target platform.
 ***************************************************************************************************
 * \brief     Stereo depth reconstruction - OpenCL version.
 * \author    Max Krichenbauer (max@krichenbauer.de)
 * \version   0.2.1
 * \date      2015-03-17
 * \copyright Max Krichenbauer 2019
 * \license   GNU General Public License v3.0
 *            https://www.gnu.org/licenses/gpl-3.0.en.html
 *            THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *            "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 *            THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 *            PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 *            CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 *            EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 *            PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 *            PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
 *            OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *            (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *            OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************************/

#ifndef __STEDERCL_H
#define __STEDERCL_H

#include <GL/gl.h>
#include <CL/opencl.h>

#include <cstdlib>
#include <ctime>
#include <map>
#ifdef __APPLE__
#else
#include <malloc.h> // For allocating LUT
#endif

typedef unsigned char   uchar;			///< unsigned 8bit integer (byte)
typedef unsigned short  ushort;			///< unsigned 16bit (short) integer
typedef unsigned int    uint;			///< unsigned 32bit integer
typedef unsigned long   ulong;			///< unsigned 64bit (long) integer

#define STRING(X) #X

/**
 * Stereo Depth Reconstruction (StDeR) class.
 */
class SteDeRCL
{
public:
    //                                                                  ________________
    //_________________________________________________________________/ enum Error
    /**
     * Reconstructor error codes.
     */
    typedef enum Error {
        SUCCESS = 0                 ///< Success
        ,
        ERROR_NOT_INITIALIZED   = 1 ///< The SteDeRCL was not correctly initialized.
        ,
        ERROR_INVALID_PARAM     = 2 ///< Invalid parameter.
        ,
        ERROR_BAD_IMAGE         = 3 ///< Could not perform reconstruction because of bad image data.
        ,
        ERROR_MALLOC_FAIL       = 4 ///< Failure to acquire required memory.
        ,
        ERROR_OPENCL_FAIL       = 5 ///< Internal OpenCL error.
    } Error;   ///< Reconstructor error codes.

    SteDeRCL();   ///< Default constructor
    SteDeRCL(const SteDeRCL& cpy); ///< Copy constructor
    ~SteDeRCL();  ///< Destructor

    SteDeRCL& operator=(const SteDeRCL& o); ///< Assignment operator.

    int init(uint w, uint h, float disparity_limit=0.5f); ///< Initialize reconstructor.
    int uninit(); ///< Uninitialize, reset object and free allocated memory.


    float baseline;     ///< Left/Right camera displacement (baseline width).
    float focal_len;    ///< Focal length (assumed to be about the same for both cameras
    float max_depth;    ///< Maximum distance (for points at infinity).
    float z_far;        ///< Far clipping plane of the depth buffer.
    float z_near;       ///< Near clipping plane of the depth buffer.
    uint tex_img_left;  ///< OpenGL texture ID of the left image.
    uint tex_img_right; ///< OpenGL texture ID of the right image.
    uint tex_z_left;    ///< OpenGL texture ID of the left z-buffer.
    uint tex_z_right;   ///< OpenGL texture ID of the left z-buffer.

    uint getWidth();    ///< Get image width in pixels.
    uint getHeight();   ///< Get image height in pixels.
    uint getMaxDisp();  ///< Get maximum considered disparity.
    bool isInitialized();///< Check wether the object was initialized successfully.

    int operator()(const uchar* img_left, const uchar* img_right, float* depth_left, float* depth_right);   ///< Execute reconstruction.

private:
    bool initialized;   ///< Whether the Reconstructor was successfully initialized.

    cl_device_id        device;     ///< OpenCL ID of the device (GPU) we're using.
    cl_context          context;    ///< Device context to work in.
    cl_command_queue    queue;      ///< Command queue to GPU.
    cl_mem              imgL;       ///< Left image buffer.
    cl_mem              imgR;       ///< Right image buffer.
    cl_mem              zL;         ///< Left z-buffer.
    cl_mem              zR;         ///< Right z-buffer.
    cl_program          program;    ///< OpenCL program to run on the GPU.
    cl_kernel           kernel;     ///< Kernel containing the depth reconstruction code.

    uint w;         ///< Width of image in pixels.
    uint h;         ///< Height of image in pixels.
    
    uint max_disp;  ///< Maximum considered pixel disparity in pixels. The maximum breadth of the error volume (+1).
    uint* stencil;  ///< Breadth of the error volume at this point (x), excluding all disparities beyond.

    static char* kernel_source; ///< Kernel source code.
};

char* SteDeRCL::kernel_source = STRING(
#define N   5 \n
#define W   2 \n
__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST; \n
__kernel void main(__read_only image2d_t    imgL,
                   __read_only image2d_t    imgR,
                   __write_only image2d_t   zL,
                   int                      dir,
                   uint                     max_d,
                   float                    fxb,
                   float                    mA,
                   float                    B) {
    
    const int2 p0  = (int2)(get_global_id(0), get_global_id(1));
    uint best_err = 0xFFFFFF00;
    uint top_err[N];
    for (int i = N-1; i>=0; i--) {
        top_err[i] = 0xFFFFFF00;
    }
    const int4 l = read_imagei(imgL, sampler, p0);
    for (uint d=0; d<=max_d; d++) {
        const int4 r = read_imagei(imgR, sampler, p0+(int2)(dir*(int)d, 0));
        uint err = abs(l.x-r.x)
                 + abs(l.y-r.y)
                 + abs(l.z-r.z);
        err = (err<<8) | d;
        top_err[N-1] = min(top_err[N-1], err);
        for (uint i=N-1; i>0; i--) {
            int err_min = min(top_err[i], top_err[i-1]);
            int err_max = max(top_err[i], top_err[i-1]);
            top_err[i]   = err_max;
            top_err[i-1] = err_min;
        }
    }
    for (uint i=0; i<N; i++) {
        uint d = top_err[i] & 0x000000FF;
        uint err = 0;
        for (int x=W; x>=-W; x--) {
            for (int y=W; y>=-W; y--) {
                const int2 p = p0 + (int2)(x, y);
                const int4 l = read_imagei(imgL, sampler, p);
                const int4 r = read_imagei(imgR, sampler, p+(int2)(dir*(int)d, 0));
                err += abs(l.x-r.x)
                    + abs(l.y-r.y)
                    + abs(l.z-r.z);
            }
        }
        err = (err<<8) | d;
        best_err = min(best_err, err);
    }
    uint best_d = best_err & 0x000000FF;
    float depth = fxb / ((float)best_d);
    float4 z = (float4)(0.5f*(mA*depth + B) / depth + 0.5f, 0, 0, 0);
    write_imagef(zL, p0, z);
    //const int2 pr = (int2)(max(p0.x-(int)best_d,0) , p0.y);*/
    //write_imagef(zR, pr, z);*/
}
);

//																			________________________
//_________________________________________________________________________/   SteDeRCL()
/**
 * Default constructor (empty).
 */
inline SteDeRCL::SteDeRCL()
{
    this->initialized = false;
    this->device    = 0;
    this->context   = 0;
    this->queue     = 0;
    this->imgL      = 0;
    this->imgR      = 0;
    this->zL        = 0;
    this->zR        = 0;
    this->program   = 0;
    this->kernel    = 0;

    this->stencil     = 0;

    this->focal_len   = 339.768f;
    this->baseline    = 50.0f;
    this->max_depth   = 10000.0f;
    this->z_far       = 10000.0f;
    this->z_near      = 1.0f;
}

//																			________________________
//_________________________________________________________________________/   SteDeRCL(cpy)
/**
 * Default constructor (empty).
 */
inline SteDeRCL::SteDeRCL(const SteDeRCL& cpy)
{
    this->initialized = false;
    this->device    = 0;
    this->context   = 0;
    this->queue     = 0;
    this->imgL      = 0;
    this->imgR      = 0;
    this->zL        = 0;
    this->zR        = 0;
    this->program   = 0;
    this->kernel    = 0;
    this->focal_len   = cpy.focal_len;
    this->baseline    = cpy.baseline;
    this->max_depth   = cpy.max_depth;
    this->z_far       = cpy.z_far;
    this->z_near      = cpy.z_near;

    if(cpy.initialized) {
        this->init(cpy.w, cpy.h, float(cpy.max_disp)/float(cpy.w));
    }
}

//																			________________________
//_________________________________________________________________________/   ~SteDeRCL()
/**
 * Destructor
 */
inline SteDeRCL::~SteDeRCL()
{
    this->uninit();
}

//																			________________________
//_________________________________________________________________________/   operator=()
/**
 * Assignment operator.
 */
inline SteDeRCL& SteDeRCL::operator=(const SteDeRCL& o)
{
    this->uninit();

    this->focal_len   = o.focal_len;
    this->baseline    = o.baseline;
    this->max_depth   = o.max_depth;
    this->z_far       = o.z_far;
    this->z_near      = o.z_near;

    if(o.initialized) {
        this->init(o.w, o.h, float(o.max_disp)/float(o.w));
    }

    return *this;
}

//																			________________________
//_________________________________________________________________________/      uninit()
/**
 * Uninitialize, reset object and free allocated memory.
 */
inline int SteDeRCL::uninit()
{
    if (this->device) {
        /// \todo   Do I have to delete the device ID?
        this->device = 0;
    }
    if(this->context) {
        clReleaseContext(this->context);
        this->context = 0;
    }
    if(this->queue) {
        clReleaseCommandQueue(this->queue);
        this->queue = 0;
    }
    if(this->imgL) {
        clReleaseMemObject(this->imgL);
        this->imgL = 0;
    }
    if(this->imgR) {
        clReleaseMemObject(this->imgR);
        this->imgR = 0;
    }
    if(this->zL) {
        clReleaseMemObject(this->zL);
        this->zL = 0;
    }
    if(this->zR) {
        clReleaseMemObject(this->zR);
        this->zR = 0;
    }
    if(this->program) {
        clReleaseProgram(this->program);
        this->program = 0;
    }
    if(this->kernel) {
        clReleaseKernel(this->kernel);
        this->kernel = 0;
    }
    
    this->initialized = false;

    return SteDeRCL::SUCCESS;
}

//																			________________________
//_________________________________________________________________________/     init()
/**
 * Initialize reconstructor object.
 * \param   w                       Image width in pixels.
 * \param   h                       Image height in pixels.
 * \param   disparity_limit         Highest possible (considered) pixel disparity.
 * \return                          Zero on success, an error code on failure.
 */    
inline int SteDeRCL::init(uint w, uint h, float disparity_limit)
{
    if(!w || !h || disparity_limit <= 0.0f || disparity_limit >= 1.0f) {
        return ERROR_INVALID_PARAM;
    }

    // If we have any previously allocated memory, free if before continuing
    this->uninit();

    
    // Calculate internal memory / volume dimensions
    this->w = w;
    this->h = h;
    this->max_disp = uint(float(w)*disparity_limit);


    cl_int error; // Return error code

    // 1: Get the device ID of the primary graphics adapter
    cl_platform_id platform;
	cl_uint num_platforms, num_devices;
	clGetPlatformIDs(1, &platform, &num_platforms);
	if (!num_platforms) {
		return SteDeRCL::ERROR_OPENCL_FAIL;
	}
    error = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &this->device, &num_devices);
	if (error != CL_SUCCESS || num_devices != 1) {
		return SteDeRCL::ERROR_OPENCL_FAIL;
	}

    // 2: Create device context
    //cl_context_properties properties[] = {
    //    CL_GL_CONTEXT_KHR, (cl_context_properties)wglGetCurrentContext(), 
    //    CL_WGL_HDC_KHR, (cl_context_properties) wglGetCurrentDC(),
    //    CL_CONTEXT_PLATFORM, (cl_context_properties) platform, 
    //    0
    //};
    //this->context = clCreateContext(properties, 1, &this->device, 0, 0, &error);
    this->context = clCreateContext(0, 1, &this->device, 0, 0, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    // 3: Create message queue
    this->queue = clCreateCommandQueue(context, this->device, 0, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    // 4: Allocate memory
    // Create an OpenCL Image / texture and transfer data to the device
    cl_image_format image_format;
    image_format.image_channel_data_type = CL_UNSIGNED_INT8; // CL_UNORM_INT8;
    image_format.image_channel_order = CL_RGBA; // CL_RGB; 
    // this->imgL = clCreateBuffer(context, CL_MEM_READ_WRITE, w*h*sizeof(uchar)*3, 0, &error);
    this->imgL = clCreateImage2D(context, CL_MEM_READ_ONLY, &image_format, w, h, 0, 0, &error);
    //this->imgL = clCreateFromGLTexture2D(context, CL_MEM_READ_ONLY, GL_TEXTURE_2D, 0, this->tex_img_left, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    // this->imgR = clCreateBuffer(context, CL_MEM_READ_WRITE, w*h*sizeof(uchar)*3, 0, &error);
    this->imgR = clCreateImage2D(context, CL_MEM_READ_ONLY, &image_format, w, h, 0, 0, &error);
    //this->imgR = clCreateFromGLTexture2D(context, CL_MEM_READ_ONLY, GL_TEXTURE_2D, 0, this->tex_img_right, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    // Allocate z-Buffer
    image_format.image_channel_data_type = CL_FLOAT; // CL_UNORM_INT16;
    image_format.image_channel_order = CL_R; // CL_INTENSITY;
    // this->zL = clCreateBuffer(context, CL_MEM_READ_WRITE, w*h*sizeof(float), 0, &error);
    this->zL = clCreateImage2D(context, CL_MEM_WRITE_ONLY, &image_format, w, h, 0, 0, &error);
    //this->zL = clCreateFromGLTexture2D(context, CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0, this->tex_z_left, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    // this->zR = clCreateBuffer(context, CL_MEM_READ_WRITE, w*h*sizeof(float), 0, &error);
    this->zR = clCreateImage2D(context, CL_MEM_WRITE_ONLY, &image_format, w, h, 0, 0, &error);
    //this->zR = clCreateFromGLTexture2D(context, CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0, this->tex_z_right, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    // 5: Create the program
    this->program = clCreateProgramWithSource(context, 1, (const char**)(&kernel_source), NULL, &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    // 6: Build the program
    error = clBuildProgram(program, 1, &device, 0, 0, 0);
    if (error != CL_SUCCESS) {
        printf("Could not build program:\n");
        size_t len;
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, 0, &len);
        char* msg;
        msg = (char*)malloc(len+1);
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, len+1, msg, 0);
        msg[len] = '\0';
        printf(msg);
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    // 7: Create the kernel
    this->kernel = clCreateKernel(program, "main", &error);
    if (error != CL_SUCCESS) {
        this->uninit();
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    this->initialized = true;
    return SUCCESS;
}




//																			________________________
//_________________________________________________________________________/    operator()
/**
 * Execute reconstruction.
 * The object must be initialized prior to calling this function.
 * \param   img_left    Pointer to left image buffer.
 * \param   img_right   Pointer to right image buffer.
 * \param   depth_left  Pointer to left depth buffer to be written (must be width*height floats).
 * \param   depth_right Pointer to right depth buffer to be written (must be width*height floats).
 * \return              Zero on success, an error code on failure.
 */
inline int SteDeRCL::operator()(const uchar* img_left, const uchar* img_right, float* depth_left, float* depth_right)
{
    if(!this->initialized)
        return ERROR_NOT_INITIALIZED;
    
    cl_int e; // error return code

    // Upload the images to the GPU
    size_t origin[3] ={ 0, 0, 0 };
    size_t region[3] ={ w, h, 1 };
    //e = clEnqueueWriteBuffer(this->queue, this->imgL, CL_TRUE, 0, w*h*sizeof(uchar)*3, img_left, 0, 0, 0);
    e = clEnqueueWriteImage(this->queue, this->imgL, CL_TRUE, origin, region, w*4, 0, img_left, 0, 0, 0);
    if (e != CL_SUCCESS) {
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    //e = clEnqueueWriteBuffer(this->queue, this->imgR, CL_TRUE, 0, w*h*sizeof(uchar)*3, img_right, 0, 0, 0);
    e = clEnqueueWriteImage(this->queue, this->imgR, CL_TRUE, origin, region, w*4, 0, img_right, 0, 0, 0);
    if (e != CL_SUCCESS) {
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    
    //e = clEnqueueAcquireGLObjects(queue, 1, &this->imgL,0,0,0);
    //e = clEnqueueAcquireGLObjects(queue, 1, &this->imgR,0,0,0);
    //e = clEnqueueAcquireGLObjects(queue, 1, &this->zL,0,0,0);
    //e = clEnqueueAcquireGLObjects(queue, 1, &this->zR,0,0,0);

    // size_t globalsize[3] ={ (w*h)+ (512-((w*h)%512)), 1, 1 }; // round up to multiple of localsize
    size_t globalsize[3] ={ w, h, 1 }; // round up to multiple of localsize
    // size_t localsize[3]  ={ 512, 1, 1 }; // less than or equal to CL_DEVICE_MAX_WORK_GROUP_SIZE from clGetDeviceInfo, each less than CL_DEVICE_MAX_WORK_ITEM_SIZES[i]

    // Set parameters and execute
    cl_int dir; // for search direction
    cl_uint max_d = this->max_disp;
    e = clSetKernelArg(kernel, 4, sizeof(cl_uint), &max_d);
    if (e != CL_SUCCESS) {
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    cl_float fxb = focal_len * baseline; // for depth calculation
    // __m128 fxb = _mm_set_ps1(focal_len * baseline);
    e = clSetKernelArg(kernel, 5, sizeof(cl_float), &fxb);
    if (e != CL_SUCCESS) {
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    //float A = -(z_far+z_near) / (z_far - z_near);
    cl_float mA = -(-(z_far+z_near) / (z_far - z_near));
    //__m128 mA = _mm_set_ps1(-(-(z_far+z_near) / (z_far - z_near)));
    e = clSetKernelArg(kernel, 6, sizeof(cl_float), &mA);
    if (e != CL_SUCCESS) {
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    cl_float B = -2.0f*z_far*z_near / (z_far - z_near);
    //__m128 B = _mm_set_ps1(-2.0f*z_far*z_near / (z_far - z_near));
    e = clSetKernelArg(kernel, 7, sizeof(cl_float), &B);
    if (e != CL_SUCCESS) {
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    clSetKernelArg(kernel, 0, sizeof(cl_mem), &this->imgL);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &this->imgR);
    clSetKernelArg(kernel, 2, sizeof(cl_mem), &this->zL);
    dir = -1;
    clSetKernelArg(kernel, 3, sizeof(cl_uint), &dir);
    e = clEnqueueNDRangeKernel(queue, kernel, 2, 0, globalsize, /*localsize*/ 0, 0, 0, 0);
    if (e != CL_SUCCESS) {
        printf("Could not clEnqueueNDRangeKernel: %d\n",e);
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    clSetKernelArg(kernel, 0, sizeof(cl_mem), &this->imgR);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &this->imgL);
    clSetKernelArg(kernel, 2, sizeof(cl_mem), &this->zR);
    dir = 1;
    clSetKernelArg(kernel, 3, sizeof(cl_uint), &dir);
    e = clEnqueueNDRangeKernel(queue, kernel, 2, 0, globalsize, /*localsize*/ 0, 0, 0, 0);
    if (e != CL_SUCCESS) {
        printf("Could not clEnqueueNDRangeKernel: %d\n",e);
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    e = clFlush(queue);
    if (e != CL_SUCCESS) {
        printf("clFlush returned %d\n",e);
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    e = clFinish(queue); // wait for commands to finish
    if (e != CL_SUCCESS) {
        printf("clFinish failed: %d\n",e);
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }

    // Download the z-buffers from the GPU
    //e =  clEnqueueReadBuffer (this->queue, this->zL, CL_TRUE, 0, w*h*sizeof(float), depth_left, 0, 0, 0);
    e = clEnqueueReadImage(this->queue, this->zL, CL_TRUE, origin, region, w*sizeof(float), 0, depth_left, 0, 0, 0);
    if (e != CL_SUCCESS) {
        printf("Failed to clEnqueueReadImage: %d\n", e);
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    //e =  clEnqueueReadBuffer (this->queue, this->zR, CL_TRUE, 0, w*h*sizeof(float), depth_right, 0, 0, 0);
    e = clEnqueueReadImage(this->queue, this->zR, CL_TRUE, origin, region, w*sizeof(float), 0, depth_right, 0, 0, 0);
    if (e != CL_SUCCESS) {
        printf("Failed to clEnqueueReadImage: %d\n", e);
        return SteDeRCL::ERROR_OPENCL_FAIL;
    }
    
    //e = clEnqueueReleaseGLObjects(queue, 1, &this->imgL,0,0,0);
    //e = clEnqueueReleaseGLObjects(queue, 1, &this->imgR,0,0,0);
    //e = clEnqueueReleaseGLObjects(queue, 1, &this->zL,0,0,0);
    //e = clEnqueueReleaseGLObjects(queue, 1, &this->zR,0,0,0);

    return SUCCESS;
}



//																			________________________
//_________________________________________________________________________/      getWidth()
/**
 * Get image width in pixels.
 */
inline uint SteDeRCL::getWidth()
{
    return this->w;
}
//																			________________________
//_________________________________________________________________________/     getHeight()
/**
 * Get image height in pixels.
 */
inline uint SteDeRCL::getHeight()
{
    return this->h;
}
//																			________________________
//_________________________________________________________________________/      getMaxDisp()
/**
 * Get maximum considered disparity.
 */
inline uint SteDeRCL::getMaxDisp()
{
    return this->max_disp;
}
//																			________________________
//_________________________________________________________________________/    isInitialized()
/**
 * Check wether the object was initialized successfully.
 */
inline bool SteDeRCL::isInitialized()
{
    return this->initialized;
}

#endif//__STEDERCL_H
