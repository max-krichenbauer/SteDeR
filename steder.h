/***********************************************************************************************//**
 * \file      steder.h
 *            Stereo depth reconstruction.
 *            Contains code for estimating the depth map of a stereo image pair.
 *            This file contains all the code necessary to use SteDeR.
 *            It can be included into any project without requiring further libraries.
 ***************************************************************************************************
 * \brief     Stereo depth reconstruction.
 * \author    Max Krichenbauer (max@krichenbauer.de)
 * \version   0.2.0
 * \date      2015-03-11
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

/***********************************************************************************************//**
 *! \mainpage SteDeR (2014)
 *
 * \section sec_intro Introduction
 * The Stereo Depth Reconstruction module (SteDeR) is a stand-alone module to estimate depth
 * from stereo image correspondence. It is aimed at real-time application and focuses on
 * on performance rather than a precise result.
 * Developed by Max Krichenbauer (max@krichenbauer.de).
 *
 * \section sec_usage Usage
 * SteDeR is implemented as a stand-alone self-contained include file (steder.h)
 * that can be included into any C++ project. No further library is required.
 * Before using the SteDeR object, it must be initialized with a call to the
 * init() function. Furthermore, the public variables focal_len,
 * baseline and max_depth should be set according to the image parameters.
 * After that, images can be fed to the object with the reconstruct() method.
 * SteDeR expects undistorted rectified images, where correspondence pixels
 * are expected to lie on the same pixel scan-line (y-coordinate).
 * Please undistort and rectify your images prior to calling reconstruct().
 * Currently only supports RGB images.
 *
 * \section sec_memory Memory Layout
 * For optimal performance, a special memory model was developed
 * that allows fast access, reducing memory page faults.
 * Firstly, this affects the images, which are internally transformed
 * to have a more helpful structure by interleaving four lines.
 * 
 */

#ifndef __STEDER_H
#define __STEDER_H

// #define __STEDER_DEBUG

#include <cstdlib>
#include <ctime>
#include <map>
#include <xmmintrin.h>
#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#ifdef __APPLE__
#else
#include <malloc.h>
#endif

typedef unsigned char   uchar;			///< unsigned 8bit integer (byte)
typedef unsigned short  ushort;			///< unsigned 16bit (short) integer
typedef unsigned int    uint;			///< unsigned 32bit integer
typedef unsigned long   ulong;			///< unsigned 64bit (long) integer


/**
 * Stereo Depth Reconstruction (StDeR) class.
 */
class SteDeR
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
        ERROR_NOT_INITIALIZED   = 1 ///< The SteDeR was not correctly initialized.
        ,
        ERROR_INVALID_PARAM     = 2 ///< Invalid parameter.
        ,
        ERROR_BAD_IMAGE         = 3 ///< Could not perform reconstruction because of bad image data.
        ,
        ERROR_MALLOC_FAIL       = 4 ///< Failure to acquire required memory.
    } Error;   ///< Reconstructor error codes.

    SteDeR();   ///< Default constructor
    SteDeR(const SteDeR& cpy); ///< Copy constructor
    ~SteDeR();  ///< Destructor

    SteDeR& operator=(const SteDeR& o); ///< Assignment operator.

    int init(uint w, uint h, float disparity_limit=0.5f); ///< Initialize reconstructor.
    int uninit(); ///< Uninitialize, reset object and free allocated memory.


    float baseline;     ///< Left/Right camera displacement (baseline width).
    float focal_len;    ///< Focal length (assumed to be about the same for both cameras
    float max_depth;    ///< Maximum distance (for points at infinity).
    float z_far;        ///< Far clipping plane of the depth buffer.
    float z_near;       ///< Near clipping plane of the depth buffer.
    uint  stencil_tolerance; ///< Tolerance of disparity changes between adjacent bands.

    uint getWidth();    ///< Get image width in pixels.
    uint getHeight();   ///< Get image height in pixels.
    uint getMaxDisp();  ///< Get maximum considered disparity.
    bool isInitialized();///< Check wether the object was initialized successfully.

    int operator()(const uchar* img_left, const uchar* img_right, float* depth_left, float* depth_right); ///< Execute reconstruction.

    static void RGBtoLAB(uchar _r, uchar _g, uchar _b, uchar& _L, uchar& _A, uchar& _B);

private:
    bool initialized;   ///< Whether the Reconstructor was successfully initialized.

    uchar*          imgL;   ///< Current left image lines, transposed.
    uchar*          imgR;   ///< Current right image lines, transposed.
    uint*           err;    ///< Dynamic Programming error matrix.

    uint*           dispL; ///< Disparity map as seen by left image.
    uint*           dispR; ///< Disparity map as seen by right image.

    uint w;         ///< Width of image in pixels.
    uint h;         ///< Height of image in pixels.
    uint h4;        ///< Padded height: adding rows to image to make height divisible by 4.
    uint max_disp;  ///< Maximum considered pixel disparity in pixels. The maximum breadth of the error volume (+1).
    uint* stencil;  ///< Breadth of the error volume at this point (x), excluding all disparities beyond.

    std::map<void*,void*> alignedMemoryPointers;   ///< Structure to remember base pointers for platform independent aligned memory allocation.
    void*   allocateAlignedMemory(size_t size, size_t alignment); ///< Like malloc(), but platform independent and aligned.
    void    freeAlignedMemory(void* m); ///< free() for memory allocated with the other function.

    void prepLineRGB(const uchar* src, uint y, uchar* dst); ///< Prepare RGB image line (4-line set) for error calculation.
    void calcErrM();  ///< Calculate and accumulate cost matrix. (Based on imgL, imgR)
    void findPath();  ///< Find the lowest-cost-path through the error matrix, store results in dispL/R
    void calcDepth(int dy, float* zL, float* zR);  ///< Calculate depth map from disparities.

    __m128i pixelDiff(__m128i l, __m128i  r); ///< Calculate pixel error, including vertical smoothing.
};

//																			________________________
//_________________________________________________________________________/   SteDeR()
/**
 * Default constructor (empty).
 */
inline SteDeR::SteDeR()
{
    this->initialized = false;
    this->err       = 0;
    this->imgL        = 0;
    this->imgR        = 0;
    this->dispL       = 0;
    this->dispR       = 0;
    this->stencil     = 0;

    this->focal_len   = 339.768f;
    this->baseline    = 50.0f;
    this->max_depth   = 10000.0f;
    this->z_far       = 10000.0f;
    this->z_near      = 1.0f;

    this->stencil_tolerance = 20;
}

//																			________________________
//_________________________________________________________________________/   SteDeR(cpy)
/**
 * Default constructor (empty).
 */
inline SteDeR::SteDeR(const SteDeR& cpy)
{
    this->initialized = false;
    this->err         = 0;
    this->imgL        = 0;
    this->imgR        = 0;
    this->dispL       = 0;
    this->dispR       = 0;
    this->stencil     = 0;
    this->focal_len   = cpy.focal_len;
    this->baseline    = cpy.baseline;
    this->max_depth   = cpy.max_depth;
    this->z_far       = cpy.z_far;
    this->z_near      = cpy.z_near;
    this->stencil_tolerance = cpy.stencil_tolerance;

    if(cpy.initialized) {
        this->init(cpy.w, cpy.h, float(cpy.max_disp)/float(cpy.w));
    }
}

//																			________________________
//_________________________________________________________________________/   ~SteDeR()
/**
 * Destructor
 */
inline SteDeR::~SteDeR()
{
    this->uninit();
}

//																			________________________
//_________________________________________________________________________/   operator=()
/**
 * Assignment operator.
 */
inline SteDeR& SteDeR::operator=(const SteDeR& o)
{
    this->uninit();

    this->focal_len   = o.focal_len;
    this->baseline    = o.baseline;
    this->max_depth   = o.max_depth;
    this->z_far       = o.z_far;
    this->z_near      = o.z_near;
    this->stencil_tolerance = o.stencil_tolerance;

    if(o.initialized) {
        this->init(o.w, o.h, float(o.max_disp)/float(o.w));
    }

    return *this;
}

//																			________________________
//_________________________________________________________________________/ allocateAlignedMemory()
/**
 * Like malloc(), but platform independent and aligned.
 * \param   size        Size of memory block to allocate (in bytes)
 * \param   alignment   Memory alignment in bytes.
 * \return              Pointer to the allocated memory location
 *                      or null if no memory could be allocated.
 */
inline void* SteDeR::allocateAlignedMemory(size_t size, size_t alignment)
{
#if defined WIN32       // WIN32
    return _aligned_malloc(size, alignment);
#elif defined __linux__ // Linux
    return memalign(alignment, size);
#else                   // Undefined platform:
    // Use platform independent hack:
    // Allocate additional (alignment-1) bytes of memory
    // and return a pointer that is aligned within this greater chunk 
    void* p = malloc(size + alignment-1);
    if (!p)
        return 0;
    size_t mod = size_t(p)%alignment;
    if (!mod)
        return p; // was lucky: it's already aligned
    void* p_aligned = (void*)(size_t(p)+(alignment-mod)); // move up to alignment
    this->alignedMemoryPointers[p_aligned] = p; // Save for later free()
    return p_aligned;
#endif
}

//																			________________________
//_________________________________________________________________________/  freeAlignedMemory(m)
/**
 * free() for memory allocated with the other function.
 * \param   m   Pointer to memory segment allocated with allocateAlignedMemory()
 */
inline void SteDeR::freeAlignedMemory(void* m)
{
    if(!m)
        return;

#if defined WIN32           // WIN32
    _aligned_free(m);
#elif defined __linux__     // Linux
    free(m);
#else                       // Undefined platform
    if (this->alignedMemoryPointers.count(m)==0) {
        free(m); // was already aligned
    } else {
        free(this->alignedMemoryPointers[m]);
        this->alignedMemoryPointers.erase(m);
    }
#endif
}

//																			________________________
//_________________________________________________________________________/      uninit()
/**
 * Uninitialize, reset object and free allocated memory.
 */
inline int SteDeR::uninit()
{
    if(this->err) {
        this->freeAlignedMemory(this->err);
        this->err = 0;
    }

    if(this->stencil) {
        this->freeAlignedMemory(this->stencil);
        this->stencil = 0;
    }

    if(this->imgL) {
        this->freeAlignedMemory(this->imgL);
        this->imgL = 0;
    }

    if(this->imgR) {
        this->freeAlignedMemory(this->imgR);
        this->imgR = 0;
    }

    if(this->dispL){
        this->freeAlignedMemory(this->dispL);
        this->dispL = 0;
    }

    if(this->dispR){
        this->freeAlignedMemory(this->dispR);
        this->dispR = 0;
    }

    this->initialized = false;

    return SteDeR::SUCCESS;
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
inline int SteDeR::init(uint w, uint h, float disparity_limit)
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
    this->h4 = h-(h%4); // height divisible by 4 (by cutting away bottom)

    // Allocate image line buffers: the internal buffers are
    // - holding 4 lines
    // - RGB0 (4byte per pixel),
    // - transposed (y-major)
    // +---+---+---+---+---+---+---+---+---+---+---+---+-----+---+---+---+---+---+---+---+---+----
    // |0,0|0,1|0,2|0,3|1,0|1,1|1,2|1,3|2,0|2,1|2,2|2,3| ... |0,4|0,5|0,6|0,7|1,4|1,5|1,6|1,7|...
    // +---+---+---+---+---+---+---+---+---+---+---+---+-----+---+---+---+---+---+---+---+---+----
    //                                                  ,--------- height in lines
    //                                                 /   ,------ width of the image
    //                                                /   /   ,--- four bytes per pixel
    this->imgL = (uchar*) this->allocateAlignedMemory(4 * w * 4 * sizeof(uchar), 16);
    this->imgR = (uchar*) this->allocateAlignedMemory(4 * w * 4 * sizeof(uchar), 16);
    if(!this->imgR || !this->imgL) {
        this->uninit();
        return ERROR_MALLOC_FAIL;
    }
    
    // Allocate DP error matrix: we perform 4 scanlines per time
    // Since we always use packets of 4 ints (4 lines at a time), every row will be 16byte aligned
    //                                                   ,-------------- Width of error volume = image width
    //                                                  /      ,-------- Maximum breadth of the volume: disparity levels
    //                                                 /      /    ,---- max_disp is the highest possible value,
    //                                                /      /    /      plus one extra slize for filtering (moving down)
    //                                               /      /    /   ,-- Four lines interleaved (4 pixel-errors per element)
    this->err = (uint*) this->allocateAlignedMemory(w*(max_disp+2)* 4 * sizeof(uint), 16);
    if(!this->err) {
        this->uninit();
        return ERROR_MALLOC_FAIL;
    }

    // Stencil values
    // For each x, the stencil variable defines the breadth of the error volume
    // (how many disparities are to be considered.
    this->stencil = (uint*) this->allocateAlignedMemory(w * sizeof(uint), 16);
    if(!this->stencil) {
        this->uninit();
        return ERROR_MALLOC_FAIL;
    }

    // Allocate disparity maps
    // - for each set four lines
    // - stores the disparity value per respective x-value.
    //                                                  ,------ Width of image (minor dimension)
    //                                                 /   ,--- 4 pixels per pack from 4 lines
    this->dispL = (uint*) this->allocateAlignedMemory(w * 4 * sizeof(uint), 16);
    this->dispR = (uint*) this->allocateAlignedMemory(w * 4 * sizeof(uint), 16);
    if(!this->dispR || !this->dispL) {
        this->uninit();
        return ERROR_MALLOC_FAIL;
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
inline int SteDeR::operator()(const uchar* img_left, const uchar* img_right, float* depth_left, float* depth_right)
{
    if(!this->initialized)
        return ERROR_NOT_INITIALIZED;

    if(!img_left || !img_right || !depth_left || !depth_right)
        return ERROR_INVALID_PARAM;

    

    #ifdef __STEDER_DEBUG
        clock_t t_start, t_end, t_task_start, t_task_prepImg, t_task_calcErrM, t_task_findPath, t_task_calcDepth;
        t_task_prepImg = t_task_calcErrM = t_task_findPath = t_task_calcDepth = 0;
        t_start = t_task_start = clock();
        for(int frame=0; frame<100; frame++) {
    #endif    
    
    uint pass = 0;
    for(int y=int(h4-4); y>=0; y-=4) { // four lines per step
        // Every eight passes: reset the stencil
        if((pass++)%8 == 0) {
            for(int x= int(w-1); x>=0; x--) {
                stencil[x] = (x<int(max_disp)) ? x+1 : max_disp+1; // number of disparitiy values for this line
            }
        }

        this->prepLineRGB(img_left,  y, this->imgL);
        this->prepLineRGB(img_right, y, this->imgR);
        
        #ifdef __STEDER_DEBUG
            t_task_prepImg += clock() - t_task_start;
            t_task_start = clock();
        #endif

        calcErrM();

        #ifdef __STEDER_DEBUG
            t_task_calcErrM += clock() - t_task_start;
            t_task_start = clock();
        #endif

        findPath();

        #ifdef __STEDER_DEBUG
            t_task_findPath += clock() - t_task_start;
            t_task_start = clock();
        #endif

        calcDepth(y, depth_left, depth_right);

        #ifdef __STEDER_DEBUG
            t_task_calcDepth += clock() - t_task_start;
            t_task_start = clock();
        #endif
    }

    #ifdef __STEDER_DEBUG
    } // } for(int frame=0; frame<100; frame++);

    t_end = clock();

    printf("----------------------------------\n");
    printf("100 frames\n");
    printf("Image size: %ux%u; max_disp=%u; stencil_tolterance=%u\n",w,h,max_disp,stencil_tolerance);
    printf("prepImg:\t\t%f s\n", float(t_task_prepImg)/CLOCKS_PER_SEC);
    printf("calcErrM:\t\t%f s\n", float(t_task_calcErrM)/CLOCKS_PER_SEC);
    printf("findPath:\t\t%f s\n", float(t_task_findPath)/CLOCKS_PER_SEC);
    printf("calcDepth:\t\t%f s\n", float(t_task_calcDepth)/CLOCKS_PER_SEC);
    printf("----------------------------------\nTOTAL TIME:\t\t%f s\n", float(t_end-t_start)/CLOCKS_PER_SEC);
    printf("----------------------------------\n");
    #endif

    return SUCCESS;
}

//																			________________________
//_________________________________________________________________________/    RGBtoLAB()
/**
 * Convert RGB colors to L*ab color space, based on some arbitrary assumptions
 * about the RGB color provided.
 * \param   _r    Input pixel red component.
 * \param   _g    Input pixel green component.
 * \param   _b    Input pixel blue component.
 * \param   _L    Output color L* component.
 * \param   _A    Output color a* component.
 * \param   _B    Output color b* component.
 */
inline void SteDeR::RGBtoLAB(uchar _r, uchar _g, uchar _b, uchar& _L, uchar& _A, uchar& _B)
{
    float r = float(_r)/255.0f;
    float g = float(_g)/255.0f;
    float b = float(_b)/255.0f;
    float L,A,B;
    float X, Y, Z, fx, fy, fz, xr, yr, zr;
    float eps = 216.f/24389.f;
    float k = 24389.f/27.f;

    float Xr = 0.964221f;  // reference white D50
    float Yr = 1.0f;
    float Zr = 0.825211f;

    // assuming sRGB (D65)
    if (r <= 0.04045f)
        r = r/12.0f;
    else
        r = pow((r+0.055f)/1.055f,2.4f);

    if (g <= 0.04045f)
        g = g/12.0f;
    else
        g = pow((g+0.055f)/1.055f,2.4f);

    if (b <= 0.04045f)
        b = b/12.0f;
    else
        b = (float) pow((b+0.055f)/1.055f,2.4f);


    X =  0.436052025f*r     + 0.385081593f*g + 0.143087414f *b;
    Y =  0.222491598f*r     + 0.71688606f *g + 0.060621486f *b;
    Z =  0.013929122f*r     + 0.097097002f*g + 0.71418547f  *b;

    // [ R ]   [  3.240479 -1.537150 -0.498535 ]   [ X ]
    // [ G ] = [ -0.969256  1.875992  0.041556 ] * [ Y ]
    // [ B ]   [  0.055648 -0.204043  1.057311 ]   [ Z ]
    
    // [ X ]   [  0.412453  0.357580  0.180423 ]   [ R ] **
    // [ Y ] = [  0.212671  0.715160  0.072169 ] * [ G ]
    // [ Z ]   [  0.019334  0.119193  0.950227 ]   [ B ]

    // XYZ to Lab
    xr = X/Xr;
    yr = Y/Yr;
    zr = Z/Zr;

    if ( xr > eps )
        fx =  pow(xr, 1.0f/3.0f);
    else
        fx = ((k * xr + 16.0f) / 116.0f);

    if ( yr > eps )
        fy = pow(yr, 1.0f/3.0f);
    else
        fy = ((k * yr + 16.0f) / 116.0f);

    if ( zr > eps )
        fz = pow(zr, 1.0f/3.0f);
    else
        fz = ((k * zr + 16.0f) / 116.0f);

    L = ( 116.0f * fy ) - 16.0f; // [0 - 100]
    A = 500.0f*(fx-fy); // [-128 - 128]
    B = 200.0f*(fy-fz);

    // for int:
    _L = (uchar)(2.55f* L + 0.5f);
    _A = (uchar)(A + 127.5f);
    _B = (uchar)(B + 127.5f);
}

//																			________________________
//_________________________________________________________________________/     prepImgRGB()
/**
 * Prepare RGB image for processing:
 * <ul>
 *    <li> Always one set of 4 lines at a time.
 *    <li> Convert to RGBA so that every pixel is exactly 4 bytes long.</li>
 *    <li> Tranposed / y-major, interleaving 4 adjacent lines per x-packets.</li>
 * </ul>
 * 
 * The resulting memory layout is illustrated below, showing the buffer from the beginning.
 * Every packet is a RGBA pixel (4byte long). The number in the box describes the
 * image location (x,y).
 * <pre>
 * +---+---+---+---+---+---+---+---+---+---+---+---+-----+---+---+---+---+---+---+---+---+----
 * |0,0|0,1|0,2|0,3|1,0|1,1|1,2|1,3|2,0|2,1|2,2|2,3| ... |0,4|0,5|0,6|0,7|1,4|1,5|1,6|1,7|...
 * +---+---+---+---+---+---+---+---+---+---+---+---+-----+---+---+---+---+---+---+---+---+----
 * </pre>
 *
 * \todo            SSE optimized version.
 * 
 * \param   src     Source image (w*h), RGB image.
 * \param   y       Base-y of the lowest line number (y+0 to y+3 will be processed).
 * \param   dst     Destination image buffer (w * h4), RGBA buffer, 16byte aligned.
 */

inline void SteDeR::prepLineRGB( const uchar* src, uint by, uchar* dst)
{
    /*
    for(int x=int(w-1); x>=0; x--) {
        for(int dy=3; dy>=0; dy--) {
            const uchar* s = &src[((dy+by)*w + x)*3];
            //int dy = y%4;      // packet index (0..4)
            //int by = (y-dy)/4; // line base address
            //                  ,-------- Packet address (x)
            //                 / ,------- Packet length (4 pixels)
            //                / /   ,---- Index within packet
            //               / /   /   ,- Bytes per pixel
            uchar* d =&dst[(x*4 + dy)*4];
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
            d[3] = 0x00;
        }
    }*/
    const uchar* s = &src[((by+3)*w + w-1)*3];
    for(int dy=3; dy>=0; dy--) {
        //                   ,-------- Packet address (x)
        //                  /    ,------- Packet length (4 pixels)
        //                 /    /   ,---- Index within packet
        //               _/_   /   /   ,- Bytes per pixel
        int* d=(int*)&dst[((w-1)*4 + dy)*4];
        for(int x=int(w-1); x>=0; x--) {
            // RGB-comparison
            *d = (int(s[2])<<16) | (int(s[1])<<8) | int(s[0]); 

            // LAB-comparison
            //uchar l, a, b; 
            //SteDeR::RGBtoLAB(s[0],s[1],s[2],l,a,b);
            //*d = (int(l)<<16) | (int(a)<<8) | int(b);
            
            // Next pixel
            s-=3;
            d-=4;
        }
    }
}



//																			________________________
//_________________________________________________________________________/   accumulateErrM()
/**
 * Calculate a accumulative cost matrix for dynamic programming (instead of normal pixel error).
 * Uses the error volume, but calculates accumulative error values.
 * Each error value includes the lowest prior error (-1px in left image, -1px in right image,
 * -1px in both images).
 * Memory layout of the cost volume is changed for optimization:
 * line major (ie. image column major).
 */
inline void SteDeR::calcErrM()
{
    // Prepare running pointer (starting at end of volume)
    __m128i* e=(__m128i*)err;
    for(int x=int(w-1); x>=0; x--) {
        e += stencil[x]; // Jump over this line...
    } // Now e points one packet beyond the error volume
    e--; // Now: last element (x==0, d==0, err=0)
    _mm_store_si128(e,_mm_set1_epi32(0));
    e--; // Now: first interesting element: x==1, d==stencil[1]-1==1

    for(int x=1; x<int(w); x++) {
        __m128i a;
        __m128i sum;
        uint s = stencil[x]; // length of this line
        uint s1 = stencil[x-1]; // length of the last line

        // Prepare image pointers for this comparison
        //                                          ,-------- Column: x
        //                                         /   ,----- Starting disparity (end of line)
        //                                        /   /   ,--- 4 pixels per packet (four lines)
        //                                       /  _/_  / ,-- 4 bytes per pixel (RGBA)
        const __m128i* l=(const __m128i*)&imgL[(x      )*4*4];
        const __m128i* r=(const __m128i*)&imgR[(x-(s-1))*4*4];
        const __m128i left = _mm_load_si128(l);

        // First value at the border of the stencil:
        // Can't go d+1,
        // but maybe x-1,d+0 or x-1,d-1
        a = pixelDiff(left,_mm_load_si128(r++)); // disp--
        if(s1 >= s-1) {
            if(s1 >= s) { // both are available
                __m128i b1 = _mm_load_si128(e+s+0); // x-1,d+0
                __m128i b2 = _mm_load_si128(e+s-1); // x-1,d-1
                sum = _mm_add_epi32(a, _mm_min_epu32(b1, b2));
                _mm_store_si128(e,sum);
            } else { // only x-1,d-1 available
                __m128i b = _mm_load_si128(e+s-1); // x-1,d-1
                sum = _mm_add_epi32(a,b);
                _mm_store_si128(e,sum);
            }
        } else { // neither available: highest possible error value/
            a = _mm_set1_epi32(0x0000FFFF); // smoothY(e); // _mm_load_si128(e);
            sum = a;
            _mm_store_si128(e, sum); // ); // -1));
        } // either way:
        // from here on a will allways contain the last (single) value
        // and sum the last accumulated value

        // Subsequent pixels: 
        e--;
        int d=s-2;
        // At the beginning, there might be a part where the previous line
        // is not long enough to allow selecting the x-1 values
        for(d; d>int(s1); d--) {
            a = pixelDiff(left,_mm_load_si128(r++)); // disp--
            // Only possible value: x-0,d+1,
            // __m128i b = _mm_load_si128(e+1);   // x-0,d+1 this value is in sum
            sum = _mm_add_epi32(a, sum); // b);
            // Write back accumulated error
            _mm_store_si128(e,sum);
            e--; // next pixel
        }
        // Now we might be at the case d==s1
        // meaning: we are right at the point where the previous line ends
        // in which case we have one additional value to consider:
        if(d==s1) {
            a = pixelDiff(left,_mm_load_si128(r++)); // disp--
            // Only possible values: x-0,d+1, x-1,d-1
            __m128i b = sum; // _mm_load_si128(e+1);   // x-0,d+1
            b = _mm_min_epu32(b, _mm_load_si128(e+s-1)); // x-1,d-1
            sum = _mm_add_epi32(a, b);
            // Write back accumulated error
            _mm_store_si128(e,sum);
            e--; // next pixel
            d--;
        }


        // Normal values
        for(d; d>0; d--) {
            // d-directional smoothing:
            // last d-value currently in 'a'
            /*
            __m128i a0 = smoothY(e);
            __m128i a1 = smoothY(e-1); // next value
            a0 = _mm_add_epi32(a0,a0); // double center weight
            a0 = _mm_add_epi32(a0,a1); // next value
            a0 = _mm_add_epi32(a0,a); // last value
            a0 = _mm_srli_epi32(a0,2); // div4
            */
            a = pixelDiff(left,_mm_load_si128(r++)); // disp--
            
            // Three possible adjacent values: x-0,d+1, x-1,d+0, x-1,d-1
            __m128i b = sum; // _mm_load_si128(e+1);   // x-0,d+1
            b = _mm_min_epu32(b, _mm_load_si128(e+s-1)); // x-1,d-1
            b = _mm_min_epu32(b, _mm_load_si128(e+s+0)); // x-1,d+0
            // Add minimum
            sum = _mm_add_epi32(a, b);
            // Write back accumulated error
            _mm_store_si128(e,sum);
            e--; // next pixel
        }
        // When we arrive here, e points at d==0 pixel
        // In the last case (d==0) there are only two adjacent values
        a = pixelDiff(left,_mm_load_si128(r));
        __m128i b1 = sum; // _mm_load_si128(e+1); // x-0,d+1
        __m128i b2 = _mm_load_si128(e+s); // x-1,d+0
        // Minimum of these two
        sum = _mm_add_epi32(a, _mm_min_epu32(b1, b2));
        _mm_store_si128(e,sum);
        e--; // after this, we'll be at the end of the next line
    }
}

//																			________________________
//_________________________________________________________________________/   findDispPathDP()
/**
 * Find the lowest-cost-path through the disparity matrix, store results in top_disp_*.
 */
inline void SteDeR::findPath()
{
    // Search by dynamic programming
    for(int dy=3; dy>=0; dy--) { // per line
        // start at: x=w-1, disp=0, which is the first value of the volume
        uint x = w-1;
        uint disp = 0;
        uint* e = err+dy; // setting internal index
        int motion = 0; // disparity motion: +1=increasing disparity, -1=decreasing disparity
        while(x) {
            // Write current point
            if(x >= disp)
                dispR[(x-disp)*4 + dy] = disp;
            dispL[x*4 + dy]        = disp;
           
            uint s = stencil[x]; // length of the current line
            uint s1 = stencil[x-1]; // length of the line below
            
            // Special case: if the line below is to short to allow any normal downward motion,
            // just jump to the firt value of the next line
            if(disp >= s1) {
                // start at highest possible disparity in next line
                e += ((s-disp)+(s1-1))*4;
                x--;
                disp = s1-1; 
                continue;
            } // this also took care of the x==s1 case, where no x-1 step was possible

            // Find next step
            // Three options:
            // 1: decrease x at same disparity
            // 2: increase disparity
            // 3: decrease disparity while decreasing x

            if(disp > 0) { // we can have the d-1,x-1 step
                if(disp < s-1) { // we can have the d+1 step
                    // Possible steps: d-1,x-1 , x-1 and d+1
                    uint k1 = *(e+s*4-4); // d-1,x-1
                    uint k2 = *(e+s*4);   // x-1
                    uint k3 = *(e+4);     // d+1
                    if(k1 < k2) {
                        if(k1 < k3) { // k1 wins (d-1,x-1)
                            e+=s*4-4;
                            disp--;
                            x--;
                        } else { // k3 wins
                            e+=4;
                            disp++;
                        }
                    } else {
                        if(k2 < k3) { // k2 wins
                            e+=s*4;
                            x--;
                        } else { // k3 wins
                            e+=4;
                            disp++;
                        }
                    }
                } else { // no d+1 step possible
                    // Possible steps: d-1,x-1 and x-1
                    uint k1 = *(e+s*4-4); // d-1,x-1
                    uint k2 = *(e+s*4);   // x-1
                    if(k1 < k2) {
                        e+=s*4-4;
                        disp--;
                        x--;
                    } else {
                        e+=s*4;
                        x--;
                    }
                }
            } else { // no d-1,x-1 step possible
                if(disp < s-1) { // we can have the d+1 step
                    // Possible steps: x-1 and d+1
                    uint k2 = *(e+s*4);   // x-1
                    uint k3 = *(e+4);     // d+1
                    if(k2 < k3) { // k2 wins
                        e+=s*4;
                        x--;
                    } else { // k3 wins
                        e+=4;
                        disp++;
                    }
                } else { // no d+1 step possible
                    // Possible steps: x-1
                    e+=s*4;
                    x--;
                }
            }
            /*
            // ALTERNATIVE: Find next step
            // Three options:
            // 1: decrease x at same disparity
            // 2: increase disparity (same x)
            // 3: decrease disparity (same x)
            uint k = err[w*4*disp + ((x-1)*4 + dy)];
            uint km = UINT_MAX;
            uint kp = UINT_MAX;
            if(disp > 0) {
                km = err[w*4*(disp-1) + ((x-1)*4 + dy)];
            }
            if(disp < max_disp) {
                kp = err[w*4*(disp+1) + (x*4 + dy)];
            }
            if(km < kp) { // walk minus direction
                if(k < km-200 || motion > 0) { // k is smallest or previously walked in other dir
                    motion=0;
                    x--;
                } else { // km is smallest
                    motion = -1;
                    disp--;
                }
            } else { // (kp < km)
                if(k < kp-200 || motion < 0) { // k is smallest or previously walked in other dir
                    motion=0;
                    x--;
                } else { // kp is smallest
                    motion = 1;
                    disp++;
                }
            }*/
            
        }
        // Write last point for x == 0
        // Write current point
        if(0 >= disp)
            dispR[(0-disp)*4 + dy] = disp;
        dispL[0*4 + dy]        = disp;
    }

    // Found path will be used for next stencil
    // (before applying optimizations to dispLR!)
    /*
    for(int x= int(w-1); x>=0; x--) {
        uint a0 = dispL[x*4+0];
        uint a1 = dispL[x*4+1];
        uint a2 = dispL[x*4+2];
        uint a3 = dispL[x*4+3];
        if(a0<a1) a0=a1; // maximum of these values
        if(a0<a2) a0=a2;
        if(a0<a3) a0=a3;
        a0 += stencil_tolerance; // tolerance
        if(a0>max_disp) a0=max_disp+1; // limits
        if(a0>uint(x)) a0=x+1;
        stencil[x] = a0;
    }
    */
    // Pixel buffer layout:
    //                                   | first quart|second quart| third quart|fourth quart|
    //                                   |  0, 1, 2, 3|  4, 5, 6, 7|  8, 9,10,11| 12,13,14,15|// in image buffer
    // original image buffer layout      | R0,G0,B0,XX| R1,G1,B1,XX| R2,G2,B2,XX| R3,G3,B3,XX|
    //                                   | 15,14,13,12| 11,10, 9, 8|  7, 6, 5, 4|  3, 2, 1, 0|// SSE re-ordering
    // shuffle pixels into integers      |   y+(q*0)  |   y+(q*1)  |   y+(q*2)  |   y+(q*3)  |
    __m128i shuffle_mask_0 = _mm_set_epi8( -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, 3, 2, 1, 0);
    __m128i shuffle_mask_1 = _mm_set_epi8( -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, 7, 6, 5, 4);
    __m128i shuffle_mask_2 = _mm_set_epi8( -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,11,10, 9, 8);
    __m128i shuffle_mask_3 = _mm_set_epi8( -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,15,14,13,12);
    __m128i* src = (__m128i*)&dispL[(w-1)*4];
    for(int x= int(w-1); x>=0; x--) {
        union {
            __m128i v;    // SSE 4 x int vector
            uint a[4];  // scalar array of 4 ints
        } u;

        u.v = _mm_load_si128(src);
        u.v = _mm_min_epu32(
                _mm_min_epu32(
                    _mm_shuffle_epi8(u.v, shuffle_mask_0)
                ,
                    _mm_shuffle_epi8(u.v, shuffle_mask_1)
                )
            ,
                _mm_min_epu32(
                    _mm_shuffle_epi8(u.v, shuffle_mask_2)
                ,
                    _mm_shuffle_epi8(u.v, shuffle_mask_3)
                )
            );
        uint a0 = u.a[0] + stencil_tolerance; // tolerance
        if(a0>max_disp) a0=max_disp+1; // limits
        if(a0>uint(x)) a0=x+1;
        stencil[x] = a0;
        src--; // next 4-line-set
    }
}

//																			________________________
//_________________________________________________________________________/calculateDepth(dy,zL,zR)
/**
 * Calculate depth map from winning disparities.
 */
inline void SteDeR::calcDepth(int by, float* zL, float* zR)
{
    // Filtering to fix some artifacts of DP in the image and get better results
    // Per line/ quart index.
    for(int dy=3; dy>=0; dy--) {
        // Protection: due to padding, some lines in the error volume may be off image borders
        if(by + dy >= int(h))
            continue;

        // Filter Depth-Map: Find disparity mismatches between left and right
        int x;
        uint last_good_disp = 0;
        uint last_good_disp_candidate = 0;

        
        // Left side
        // 1: Image border: find the first good value...
        for(x=0; x<int(w); x++) {
            
            int dL = (int)dispL[x*4 + dy];
            if(dL <=1)
                continue; // non-zero dispariy
            if(dL > x) // lies beyond the image border
                continue; // we have to trust it

            int dR = (int)dispR[(x-dL)*4 + dy];
            if(abs(dL-dR) > 1) // no good value
                continue;
            
            // else: found a good value!
            last_good_disp = dL;
            //printf("left side first good d=%i\n",last_good_disp);
            // go back to beginning, filling the border
            for(int x2=x-1; x2>=0; x2--)
                dispL[x2*4 + dy] = last_good_disp;
            break;
        }
        // 2: Continue with the rest, filling occlusions with background
        for(x; x<int(w); x++) {
            
            int dL = (int)dispL[x*4 + dy];
            if(dL > x) // lies beyond the image border
                continue; // we have to trust it

            int dR = (int)dispR[(x-dL)*4 + dy];
            if(abs(dL-dR) <= 1) {
                last_good_disp = last_good_disp_candidate;
                last_good_disp_candidate = dL;
                continue; // check passed
            }
            
            // else: reuse last disparity
            dispL[x*4 + dy] = last_good_disp;
            dispL[(x-1)*4 + dy] = last_good_disp; // including candidate
            
        }

        // Right side
        // 1: Image border: find the first good value...
        for(x = int(w-1); x>=0; x--) {
            int dR = (int)dispR[x*4 + dy];
            if(dR <= 1)
                continue; // non-zero disparity
            if(x+dR >= int(w)) // lies beyond the image border
                continue; // we have to trust it
            
            int dL = (int)dispL[(x+dR)*4 + dy];
            if(abs(dL-dR) > 1)
                continue; // no good value

            // found good value!
            last_good_disp = dR;
            // go back to beginning, filling the border
            for(int x2=x+1; x2<int(w); x2++)
                dispR[x2*4 + dy] = last_good_disp;
            break;
        }
        // 2: Continue with the rest, filling occlusions with background
        for(x; x>=0; x--) {
            int dR = (int)dispR[x*4 + dy];
            if(x+dR >= int(w)) // lies beyond the image border
                continue; // we have to trust it
            
            int dL = (int)dispL[(x+dR)*4 + dy];
            if(abs(dL-dR) <= 1) {
                last_good_disp = last_good_disp_candidate;
                last_good_disp_candidate = dR;
                continue; // check passed
            }

            // else: reuse last disparity
            dispR[x*4 + dy] = last_good_disp;
            dispR[(x+1)*4 + dy] = last_good_disp; // including last candidate
        }
        
        

        
    }
    /*
    // Calculate depth
    float fxb = focal_len * baseline; // for depth calculation
    float A = -(z_far+z_near) / (z_far - z_near);
    float B = -2.0f*z_far*z_near / (z_far - z_near);
    for(int dy=3; dy>=0; dy--) {
        for(int x = int(w-1); x>=0; x--) {
            uint dL = dispL[x*4 + dy];
            uint dR = dispR[x*4 + dy];

            // TODO: Could save some calculation here maybe by resuing L/R result
            //zL[(by+dy)*w + x] = (dL==0) ? max_depth : (fxb / float(dL));
            //zR[(by+dy)*w + x] = (dR==0) ? max_depth : (fxb / float(dR));
            float depthL = (dL==0) ? max_depth : (fxb / float(dL));
            float depthR = (dR==0) ? max_depth : (fxb / float(dR));

            // To Z-Values
            zL[(by+dy)*w + x] = 0.5f*(-A*depthL + B) / depthL + 0.5f; // alternative: (-depthL*A+B)/(depthL);
            zR[(by+dy)*w + x] = 0.5f*(-A*depthR + B) / depthR + 0.5f; // alternative: (-depthR*A+B)/(depthR);          
        }
    }
    */

    __m128 _half = _mm_set_ps1(0.5f);
    __m128 _max_depth = _mm_set_ps1(max_depth);
    //float fxb = focal_len * baseline; // for depth calculation
    __m128 fxb = _mm_set_ps1(focal_len * baseline);
    //float A = -(z_far+z_near) / (z_far - z_near);
    __m128 mA = _mm_set_ps1(-(-(z_far+z_near) / (z_far - z_near)));
    //float B = -2.0f*z_far*z_near / (z_far - z_near);
    __m128 B = _mm_set_ps1(-2.0f*z_far*z_near / (z_far - z_near));

    const __m128i* dL= (const __m128i*) &dispL[w*4-4];
    const __m128i* dR= (const __m128i*) &dispR[w*4-4];

    for(int x = int(w-1); x>=0; x--) {
        //uint dL = dispL[x*4 + dy];
        //uint dR = dispR[x*4 + dy];

        //float depthL = (dL==0) ? max_depth : (fxb / float(dL));
        //float depthR = (dR==0) ? max_depth : (fxb / float(dR));
        __m128 depthL = _mm_min_ps(_mm_div_ps(fxb, _mm_cvtepi32_ps(_mm_load_si128(dL))), _max_depth);
        __m128 depthR = _mm_min_ps(_mm_div_ps(fxb, _mm_cvtepi32_ps(_mm_load_si128(dR))), _max_depth);

        // To Z-Values
        //zL[(by+dy)*w + x] = 0.5f*(-A*depthL + B) / depthL + 0.5f; // alternative: (-depthL*A+B)/(depthL);
        //zR[(by+dy)*w + x] = 0.5f*(-A*depthR + B) / depthR + 0.5f; // alternative: (-depthR*A+B)/(depthR);
        depthL = _mm_add_ps(_mm_div_ps(_mm_mul_ps(_half,_mm_add_ps(_mm_mul_ps(mA,depthL),B)) , depthL),_half);
        depthR = _mm_add_ps(_mm_div_ps(_mm_mul_ps(_half,_mm_add_ps(_mm_mul_ps(mA,depthR),B)) , depthR),_half);

        // Extract and write to z-Map
        union {
            __m128 v;    // SSE 4 x float vector
            float a[4];  // scalar array of 4 floats
        } u;
        u.v = depthL;
        zL[(by+0)*w + x] = u.a[0];
        zL[(by+1)*w + x] = u.a[1];
        zL[(by+2)*w + x] = u.a[2];
        zL[(by+3)*w + x] = u.a[3];
        u.v = depthR;
        zR[(by+0)*w + x] = u.a[0];
        zR[(by+1)*w + x] = u.a[1];
        zR[(by+2)*w + x] = u.a[2];
        zR[(by+3)*w + x] = u.a[3];
       

        // Next disparities
        dL--;
        dR--;
    }
    
}

//																			________________________
//_________________________________________________________________________/    pixelDiff(p)
/**
 * Perform vertical (y) smoothing within one packet of four interleaved lines error values.
 * \param   p   Pointer to a 4-pack of 32bit values of vertical adjacent (interleaved)
 *              error values. Pointer must be 16byte aligned.
 * \return      4Pack of error values, vertically smoothed.
 */
inline __m128i SteDeR::pixelDiff(__m128i left, __m128i right)
{
    
    // Pixel buffer layout:
    //                                       | first quart|second quart| third quart|fourth quart|
    //                                       |  0, 1, 2, 3|  4, 5, 6, 7|  8, 9,10,11| 12,13,14,15|// in image buffer
    // original image buffer layout          | R0,G0,B0,XX| R1,G1,B1,XX| R2,G2,B2,XX| R3,G3,B3,XX|
    //                                       | 15,14,13,12| 11,10, 9, 8|  7, 6, 5, 4|  3, 2, 1, 0|// SSE re-ordering
    // shuffle pixels into integers          |   y+(q*0)  |   y+(q*1)  |   y+(q*2)  |   y+(q*3)  |
    __m128i shuffle_mask_red   = _mm_set_epi8( -1,-1,-1,12, -1,-1,-1, 8, -1,-1,-1, 4, -1,-1,-1, 0);
    __m128i shuffle_mask_green = _mm_set_epi8( -1,-1,-1,13, -1,-1,-1, 9, -1,-1,-1, 5, -1,-1,-1, 1);
    __m128i shuffle_mask_blue  = _mm_set_epi8( -1,-1,-1,14, -1,-1,-1,10, -1,-1,-1, 6, -1,-1,-1, 2);
    //__m128i shuffle_mask_alpha = _mm_set_epi8( -1,-1,-1,15, -1,-1,-1,11, -1,-1,-1, 7, -1,-1,-1, 3);
    /*
    //__m128i left  = _mm_load_si128(l);
    //__m128i right = _mm_load_si128(r);
    __m128i max   = _mm_max_epu8(left, right);
    __m128i min   = _mm_min_epu8(left, right);
    __m128i diff  = _mm_sub_epi8(max, min);
    __m128i red   = _mm_shuffle_epi8(diff, shuffle_mask_red); // separate pixel errors by channel
    __m128i green = _mm_shuffle_epi8(diff, shuffle_mask_green);
    __m128i blue  = _mm_shuffle_epi8(diff, shuffle_mask_blue);
    __m128i a     = _mm_adds_epu16(red,_mm_adds_epu16(green,blue));
    */
    __m128i diff  = _mm_sub_epi8(
                        _mm_max_epu8(left, right),
                        _mm_min_epu8(left, right)
                    );
    // // Sum of absolute differences (SAD)
    __m128i a     = _mm_adds_epu16(
                        _mm_adds_epu16(
                            _mm_shuffle_epi8(diff, shuffle_mask_red),
                            _mm_shuffle_epi8(diff, shuffle_mask_green)
                        ),
                        //_mm_adds_epu16(
                            _mm_shuffle_epi8(diff, shuffle_mask_blue)//,
                        //    _mm_shuffle_epi8(diff, shuffle_mask_alpha)
                        //)
                    );
    /*
    // Sum of squared differences (SSD)
    __m128i red   = _mm_shuffle_epi8(diff, shuffle_mask_red); // separate pixel errors by channel
    __m128i green = _mm_shuffle_epi8(diff, shuffle_mask_green);
    __m128i blue  = _mm_shuffle_epi8(diff, shuffle_mask_blue);
    red   = _mm_mullo_epi16(red, red);
    green = _mm_mullo_epi16(green, green);
    blue  = _mm_mullo_epi16(blue, blue);
    __m128i a     = _mm_adds_epu16(red,_mm_adds_epu16(green,blue));
    */

    // Vertical smoothing
    /*
    // rotational left-shift of packets           11 10 01 00
    __m128i sl1 = _mm_shuffle_epi32(a, 0x93); //  10 01 00 11
    __m128i sl2 = _mm_shuffle_epi32(a, 0x4E); //  01 00 11 10
    __m128i sl3 = _mm_shuffle_epi32(a, 0x39); //  00 11 10 01
    a = _mm_add_epi32(a,a); // center weight x2
    a = _mm_add_epi32(a,sl1);
    a = _mm_add_epi32(a,sl2);
    a = _mm_add_epi32(a,sl3);
    a = _mm_srli_epi32(a,2); // div4
    */
    a = _mm_add_epi32(a,a); // center weight x2
    // rotational left-shift of packets                   11 10 01 00
    a = _mm_add_epi32(a, _mm_shuffle_epi32(a, 0x93)); //  10 01 00 11
    a = _mm_add_epi32(a, _mm_shuffle_epi32(a, 0x4E)); //  01 00 11 10
    a = _mm_add_epi32(a, _mm_shuffle_epi32(a, 0x39)); //  00 11 10 01
    a = _mm_srli_epi32(a,2); // div4
    return a;
    
    
}


//																			________________________
//_________________________________________________________________________/      getWidth()
/**
 * Get image width in pixels.
 */
inline uint SteDeR::getWidth()
{
    return this->w;
}
//																			________________________
//_________________________________________________________________________/     getHeight()
/**
 * Get image height in pixels.
 */
inline uint SteDeR::getHeight()
{
    return this->h;
}
//																			________________________
//_________________________________________________________________________/      getMaxDisp()
/**
 * Get maximum considered disparity.
 */
inline uint SteDeR::getMaxDisp()
{
    return this->max_disp;
}
//																			________________________
//_________________________________________________________________________/    isInitialized()
/**
 * Check wether the object was initialized successfully.
 */
inline bool SteDeR::isInitialized()
{
    return this->initialized;
}

#endif//__STEDER_H
