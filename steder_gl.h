/***********************************************************************************************//**
 * \file      steder_gl.h
 *            Stereo depth reconstruction: OpenGL (GLSL) version.
 *            Contains code for estimating the depth map of a stereo image pair.
 *            This file contains all the code necessary to use SteDeR.
 *            It can be included into any project without requiring further libraries
 *            (Except OpenGL / OpenGL extensions).
 *            However, it does require OpenGL 1.5 or greater on the target platform.
 ***************************************************************************************************
 * \brief     Stereo depth reconstruction - OpenGL (GLSL) version.
 * \author    Max Krichenbauer (max@krichenbauer.de)
 * \version   0.2.4
 * \date      2015-04-24
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

#ifndef __STEDERGL_H
#define __STEDERGL_H

#include <GL/gl.h>
#include <GL/glext.h>

#ifdef _WIN32
#elif defined __APPLE__
#else
#endif

#include <cstdlib>
#include <ctime>
#include <map>

#include <malloc.h>

typedef unsigned char   uchar;			///< unsigned 8bit integer (byte)
typedef unsigned short  ushort;			///< unsigned 16bit (short) integer
typedef unsigned int    uint;			///< unsigned 32bit integer
typedef unsigned long   ulong;			///< unsigned 64bit (long) integer

#define STRING(X) #X

/**
 * Stereo Depth Reconstruction (StDeR) class.
 */
class SteDeRGL
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
        ERROR_NOT_INITIALIZED   = 1 ///< The SteDeRGL was not correctly initialized.
        ,
        ERROR_INVALID_PARAM     = 2 ///< Invalid parameter.
        ,
        ERROR_BAD_IMAGE         = 3 ///< Could not perform reconstruction because of bad image data.
        ,
        ERROR_MALLOC_FAIL       = 4 ///< Failure to acquire required memory.
        ,
        ERROR_OPENGL_FAIL       = 5 ///< Internal OpenCL error.
    } Error;   ///< Reconstructor error codes.

    //                                                                  ________________
    //_________________________________________________________________/ struct Viewport
    /**
     * OpenGL rendering viewport.
     */
    typedef struct Viewport {
        GLint   x;  ///< Viewport origin (x)
        GLint   y;  ///< Viewport origin (y)
        GLsizei w;  ///< Viewport width in pixels.
        GLsizei h;  ///< Viewport height in pixels.
    } Viewport; ///< OpenGL rendering viewport.

    //                                                              ____________________
    //_____________________________________________________________/ struct Framebuffer
    /**
     * OpenGL framebuffer object.
     */
    typedef struct Framebuffer {
        GLuint   fb;  ///< Framebuffer object handle.
        GLuint   tx;  ///< Texture ID.
        Viewport vp;  ///< Rendering viewport.
        GLenum   iformat;///< Internal format.
        GLenum   format; ///< Pixel format.
        GLenum   type;   ///< Data type.
        Framebuffer();  ///< Default constructor.
        ~Framebuffer(); ///< Default constructor.
        int create(GLsizei w, GLsizei h, GLenum iformat, GLenum format, GLenum type, const void* pixels=0); ///< Create frame buffer.
        int release();///< Release allocated objects.
    } Framebuffer; ///< OpenGL framebuffer object.

    //                                                                  ________________
    //_________________________________________________________________/ struct Shader
    /**
     * OpenGL shader set.
     */
    typedef struct Shader {
        GLuint program;         ///< Shader program handle.
        GLuint vertex_shader;   ///< Vertex shader handle.
        GLuint fragment_shader; ///< Fragment shader handle.
        Shader();               ///< Default constructor.
        ~Shader();               ///< Default constructor.
        int create(const GLchar* vs, const GLchar* fs); ///< Create shader program from source.
        int release(); ///< Release allocated objects.
    } Shader; ///< OpenGL shader set.

    SteDeRGL();   ///< Default constructor
    SteDeRGL(const SteDeRGL& cpy); ///< Copy constructor
    ~SteDeRGL();  ///< Destructor

    SteDeRGL& operator=(const SteDeRGL& o); ///< Assignment operator.

    int init(uint w, uint h, float disparity_limit=0.5f); ///< Initialize reconstructor.
    int uninit(); ///< Uninitialize, reset object and free allocated memory.

    float baseline;     ///< Left/Right camera displacement (baseline width).
    float focal_len;    ///< Focal length (assumed to be about the same for both cameras
    float max_depth;    ///< Maximum distance (for points at infinity).
    float z_far;        ///< Far clipping plane of the depth buffer.
    float z_near;       ///< Near clipping plane of the depth buffer.

    uint getWidth();    ///< Get image width in pixels.
    uint getHeight();   ///< Get image height in pixels.
    uint getMaxDisp();  ///< Get maximum considered disparity.
    bool isInitialized();///< Check wether the object was initialized successfully.

    Framebuffer imgL;        ///< Left image (input texture and output frame buffer).
    Framebuffer imgR;        ///< Right image (input texture and output frame buffer).    

    int operator()();   ///< Execute reconstruction.

private:
    bool initialized;   ///< Whether the Reconstructor was successfully initialized.

    uint w;         ///< Width of image in pixels.
    uint h;         ///< Height of image in pixels.
    uint max_disp;  ///< Maximum considered pixel disparity in pixels.

    Shader  sdr;    ///< Shader for stereo depth reconstruction.
    Shader  mip;    ///< Mipmap shader for image downsampling.
    Shader  dip;    ///< Shader for disparity map generation.
    
    Framebuffer mipL; ///< Down-sampled image texture (pyramid/mipmap), left image.
    Framebuffer mipR; ///< Down-sampled image texture (pyramid/mipmap), right image.
    Framebuffer dipL; ///< Disparity map of of the downsampled image, left.
    Framebuffer dipR; ///< Disparity map of of the downsampled image, right.

    static const GLchar* sdr_vshader_source();  ///< Stereo depth reconstruction vertex shader source code.
    static const GLchar* sdr_fshader_source();  ///< Stereo depth reconstruction fragment shader source code.

    static const GLchar* mip_vshader_source();  ///< Mipmap vertex shader source code.
    static const GLchar* mip_fshader_source();  ///< Mipmap fragment shader source code.

    static const GLchar* dip_vshader_source();  ///< Disparity map vertex shader source code.
    static const GLchar* dip_fshader_source();  ///< Disparity map fragment shader source code.
};


//																			________________________
//_________________________________________________________________________/  dip_vshader_source
/**
 * Downsampling vertex shader source code.
 * This a very primitive vertex shader used for texture downsampling.
 */
inline const GLchar* SteDeRGL::dip_vshader_source() {
    return STRING(
    #version 150\n
    in      vec2        pos;
    out     vec2        p0;
    void main() {
        p0 = (pos*0.5f + 0.5f);
        gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
    });
}


//																			________________________
//_________________________________________________________________________/   dip_fshader_source
/**
 * 
 */
inline const GLchar* SteDeRGL::dip_fshader_source() {
    return STRING(
    #version 150\n
    #define N0          11\n
    #define W0X         1\n
    #define W0Y         1\n
    #define W0WIDTH     (2*W0X+1) \n
    #define W0HEIGHT    (2*W0Y+1) \n
    #define W0AREA      (W0WIDTH*W0HEIGHT) \n
    #define W0CENTER    (W0Y*W0WIDTH+W0X) \n
    #define N1          4\n
    #define W1X         7\n
    #define W1Y         7\n
    #define W1WIDTH     (2*W1X+1) \n
    #define W1HEIGHT    (2*W1Y+1) \n
    #define W1AREA      (W1WIDTH+W1HEIGHT) \n
    #define W1CENTER    (W1X) \n
    uniform sampler2D   img0;
    uniform sampler2D   img1;
    uniform uint        max_d;
    uniform float       u;
    uniform float       v;
    in vec2             p0;
    out vec4            disp;
    void main()
    {
        // FIRST STAGE: Small window errors
        uint top_err0[N0];
        for (int i=N0-1; i>=0; i--) {
            top_err0[i] = uint(0xFFFFFF00);
        }
        {
            vec4 c0[W0AREA];
            for (int i=0,x=W0X; x>=-W0X; x--) {
                for (int y=W0Y; y>=-W0Y; y--,i++) {
                    c0[i] = texture(img0, p0+vec2(float(x)*u, float(y)*v));
                }
            }
            vec2 p1 = p0;
            for (uint d=uint(0); d<=max_d; d++) {
                vec4 c1[W0AREA];
                for (int i=0,x=W0X; x>=-W0X; x--) {
                    for (int y=W0Y; y>=-W0Y; y--, i++) {
                        c1[i] = texture(img1, p1+vec2(float(x)*u, float(y)*v));
                    }
                }
                float w = 0.0f;
                float e=0.0f;
                for (int i=W0AREA-1; i>=0; i--) {
                    vec4 c0d = c0[i];
                    vec4 c1d = c1[i];
                    e += dot(abs(c0d-c1d), vec4(0.25));
                }
                // e is now 0~WAREA
                // w is now 1~WAREA (min 1 for center pixel)
                // e/w is 0~WAREA
                //e = (e/w)*(524287.0f/W0AREA);
                e *= 524287.0f/W0AREA;
                uint err = (uint(e)<<8) | d;
                top_err0[N0-1] = min(top_err0[N0-1], err);
                for (int i=N0-1; i>0; i--) {
                    uint err_min = min(top_err0[i], top_err0[i-1]);
                    uint err_max = max(top_err0[i], top_err0[i-1]);
                    top_err0[i]   = err_max;
                    top_err0[i-1] = err_min;
                }
                p1.x += u; // Next pixel
            }
        }
        // SECOND STAGE: Larger window errors
        uint top_err1[N1];
        for (int i=N1-1; i>=0; i--) {
            top_err1[i] = uint(0xFFFFFF00);
        }
        {
            vec4 c0[W1AREA];
            for (int i=0,x=W1X; x>=-W1X; x--,i++) {
                c0[i] = texture(img0, p0+vec2(float(x)*u, 0));
            }
            for (int i=W1WIDTH, y=W1Y; y>=-W1Y; y--,i++) {
                c0[i] = texture(img0, p0+vec2(0, float(y)*v));
            }
            
            for (int k=N0-1; k>=0; k--) {
                uint d = top_err0[k] & uint(0x000000FF);
                vec2 p1 = p0+vec2(float(d)*u,0);
                vec4 c1[W1AREA];
                for (int i=0,x=W1X; x>=-W1X; x--,i++) {
                    c1[i] = texture(img1, p1+vec2(float(x)*u, 0));
                }
                for (int i=W1WIDTH,y=W1Y; y>=-W1Y; y--, i++) {
                    c1[i] = texture(img1, p1+vec2(0, float(y)*v));
                }
                //float w = 0.0f;
                float e=0.0f;
                for (int i=W1AREA-1; i>=0; i--) {
                    vec4 c0d = c0[i];
                    vec4 c1d = c1[i];
                    e += dot(abs(c0d-c1d), vec4(0.25));
                }
                // e is now 0~WAREA
                // w is now 1~WAREA (min 1 for center pixel)
                // e/w is 0~WAREA
                //e = (e/w)*(524287.0f/W1AREA);
                e *= 524287.0f/W1AREA;
                
                uint err = (uint(e)<<8) | d;
                top_err1[N1-1] = min(top_err1[N1-1], err);
                for (int i=N1-1; i>0; i--) {
                    uint err_min = min(top_err1[i], top_err1[i-1]);
                    uint err_max = max(top_err1[i], top_err1[i-1]);
                    top_err1[i]   = err_max;
                    top_err1[i-1] = err_min;
                }
            }
        }
        disp.r = float(top_err1[0]&uint(0x000000ff))/255.0f;
        disp.g = float(top_err1[1]&uint(0x000000ff))/255.0f;
        disp.b = float(top_err1[2]&uint(0x000000ff))/255.0f;
        disp.a = float(top_err1[3]&uint(0x000000ff))/255.0f;
    });
}


//																			________________________
//_________________________________________________________________________/  mip_vshader_source
/**
 * Downsampling vertex shader source code.
 * This a very primitive vertex shader used for texture downsampling.
 */
inline const GLchar* SteDeRGL::mip_vshader_source() {
    return STRING(
    #version 150\n
    in      vec2        pos;
    out     vec2        p0;
    void main() {
        p0 = (pos*0.5f + 0.5f);
        gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
    });
}


//																			________________________
//_________________________________________________________________________/   mip_fshader_source
/**
 * Downsampling fragment shader source code.
 * This a very primitive fragment shader for texture downsampling.
 */
inline const GLchar* SteDeRGL::mip_fshader_source() {
    return STRING(
    #version 150            \n
    in vec2            p0;
    uniform float      u;   // Size of one pixel in UV coordinates, horizontally. \n
    uniform float      v;   // Size of one pixel in UV coordinates, vertically. \n
    uniform sampler2D  src;
    out vec4           dst; 
    void main()
    {               
        vec4 c=vec4(0);
        vec4 img[16];
        for (int i=0,x=3;x>=0;x--) {
            for (int y=3;y>=0;y--,i++) {
                img[i] = texture2D(src, p0+vec2(float(x)*u, float(y)*v));
                c += img[i];
            }
        }
        c = c/16.0f;
        float var=0.0f;
        for (int i=15; i>=0; i--) {
            vec4 d = img[i]-c;
            float diff = length(d);
            var += diff*diff;
        }
        var *= 4.0f/16.0f;

        ////////////////////////////
        // RGB to L*ab
        float r = c[0];
        float g = c[1];
        float b = c[2];
        float L; float A; float B;
        float X; float Y; float Z; float fx; float fy; float fz; float xr; float yr; float zr;
        float eps = 216.f/24389.f;
        float k = 24389.f/27.f;
        float Xr = 0.964221f;
        float Yr = 1.0f;
        float Zr = 0.825211f;
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
            b = float(pow((b+0.055f)/1.055f,2.4f));
        X =  0.436052025f*r     + 0.385081593f*g + 0.143087414f *b;
        Y =  0.222491598f*r     + 0.71688606f *g + 0.060621486f *b;
        Z =  0.013929122f*r     + 0.097097002f*g + 0.71418547f  *b;
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
        A = 500.0f*(fx-fy); // [-79.283653 ~ 93.549309] ??
        B = 200.0f*(fy-fz); // [-112.028458 ~ 93.391396] ??
        // cast to 0~1:
        //c[0] = L/100.0f;
        ////////////////////////////

        // RGB to HSV
        float v = max(max(c.r, max(c.g, c.b)), 0.001f);
        c /= v;
        float rgb_min = min(c.r, min(c.g, c.b));
        float rgb_max = max(c.r, max(c.g, c.b));
        float s = max(rgb_max - rgb_min, 0.01f);
        // cast to 0~1:
        dst.r = (A / 128.0f)+0.5f;
        //dst.r = (dst.r+79.283653f) / (79.283653f + 93.549309f);
        dst.g = (B / 128.0f)+0.5f;
        //dst.g = (dst.g+112.028458f)/ (112.028458f + 93.391396f);
        //dst.b = sqrt(s*s+v*v);
        dst.b = exp(-log(1.0f-v) / log(s)); // gamma value of saturation and brightness
        dst.a = sqrt(var); // add variance
    });
}


//																			________________________
//_________________________________________________________________________/ vertex_shader_source
/**
 * Vertex shader source code.
 * This a very primitive vertex shader that is only used to calculate the texture coordinate
 * of the vertex / fragment.
 */
inline const GLchar* SteDeRGL::sdr_vshader_source() {
    return STRING(
    #version 150\n
    in vec2 pos;
    out vec2 p0;
    void main() {
        p0 = pos*0.5f + 0.5f;
        gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
    });
}
//																			________________________
//_________________________________________________________________________/ sdr_fshader_source
/**
 * Fragment shader source code.
 * This is the main algorithm for stereo depth reconstruction.
 */
inline const GLchar* SteDeRGL::sdr_fshader_source() {
    return STRING(
    #version 150\n
    #define WX          0\n
    #define WY          8\n
    #define WWIDTH      (2*WX+1) \n
    #define WHEIGHT     (2*WY+1) \n
    #define WAREA       (WWIDTH*WHEIGHT) \n
    #define WCENTER     (WY*WWIDTH+WX) \n
    #define K           2 \n // Number of proposed disparity levels to test
    #define DMIN        (-4) \n // Minimum disparity offset to test
    #define DMAX        (4) \n // Maximum disparity offset to test
    #define DSTEP       (4) \n // Disparity offset stepping
    uniform sampler2D   img0;
    uniform sampler2D   img1;
    uniform sampler2D   dmap;
    uniform float       u;
    uniform float       v;
    uniform float       fxb;
    uniform float       mA;
    uniform float       B;
    in vec2             p0;
    void main()
    {
        vec3 c0[WAREA];
        vec3 c0_mean = vec3(0);
        for (int i=0,x=WX;x>=-WX;x--) {
            for (int y=WY;y>=-WY;y--,i++) {
                c0[i] = texture(img0, p0+vec2(float(x)*u, float(y)*v)).rgb;
            }
        }
        c0_mean = (c0[WCENTER-2]+c0[WCENTER-1]+c0[WCENTER]+c0[WCENTER+1]+c0[WCENTER+2])/5.0f;
        float c0_mean_len = length(c0_mean);

        // Load proposed disparities from lower pyramid level
        vec4 disp = texture(dmap, p0);
        uint d[4];
        d[0] = uint(255.0f*disp.r*8.0f);
        d[1] = uint(255.0f*disp.g*8.0f);
        d[2] = uint(255.0f*disp.b*8.0f);
        d[3] = uint(255.0f*disp.a*8.0f);
        uint best_err = uint(0xFFFFFF00);
        for (int k=K-1; k>=0; k--) {
            for (int dd=DMIN; dd<=DMAX; dd+=DSTEP) {
                int disp = abs(int(d[k])+dd);
                vec2 p1 = vec2(p0.x+float(disp)*u, p0.y);
                vec3 c1[WAREA];
                vec3 c1_mean = vec3(0);
                for (int i=0,x=WX;x>=-WX;x--) {
                    for (int y=WY;y>=-WY;y--,i++) {
                        /*c1_mean +=*/ c1[i] = texture(img1, p1+vec2(float(x)*u, float(y)*v)).rgb;
                    }
                }
                //c1_mean /= WAREA;
                c1_mean = (c1[WCENTER-2]+c1[WCENTER-1]+c1[WCENTER]+c1[WCENTER+1]+c1[WCENTER+2])/5.0f;
                c1_mean = c0_mean * (length(c1_mean) / c0_mean_len); // rebase: linear with c0_mean
                //float w = 0.0f;
                float e = 0.0f;
                for (int i=WAREA-1; i>=0; i--) {
                    //vec3 diff0 = vec3(1)-abs(c0[WCENTER] - c0[i]);
                    //vec3 diff1 = vec3(1)-abs(c1[WCENTER] - c1[i]);
                    //float wi = (diff0.r*diff0.g*diff0.b)
                    //         * (diff1.r*diff1.g*diff1.b);
                    vec3 diff = abs((c0[i]-c0_mean) - (c1[i]-c1_mean));
                    e += (diff.r+diff.g+diff.b); // *wi;
                    //w += wi;
                }
                // e is now 0 ~ 3*WAREA
                // w is now 1~WAREA (min 1 for center pixel)
                // e/w is 0~WAREA
                //e = (e/w)*(524287.0f/W1AREA);
                e *= 524287.0f/(3.0f*WAREA);
                //uint err = (uint(524287.0f*e*(WAREA/w))<<8) | uint(disp);
                uint err = (uint(e)<<8) | uint(disp);
                best_err = min(err, best_err);
            }
        }
        
        float best_d = max(0.1f, float(best_err & uint(0x000000FF)));
        float depth = fxb / best_d;
        float z = 0.5f*(mA*depth + B) / depth + 0.5f;
        gl_FragDepth = z;
    });
}

//																			________________________
//_________________________________________________________________________/   Framebuffer()
/**
 * Default constructor.
 */
inline SteDeRGL::Framebuffer::Framebuffer()
: fb(0), tx(0)
{
    // 
}

//																			________________________
//_________________________________________________________________________/   ~Framebuffer()
/**
 * Default destructor.
 */
inline SteDeRGL::Framebuffer::~Framebuffer()
{
    this->release();
}

//																			________________________
//_________________________________________________________________________/   release()
/**
 * Release allocated objects.
 */
inline int SteDeRGL::Framebuffer::release()
{
    if (this->tx) {
        glDeleteTextures(1, &this->tx);
        this->tx = 0;
    }
    if (this->fb) {
        glDeleteFramebuffers(1, &this->fb);
        this->fb = 0;
    }
    return SteDeRGL::SUCCESS;
}

//																			________________________
//_________________________________________________________________________/   create()
/**
 * Create frame buffer.
 */
inline int SteDeRGL::Framebuffer::create(GLsizei w, GLsizei h, GLenum iformat, GLenum format, GLenum type, const void* pixels)
{
    GLenum e;
    this->release();

    this->vp.x = 0;
    this->vp.y = 0;
    this->vp.w = w;
    this->vp.h = h;
    this->iformat = iformat;
    this->format = format;
    this->type = type;

    glGenTextures(1, &this->tx);
    if ((e = glGetError()) != GL_NO_ERROR) {
        return SteDeRGL::ERROR_OPENGL_FAIL;
    }
    glBindTexture(GL_TEXTURE_2D, tx);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); // GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, iformat, w, h, 0, format, type, pixels);
    if ((e = glGetError()) != GL_NO_ERROR) {
        this->release();
        return SteDeRGL::ERROR_OPENGL_FAIL;
    }
    glGenFramebuffers(1, &this->fb);
    if ((e = glGetError()) != GL_NO_ERROR) {
        this->release();
        return SteDeRGL::ERROR_OPENGL_FAIL;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tx, 0);
    if ((e = glGetError()) != GL_NO_ERROR) {
        this->release();
        return SteDeRGL::ERROR_OPENGL_FAIL;
    }
    return SteDeRGL::SUCCESS;
}

//																			________________________
//_________________________________________________________________________/      Shader()
/**
 * Default constructor.
 */
inline SteDeRGL::Shader::Shader()
: fragment_shader(0), vertex_shader(0), program(0)
{
    // 
}

//																			________________________
//_________________________________________________________________________/     ~Shader()
/**
 * Default destructor.
 */
inline SteDeRGL::Shader::~Shader()
{
    this->release();
}

//																			________________________
//_________________________________________________________________________/   Shader::create()
/**
 * Create shader program from source.
 */
inline int SteDeRGL::Shader::create(const GLchar* vss, const GLchar* fss)
{
    this->release();

    // Initialize the depth reconstruction shaders
    this->program = glCreateProgram();
    this->vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    this->fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertex_shader, 1, &vss, 0);
    glShaderSource(fragment_shader, 1, &fss, 0);

    GLint ret;

    glCompileShader(vertex_shader);
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &ret);
    if (!ret) {
        GLchar error[256];
        GLsizei len;
        glGetShaderInfoLog(vertex_shader, 256, &len, error);
        error[255]=0;
        printf("Error compiling vertex shader: \n%s\n", error);
        return ERROR_OPENGL_FAIL;
    }
    glAttachShader(program, vertex_shader);

    glCompileShader(fragment_shader);
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &ret);
    if (!ret) {
        GLchar error[256];
        GLsizei len;
        glGetShaderInfoLog(fragment_shader, 256, &len, error);
        error[255]=0;
        printf("Error compiling fragment shader: \n%s\n", error);
        return ERROR_OPENGL_FAIL;
    }
    glAttachShader(program, fragment_shader);
    
    //glBindFragDataLocation(program, 0, "color");

    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &ret);
    if (!ret) {
        GLchar error[256];
        GLsizei len;
        glGetProgramInfoLog(program, 256, &len, error);
        error[255]=0;
        printf("Error linking GLSL program:\n%s\n", error);
        return ERROR_OPENGL_FAIL;
    }

    return SteDeRGL::SUCCESS;
}

//																			________________________
//_________________________________________________________________________/   Shader::release()
/**
 * Release allocated objects.
 */
inline int SteDeRGL::Shader::release()
{
    if (this->vertex_shader) {
        glDeleteShader(this->vertex_shader);
        this->vertex_shader = 0;
    }
    if (this->fragment_shader) {
        glDeleteShader(this->fragment_shader);
        this->fragment_shader = 0;
    }
    if (this->program) {
        glDeleteProgram(this->program);
        this->program = 0;
    }
    return SteDeRGL::SUCCESS;
}

//																			________________________
//_________________________________________________________________________/   SteDeRGL()
/**
 * Default constructor (empty).
 */
inline SteDeRGL::SteDeRGL() :
imgL(), imgR(), mipL(), mipR(), dipL(), dipR(), sdr(), mip(), dip()
{
    this->initialized = false;

    this->focal_len   = 348.85096f;
    this->baseline    = 49.7642237f;
    this->max_depth   = 10000.0f;
    this->z_far       = 10000.0f;
    this->z_near      = 1.0f;
}

//																			________________________
//_________________________________________________________________________/   SteDeRGL(cpy)
/**
 * Copy constructor.
 */
inline SteDeRGL::SteDeRGL(const SteDeRGL& cpy) :
imgL(), imgR(), mipL(), mipR(), dipL(), dipR(), sdr(), mip(), dip()
{
    this->initialized = false;
    // Use the designated render target of the copy
    this->imgL        = cpy.imgL;
    this->imgR        = cpy.imgR;
    // Use the rendering details of the copy
    this->focal_len   = cpy.focal_len;
    this->baseline    = cpy.baseline;
    this->max_depth   = cpy.max_depth;
    this->z_far       = cpy.z_far;
    this->z_near      = cpy.z_near;
    // If the master is initialized, try to init too.
    if(cpy.initialized) {
        this->init(cpy.w, cpy.h, float(cpy.max_disp)/float(cpy.w));
    }
}

//																			________________________
//_________________________________________________________________________/   ~SteDeRGL()
/**
 * Destructor
 */
inline SteDeRGL::~SteDeRGL()
{
    this->uninit();
}

//																			________________________
//_________________________________________________________________________/   operator=()
/**
 * Assignment operator.
 */
inline SteDeRGL& SteDeRGL::operator=(const SteDeRGL& o)
{
    // Delete your own objects, if any.
    this->uninit();
    // Use the render target from the other object
    this->imgL        = o.imgL;
    this->imgR        = o.imgR;
    // Use the same reconstruction parameters
    this->focal_len   = o.focal_len;
    this->baseline    = o.baseline;
    this->max_depth   = o.max_depth;
    this->z_far       = o.z_far;
    this->z_near      = o.z_near;
    // If the other is initialized, try to init too.
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
inline int SteDeRGL::uninit()
{
    mipL.release();
    mipR.release();
    dipL.release();
    dipR.release();
    sdr.release();
    mip.release();
    dip.release();

    this->initialized = false;

    return SteDeRGL::SUCCESS;
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
inline int SteDeRGL::init(uint w, uint h, float disparity_limit)
{
    if(!w || !h || disparity_limit <= 0.0f || disparity_limit >= 1.0f) {
        return ERROR_INVALID_PARAM;
    }
    
    // Save prior shader and return to it later
    GLint default_shader;
    glGetIntegerv(GL_CURRENT_PROGRAM, (GLint*)&default_shader);
    GLuint bound_texture;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLint*)&bound_texture);
    GLenum active_texture;
    glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&active_texture);
    GLuint old_draw_framebuffer;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, (GLint*)&old_draw_framebuffer);
    GLuint old_read_framebuffer;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, (GLint*)&old_read_framebuffer);

    // If we have any previously allocated memory, free if before continuing
    this->uninit();

    
    // Calculate internal memory / volume dimensions
    this->w = w;
    this->h = h;
    this->max_disp = uint(float(w)*disparity_limit);
    
    
    // Initialize the shaders
    if (this->sdr.create(SteDeRGL::sdr_vshader_source(), SteDeRGL::sdr_fshader_source()))
        return SteDeRGL::ERROR_OPENGL_FAIL;

    if (this->mip.create(SteDeRGL::mip_vshader_source(), SteDeRGL::mip_fshader_source()))
        return SteDeRGL::ERROR_OPENGL_FAIL;

    if (this->dip.create(SteDeRGL::dip_vshader_source(), SteDeRGL::dip_fshader_source()))
        return SteDeRGL::ERROR_OPENGL_FAIL;

    // Initialize the render buffers
    this->mipL.create(w/8, h/8, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    this->mipR.create(w/8, h/8, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    this->dipL.create(w/8, h/8, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    this->dipR.create(w/8, h/8, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);

    // Reset to old state
    glUseProgram(default_shader);
    glBindTexture(GL_TEXTURE_2D, bound_texture);
    glActiveTexture(active_texture);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_draw_framebuffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, old_read_framebuffer);

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
inline int SteDeRGL::operator()()
{
    if(!this->initialized)
        return ERROR_NOT_INITIALIZED;

    float pixel_size = 1.0f/float(w); // pixel size in texture coordinates [0..1]

    // Save prior shader and return to it later
    GLint default_shader;
    glGetIntegerv(GL_CURRENT_PROGRAM, (GLint*)&default_shader);

    // Prepare z-buffer for rendering
    GLboolean depth_test;
    glGetBooleanv(GL_DEPTH_TEST,&depth_test);
    if (!depth_test) {
        glEnable(GL_DEPTH_TEST); // Must be enabled in order to write to depth buffer
    }
    GLenum depth_func;
    glGetIntegerv(GL_DEPTH_FUNC,(GLint*)&depth_func);
    if (depth_func!=GL_ALWAYS) {
        glDepthFunc(GL_ALWAYS);  // Always pass depth depths test
    }
    GLboolean cull_face = glIsEnabled(GL_CULL_FACE);
    if (cull_face) {
        glDisable(GL_CULL_FACE);
    }
    GLboolean blend = glIsEnabled(GL_BLEND);
    if (blend) {
        glDisable(GL_BLEND);
    }
    GLboolean framebuffer_srgb = glIsEnabled(GL_FRAMEBUFFER_SRGB);
    if (framebuffer_srgb) {
        glDisable(GL_FRAMEBUFFER_SRGB);
    }
    GLboolean stencil_test = glIsEnabled(GL_STENCIL_TEST);
    if (stencil_test) {
        glDisable(GL_STENCIL_TEST);
    }
    GLboolean program_point_size = glIsEnabled(GL_PROGRAM_POINT_SIZE);
    if (program_point_size) {
        glDisable(GL_PROGRAM_POINT_SIZE);
    }

    GLenum old_activetexture;
    glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&old_activetexture);


    GLenum old_drawbuffer; // will be saved and restored later for each framebuffer

    GLuint old_framebuffer;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, (GLint*)&old_framebuffer);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // Downsample textures
    //glUseProgram(0);
    //glColor4f(1.0, 1.0, 1.0, 1.0);
    //glEnable(GL_TEXTURE_2D);
    //glDisable(GL_LIGHTING);
    glUseProgram(this->mip.program);
    glUniform1f(glGetUniformLocation(mip.program, "u"),  1.0f/float(w));
    glUniform1f(glGetUniformLocation(mip.program, "v"),  1.0f/float(h));
    glUniform1i(glGetUniformLocation(mip.program, "src"), 0);
    glBindFragDataLocation(mip.program, 0, "dst");
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mipL.fb);
    glViewport(mipL.vp.x, mipL.vp.y, mipL.vp.w, mipL.vp.h);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imgL.tx);
    glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 1.0f);   glVertex3f( -1,	 1,  0 );
	glTexCoord2f(1.0f, 1.0f);   glVertex3f(  1,  1,  0 );
	glTexCoord2f(1.0f, 0.0f);   glVertex3f(  1,	-1,  0 );
	glTexCoord2f(0.0f, 0.0f);   glVertex3f( -1, -1,  0 );
	glEnd();
    
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mipR.fb);
    glViewport(mipR.vp.x, mipR.vp.y, mipR.vp.w, mipR.vp.h);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imgR.tx);
    glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 1.0f);   glVertex3f( -1,	 1,  0 );
	glTexCoord2f(1.0f, 1.0f);   glVertex3f(  1,  1,  0 );
	glTexCoord2f(1.0f, 0.0f);   glVertex3f(  1,	-1,  0 );
	glTexCoord2f(0.0f, 0.0f);   glVertex3f( -1, -1,  0 );
	glEnd();
    
    // Render disparity map of the pyramid level
    glUseProgram(this->dip.program);
    glUniform1ui(glGetUniformLocation(dip.program, "max_d"), this->max_disp/8);
    glUniform1f(glGetUniformLocation(dip.program, "u"), -1.0f/float(dipL.vp.w));
    glUniform1f(glGetUniformLocation(dip.program, "v"), -1.0f/float(dipL.vp.h));
    glUniform1i(glGetUniformLocation(dip.program, "img0"), 0);
	glUniform1i(glGetUniformLocation(dip.program, "img1"), 1);
    glBindFragDataLocation(dip.program, 0, "disp");
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dipL.fb);
    glViewport(dipL.vp.x, dipL.vp.y, dipL.vp.w, dipL.vp.h);;
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mipL.tx);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mipR.tx);
    glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	glVertex3f( -1,	 1,  0 );
	glVertex3f(  1,  1,  0 );
	glVertex3f(  1,	-1,  0 );
	glVertex3f( -1, -1,  0 );
	glEnd();
    // right-left
    glUniform1f(glGetUniformLocation(dip.program, "u"), 1.0f/float(dipL.vp.w));
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dipR.fb);
    glViewport(dipR.vp.x, dipR.vp.y, dipR.vp.w, dipR.vp.h);;
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mipR.tx);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mipL.tx);
    glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	glVertex3f( -1,	 1,  0 );
	glVertex3f(  1,  1,  0 );
	glVertex3f(  1,	-1,  0 );
	glVertex3f( -1, -1,  0 );
	glEnd();

    // Stereo depth reconstruction
    glUseProgram(this->sdr.program);
    
    // glUniform1f(glGetUniformLocation(program, "u"), -1.0f/float(w)); depends on side...
    glUniform1f(glGetUniformLocation(sdr.program, "v"),  1.0f/float(h));
    glUniform1ui(glGetUniformLocation(sdr.program, "max_d"), this->max_disp);
    glUniform1f(glGetUniformLocation(sdr.program, "fxb"), focal_len * baseline);
    glUniform1f(glGetUniformLocation(sdr.program, "mA"), -(-(z_far+z_near) / (z_far - z_near)));
    glUniform1f(glGetUniformLocation(sdr.program, "B"), -2.0f*z_far*z_near / (z_far - z_near));
	glUniform1i(glGetUniformLocation(sdr.program, "img0"), 0);
	glUniform1i(glGetUniformLocation(sdr.program, "img1"), 1);
    glUniform1i(glGetUniformLocation(sdr.program, "dmap"), 2);
    
    
    // Left-right
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, imgL.fb);
    glViewport(imgL.vp.x, imgL.vp.y, imgL.vp.w, imgL.vp.h);
    glGetIntegerv(GL_DRAW_BUFFER, (GLint*)&old_drawbuffer);
    if (old_drawbuffer != GL_NONE)
        glDrawBuffer(GL_NONE);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imgL.tx);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, imgR.tx);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, dipL.tx);

    glUniform1f(glGetUniformLocation(sdr.program, "u"), -pixel_size);
    glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	glVertex3f( -1,	 1,  0 );
	glVertex3f(  1,  1,  0 );
	glVertex3f(  1,	-1,  0 );
	glVertex3f( -1, -1,  0 );
	glEnd();
    if (old_drawbuffer != GL_NONE)
        glDrawBuffer(old_drawbuffer);
   
        
    // Right-left
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, imgR.fb);
    glGetIntegerv(GL_DRAW_BUFFER, (GLint*)&old_drawbuffer);
    if (old_drawbuffer != GL_NONE)
        glDrawBuffer(GL_NONE);
    glViewport(imgR.vp.x, imgR.vp.y, imgR.vp.w, imgR.vp.h);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imgR.tx);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, imgL.tx);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, dipR.tx);
    glUniform1f(glGetUniformLocation(sdr.program, "u"), pixel_size);
    glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	glVertex3f( -1,	 1,  0 );
	glVertex3f(  1,  1,  0 );
	glVertex3f(  1,	-1,  0 );
	glVertex3f( -1, -1,  0 );
	glEnd();
    if (old_drawbuffer != GL_NONE)
        glDrawBuffer(old_drawbuffer);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();


    // Reset to previous z-buffer settings, if necessary
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_framebuffer);
    if (!depth_test) {
        glDisable(GL_DEPTH_TEST);
    }
    if (depth_func!=GL_ALWAYS) {
        glDepthFunc(depth_func);
    }
    if (cull_face) {
        glEnable(GL_CULL_FACE);
    }
    if (blend) {
        glEnable(GL_BLEND);
    }
    if (framebuffer_srgb) {
        glEnable(GL_FRAMEBUFFER_SRGB);
    }
    if (stencil_test) {
        glEnable(GL_STENCIL_TEST);
    }
    if (program_point_size) {
        glEnable(GL_PROGRAM_POINT_SIZE);
    }
    glActiveTexture(old_activetexture);

    // Reset prior shader program
    glUseProgram(default_shader);
    
    return SUCCESS;
}

//																			________________________
//_________________________________________________________________________/      getWidth()
/**
 * Get image width in pixels.
 */
inline uint SteDeRGL::getWidth()
{
    return this->w;
}
//																			________________________
//_________________________________________________________________________/     getHeight()
/**
 * Get image height in pixels.
 */
inline uint SteDeRGL::getHeight()
{
    return this->h;
}
//																			________________________
//_________________________________________________________________________/      getMaxDisp()
/**
 * Get maximum considered disparity.
 */
inline uint SteDeRGL::getMaxDisp()
{
    return this->max_disp;
}
//																			________________________
//_________________________________________________________________________/    isInitialized()
/**
 * Check wether the object was initialized successfully.
 */
inline bool SteDeRGL::isInitialized()
{
    return this->initialized;
}

#endif//__STEDERGL_H
