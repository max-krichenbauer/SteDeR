#include "../steder.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstdio>
#include <cfloat>

int main(int argc, char** argv)
{
    // Load color stereo images
    unsigned char *left_image, *right_image;
    int left_width, left_height, left_channels;
    left_image  = stbi_load("left.png",  &left_width, &left_height, &left_channels, 3);
    right_image = stbi_load("right.png", &left_width, &left_height, &left_channels, 3);

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

    // Convert depth buffer to RGB image
    // Find nearest and farthest points:
    float z_max = FLT_MIN;
    float z_min = FLT_MAX;
    for(int i = left_width*left_height-1; i>=0; i--) {
        if(zL[i] > z_max) z_max = zL[i];
        if(zL[i] < z_min) z_min = zL[i];
    }
    // Convert z (float) to pixel color (uchar)
    for(int i = left_width*left_height-1; i>=0; i--) {
        uchar c;
        c = (zL[i]>z_max) ? 255 : uchar(255.0f*(zL[i]-z_min)/(z_max-z_min));
        left_image[i*3+0] = c;
        left_image[i*3+1] = c;
        left_image[i*3+2] = c;
        c = (zR[i]>z_max) ? 255 : uchar(255.0f*(zR[i]-z_min)/(z_max-z_min));
        right_image[i*3+0] = c;
        right_image[i*3+1] = c;
        right_image[i*3+2] = c;
    }

    stbi_write_png("z_left.png",  left_width, left_height, 3, left_image, left_width*3);
    stbi_write_png("z_right.png", left_width, left_height, 3, right_image, left_width*3);
}
