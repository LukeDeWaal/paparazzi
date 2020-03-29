/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "pthread.h"

#define PRINT(string, ...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

uint8_t cod_lum_min3 = 0;
uint8_t cod_lum_max3 = 0;
uint8_t cod_cb_min3 = 0;
uint8_t cod_cb_max3 = 0;
uint8_t cod_cr_min3 = 0;
uint8_t cod_cr_max3 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct color_object_t {
    int32_t x_c;
    int32_t y_c;
    uint32_t color_count;
    bool updated;
};
struct color_object_t global_filters[3];

uint8_t first = 0;


uint32_t left_green_count = 0;
uint32_t right_green_count = 0;
uint32_t left_orange_count = 0;
uint32_t right_orange_count = 0;

int32_t x_c_g, y_c_g, x_c_o, y_c_o, x_c_g_L, y_c_g_L, x_c_g_R, y_c_g_R, x_c_o_L, y_c_o_L, x_c_o_R, y_c_o_R;

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t *p_xc, int32_t *p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint16_t x_min, uint16_t x_max,
                              uint16_t y_min, uint16_t y_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter) {


    uint8_t lum_min, lum_max;
    uint8_t cb_min, cb_max;
    uint8_t cr_min, cr_max;
    uint8_t lum_min_2, lum_max_2;
    uint8_t cb_min_2, cb_max_2;
    uint8_t cr_min_2, cr_max_2;
    bool draw;

    switch (filter) {
        case 1:
            lum_min = cod_lum_min1;
            lum_max = cod_lum_max1;
            cb_min = cod_cb_min1;
            cb_max = cod_cb_max1;
            cr_min = cod_cr_min1;
            cr_max = cod_cr_max1;
            lum_min_2 = cod_lum_min3;
            lum_max_2 = cod_lum_max3;
            cb_min_2 = cod_cb_min3;
            cb_max_2 = cod_cb_max3;
            cr_min_2 = cod_cr_min3;
            cr_max_2 = cod_cr_max3;

            draw = cod_draw1;
            break;
        case 2:
            lum_min = cod_lum_min2;
            lum_max = cod_lum_max2;
            cb_min = cod_cb_min2;
            cb_max = cod_cb_max2;
            cr_min = cod_cr_min2;
            cr_max = cod_cr_max2;
//      lum_min_2 = cod_lum_min3;
//      lum_max_2 = cod_lum_max3;
//      cb_min_2 = cod_cb_min3;
//      cb_max_2 = cod_cb_max3;
//      cr_min_2 = cod_cr_min3;
//      cr_max_2 = cod_cr_max3;

            draw = cod_draw2;
            break;
        default:
            return img;
    };

    // Filter and find centroid of green and orange colours in centre, left and right parts of image.
    // Centre:
    //      -Green:  Bottom 50% of image;   centered 50% of image;  total 25% of image
    //      -Orange: Bottom 87.5% of image; centered 50% of image;  total 43.75% of image
    uint32_t central_green_count = find_object_centroid(img, &x_c_g, &y_c_g, draw, lum_min, lum_max, cb_min, cb_max,
                                                        cr_min, cr_max, 0, (uint16_t)(img->w / 2),
                                                        (uint16_t)(img->h / 4), (uint16_t)(3 * img->h / 4));
    uint32_t central_orange_count = find_object_centroid(img, &x_c_o, &y_c_o, draw, lum_min_2, lum_max_2, cb_min_2,
                                                         cb_max_2, cr_min_2, cr_max_2, 0, (uint16_t)(7 * img->w / 8),
                                                         (uint16_t)(img->h / 4), (uint16_t)(3 * img->h / 4));


    pthread_mutex_lock(&mutex);

    // Left:
    //      -Green:  Bottom 50% of image;   left 25% of image;  total 12.5% of image
    //      -Orange: Bottom 50% of image;   left 25% of image;  total 12.5% of image
    left_green_count = find_object_centroid(img, &x_c_g_L, &y_c_g_L, draw, lum_min, lum_max, cb_min, cb_max, cr_min,
                                            cr_max, 0, (uint16_t)(img->w / 2), 0, (uint16_t)(img->h / 4));
    left_orange_count = find_object_centroid(img, &x_c_o_L, &y_c_o_L, draw, lum_min_2, lum_max_2, cb_min_2, cb_max_2,
                                             cr_min_2, cr_max_2, 0, (uint16_t)(img->w / 2), 0, (uint16_t)(img->h / 4));

    // Right:
    //      -Green:  Bottom 50% of image;   right 25% of image; total 12.5% of image
    //      -Orange: Bottom 50% of image;   right 25% of image; total 12.5% of image
    right_green_count = find_object_centroid(img, &x_c_g_R, &y_c_g_R, draw, lum_min, lum_max, cb_min, cb_max, cr_min,
                                             cr_max, 0, (uint16_t)(img->w / 2), (uint16_t)(3 * img->h / 4),
                                             (uint16_t)(img->h));
    right_orange_count = find_object_centroid(img, &x_c_o_R, &y_c_o_R, draw, lum_min_2, lum_max_2, cb_min_2, cb_max_2,
                                              cr_min_2, cr_max_2, 0, (uint16_t)(img->w / 2), (uint16_t)(3 * img->h / 4),
                                              (uint16_t)(img->h));

    global_filters[filter - 1].color_count = central_green_count;
    global_filters[filter - 1].x_c = x_c_g;
    global_filters[filter - 1].y_c = y_c_g;
    global_filters[filter - 1].updated = true;
    global_filters[2].color_count = central_orange_count;
    global_filters[2].x_c = x_c_o;
    global_filters[2].y_c = y_c_o;
    global_filters[2].updated = true;
    pthread_mutex_unlock(&mutex);

    return img;
}

struct image_t *object_detector1(struct image_t *img);

struct image_t *object_detector1(struct image_t *img) {
    return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img);

struct image_t *object_detector2(struct image_t *img) {
    return object_detector(img, 2);
}

void color_object_detector_init(void) {
    memset(global_filters, 0, 3 * sizeof(struct color_object_t));
    pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
    cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
    cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
    cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
    cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
    cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
    cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
    cod_lum_min3 = COLOR_OBJECT_DETECTOR_LUM_MIN3;
    cod_lum_max3 = COLOR_OBJECT_DETECTOR_LUM_MAX3;
    cod_cb_min3 = COLOR_OBJECT_DETECTOR_CB_MIN3;
    cod_cb_max3 = COLOR_OBJECT_DETECTOR_CB_MAX3;
    cod_cr_min3 = COLOR_OBJECT_DETECTOR_CR_MIN3;
    cod_cr_max3 = COLOR_OBJECT_DETECTOR_CR_MAX3;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
    cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

    cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
    cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
    cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
    cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
    cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
    cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
    cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
  //#endif
  //  #ifdef COLOR_OBJECT_DETECTOR_LUM_MIN3
    cod_lum_min3 = COLOR_OBJECT_DETECTOR_LUM_MIN3;
    cod_lum_max3 = COLOR_OBJECT_DETECTOR_LUM_MAX3;
    cod_cb_min3 = COLOR_OBJECT_DETECTOR_CB_MIN3;
    cod_cb_max3 = COLOR_OBJECT_DETECTOR_CB_MAX3;
    cod_cr_min3 = COLOR_OBJECT_DETECTOR_CR_MIN3;
    cod_cr_max3 = COLOR_OBJECT_DETECTOR_CR_MAX3;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
    cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

    cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2);
#endif

}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t *p_xc, int32_t *p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint16_t x_min, uint16_t x_max,
                              uint16_t y_min, uint16_t y_max) {
    uint32_t cnt = 0;
    uint32_t tot_x = 0;
    uint32_t tot_y = 0;
    uint8_t *buffer = img->buf;

    // Go through pixels
    for (uint16_t y = y_min; y < y_max; y++) {
        for (uint16_t x = x_min; x < x_max; x++) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;

            // printf("\n x,y = (%d, %d) \n", x, y);
            // Check if x,y in image
            if (x < 0 || x >= img->w || y < 0 || y >= img->h) continue;


            if (x % 2 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }
            if ((*yp >= lum_min) && (*yp <= lum_max) &&
                (*up >= cb_min) && (*up <= cb_max) &&
                (*vp >= cr_min) && (*vp <= cr_max)) {
                cnt++;
                tot_x += x;
                tot_y += y;
                if (draw) {
                    *yp = 255;  // make pixel brighter in image
                }
            }
        }
    }

    if (cnt > 0) {
        *p_xc = (int32_t) roundf(tot_x / ((float) cnt) - img->w * 0.5f);
        *p_yc = (int32_t) roundf(img->h * 0.5f - tot_y / ((float) cnt));
    } else {
        *p_xc = 0;
        *p_yc = 0;
    }
    return cnt;
}

void color_object_detector_periodic(void) {



    static struct color_object_t local_filters[3];

    pthread_mutex_lock(&mutex);
    memcpy(local_filters, global_filters, 3 * sizeof(struct color_object_t));
    pthread_mutex_unlock(&mutex);

    //printf("IMG WIDTH LOCAL = %d\n", local_buffer.buffer[0].w);

    if (local_filters[0].updated) {
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
                                   0, 0, local_filters[0].color_count, 0);
        local_filters[0].updated = false;
    }
    if (local_filters[1].updated) {
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
                                   0, 0, local_filters[1].color_count, 1);
        local_filters[1].updated = false;

    }
    if (local_filters[2].updated) {
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION3_ID, local_filters[2].x_c, local_filters[2].y_c,
                                   0, 0, local_filters[2].color_count, 2);
        local_filters[2].updated = false;

    }


}
