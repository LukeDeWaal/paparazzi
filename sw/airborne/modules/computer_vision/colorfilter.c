/*
 * Copyright (C) 2015
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
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 50;
uint8_t color_lum_max = 200;
uint8_t color_cb_min  = 120;
uint8_t color_cb_max  = 130;
uint8_t color_cr_min  = 120;
uint8_t color_cr_max  = 130;

// Result
volatile int color_count = 0;

#include "subsystems/abi.h"

static void colorfilter(struct image_t *input, struct image_t *output, int Y_min, int Y_max, int U_min, int U_max, int V_min, int V_max){

    for(int y = (int)(input->h/2); y < input->h; ++y){ // Just bottom Half of picture
        for(int x = (int)(input->w/4); x < (int)3*input->w/4; ++x){ // Just Central part of picture
            printf("\n x,y = (%d, %d) \n", x, y);
            // Check if x,y in image
            if (x < 0 || x >= input->w || y < 0 || y >= input->h) continue;

            if (check_color_yuv422(input, x, y, Y_min, Y_max, U_min, U_max, V_min, V_max)){
                set_color_yuv422(output, x, y, 255, 128, 128);  // If pixel within bounds, make it white
                color_count++;
            }
            else{
                set_color_yuv422(output, x, y, 0, 128, 128);    // Else make it black
            }
        }
    }
}

// Function
static struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
    colorfilter(img, img,
                color_lum_min, color_lum_max,
                color_cb_min,  color_cb_max,
                color_cr_min,  color_cr_max
                );

  if (COLORFILTER_SEND_OBSTACLE) {
    if (color_count > 20)
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
    }
    else
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
    }
  }

  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}
