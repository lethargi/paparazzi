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

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result
int color_count = 0;

uint16_t colcnt[3];
uint16_t colcnt_grids[3][3];


void my_image_yuv422_colorcounter(struct image_t *input);
void my_image_yuv422_colorcounter(struct image_t *input)
{
    uint8_t *source = input->buf;
    float red,green,blue,pixcolsum;
    float f_red,f_green,f_blue;
    // printf("Inside colorcounter\n");
    for (uint8_t ii = 0; ii < 3; ii++) { colcnt[ii] = 0; }
    for (uint8_t ii = 0; ii < 3; ii++) {
        for (uint8_t jj = 0; jj <3; jj++) {
            colcnt_grids[ii][jj] = 0;
        }
    }

    uint16_t w_first, w_second;
    uint8_t coltoapp;
    w_first = input->w/3;
    w_second = 2*w_first;

    // Copy the creation timestamp (stays the same)
    // output->ts = input->ts;

  // Go trough all the pixels
    for (uint16_t y = 0; y < input->h; y++) {
        for (uint16_t x = 0; x < input->w; x += 2) {
        // Check if the color is inside the specified values
        /*
            if (
            (dest[1] >= y_m)
            && (dest[1] <= y_M)
            && (dest[0] >= u_m)
            && (dest[0] <= u_M)
            && (dest[2] >= v_m)
            && (dest[2] <= v_M)
            ) {
            cnt ++;
            // UYVY
            dest[0] = 64;        // U
            dest[1] = source[1];  // Y
            dest[2] = 255;        // V
            dest[3] = source[3];  // Y
            } else {
            // UYVY
            char u = source[0] - 127;
            u /= 4;
            dest[0] = 127;        // U
            dest[1] = source[1];  // Y
            u = source[2] - 127;
            u /= 4;
            dest[2] = 127;        // V
            dest[3] = source[3];  // Y
            }
            */

            // printf("Inside colorcounter: bef red \n");
            // printf("%d,%d : %d %d", y,x,source[1], source[2]);
            red = (float )source[1] + 1.4075*((float )source[2] - (float )128);
            // printf(" %2.3f \n", red);
            green = (float )source[1] - 0.3455*((float )source[0] - (float )128) - ((float )0.7169 * ((float )source[2] - (float )128));
            blue = (float )source[1] + 1.779*((float )source[0] - (float )128);
            pixcolsum = red+green+blue;

            // printf("Inside colorcounter: bef float \n");
            f_red = red/pixcolsum;
            f_green = green/pixcolsum;
            f_blue = blue/pixcolsum;

            // printf("Inside colorcounter: bef if \n");
            if (x < w_first) {
                coltoapp = 0;
            } else if (x < w_second) {
                coltoapp = 1;
            } else {
                coltoapp = 2;
            }

            if (f_red > 0.40) {
                source[0] = 10;        // U
                source[2] = 240;        // V
                colcnt_grids[coltoapp][0]++;
            }
            else if (f_green > 0.40) {
                source[0] = 10;        // U
                source[2] = 10;        // V
                colcnt_grids[coltoapp][1]++;
            }
            else if (f_blue > 0.40) {
                source[0] = 240;        // U
                source[2] = 10;        // V
                colcnt_grids[coltoapp][2]++;
            } else {
                source[0] = 127;        // U
                source[2] = 127;        // V
            }
            /*
            */


            // Go to the next 2 pixels
            // dest += 4;
            source += 4;
        }
    }
    // printf("End for \n");
    // return false;
}

// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
  // int red_cnt = 0;
  my_image_yuv422_colorcounter(img);
//   color_count = image_yuv422_colorfilt(img, img,
//                                        color_lum_min, color_lum_max,
//                                        color_cb_min, color_cb_max,
//                                        color_cr_min, color_cr_max
//                                       );


  printf("\tR\tG\tB\n");
  for (int ri = 0; ri < 3; ri++) {
      printf("col%d:",ri);
      for (int ci = 0; ci <3; ci++) {
        printf("\t%d",colcnt_grids[ri][ci]);
      }
    printf("\n");
  }
  // printf("InColorfilter R:%d G:%d B:%d\n",colcnt[0],colcnt[1],colcnt[2]);
  // printf("InColorfilter %d\n",color_count);
  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}
