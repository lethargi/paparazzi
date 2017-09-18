/*
 * Copyright (C) paki
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/manan_test/manan_test.h"
 * @author paki
 * testing if nps can compile custom modules
 */

#ifndef VIS_NPS_H
#define VIS_NPS_H

#include <inttypes.h>
#include <unistd.h>

extern uint32_t count_arr[3][3];
extern uint32_t sumcount_arr[3];
extern uint8_t domcol_arr[3];

extern uint8_t print_cvarrs(void);
extern uint8_t cv_3grids(void);

extern float red_thresh, blue_thresh;

#endif
