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
#include <inttypes.h>
#include <unistd.h>

#ifndef MANAN_TEST_H
#define MANAN_TEST_H

extern void my_init(void);

// Making amacro to get current heading (phi)
// use ANGLE_FLOAT_OF_BFP for real value??
#define GetCurHeading() (stateGetNedToBodyEulers_f()->psi)

extern uint8_t set_nav_heading(float newheading);
extern uint8_t increase_nav_heading(float increment);
extern uint8_t setHeadingNorth(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointLeftwards(uint8_t waypoint, float distanceMeters);
extern uint8_t setAltToWp(uint8_t waypoint_toset, uint8_t waypoint_ref);

extern uint32_t count_arr[3][3];
extern uint32_t sumcount_arr[3];
extern uint8_t domcol_arr[3];

extern uint8_t print_cvarrs(void);
extern uint8_t cv_3grids(void);

#endif
