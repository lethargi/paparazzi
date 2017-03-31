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
//extern void my_period(void);

// Making amacro to get current heading (phi)
#define GetCurHeading() (stateGetNedToBodyEulers_i()->psi)

extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t increase_nav_heading_deg(int32_t *heading, int32_t increment);

extern int printheading(int32_t *heading);

extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointBackwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointLeftwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointRightwards(uint8_t waypoint, float distanceMeters);


//extern size_t write_data(void *ptr, size_t size, size_t nmemb, void *stream);
//extern int runcurltomem(void);

extern uint8_t update_redsatheading(void);
extern uint8_t print_redsatheading(void);
extern uint8_t cv_task(void);

#endif

