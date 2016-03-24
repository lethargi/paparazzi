/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "subsystems/datalink/telemetry.h"

// FOR ALGEBRA WITH HEADING ANGLES
#include "math/pprz_algebra_int.h"
#include "modules/computer_vision/opencv_example.h"

uint8_t safeToGoForwards=FALSE;
int tresholdColorCount = 4000;
int32_t incrementForAvoidance;
int32_t incrementForBehind = 180;

void send_msg(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_GROUP_NINE(trans, dev, AC_ID, &isSafeToGoForwards);
}

void orange_avoider_init() {
	// Initialise the variables of the colorfilter to accept orange
	// color_lum_min=0;
	// color_lum_max=131;
	// color_cb_min=93;
	// color_cb_max=255;
	// color_cr_min=134;
	// color_cr_max=255;
	// // Initialise random values
	// srand(time(NULL));
	// chooseRandomIncrementAvoidance();
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GROUP_NINE, send_msg);
}
void orange_avoider_periodic() {
	// Check the amount of orange. If this is above a threshold
	// you want to turn a certain amount of degrees
	// safeToGoForwards = color_count < tresholdColorCount;
    // printf("Heading: %d", nav_heading);
	// printf("Checking if this funciton is called %d treshold: %d now: %d \n", color_count, tresholdColorCount, safeToGoForwards);
}


/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */

uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}

/* The following functions are to yaw the craft in different directions */
uint8_t increase_nav_heading_deg(int32_t *heading, int32_t increment)
{
  // calculate new heading in weird units
  int32_t increment_int32t = ANGLE_BFP_OF_REAL(RadOfDeg(increment));
  // int32_t new_heading = *heading + increment_int32t;
  *heading = *heading + increment_int32t;
  // Check if your turn made it go out of bounds...
  // INT32_COURSE_NORMALIZE(*new_heading);
  INT32_COURSE_NORMALIZE(*heading);

  // upload the nav_heading
  // *heading = new_heading;
  return FALSE;
}
/*
uint8_t yaw(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}
*/

/* The following functions are to move the waypoints in 4 directions relative
 * to the craft
 */
uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t moveWaypointBackwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x - POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y - POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t moveWaypointLeftwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x - POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t moveWaypointRightwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.y = pos->y - POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t chooseRandomIncrementAvoidance(){

	int r = rand() % 2;
	if(r==0){
		incrementForAvoidance=350;
	}
	else{
		incrementForAvoidance=-350;
	}
	return FALSE;
}
