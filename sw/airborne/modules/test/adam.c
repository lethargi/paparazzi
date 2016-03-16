/*
 * Copyright (C) Dr Manan, Dr Stephen
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/test/adam.c"
 * @author Dr Manan, Dr Stephen
 * a module for foreplay with paparazzi
 */

#include <stdio.h>
#include "modules/test/adam.h"

// for sending telemetry
#include "subsystems/datalink/telemetry.h"

// for flight plan
#include "generated/flight_plan.h"

// cause Martin imports it
#include "firmwares/rotorcraft/navigation.h"

double myvar = 2.51;

void send_msg(struct transport_tx *trans, struct link_device *dev)
{
    // int aba = 5;
    pprz_msg_send_GROUP_NINE(trans, dev, AC_ID, &myvar);
}

void increase_myvar(double *x)
{
    *x = *x + 1.5;
}

void init_adam() {
    printf("We have liftoff!");
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GROUP_NINE, send_msg);
}


void print_data() {
    // printf("UR STUPID!");
    increase_myvar(&myvar);
}


