#include "modules/manan_test/visrl.h"
#include "modules/manan_test/manan_test.h"
#include <stdlib.h>
#include <stdio.h>

// think this contains state information from that file
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

#include "generated/flight_plan.h"


static uint8_t ac_for = 0;
static uint8_t ac_lef = 1;
static uint8_t ac_rig = 2;

float qtab[5][3];

static uint8_t rl_isterminal = 0;

static int8_t reward_function[5] = {-20, -15, -10, -5, 0};

static uint8_t cur_sta;
static uint8_t last_sta;
static uint8_t cur_rew;

static float rl_gamma;

/* States redcount      reward
 * 0 - x==0         - -20
 * 1 - 0<x<20k      - -15
 * 2 - 20k<x<30k    - -10
 * 3 - 40k<x<50k    - -5
 * 4 - x>50k        -  0
 */

uint8_t pick_action(void)
{
    uint8_t policy_roll = rand() % 100;
    printf("Policy roll: %d\n", policy_roll);
    return 0;
}

uint8_t do_visrl(void)
{
    last_sta = get_state();
    pick_action();
    // printf("2 C files in 1 modules: %d\n", last_sta);
    return 0;
}

uint8_t get_state(void)
{
    cv_task();
    uint8_t state;

    if (curredcount == 0) {
        state = 0;
    }
    else if (curredcount < 10000) {
        state = 1;
    }
    else if (curredcount < 20000) {
        state = 2;
    }
    else if (curredcount < 30000) {
        state = 3;
    }
    else if (curredcount < 40000) {
        state = 4;
    }
    else if (curredcount < 50000) {
        state = 5;
    }
    else {
        /* Terminal state */
        rl_isterminal = 1;
    }
    return state;
}

