#include "modules/manan_test/visrl.h"
#include "modules/manan_test/manan_test.h"
#include <stdlib.h>
#include <stdio.h>

static uint8_t ac_for = 0;
static uint8_t ac_lef = 1;
static uint8_t ac_rig = 2;

extern int16_t qtab[5][3];

static uint8_t cur_sta;
static uint8_t last_sta;
static uint8_t cur_rew;

static int8_t reward_function[5] = {-20, -15, -10, -5, 0};

/* States redcount      reward
 * 0 - x==0         - -20
 * 1 - 0<x<20k      - -15
 * 2 - 20k<x<30k    - -10
 * 3 - 40k<x<50k    - -5
 * 4 - x>50k        -  0
 */

int do_visrl(void)
{
    printf("2 C files in 1 modules\n");
    return 0;
}
