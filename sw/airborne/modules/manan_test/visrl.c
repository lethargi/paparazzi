#include "modules/manan_test/visrl.h"
#include "modules/manan_test/manan_test.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// #include <limits.h>
#include <float.h>

// think this contains state information from that file
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

#define NAV_C
#include "generated/flight_plan.h"


uint32_t totredcount;

uint8_t qtab_rows = 7;
uint8_t qtab_cols = 3;
float qtab[7][3] = { { 0} };  // qtable intialized at 0
// Mapping from state to rewardz
static float reward_function[7] = {-10, -8, -6, -4, -2, -1, 0};

// States, actions and reward
static uint8_t cur_sta, nxt_sta, cur_act, nxt_act;
static uint8_t hitwall = 0;
uint8_t rl_isterminal = 0;
static float cur_rew;
// nxt_sta = 0;

// RL parameters
static float rl_gamma = 0.95;
static float rl_alp = 0.3;
static uint8_t rl_eps = 50;

// counter for steps and episodes
static uint16_t steps_taken = 0;
uint8_t episodes_simulated = 0;
static float sumofQchanges = 0;

// file to write and qtable
char qtab_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/cur_qtab.dat";
char qtab_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qtab.txt";
char log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/log.txt";
char savelocation[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/";
FILE *qtab_file;
FILE *qtab_txt_file;
FILE *log_file;


// uint8_t rl_stepsinep = 0;
// uint8_t rl_  = 0;

uint8_t pick_action(uint8_t state)
{
    uint8_t picked_action = 0;
    // printf(" MAXQ: %f ",maxq);
    uint8_t policy_roll = rand() % 100;
    if (policy_roll < rl_eps) {
        // do greedy
        float maxq = -FLT_MAX;
        for (int i=0; i < 3; i++) {
            if (qtab[state][i] > maxq) {
                picked_action = i;
                maxq = qtab[state][i];
            }
        }
        // printf("Greed: %d\n", picked_action);
    }
    else {
        picked_action = rand() % 3;
        // printf("Rand: %d\n", picked_action);
    }
    return picked_action;
}

uint8_t do_visrl(void)
{
//     cur_sta = get_state();
//     cur_act = pick_action(cur_sta);
    // uint8_t ami_inside = InsideMyWorld(GetPosX(),GetPosY());
    // printf("Inside: %d\n", ami_inside);
    for(uint8_t i = 0; i<5; i++) {
        printf("RewFunc: %3f",reward_function[i]);
    }
    return 0;
}

uint8_t get_state(void)
{
    cv_task();
    uint8_t state = 6;

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
    // printf(": State: %d :",state);
    return state;
}

int visrl_power(int base, int exp);

uint8_t get_state2(void)
{
    cv_3grids();
    // uint8_t state = 6;
    totredcount = 0;
    // uint8_t redpresence[3] = {0,0,0}
    // uint8_t state_binary_array[4] = {0,0,0,0}
    uint8_t state_binary_array[3] = {0,0,0};
    uint8_t stateind = 0;

    // see if each grid has red
    for(int i=0; i<3; i++) {
        uint32_t redcount = redcount_arr[i];
        totredcount += redcount;
        if (redcount > 0) {
            state_binary_array[i] = 1;
            stateind += visrl_power(2,i);
        }
    }

    //normalizing stateind to start from 0
//     if (stateind != 0) {
//         stateind -= 1;
//     }
    printf("\n Totredcount: %d; Stateind: %d \n",totredcount,stateind);

    return 0;
}

int visrl_power(int base, int exp)
{
    if (exp == 0)
        return 1;
    else if (exp % 2)
        return base * visrl_power(base, exp - 1);
    else {
        int temp = visrl_power(base, exp / 2);
        return temp * temp;
    }
}

uint8_t rl_reset_episode(void)
{
        rl_isterminal = 0;
        return 0;
}

uint8_t rl_init(void)
{
    steps_taken = 0;
    sumofQchanges = 0;
    // episodes_simulated++;
    nxt_sta = get_state();
    nxt_act = pick_action(nxt_sta);
    printf("\n RL initialized \n");
    return 0;
}

uint8_t rl_set_cur(void)
{
    cur_sta = nxt_sta;
    cur_act = nxt_act;
    printf("Step:%3u, Reds:%6d, C_sta:%1d C_act:%1d :: ", steps_taken, curredcount, cur_sta, cur_act);
    return 0;
}

uint8_t rl_set_nxt(void)
{
    nxt_sta = get_state();
    nxt_act = pick_action(nxt_sta);
    if (hitwall == 0) {
        cur_rew = reward_function[nxt_sta];
    }
    else {
        hitwall = 0;
        cur_rew = -25;
    }
    printf("N_sta:%1d N_act:%1d Rew:%03.1f :: ",nxt_sta, nxt_act, cur_rew);
    return 0;
}

uint8_t rl_take_cur_action(void)
{
    if (cur_act == 0) {
        NavSetWaypointHere(WP_BoundaryChecker);
        moveWaypointForwards(WP_BoundaryChecker,2);
        // logic for detecting hitting wall
        if (!InsideMyWorld(WaypointX(WP_BoundaryChecker),WaypointY(WP_BoundaryChecker))) {
            hitwall = 1;
            printf(" HITWALL ");
        }
        else {
            moveWaypointForwards(WP_GOAL,2);
        }
    }
    else if (cur_act == 1) {
      increase_nav_heading_deg(&nav_heading, -30);
    }
    else if (cur_act == 2) {
      increase_nav_heading_deg(&nav_heading, 30);
    }
    steps_taken++;
    return 0;
}

uint8_t rl_update_qtab(void)
{
    float Qcur, Qnxt, Qcur1;
    Qcur = qtab[cur_sta][cur_act];
    Qnxt = qtab[nxt_sta][nxt_act];
    Qcur1 = Qcur + rl_alp*(cur_rew + rl_gamma*Qnxt - Qcur);
    qtab[cur_sta][cur_act] = Qcur1;
    sumofQchanges += abs(Qcur1 - Qcur);
    printf("Qcur:%03.1f Qnxt:%03.1f Qcur1:%03.1f :: ", Qcur, Qnxt, Qcur1);
    return 0;
}

uint8_t rl_check_terminal(void)
{
    if (nxt_sta < 5) {
        printf("non-terminal \n");
        rl_isterminal = 0;
    }
    else {
        printf("TERMINAL \n");
        rl_isterminal = 1;
        episodes_simulated++;
    }
    return 0;
}

uint8_t rl_print_qtab(void)
{
    printf("\n");
    printf("================\n");
    printf("Q table\n");
    for(int i = 0; i < 8; i++) {
        printf("%2d : ", i);
        for(int j = 0; j < 3; j++) {
            printf("%3f  ",qtab[i][j]);

        }
        printf("\n");
    }
    printf("\n");
    printf("===============\n");
    return 0;
}

uint8_t rl_write_qtab(void)
{
    // need to include error checks
    printf("===============\n");
    printf("\nWriting to file;");
    qtab_file = fopen(qtab_file_addrs,"wb");
    fwrite(qtab,sizeof(float),sizeof(qtab)/sizeof(float),qtab_file);
    fclose(qtab_file);
    printf("Done\n");
    printf("===============\n");
    return 0;
}

uint8_t rl_read_qtab(void)
{
    // need to include error checks
    printf("===============\n");
    printf("\nReading from file;");
    qtab_file = fopen(qtab_file_addrs,"rb");
    float* mybuffer = (float *)malloc(sizeof(qtab));
//     if (qtab_file == NULL) {
//         printf("\nQ-table Read error\n");
//     }
// 
//     if (fread(qtab,sizeof(float),sizeof(qtab),qtab_file) != sizeof(qtab)) {
//         printf("\nQ-table Read error\n");
//     }


    printf("\nFile opened\n");
    fread(mybuffer,sizeof(float),sizeof(qtab)/sizeof(float),qtab_file);

    uint8_t poscounter = 0;
    for (uint8_t ri = 0; ri < qtab_rows; ri++) {
        for (uint8_t ci = 0; ci < qtab_cols; ci++) {
            qtab[ri][ci] = *(mybuffer+poscounter);
            printf("%f  ",qtab[ri][ci]);
            poscounter++;
        }
        printf("\n");
    }
    free(mybuffer);
    // printf("Freed\n");
    fclose(qtab_file);
    // printf("Closed\n");


    printf("Done\n");
    printf("===============\n");
    return 0;

}

uint8_t rl_write_episode_log(void)
{
    // need to include error checks
    printf("===============\n");
    printf("\nWriting to file;");
    log_file = fopen(log_file_addrs,"a");
    fprintf(log_file,"%d %d %f \n",episodes_simulated,steps_taken,sumofQchanges);
    // fwrite(qtab,sizeof(float),sizeof(qtab)/sizeof(float),qtab_file);
    fclose(log_file);
    printf("Done\n");
    printf("===============\n");
    return 0;
}

uint8_t rl_print_qtab_to_file(void)
{
    printf("\n");
    printf("================\n");
    printf("Q table\n");
    qtab_txt_file = fopen(qtab_txt_file_addrs,"w");
    for(int i = 0; i < 8; i++) {
        fprintf(qtab_txt_file,"%2d : ", i);
        for(int j = 0; j < 3; j++) {
            fprintf(qtab_txt_file,"%3f  ",qtab[i][j]);

        }
        printf("\n");
    }
    printf("\n");
    printf("===============\n");
    return 0;
}


/* TrashBin
// uint8_t rl_print_test(void)
// {
//     static int16_t inttoprint = 0;
//     inttoprint++;
//     printf("The int is: %d\n", inttoprint);
//     return 0;
// }
//
// uint8_t test_pick_action(void) 
// {
//     uint8_t anaction;
//     anaction = pick_action(nxt_sta);
//     return 0;
// }
//
*/
