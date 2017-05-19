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


// counters for vision, state and terminal condition
uint32_t totredcount;

uint8_t qtab_rows = 7;
uint8_t qtab_cols = 3;
// float qtab[7][3] = { { 0} };  // qtable intialized at 0
float qtab[8][3] = { { 0} };  // qtable intialized at 0
// Mapping from state to rewardz
// static float reward_function[7] = {-10, -8, -6, -4, -2, -1, 0};

// States, actions and reward
static uint8_t cur_act, nxt_act;
static char *cur_sta, *nxt_sta;
static uint8_t hitwall = 0;
uint8_t rl_isterminal = 0;
static float cur_rew;
// nxt_sta = 0;

// RL parameters
static float rl_gamma = 0.95;
static float rl_alp = 0.35;
static uint8_t rl_eps = 75;

// Some vars for state
// goals_visited 0 for none; 1 for red; 2 for green; 3 for both
uint8_t goals_visited = 0;
// counter for steps and episodes
static uint16_t steps_taken = 0;
uint16_t episodes_simulated = 0;
float episode_rewards = 0;
static float sumofQchanges = 0;

// file to write and qtable
char qtab_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/cur_qtab.dat";
char statevisits_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits.txt";
char qtab_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qtab.txt";
char log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/log.txt";
char savelocation[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/";

FILE *qtab_file;
FILE *qtab_txt_file;
FILE *statevisits_txt_file;
FILE *log_file;

GHashTable *myqdict;
GHashTable *mystatevisitsdict;
// GSList *rewardlist = NULL;

uint8_t counter = 0;

uint8_t init_qdict(void)
{
    myqdict = g_hash_table_new(g_str_hash, g_str_equal);
    mystatevisitsdict = g_hash_table_new(g_str_hash, g_str_equal);
    printf("SizeOfQdict:%d\n",g_hash_table_size(myqdict));
    return 0;
}

uint8_t size_qdict(void)
{
    printf("SizeOfQdict:%d\n",g_hash_table_size(myqdict));
    return 0;
}

uint8_t pick_action(char *state)
{
    uint8_t picked_action = 0;
    uint8_t policy_roll = rand() % 100;

    // float *currow;
    float *currow = (float *)g_hash_table_lookup(myqdict,state);
    uint16_t *currow_statevisits = (uint16_t *)g_hash_table_lookup(mystatevisitsdict,state);
    if (currow == NULL) {
        currow = (float *)calloc(3,sizeof(float));
        currow_statevisits = (uint16_t *)calloc(3,sizeof(uint16_t));
        for (int i = 0; i < 3; i++) {
            currow[i] = 0;
            currow_statevisits[i] = 0;
        }
        g_hash_table_insert(myqdict,g_strdup(state),currow);

        picked_action = rand() % 3;
        currow_statevisits[picked_action]++;
        g_hash_table_insert(mystatevisitsdict,g_strdup(state),currow_statevisits);
    }
    else {
        if (policy_roll < rl_eps) {
        // do greedy
            float maxq = -FLT_MAX;
            for (int i=0; i < 3; i++) {
                if (currow[i] > maxq) {
                    picked_action = i;
                    maxq = currow[i];
                }
            }
        }
        else {
            // do random
            picked_action = rand() % 3;
        }
        currow_statevisits[picked_action]++;
    }
    printf(" Visits:%u ", currow_statevisits[picked_action]);
    printf(" Action:%u ", picked_action);
    return picked_action;
}

uint8_t get_state_old(void)
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

uint8_t get_state(void)
{
    cv_3grids();
    totredcount = 0;
    // uint8_t state_binary_array[3] = {0,0,0};
    uint8_t stateind = 0;

    // see if each grid has red
    for(int i=0; i<3; i++) {
        if (redcount_arr[i] > 0) {
            totredcount += redcount_arr[i];
            // state_binary_array[i] = 1;
            stateind += visrl_power(2,i);
        }
    }
    return stateind;
}

char *get_state_ext(void)
{
    cv_3grids();
    uint8_t countfracs[3] = {0,0,0};
    for (int i = 0; i < 3; i++) {
        countfracs[i] = sumcount_arr[i]/5000;
    }
    char *curstate = g_strdup_printf("%d,%d,%d;%d,%d,%d;%d",
            domcol_arr[0],domcol_arr[1],domcol_arr[2],
            countfracs[0],countfracs[1],countfracs[2],
            goals_visited);
    printf("Ep:%d Step:%d State:%s ",episodes_simulated+1,steps_taken,curstate);
    return curstate;
}

int visrl_power(int base, int exp)
{
    /* function to calculate exponents */
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
    episode_rewards = 0;
    // episodes_simulated++;
    nxt_sta = get_state_ext();
    nxt_act = pick_action(nxt_sta);
    printf("\n RL initialized \n");
    return 0;
}

uint8_t rl_set_cur(void)
{
    cur_sta = nxt_sta;
    cur_act = nxt_act;
    // printf("Ep:%2u, Step:%3u, Reds:%6d, C_sta:%1d C_act:%1d :: ", episodes_simulated, steps_taken, totredcount, cur_sta, cur_act);
    return 0;
}

uint8_t rl_set_nxt(void)
{
    nxt_sta = get_state_ext();
    nxt_act = pick_action(nxt_sta);
    if (hitwall == 0) {
        // cur_rew = reward_function[nxt_sta];
        if (sumcount_arr[0] > 20000) {
            cur_rew = -2;
        }
        if (sumcount_arr[0] > 10000) {
            cur_rew = -4;
        }
        else if (sumcount_arr[0] > 1000) {
            cur_rew = -6;
        }
        else if (sumcount_arr[0] > 500) {
            cur_rew = -8;
        }
        else {
            cur_rew = -10;
        }
    }
    else {
        hitwall = 0;
        cur_rew = -20;
    }
    printf("Rew:%02.1f ",cur_rew);


    episode_rewards += cur_rew;
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

// uint8_t rl_update_qtab(void)
// {
//     float Qcur, Qnxt, Qcur1;
//     Qcur = qtab[cur_sta][cur_act];
//     Qnxt = qtab[nxt_sta][nxt_act];
//     Qcur1 = Qcur + rl_alp*(cur_rew + rl_gamma*Qnxt - Qcur);
//     qtab[cur_sta][cur_act] = Qcur1;
//     sumofQchanges += abs(Qcur1 - Qcur);
//     printf("Qcur:%03.1f Qnxt:%03.1f Qcur1:%03.1f :: ", Qcur, Qnxt, Qcur1);
//     return 0;
// }

uint8_t rl_update_qdict(void)
{
    printf("SizeOfQdict:%d ;",g_hash_table_size(myqdict));
    float Qcur, Qnxt, Qcur1;
    float *qtab_currow =  (float *)g_hash_table_lookup(myqdict,cur_sta);
    float *qtab_nxtrow =  (float *)g_hash_table_lookup(myqdict,nxt_sta);
    Qcur = qtab_currow[cur_act];
    Qnxt = qtab_nxtrow[nxt_act];
    Qcur1 = Qcur + rl_alp*(cur_rew + rl_gamma*Qnxt - Qcur);
    qtab_currow[cur_act] = Qcur1;
    sumofQchanges += abs(Qcur1 - Qcur);
    printf("Qcur:%03.1f Qnxt:%03.1f Qcur1:%03.1f :: \n", Qcur, Qnxt, Qcur1);
    return 0;
}

uint8_t rl_check_terminal(void)
{
    // if (totredcount < 35000) {
    if (sumcount_arr[0] < 45000) {
        // printf("non-terminal \n");
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
    size_t readbytes = fread(mybuffer,sizeof(float),sizeof(qtab)/sizeof(float),qtab_file);

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
    fprintf(log_file,"%u %u %f %f\n",episodes_simulated,steps_taken,sumofQchanges,episode_rewards);
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
        fprintf(qtab_txt_file,"\n");
    }
    fclose(qtab_txt_file);
    printf("\n");
    printf("===============\n");
    return 0;
}

void qdict_printer(gpointer key, gpointer value, gpointer user_data)
{
    float *curarr = (float *)value;
    // printf(user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
    fprintf(qtab_txt_file,user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
}
void qdict_printer2(gpointer key, gpointer value, gpointer user_data)
{
    uint16_t *curarr = (uint16_t *)value;
    // printf(user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
    fprintf(qtab_txt_file,user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
}

uint8_t print_qdict(void)
{
    // printf("\n State \t | For \t | Lef \t | Rig");
    qtab_txt_file = fopen(qtab_txt_file_addrs,"w");
    fprintf(qtab_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");
    g_hash_table_foreach(myqdict, (GHFunc)qdict_printer, "%s \t | %03.3f | %03.3f | %03.3f \n");
    fclose(qtab_txt_file);
    statevisits_txt_file = fopen(statevisits_file_addrs,"w");
    fprintf(statevisits_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");
    // g_hash_table_foreach(mystatevisitsdict, (GHFunc)qdict_printer, "%s \t | %04.3f | %04.3f | %04.3f \n");
    g_hash_table_foreach(mystatevisitsdict, (GHFunc)qdict_printer2, "%s \t | %4u | %4u | %4u \n");
    fclose(statevisits_txt_file);
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
