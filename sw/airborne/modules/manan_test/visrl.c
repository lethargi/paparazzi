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
static float cur_rew = 0;
// nxt_sta = 0;

// RL parameters
static float rl_gamma = 0.95;
static float rl_alp = 0.35;
static uint8_t rl_eps = 75;

// Some vars for state
// goals_visited 0 for none; 1 for red; 2 for green; 3 for both
uint8_t goals_visited = 0;
uint8_t countfracs[3] = {0,0,0};
// counter for steps and episodes
static uint16_t steps_taken = 0;
uint16_t episodes_simulated = 0;
float episode_rewards = 0;
static float sumofQchanges = 0;
uint32_t total_state_visits = 0;

// file to write and qtable; add descriptions of these files
char qtab_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/cur_qtab.dat";
char qdict_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict.txt";
char qdictkeys_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict_keys.dat";
char qdictvalues_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict_values.dat";
char statevisitsvalues_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits_values.dat";
char statevisits_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits.txt";
char qtab_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qtab.txt";
char log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/log.txt";
char epi_log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/epi_log.txt";
char savelocation[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/";

FILE *qtab_file;
FILE *qdict_txt_file;
FILE *qdictkeys_file;
FILE *qdictvalues_file;
FILE *statevisitsvalues_file;
FILE *qtab_txt_file;
FILE *statevisits_txt_file;
FILE *log_file;
FILE *epi_log_file;

// POSSIBLE MEMORY LEAK!!
// I dont free the dictionary in the end of the program; If further development
// is desired, the dictionary needs to be freed using g_hash_table_destroy(myqdict);
// maybe each element also needs to be freed since they were allocated with calloc.
GHashTable *myqdict;
GHashTable *mystatevisitsdict;
// GSList *rewardlist = NULL;

uint8_t counter = 0;

uint8_t init_qdict(void)
{
    // run only once per training
    myqdict = g_hash_table_new(g_str_hash, g_str_equal);
    mystatevisitsdict = g_hash_table_new(g_str_hash, g_str_equal);
    printf("SizeOfQdict:%d\n",g_hash_table_size(myqdict));
    log_file = fopen(log_file_addrs,"w"); //create or reset logfile
    epi_log_file = fopen(epi_log_file_addrs,"w"); //create or reset epi logfile
    fclose(log_file);
    fclose(epi_log_file);
    return 0;
}

uint8_t size_qdict(void)
{
    printf("SizeOfQdict:%d\n",g_hash_table_size(myqdict));
    return 0;
}

uint8_t rl_dec_eps(void)
{
    rl_eps -= 10;
    printf("\nEps:%d\n",rl_eps);
    return 0;
}
uint8_t rl_inc_eps(void)
{
    rl_eps += 10;
    printf("\nEps:%d\n",rl_eps);
    return 0;
}

uint8_t pick_action(char *state)
{
    uint8_t picked_action = 0;
    uint8_t policy_roll = rand() % 100;
    total_state_visits++;

    // float *currow;
    // printf("\n InPickAction \n");
    float *currow = (float *)g_hash_table_lookup(myqdict,state);
    // printf("\nSizeOfCurrow:%lu\n",sizeof(currow));
    uint16_t *currow_statevisits = (uint16_t *)g_hash_table_lookup(mystatevisitsdict,state);
    // printf("\n BefIf \n");
    if (currow == NULL) {
        // printf("\n NULL \n");
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
        // printf("\n NotNULL \n");
        if (policy_roll < rl_eps) {
        // do greedy
            float maxq = -FLT_MAX;
            for (int i=0; i < 3; i++) {
                // printf("\n%f\n",currow[i]);
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
        // printf("\n BefStateVisits \n");
        // currow_statevisits[picked_action]++;
        // printf("\n AftStateVisits \n");
    }
    // printf(" Visits:%u ", currow_statevisits[picked_action]);
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
    for (int i = 0; i < 3; i++) {
        countfracs[i] = sumcount_arr[i]/5000;
    }

    //check for goals visited
    if (goals_visited == 0) {
        if (countfracs[0] > 7) {
            goals_visited = 1;
        }
        if (countfracs[1] > 7) {
            goals_visited = 2;
        }
    }
    else if (goals_visited == 1) {
        if (countfracs[1] > 7) {
            goals_visited = 3;
        }
    }
    else if (goals_visited == 2) {
        if (countfracs[0] > 7) {
            goals_visited = 3;
        }
    }

    // curstate[30];
    char *curstate = g_strdup_printf("%d,%d,%d;%d,%d;%d;%d",
            domcol_arr[0],domcol_arr[1],domcol_arr[2],
            countfracs[0],countfracs[1],//countfracs[2],
            goals_visited,hitwall);
    printf("Ep:%d Step:%d State:%s ",episodes_simulated+1,steps_taken,curstate);
    return curstate;
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
    cur_rew = 0;
    goals_visited = 0;
    nxt_sta = get_state_ext();
    nxt_act = pick_action(nxt_sta);
    printf("\n RL initialized ");
    printf(" TotVis:%u \n", total_state_visits);
    return 0;
}

uint8_t rl_set_cur(void)
{
    cur_sta = nxt_sta;
    cur_act = nxt_act;
    // printf("Ep:%2u, Step:%3u, Reds:%6d, C_sta:%1d C_act:%1d :: ", episodes_simulated, steps_taken, totredcount, cur_sta, cur_act);
    return 0;
}

uint8_t rl_get_reward(void)
{
    cur_rew = -10;
    if (hitwall == 0) {
        // cur_rew = reward_function[nxt_sta];
        if (goals_visited == 0) {
            cur_rew += countfracs[0]+countfracs[1];
        }
        else if (goals_visited == 1) {
            cur_rew += countfracs[1]; //get reward for green
            cur_rew -= countfracs[0]; //deduct for seeing red
        }
        else if (goals_visited == 2) {
            cur_rew += countfracs[0]; //reward for red
            cur_rew -= countfracs[1]; //deduct for green
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

uint8_t rl_set_nxt(void)
{
    nxt_sta = get_state_ext();
    nxt_act = pick_action(nxt_sta);
    rl_get_reward();

    return 0;
}

uint8_t rl_take_cur_action(void)
{
    if (cur_act == 0) {
        NavSetWaypointHere(WP_BoundaryChecker);
        moveWaypointForwards(WP_BoundaryChecker,0.5);
        // logic for detecting hitting wall
        if (!InsideMyWorld(WaypointX(WP_BoundaryChecker),WaypointY(WP_BoundaryChecker))) {
            hitwall = 1;
            printf(" HITWALL ");
        }
        else {
            moveWaypointForwards(WP_GOAL,0.5);
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
    // this can be stated better
    // if (totredcount < 35000) {
    if (goals_visited != 3) {
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

uint8_t rl_write_step_log(void)
{
    // need to include error checks
    epi_log_file = fopen(epi_log_file_addrs,"a");
    // fprintf(log_file,"%u %u %f %f\n",episodes_simulated,steps_taken,sumofQchanges,episode_rewards);
    fprintf(epi_log_file,"%u %u %s %d %f %f %f \n", episodes_simulated, steps_taken,
            cur_sta, cur_act, cur_rew, GetPosX(), GetPosY());
    fclose(epi_log_file);
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
void statevisits_printer(gpointer key, gpointer value, gpointer user_data)
{
    uint16_t *curarr = (uint16_t *)value;
    // printf(user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
    fprintf(statevisits_txt_file,user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
}
void qdict_writer(gpointer key, gpointer value, gpointer user_data)
{
    uint16_t *curarr = (uint16_t *)value;
    // printf(user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
    fprintf(statevisits_txt_file,user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
}

uint8_t print_qdict(void)
{
    // printf("\n State \t | For \t | Lef \t | Rig");
    qdict_txt_file = fopen(qdict_txt_file_addrs,"w");
    fprintf(qdict_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");
    g_hash_table_foreach(myqdict, (GHFunc)qdict_printer, "%s %03.3f %03.3f %03.3f \n");
    fclose(qdict_txt_file);
    statevisits_txt_file = fopen(statevisits_file_addrs,"w");
    fprintf(statevisits_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");
    // g_hash_table_foreach(mystatevisitsdict, (GHFunc)qdict_printer, "%s \t | %04.3f | %04.3f | %04.3f \n");
    g_hash_table_foreach(mystatevisitsdict, (GHFunc)statevisits_printer, "%s %4u %4u %4u \n");
    fclose(statevisits_txt_file);

    return 0;
}

uint8_t write_qdict(void)
{
    GHashTableIter iter;
    gpointer key, value;
    uint16_t dictsize = g_hash_table_size(myqdict);

    float valarr[dictsize][3];
    uint16_t valvisitsarr[dictsize][3];
    char keyarr[dictsize][30];

    uint16_t dicti = 0;
    g_hash_table_iter_init(&iter, myqdict);
    printf("\n LetsLookInside \n");
    while (g_hash_table_iter_next (&iter, &key, &value))
    {
        float *currow = (float *)value;
        uint16_t *currow_statevisits = (uint16_t *)g_hash_table_lookup(mystatevisitsdict,(char *)key);
        printf("%s: ",(char *)key);
        strcpy(keyarr[dicti],(char *)key);
        for (int i = 0; i < 3; i++) {
            valarr[dicti][i] = currow[i];
            valvisitsarr[dicti][i] = currow_statevisits[i];
            printf("%f ",valarr[dicti][i]);
        }
        printf("\n");
        dicti++;
    }
    printf("\n ===DONE=== \n");

    printf("===============\n");
    printf("\nWriting to file;");
    qdictkeys_file = fopen(qdictkeys_file_addrs,"wb");
    // fwrite(keyarr,sizeof(keyarr[0]),sizeof(keyarr)/sizeof(keyarr[0]),qdictkeys_file);
    fwrite(keyarr,sizeof(char),sizeof(keyarr)/sizeof(char),qdictkeys_file);
    fclose(qdictkeys_file);

    qdictvalues_file = fopen(qdictvalues_file_addrs,"wb");
    fwrite(valarr,sizeof(float),sizeof(valarr)/sizeof(float),qdictvalues_file);
    fclose(qdictvalues_file);

    statevisitsvalues_file = fopen(statevisitsvalues_file_addrs,"wb");
    fwrite(valvisitsarr,sizeof(uint16_t),sizeof(valvisitsarr)/sizeof(uint16_t),statevisitsvalues_file);
    fclose(statevisitsvalues_file);
    printf("Done\n");
    printf("===============\n");
    return 0;
}

uint8_t load_qdict(void)
{
    //safety feature about testing if "myqdict" already exists is unimplemented
    printf("\n ===LoadingQDICT=== \n");

    qdictkeys_file = fopen(qdictkeys_file_addrs,"rb");
    qdictvalues_file = fopen(qdictvalues_file_addrs,"rb");
    statevisitsvalues_file = fopen(statevisitsvalues_file_addrs,"wb");
//     fwrite(valvisitsarr,sizeof(uint16_t),sizeof(valvisitsarr)/sizeof(uint16_t),statevisitsvalues_file);

    char akey[30];
    size_t dummy;

    printf("SizeOfQdict:%d \n",g_hash_table_size(myqdict));
    if ((qdictkeys_file != NULL) && (qdictvalues_file != NULL) && (statevisitsvalues_file != NULL)) {
        while (!feof(qdictkeys_file)) {
            dummy = fread(&akey,sizeof(char[30]),1,qdictkeys_file);
            float *aval = (float *)calloc(3,sizeof(float));
            uint16_t *asvval = (uint16_t *)calloc(3,sizeof(uint16_t));
            for (int i=0 ; i<3 ; i++) {
                dummy = fread(&aval[i],sizeof(float),1,qdictvalues_file);
                dummy = fread(&asvval[i],sizeof(float),1,statevisitsvalues_file);
            }
            g_hash_table_insert(myqdict,g_strdup(akey),aval);
            g_hash_table_insert(mystatevisitsdict,g_strdup(akey),asvval);
        }
        printf("Done\n");
        printf("SizeOfQdict:%d \n",g_hash_table_size(myqdict));
        printf("===============\n");
    }
    else {
        printf("\n NO FILE TO READ \n");
    }
    fclose(qdictkeys_file);
    fclose(qdictvalues_file);
    fclose(statevisitsvalues_file);
    return 0;
}

uint8_t load_qdict_fromtxt(void)
{
    size_t dummy;
    qdict_txt_file = fopen(qdict_txt_file_addrs,"r");
    statevisits_txt_file = fopen(statevisits_file_addrs,"r");
    // qtab_txt_file = fopen(qtab_txt_file_addrs,"r");
    dummy = fscanf(qdict_txt_file, "%*[^\n]");
    dummy = fscanf(statevisits_txt_file, "%*[^\n]");

    char akey[30];
    // char dumm[30];
        while (!feof(qdict_txt_file)) {
            // curline = fscanf(qtab_txt_file, "%*[^\n]");
            float *aval = (float *)calloc(3,sizeof(float));
            uint16_t *asvval = (uint16_t *)calloc(3,sizeof(uint16_t));

            dummy = fscanf(qdict_txt_file, "%s %f %f %f",akey, &aval[0], &aval[1],
                    &aval[2]);
            // fgets(curline,400,qtab_txt_file);

            // sets all state visit values to 0
//             dummy = fscanf(statevisits_txt_file, "%s %f %f %f",akey, &asvval[0], &asvval[1],
//                     &asvval[2]);
            asvval[0] = 0;
            asvval[1] = 0;
            asvval[2] = 0;

            g_hash_table_insert(myqdict,g_strdup(akey),aval);
            g_hash_table_insert(mystatevisitsdict,g_strdup(akey),asvval);

            printf("%s %f %f %f \n",akey, aval[0], aval[1], aval[2]);
        }
    fclose(qtab_txt_file);
    return 0;
}

void qdict_remove(gpointer key, gpointer value, gpointer user_data)
{
    // uint16_t *curarr = (uint16_t *)value;
    free(value);
    free(key);
    // printf(user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
    // fprintf(statevisits_txt_file,user_data, (char *)key, curarr[0], curarr[1], curarr[2]);
}
uint8_t free_qdict(void)
{
    g_hash_table_foreach(myqdict, (GHFunc)qdict_remove, "");
    g_hash_table_destroy(myqdict);
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
