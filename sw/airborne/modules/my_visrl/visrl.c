#include "modules/my_visrl/visrl.h"

#include "modules/my_visrl/mynavfuncs.h"
#include "modules/my_visrl/mydict.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// #include <limits.h>
#include <float.h>


// think this contains state information from that file
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot.h"

#define NAV_C
#include "generated/flight_plan.h"

#include "subsystems/datalink/telemetry.h"


float headings_rad[8] = {0, M_PI/4., M_PI/2., 3*M_PI/4.,
                        M_PI, 5*M_PI/4. , 3*M_PI/2. , 7*M_PI/4.};
uint8_t len_headings = 8;
int8_t headind = 0;
uint8_t headatcompass = 1;

// States, actions and reward
char *state_buffer;
static char *act_type = "R";
static uint8_t cur_act = 0;
static uint8_t nxt_act = 0;
static uint8_t start_option = 0; //boolean to check if performing option
static uint8_t end_option = 1; //boolean to check if performing option
static char cur_sta[VISRL_STATESIZE], nxt_sta[VISRL_STATESIZE], option_start_sta[VISRL_STATESIZE];
// static char *cur_sta, *nxt_sta;
// static char *option_start_sta;  //stores the starting state of options for Qtab updates
static uint8_t hitwall = 0;
uint8_t rl_isterminal = 0;
static float cur_rew = 0;

uint16_t rl_maxepochs = 50;

float rl_gamma = 0.9;
float rl_alp = 0.3;
uint8_t rl_eps = 60;

// Some vars for state
// goals_visited 0 for none; 1 for red; 2 for green; 3 for both
uint8_t goals_visited = 0;
float countfracs[3] = {0,0,0};
// counter for steps and episodes
static uint16_t steps_taken = 0;
uint16_t episodes_simulated = 0;
float episode_rewards = 0;
static float sumofQchanges = 0;
uint32_t total_state_visits = 0;

int8_t head_roll;

// file to write and qtable; add descriptions of these files
// FILE LOCATIONS DURING NPS RUNS
#ifdef VISRL_NPS
char qdict_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict.txt";
char qdictkeys_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict_keys.dat";
char qdictvalues_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict_values.dat";
char statevisitsvalues_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits_values.dat";
char statevisits_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits.txt";
char log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/log.txt";
char epi_log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/epi_log.txt";
char savelocation[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/";
char copy_location[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/__LastSaves/";
#else
// FILE LOCATION WHEN RUNNING IN THE UAV
char qdict_txt_file_addrs[] = "/home/default/qdict.txt";
char qdictkeys_file_addrs[] = "/home/default/qdict_keys.dat";
char qdictvalues_file_addrs[] = "/home/default/qdict_values.dat";
char statevisitsvalues_file_addrs[] = "/home/defaultstatevisits_values.dat";
char statevisits_file_addrs[] = "/home/default/statevisits.txt";
char log_file_addrs[] = "/home/default/log.txt";
char epi_log_file_addrs[] = "/home/default/epi_log.txt";
char copy_location[] = "/home/default/";
// savelocation commented out for UAV implementation; may need fixing
// char savelocation[] = "/home/default/_Study/AE9999_Thesis/playground/SavedQtabs/";
#endif


FILE *qdict_txt_file;
FILE *qdictkeys_file;
FILE *qdictvalues_file;
FILE *statevisitsvalues_file;
FILE *statevisits_txt_file;
FILE *log_file;
FILE *epi_log_file;

md_linkedlist *ll_qdict;


//send message about vision output
static void send_visrl(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VISRL(trans, dev, AC_ID,
                            &count_arr[0][0],&count_arr[0][1],&count_arr[0][2],
                            &count_arr[1][0],&count_arr[1][1],&count_arr[1][2],
                            &domcol_arr[0],&domcol_arr[1],&domcol_arr[2]);
//                         &airspeed, &v_ctl_auto_airspeed_setpoint,
//                         &v_ctl_auto_airspeed_controlled, &v_ctl_auto_groundspeed_setpoint);
}

void visrl_init(void)
{
    // send message
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISRL, send_visrl);

    // initalize other required stuffs
    log_file = fopen(log_file_addrs,"w"); //create or reset logfile
    epi_log_file = fopen(epi_log_file_addrs,"w"); //create or reset epi logfile
    fclose(log_file);
    fclose(epi_log_file);

    ll_qdict = md_init_linkedlist();
}

uint8_t rl_dec_eps(void)
{
    rl_eps -= 5;
    printf("\nEps:%d\n",rl_eps);
    return 0;
}
uint8_t rl_inc_eps(void)
{
    rl_eps += 5;
    printf("\nEps:%d\n",rl_eps);
    return 0;
}
uint8_t rl_inc_maxepochs(void)
{
    rl_maxepochs += 50;
    printf("\nMaxEpochs:%d\n",rl_maxepochs);
    return 0;
}

uint8_t pick_action(char *mystate)
{
#ifdef VISRL_USEOPTIONS
    // if performing option of turning till color, return option action
    if (cur_act == 3) {
        start_option = 0; //already performing option
        uint8_t seeing_color = 0;
        // printf(" Colors: ");
        for (int i = 0; i < 3; i++) {
            if (domcol_arr[i]) { // using the basic color array; maybe shud use the state info
                seeing_color = 1;
            }
            // printf(" %d ",seeing_color);
        }
        if (!seeing_color) {
            printf(" %d %d ",start_option, end_option);
            return 3;
        }
        else {
            end_option = 1;
        }
    }
#endif

    uint8_t picked_action = 0;
    total_state_visits++;

    md_node *curnode;
    curnode = md_search(ll_qdict,mystate);

    if (curnode == NULL) {
        curnode = md_prepend_list(ll_qdict,mystate);
        picked_action = rand() % 4;
    }
    else {
        uint8_t policy_roll = rand() % 100;
        if (policy_roll < rl_eps) {
            // do greedy
            act_type = "G";
            md_node_best_action(curnode);
            picked_action = curnode->best_action;
        }
        else {
            // do random
            act_type = "R";
            picked_action = rand() % 4;
        }
        curnode->visits[picked_action]++;
    }
    // printf(" Visits:%u ", currow_statevisits[picked_action]);

#ifdef VISRL_USEOPTIONS
    // if picking option, set boolean to true;
    if ((picked_action == 3) && (cur_act == 3)) {
        start_option = 1;
        end_option = 1;
    }
    else if (picked_action == 3) {
        start_option = 1;
        end_option = 0;
    }
    printf(" %d %d ",start_option, end_option);
#endif
    return picked_action;
}

void get_state_ext(char *curstate)
{
    cv_3grids();        // function only used to get cv data during simulation
    for (int i = 0; i < 3; i++) {
        countfracs[i] = (float)sumcount_arr[i]/(float)5000;
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

    // char curstate[30];
    // char *curstate = malloc( sizeof(char)*20 );
    sprintf(curstate,"%d,%d,%d;%d;%d",
            domcol_arr[0],domcol_arr[1],domcol_arr[2],
            // countfracs[0],countfracs[1],//countfracs[2],
            // goals_visited,hitwall,headind); // with headindex
            goals_visited,hitwall);      // without headingindex

    // printf("Ep:%d Step:%d State:%s ",episodes_simulated+1,steps_taken,curstate);
    printf("%3d %4d",episodes_simulated+1,steps_taken);
}

float get_myheading(void)
{
    float myhead = GetCurHeading();
    myhead = (myhead > 0)? myhead : 2*M_PI + myhead;
    return myhead;
}

void update_headind(void)
{
    float myhead = get_myheading();
    float hby2 = M_PI/8;
    uint8_t bin_i = 0;
    if ((myhead < hby2) || (myhead >= headings_rad[7] + hby2)) {
        headind = bin_i;
    }
    else {
        for (bin_i=1; bin_i<8;bin_i++){
            if ((myhead >= headings_rad[bin_i-1] + hby2) &&
                (myhead < headings_rad[bin_i] + hby2)) {
                    headind = bin_i;
                }
        }
    }
}

uint8_t rl_smooth_turn(uint8_t targhead_ind)
{
    update_headind();
    int8_t dh_i = targhead_ind - headind;
    if (dh_i == 0) {
        return FALSE;
    }

    if (abs(dh_i) > 4) {
        dh_i = (dh_i > 0)? 8 - dh_i : 8 + dh_i;
    }

    dh_i = GetSign(dh_i);

    float targ_heading = headings_rad[headind+dh_i];
    set_nav_heading(targ_heading);
    return TRUE;
}

uint8_t rl_randomize_start(uint8_t waypoint, uint8_t altref_wp)
{
    struct EnuCoor_i new_coor;

    float x_roll = (float) (rand() % 70 - 35)/10;
    float y_roll = (float) (rand() % 70 - 35)/10;
    while (!InsideMyWorld(x_roll,y_roll)) {
        x_roll = (float) (rand() % 70 - 35)/10;
        y_roll = (float) (rand() % 70 - 35)/10;
    }
    float refalt = waypoint_get_alt(altref_wp);
    head_roll = rand() % 8;
    printf("\n %f %f %f %d %f \n", x_roll,y_roll,refalt,head_roll,headings_rad[head_roll]);

    // Now determine where to place the waypoint you want to go to
    new_coor.x = POS_BFP_OF_REAL(x_roll);
    new_coor.y = POS_BFP_OF_REAL(y_roll);
    new_coor.z = refalt; // Keep the height the same
    waypoint_move_enu_i(waypoint,&new_coor);

//     uint8_t notdone = TRUE;
//     while(notdone) {
//         notdone = rl_smooth_turn(head_roll);
//     };

//     int8_t turndir = head_roll - headind;
//     int8_t turndir = (head_roll > headind)? +1 : -1;
//     int8_t curtargdir = heading + turndir;
//     curtargdir = (curtargdir>7)? 0 : curtargdir;
//     curtargdir = (curtargdir<0)? 7 : curtargdir;
//     while (headind != head_roll) {
// 
//         set_nav_heading(headings_rad[curtargdir])
//     }
//     set_nav_heading(headings_rad[head_roll]);
    // autopilot_guided_goto_ned(x_roll, y_roll, refalt, headings_rad[head_roll]);
    return FALSE;
}

uint8_t rl_init(void)
{
    rl_isterminal = 0;
    steps_taken = 0;
    sumofQchanges = 0;
    episode_rewards = 0;
    cur_rew = 0;
    goals_visited = 0;
    rl_set_nxt();
    // headind = 0;
    update_headind();
    printf("\n RL initialized ");
    printf(" TotVis:%u \n", total_state_visits);
    printf("Ep Steps Options Rew :: cur_sta cur_act nxt_sta nxt_act : Qcur Qnxt Qcur1 ::\n");
    return 0;
}

uint8_t rl_set_cur(void)
{
    strcpy(cur_sta,nxt_sta);
    cur_act = nxt_act;
    // If starting an options copy the starting state into a buffer
#ifdef VISRL_USEOPTIONS
    if (start_option) {
        strcpy(option_start_sta,cur_sta);
    }
#endif
    return 0;
}

uint8_t rl_get_reward(void)
{
    cur_rew = -10;
    if (hitwall == 0) {
        // cur_rew = reward_function[nxt_sta];
        if (goals_visited == 0) {
            cur_rew += countfracs[0]+countfracs[1];
            //cur_rew += (countfracs[0]+countfracs[1])*3;
        }
        else if (goals_visited == 1) {
            cur_rew += countfracs[1]; //get reward for green
            cur_rew -= countfracs[0]; //deduct for seeing red
        }
        else if (goals_visited == 2) {
            cur_rew += countfracs[0]; //reward for red
            cur_rew -= countfracs[1]; //deduct for green
        }

        // if we are selecting an option include extra penalty
        if (cur_act == 3) {
            cur_rew -= 10;
        }
    }
    else {
        hitwall = 0;
        cur_rew = -20;
    }
    // printf("Rew:%02.1f ",cur_rew);
    printf(" %03.1f ",cur_rew);
#ifdef VISRL_USEOPTIONS
    if (end_option == 1) {
        episode_rewards += cur_rew;
    }
#else
    episode_rewards += cur_rew;
#endif
    return 0;
}

uint8_t rl_set_nxt(void)
{
//     state_buffer = get_state_ext();
//     strcpy(nxt_sta,state_buffer);
    get_state_ext(nxt_sta);
    nxt_act = pick_action(nxt_sta);
    rl_get_reward();
    // free(state_buffer);

    return 0;
}

uint8_t rl_take_cur_action(void)
{
    if (cur_act == 0) {
        NavSetWaypointHere(WP_BoundaryChecker);
        moveWaypointForwards(WP_BoundaryChecker,0.6);
        // logic for detecting hitting wall
        if (!InsideMyWorld(WaypointX(WP_BoundaryChecker),WaypointY(WP_BoundaryChecker))) {
            hitwall = 1;
        }
        else {
            if (headatcompass == 1) {
                moveWaypointForwards(WP_GOAL,0.5);
            }
            else {
                moveWaypointForwards(WP_GOAL,0.70710678118);
            }
        }
    }
    else {
        headatcompass = (headatcompass == 1) ? 0 : 1;
        // select index of heading from headings_rad array
        if (cur_act == 1) {
            // increase_nav_heading_deg(&nav_heading, -45);
            headind--;
            if (headind == -1) {
                headind = len_headings - 1;
            }
        }
        else if ((cur_act == 2) || (cur_act == 3)) {
            // increase_nav_heading_deg(&nav_heading, 45);
            headind++;
            if (headind == len_headings) {
                headind = 0;
            }
        }
        // set the heading
        set_nav_heading(headings_rad[headind]);

    }
    steps_taken++;
    return FALSE;
}

uint8_t rl_update_qdict(void)
{
    printf(":: %s %u  %s %u :", cur_sta, cur_act, nxt_sta, nxt_act);
#ifdef VISRL_USEOPTIONS
    // if running options
    if (cur_act == 3) {
        // if option didnt end, dont update qtab
        if (!end_option) {
            printf(" No Q updates \n");
            return 0;
        }
        else {
            // starting one option and ending another; must update for t-1
            // (i.e. Qcur).
            if (start_option) {
                end_option = 0;
            }
            strcpy(cur_sta,option_start_sta);
        }
    }
#endif
    // printf("SizeOfQdict:%d ;",g_hash_table_size(myqdict));
    float Qcur, Qnxt, Qcur1;

    md_node *qtab_curnode = md_search(ll_qdict,cur_sta); //(float *)g_hash_table_lookup(myqdict,cur_sta);
    md_node *qtab_nxtnode = md_search(ll_qdict,nxt_sta); // (float *)g_hash_table_lookup(myqdict,nxt_sta);
    Qcur = qtab_curnode->values[cur_act];
    Qnxt = qtab_nxtnode->values[nxt_act];
    Qcur1 = Qcur + rl_alp*(cur_rew + rl_gamma*Qnxt - Qcur);
    qtab_curnode->values[cur_act] = Qcur1;
    sumofQchanges += abs(Qcur1 - Qcur);
    printf("%03.1f %03.1f %03.1f :: \n", Qcur, Qnxt, Qcur1);
    return 0;
}

uint8_t rl_check_terminal(void)
{
    // this can be stated better
    // if (goals_visited != 3) {
    if (goals_visited != 1) {
        rl_isterminal = 0;
    }
    else {
        printf("TERMINAL :: Sum of rewards: %f\n",episode_rewards);
        rl_isterminal = 1;
        episodes_simulated++;
    }
    return 0;
}

uint8_t rl_write_episode_log(void)
{
    // need to include error checks
    printf("===============\n");
    printf("\nWriting to file;");
    log_file = fopen(log_file_addrs,"a");
    fprintf(log_file,"%u %u %.1f %.1f %u\n",episodes_simulated,steps_taken,
            sumofQchanges,episode_rewards,ll_qdict->length);
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
    // Save the RL data for the step
    fprintf(epi_log_file,"%3u %4u %s %d %s % 3.0f", episodes_simulated, steps_taken,
            cur_sta, cur_act, act_type, cur_rew);
    // Save absolute state data
    fprintf(epi_log_file," % 05.2f % 04.3f % 04.3f", DegOfRad(GetCurHeading()), GetPosX(),
            GetPosY());
    // save comamand data
    fprintf(epi_log_file," % 05.2f % 04.3f % 04.3f",
            DegOfRad(ANGLE_FLOAT_OF_BFP(nav_heading)), WaypointX(WP_GOAL),
            WaypointY(WP_GOAL));
    fprintf(epi_log_file,"\n");
    fclose(epi_log_file);
    return 0;
}

// functions to read and write Q-tables; can be improved overall
uint8_t print_qdict(void)
{
    // printf("\n====== Writing to txt file =========\n");
    md_export_to_text(ll_qdict, qdict_txt_file_addrs, statevisits_file_addrs);
    return 0;
}

uint8_t write_qdict(void)
{
    // printf("\n====== Writing to dat file =========\n");
    md_export_to_dat(ll_qdict, qdictkeys_file_addrs, qdictvalues_file_addrs,
            statevisitsvalues_file_addrs);
    // printf("======= Done ==========\n");
    return 0;
}

uint8_t load_qdict(void)
{
    //safety feature about testing if "myqdict" already exists is unimplemented
    // printf("\n ===Loading QDICT from dat=== \n");
    md_import_from_dat(ll_qdict, qdictkeys_file_addrs, qdictvalues_file_addrs,
            statevisitsvalues_file_addrs);
    // printf("\n ===Done=== \n");
    return 0;
}

uint8_t load_qdict_fromtxt(void)
{
    // printf("\n ===Loading QDICT from txt=== \n");
    md_import_from_text(ll_qdict, qdict_txt_file_addrs,
            statevisits_file_addrs);
    // printf("\n ===Done=== \n");
    return 0;
}

uint8_t copy_file(char *old_filename, char  *new_filename)
{
    FILE  *ptr_old, *ptr_new;
    int  a;

    ptr_old = fopen(old_filename,"rb");
    ptr_new = fopen(new_filename,"wb");

    while(1)
    {
        a  =  fgetc(ptr_old);

        if(!feof(ptr_old))
            fputc(a, ptr_new);
        else
            break;
    }

    fclose(ptr_new);
    fclose(ptr_old);
    return  0;
}

uint8_t copy_qdict(void)
{
    char acopyloc[200];
    //https://stackoverflow.com/questions/8257714/how-to-convert-an-int-to-string-in-c
    int length = snprintf( NULL, 0, "%s%d/", copy_location, episodes_simulated);
    char* path = malloc( length + 1 );
    snprintf( path, length + 1, "%s%d/", copy_location, episodes_simulated);
    mkdir(path,0777);

    strcpy(acopyloc,path);
    strcat(acopyloc,"statevisits.txt");
    copy_file(statevisits_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict.txt");
    copy_file(qdict_txt_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict_keys.dat");
    copy_file(qdictkeys_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict_values.dat");
    copy_file(qdictvalues_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"statevisits.dat");
    copy_file(statevisitsvalues_file_addrs,acopyloc);

    free(path);
    return FALSE;


}

uint8_t copy_logs(void)
{
    char acopyloc[200];

    strcpy(acopyloc,copy_location);
    strcat(acopyloc,"epi_log_cp.txt");
    copy_file(epi_log_file_addrs,acopyloc);

    strcpy(acopyloc,copy_location);
    strcat(acopyloc,"log_cp.txt");
    copy_file(log_file_addrs,acopyloc);

    return FALSE;
}
