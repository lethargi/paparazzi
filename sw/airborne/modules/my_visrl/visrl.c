#include "modules/my_visrl/visrl.h"

#include "modules/my_visrl/mynavfuncs.h"

#include "modules/my_visrl/simsoft.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// #include <limits.h>
#include <float.h>


// think this contains state information from that file
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"

#include "state.h"
#include "autopilot.h"

#define NAV_C
#include "generated/flight_plan.h"

#include "subsystems/datalink/telemetry.h"


// float headings_rad[8] = {0, M_PI/4., M_PI/2., 3*M_PI/4.,
//                         M_PI, 5*M_PI/4. , 3*M_PI/2. , 7*M_PI/4.};
// uint8_t len_headings = 8;
float headings_rad[16] = {0, M_PI/8., M_PI/4., 3*M_PI/8., M_PI/2., 5*M_PI/8.,
        3*M_PI/4., 7*M_PI/8., M_PI, 9*M_PI/8., 5*M_PI/4.,  11*M_PI/8, 3*M_PI/2. ,
        13*M_PI/8, 7*M_PI/4., 15*M_PI/8};
uint8_t len_headings = 16;
int8_t headind = 0;
int8_t cur_targ_headind = 0;
// uint8_t headatcompass = 1;
uint8_t init_headind;

// States, actions and reward
char *state_buffer;
char *act_type = "R";
uint8_t cur_act = 0;
uint8_t nxt_act = 0;
char cur_sta[VISRL_STATESIZE], nxt_sta[VISRL_STATESIZE];
float step_wait_time = 1.0;

#ifdef VISRL_USEOPTIONS
uint8_t start_option = 0; //boolean to check if performing option
uint8_t end_option = 1; //boolean to check if performing option
char option_start_sta[VISRL_STATESIZE];
#endif

// static char *cur_sta, *nxt_sta;
// static char *option_start_sta;  //stores the starting state of options for Qtab updates
uint8_t hitwall = 0;
uint8_t rl_isterminal = 0;
float cur_rew = 0;
float opts_rew = 0;

// uint16_t rl_maxepsinc = 1;
// uint16_t rl_maxeps = 1;

uint16_t rl_maxsteps = 5000;

float rl_gamma = 0.9;
float rl_alp = 0.3;
int8_t rl_eps = 60;

#ifdef VISRL_NPS
uint8_t red_goal_reach_thresh = 4;
uint8_t blue_goal_reach_thresh = 3;
#else
uint8_t red_goal_reach_thresh = 4;
uint8_t blue_goal_reach_thresh = 3;
#endif

int16_t entanglement_count = 0;
int8_t stop_after_episode = 0;

// Some vars for state
// goals_visited 0 for none; 1 for red; 2 for green; 3 for both
uint8_t goals_visited = 0;
float countfracs[2] = {0,0};
// counter for steps and episodes
uint16_t steps_taken = 0;
uint16_t epinum = 0;
float episode_rewards = 0;
float sum_dQ = 0;

uint32_t total_state_visits;

md_linkedlist *ll_qdict;

//send message about vision output
static void send_visrl(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VISRL(trans, dev, AC_ID, 3, count_arr[0], 3, count_arr[1], 3, domcol_arr, 2, sumcount_arr);
}

void visrl_init(void)
{
    // send message
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISRL, send_visrl);

    // initalize other required stuffs
    simsoft_init();

    // rl_initmaxeps = rl_curmaxeps;
    ll_qdict = md_init_linkedlist();

    total_state_visits = 0;
    // rl_curmaxeps = rl_maxepsinc;
    endsess = 0;
#ifdef VISRL_AP
    vis_ap_init();
#endif
    printf("\nmy_visrl intialized \n");
}

void rl_get_random_cords(float *cords)
{
    // float cords[2];
    cords[0] = (float) (rand() % 70 - 35)/10;
    cords[1] = (float) (rand() % 70 - 35)/10;
    while (!InsideMyWorld(cords[0],cords[1])) {
        cords[0] = (float) (rand() % 70 - 35)/10;
        cords[1] = (float) (rand() % 70 - 35)/10;
    }
}

uint8_t rl_init_uav(void)
{
    struct EnuCoor_i new_coor;
    float init_cords[2];
    float refalt;

#ifdef VISRL_RANDOMSTARTS
    rl_get_random_cords(init_cords);
    init_headind = rand() % 16;
    // rl_randomize_start(WP_GOAL);
#else
    init_cords[0] = waypoints[WP_StartPos].enu_f.x;
    init_cords[1] = waypoints[WP_StartPos].enu_f.y;

#ifdef VISRL_NPS
    init_headind = 0; //dont understan why these are these values
#else
    init_headind = 10; // will need to check for real implement
#endif

#endif

    refalt = waypoints[WP_StartPos].enu_i.z;
    new_coor.x = POS_BFP_OF_REAL(init_cords[0]);
    new_coor.y = POS_BFP_OF_REAL(init_cords[1]);
    new_coor.z = refalt; // Keep the height the same

    waypoint_move_enu_i(WP_GOAL,&new_coor);
    // rl_smooth_turn(init_headind);
    return 0;
}

uint8_t pick_action(char *mystate)
{
    uint8_t possible_actions = VISRL_ACTIONS;
    // uint8_t possible_actions;
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
            // step_wait_time = 1.0;
            printf(" %d %d ",start_option, end_option);
            return 3;
        }
        else {
            end_option = 1;
        }
    }

    // if seeing any color, do not use options
    if ((*(mystate) != '0') || (*(mystate+2) != '0') || (*(mystate+4) != '0')) {
        possible_actions = 3;
        // printf("NoOpt: SeeingColor ");
    }
#endif

//     printf("\nChars %c %c %c %c %c\n", *(mystate),*(mystate+1),*(mystate+2),*(mystate+3),*(mystate+4));
    uint8_t picked_action = 0;
    total_state_visits++;

    md_node *curnode;
    curnode = md_search(ll_qdict,mystate);

    if (curnode == NULL) {
        printf(" NewState ");
        curnode = md_prepend_list(ll_qdict,mystate);
        picked_action = rand() % possible_actions;
        act_type = "R";
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
            picked_action = rand() % possible_actions;
        }
    }
    curnode->visits[picked_action]++;

    // set time to wait; if forward 1sec; if turn 0.15 sec
    if ((picked_action == 0)) { // || (picked_action == 3)) {
        step_wait_time = 1.0;
    }
    else {
        step_wait_time = 0.5;
    }

#ifdef VISRL_USEOPTIONS
    // if picking option, set boolean to true;
    if (picked_action == 3) {
        // this never happens as its not possible to select options twice in
        // row
        if (cur_act == 3) {
            start_option = 1;
            end_option = 1;
            printf("-------INSIDE REDUNDANT LOOP------");
        }
        else {
            start_option = 1;
            end_option = 0;
        }
        opts_rew = 0;
    }
//     if ((picked_action == 3) && (cur_act == 3)) {
//     }
//     else if (picked_action == 3) {
//     }
    printf(" %d %d ",start_option, end_option);
#endif
    return picked_action;
}

void get_state_ext(char *curstate)
{
    uint8_t sta_cfrac;
// #ifdef VISRL_NPS
    cv_3grids();        // function only used to get cv data during simulation
// #endif
    for (int i = 0; i < 2; i++) {
        countfracs[i] = (float)sumcount_arr[i]/(float)5000;
    }

    //check for goals visited
    if (goals_visited == 0) {
        if (countfracs[0] > red_goal_reach_thresh) {
            goals_visited = 1;
        }
#ifdef VISRL_TWOGOALS
        if (countfracs[1] > blue_goal_reach_thresh) {
            goals_visited = 2;
        }
    }
    else if (goals_visited == 1) {
        if (countfracs[1] > blue_goal_reach_thresh) {
            goals_visited = 3;
        }
    }
    else if (goals_visited == 2) {
        if (countfracs[0] > red_goal_reach_thresh) {
            goals_visited = 3;
        }
    }
    sta_cfrac = ((int) floor(countfracs[0]+countfracs[1]) > red_goal_reach_thresh) ? red_goal_reach_thresh: countfracs[0]+countfracs[1] ;
#else
    }
    sta_cfrac = ((int) floor(countfracs[0]) > red_goal_reach_thresh) ? red_goal_reach_thresh: countfracs[0] ;
#endif

    // uint8_t sta_cfrac = ((int) floor(countfracs[0]) > red_goal_reach_thresh) ? red_goal_reach_thresh: countfracs[0] ;
    // sprintf(curstate,"%d,%d,%d;%d;%d",
    sprintf(curstate,"%d,%d,%d;%d;%d;%d",
            domcol_arr[0],domcol_arr[1],domcol_arr[2],
            sta_cfrac,
            // countfracs[0],countfracs[1],//countfracs[2],
            // goals_visited,hitwall,headind); // with headindex
            goals_visited,hitwall);      // without headingindex


    printf("%3d %4d",epinum,steps_taken);
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
    // float hby2 = M_PI/8;
    float hby2 = M_PI/16;
    uint8_t bin_i = 0;
    if ((myhead < hby2) || (myhead >= headings_rad[15] + hby2)) {
        headind = bin_i;
    }
    else {
        for (bin_i=1; bin_i<16;bin_i++){
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

    if (abs(dh_i) > 8) {
        dh_i = (dh_i > 0)? 16 - dh_i : 16 + dh_i;
    }

    dh_i = GetSign(dh_i);

    float targ_heading = headings_rad[headind+dh_i];
    nav_set_heading_rad(targ_heading);
    return TRUE;
}

uint8_t rl_init_ep(void)
{
    cv_3grids();        // function only used to get cv data during simulation
#ifdef VISRL_USEOPTIONS
    start_option = 0;
    end_option = 0;
#endif
    rl_isterminal = 0;
    steps_taken = 0;
    sum_dQ = 0;
    episode_rewards = 0;
    cur_rew = 0;
    goals_visited = 0;
    ep_success = 1;
    epinum++;
    // headind = 0;
    rl_set_nxt();
    update_headind();
    printf("\n Episode initialized \n");
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
    // Increased penalty if going forward while not seeing color
    // float rew_base; turing always has a lower penalty
    if (cur_act == 0){
        // small base penalty if going univisted goal in sight
        if (((goals_visited == 0) && ((domcol_arr[0]+domcol_arr[1]+domcol_arr[2]) > 0 )) ||
            ((goals_visited == 1) && ((domcol_arr[0] == 2) || (domcol_arr[1] == 2) || (domcol_arr[2] == 2))) ||
            ((goals_visited == 2) && ((domcol_arr[0] == 1) || (domcol_arr[1] == 1) || (domcol_arr[2] == 1)))) {
            cur_rew = -10;
        }
        else {
            cur_rew = -20;
        }
    }
    else {
        cur_rew = -15;
    }

    // flat big penalty if hit a wall in last state
    // May consider this putting this at the end so that hitting walls is
    // always strongly penalized;regardless of seeing goals or not.
    if (hitwall) {
        hitwall = 0;
        cur_rew = -100;
    }

    // reward for seeing unvisited goal and penalty for seeing visited goal
    // (CAN MAYBE MAKE THESE GLOBAL)
    float rew_factor = 2;
    float same_goal_pen_factor = 2;
    if (goals_visited == 0) {
        cur_rew += (countfracs[0]+countfracs[1])*rew_factor;
    }
    else if (goals_visited == 1) {
        cur_rew += (countfracs[1])*rew_factor; //get reward for blue
        if ((domcol_arr[0] == 1) || (domcol_arr[1] == 1) || (domcol_arr[2] == 1)) {
            cur_rew -= (countfracs[0])*same_goal_pen_factor + 10; //deduct for seeing red
        }
    }
    else if (goals_visited == 2) {
        cur_rew += (countfracs[0])*rew_factor; //reward for red
        if ((domcol_arr[0] == 2) || (domcol_arr[1] == 2) || (domcol_arr[2] == 2)) {
            cur_rew -= (countfracs[1])*same_goal_pen_factor + 10; //deduct for blue
        }
    }

    // Big reward if just visited a new goal
    if (*(cur_sta+8) != *(nxt_sta+8)) {
        cur_rew += 100;
    }
    // if (() || (*(mystate+2) != '0') || (*(mystate+4) != '0')) {
    // if we are selecting an option include extra penalty

#ifdef VISRL_USEOPTIONS
    if (cur_act == 3) {
        opts_rew += cur_rew;
        printf(" OptsRew:%.1f ", opts_rew);
        cur_rew = opts_rew;
        // cur_rew -= 20;
    }

    if (end_option == 1) {
        episode_rewards += cur_rew;
    }
#else
    episode_rewards += cur_rew;
#endif
    printf(" %03.1f ",cur_rew);
    return 0;
}

uint8_t rl_set_nxt(void)
{
//     state_buffer = get_state_ext();
//     strcpy(nxt_sta,state_buffer);
    get_state_ext(nxt_sta);
    nxt_act = pick_action(nxt_sta);
    rl_get_reward();
    rl_write_step_log();
    // free(state_buffer);

    return 0;
}

uint8_t rl_headind_normalize(int8_t inheadind)
{
    uint8_t outheadind;
    if (inheadind == -1) {
        outheadind = len_headings - 1;
    }
    else if (inheadind == len_headings) {
        outheadind = 0;
    }
    else {
        outheadind = inheadind;
    }
    return outheadind;
}

uint8_t rl_turn_to_targheadind(void)
{
    // printf("\n INSIDE TURN_TO_TARGHEADIND \n");
    if (headind == cur_targ_headind) {
        return FALSE;
    }

    float targ_heading = headings_rad[cur_targ_headind];
    nav_set_heading_rad(targ_heading);
    update_headind();
    return TRUE;
}

void rl_action_forward(void)
{
    NavSetWaypointHere(WP_BoundaryChecker);
    moveWaypointForwards(WP_BoundaryChecker,0.6);
    // logic for detecting hitting wall
    if (!InsideMyWorld(WaypointX(WP_BoundaryChecker),WaypointY(WP_BoundaryChecker))) {
        hitwall = 1;
    }
    else {
        moveWaypointForwards(WP_GOAL,0.5);
//         if (headatcompass == 1) {
//             moveWaypointForwards(WP_GOAL,0.5);
//         }
//         else {
//             moveWaypointForwards(WP_GOAL,0.70710678118);
//         }
    }
}

void rl_left2(void)
{
    cur_targ_headind = headind - 1;
    cur_targ_headind = rl_headind_normalize(cur_targ_headind);
    entanglement_count++;
    // printf("\n RL_LEFT2:: ENTANG:%d :: headind:%d :: targHeadind: %d\n", entanglement_count,headind,cur_targ_headind);
}

void rl_right2(void)
{
    cur_targ_headind = headind + 1;
    cur_targ_headind = rl_headind_normalize(cur_targ_headind);
    entanglement_count--;
    // printf("\n RL_RIGHT2:: ENTANG:%d :: headind:%d :: targHeadind: %d\n", entanglement_count,headind,cur_targ_headind);
}

uint8_t rl_action_left(void)
{
    rl_left2();
    return rl_turn_to_targheadind();
//     headind--;
//     if (headind == -1) {
//         headind = len_headings - 1;
//     }
//     nav_set_heading_rad(headings_rad[headind]);
//     entanglement_count++;
}

void rl_action_right(void)
{
    headind++;
    if (headind == len_headings) {
        headind = 0;
    }
    nav_set_heading_rad(headings_rad[headind]);
    entanglement_count--;
}

uint8_t rl_unentangle_tether(void)
{
    // printf("ENTANG:%d :: headind:%d :: targHeadind: %d\n", entanglement_count,headind,cur_targ_headind);
    if (entanglement_count) {
        if (headind == cur_targ_headind) {
            if (entanglement_count > 0) {
                rl_right2();
            }
            else if (entanglement_count < 0) {
                rl_left2();
            }
        }
    }
    else {
        // update_headind();
        headind = cur_targ_headind;
        return FALSE;
    }
    rl_turn_to_targheadind();
    return TRUE;
    // }
}

uint8_t rl_take_cur_action(void)
{
    steps_taken++;
    if (cur_act == 0) {
        rl_action_forward();
    }
    else {
        // headatcompass = (headatcompass == 1) ? 0 : 1;
        // select index of heading from headings_rad array
        if (cur_act == 1) {
            rl_left2();
            // return rl_action_left();
        }
        else if ((cur_act == 2) || (cur_act == 3)) {
            rl_right2();
            // rl_action_right();
        }
        rl_turn_to_targheadind();
        headind = cur_targ_headind;
    }
    return FALSE;
}

uint8_t rl_update_qdict(void)
{
    printf(":: %s %u  %s %u :", cur_sta, cur_act, nxt_sta, nxt_act);
    uint8_t update_act;
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

    // QLEARNING
    // Maybe i shouldnt be using strcmp here
#ifdef VISRL_SARSA
    update_act = nxt_act;
#else
    if (!strcmp(act_type,"G")) {
       update_act = nxt_act;
    }
    else {
       md_node_best_action(qtab_nxtnode);
       update_act = qtab_nxtnode->best_action;
    }
#endif

    Qcur = qtab_curnode->values[cur_act];
    Qnxt = qtab_nxtnode->values[update_act];
    Qcur1 = Qcur + rl_alp*(cur_rew + rl_gamma*Qnxt - Qcur);
    qtab_curnode->values[cur_act] = Qcur1;
    sum_dQ += abs(Qcur1 - Qcur);
    printf("%03.1f %03.1f %03.1f :: \n", Qcur, Qnxt, Qcur1);
    return 0;
}

uint8_t rl_check_terminal(void)
{
    /* This bit to or development; Makes episodes end fast
    if (steps_taken > 3) {
        rl_isterminal = 1;
        // goals_visited = 1;
//         if (steps_taken < rl_maxsteps) {
//             steps_taken += rl_maxsteps;
//         }
//         else {
//             return 0;
//         }
    }
    */

    // check for end of run and session
    // printf("\n Epinum:%d :: rl_maxeps-1:%d :: endrun:%d \n",epinum,rl_maxeps-1,endrun);
    if (epinum > rl_max_episodes_limit-1) { endrun = 1; }
    if (runnum > rl_maxruns-1) { endsess = 1; }

#ifdef VISRL_TWOGOALS
    if (goals_visited == 3) {
#else
    if (goals_visited == 1) {
#endif
        printf("TERMINAL :: Sum of rewards: %f\n",episode_rewards);
        rl_isterminal = 1;
        sequential_failed_episodes = 0;
    }
    else if (steps_taken > rl_maxsteps-1) {
        ep_success = 0;
        printf("\n====MaxSteps reached :: FORCED TERMINATION====\n");
        rl_isterminal = 1;
        ++sequential_failed_episodes;
        ++failed_episodes_count;
        if (sequential_failed_episodes > 3) {
            endrun = 1;
            run_success = 0;
        }
    }
    return 0;
}
