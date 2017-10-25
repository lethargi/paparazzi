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
uint8_t headatcompass = 1;

// States, actions and reward
char *state_buffer;
char *act_type = "R";
uint8_t cur_act = 0;
uint8_t nxt_act = 0;
char cur_sta[VISRL_STATESIZE], nxt_sta[VISRL_STATESIZE];

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

// uint16_t rl_max_eps_epochs = 100;
// uint16_t rl_max_eps_epochs_increase = 100;
// uint16_t rl_max_train_epochs = 900;
// uint8_t rl_eps_increase = 5;


uint16_t rl_curmaxeps = 50;
uint16_t rl_maxepsinc = 50;
uint16_t rl_maxeps = 450;
uint8_t rl_eps_increase = 5;

float rl_gamma = 0.9;
float rl_alp = 0.3;
uint8_t rl_eps = 60;

#ifdef VISRL_NPS
uint8_t red_goal_reach_thresh = 5;
uint8_t blue_goal_reach_thresh = 3;
#else
uint8_t red_goal_reach_thresh = 4;
uint8_t blue_goal_reach_thresh = 3;
#endif

// Some vars for state
// goals_visited 0 for none; 1 for red; 2 for green; 3 for both
uint8_t goals_visited = 0;
float countfracs[2] = {0,0};
// counter for steps and episodes
uint16_t steps_taken = 0;
uint16_t episodes_simulated = 0;
float episode_rewards = 0;
float sum_dQ = 0;
uint32_t total_state_visits = 0;

int8_t head_roll;

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

    ll_qdict = md_init_linkedlist();
#ifdef VISRL_AP
    vis_ap_init();
#endif
}

uint8_t rl_dec_eps(void)
{
    rl_eps -= 5;
    printf("\nEps:%d\n",rl_eps);
    return 0;
}
uint8_t rl_inc_eps(void)
{
    rl_eps += rl_eps_increase;
    printf("\nEps:%d\n",rl_eps);
    return 0;
}
uint8_t rl_inc_maxepochs(void)
{
    rl_curmaxeps += rl_maxepsinc;
    printf("\nMaxEpochs:%d\n",rl_curmaxeps);
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
        printf(" NewState ");
        curnode = md_prepend_list(ll_qdict,mystate);
        picked_action = rand() % VISRL_ACTIONS;
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
            picked_action = rand() % VISRL_ACTIONS;
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
#ifdef VISRL_NPS
    cv_3grids();        // function only used to get cv data during simulation
#endif
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
#endif
    }
#ifdef VISRL_TWOGOALS
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
#endif

    // char curstate[30];
    // char *curstate = malloc( sizeof(char)*20 );
    // sprintf(curstate,"%d,%d,%d;%d;%d",
    uint8_t sta_cfrac = ((int) floor(countfracs[0]) > red_goal_reach_thresh) ? red_goal_reach_thresh: countfracs[0] ;
    sprintf(curstate,"%d,%d,%d;%d;%d;%d",
            domcol_arr[0],domcol_arr[1],domcol_arr[2],
            sta_cfrac,
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

uint8_t rl_reset_heading(void)
{
    uint8_t targhead;
#ifdef VISRL_NPS
    targhead = 0; //dont understan why these are these values
#else
    targhead = 10; // will need to check for real implement
#endif
    return rl_smooth_turn(targhead);;
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
    head_roll = rand() % 16;
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
     sum_dQ = 0;
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
    float rew_factor = 1.5;
    if (hitwall == 0) {
        // cur_rew = reward_function[nxt_sta];
        if (goals_visited == 0) {
            cur_rew += (countfracs[0]+countfracs[1])*rew_factor;
            //cur_rew += (countfracs[0]+countfracs[1])*3;
        }
        else if (goals_visited == 1) {
            cur_rew += (countfracs[1])*rew_factor; //get reward for green
            cur_rew -= (countfracs[0])*rew_factor; //deduct for seeing red
        }
        else if (goals_visited == 2) {
            cur_rew += (countfracs[0])*rew_factor; //reward for red
            cur_rew -= (countfracs[1])*rew_factor; //deduct for green
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

void rl_action_forward(void)
{
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

void rl_action_left(void)
{
    headind--;
    if (headind == -1) {
        headind = len_headings - 1;
    }
    nav_set_heading_rad(headings_rad[headind]);
}

void rl_action_right(void)
{
    headind++;
    if (headind == len_headings) {
        headind = 0;
    }
    nav_set_heading_rad(headings_rad[headind]);
}

uint8_t rl_take_cur_action(void)
{
    if (cur_act == 0) {
        rl_action_forward();
    }
    else {
        headatcompass = (headatcompass == 1) ? 0 : 1;
        // select index of heading from headings_rad array
        if (cur_act == 1) {
            rl_action_left();
        }
        else if ((cur_act == 2) || (cur_act == 3)) {
            rl_action_right();
        }
    }
    steps_taken++;
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
    // this can be stated better
#ifdef VISRL_TWOGOALS
    if (goals_visited != 3) {
#else
    if (goals_visited != 1) {
#endif
        rl_isterminal = 0;
    }
    else {
        printf("TERMINAL :: Sum of rewards: %f\n",episode_rewards);
        rl_isterminal = 1;
        episodes_simulated++;
    }
    return 0;
}
