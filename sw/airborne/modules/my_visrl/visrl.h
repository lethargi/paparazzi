#ifndef MYVISRL_H
#define MYVISRL_H

#define VISRL_STATESIZE 20

// #define VISRL_USEOPTIONS
#ifdef VISRL_USEOPTIONS
#define VISRL_ACTIONS 4
#else
#define VISRL_ACTIONS 3
#endif

#include <stdint.h>
// #include <glib.h>

#ifdef VISRL_NPS
#include "modules/my_visrl/vis_nps.h"
#endif
// #else
#ifdef VISRL_AP
#include "modules/my_visrl/vis_ap.h"
#endif

#define GetSign(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )
// #define GetCurX() (stateGetNedToBodyEulers_i()->psi)
// #define GetCurY() (stateGetNedToBodyEulers_i()->psi)

// #define VISRL_USEOPTIONS
#include "modules/my_visrl/mydict.h"


// Variables for the CV functions
extern uint32_t count_arr[2][3];
extern uint32_t sumcount_arr[2];
extern uint8_t domcol_arr[3];

extern float red_thresh, blue_thresh;
extern uint16_t min_pix_thresh;
extern uint8_t red_goal_reach_thresh;
extern uint8_t blue_goal_reach_thresh;

// extern GHashTable *myqdict;
extern uint16_t rl_curmaxeps;
extern uint16_t rl_maxepsinc;
extern uint16_t rl_maxeps;
extern uint8_t rl_eps_increase;

extern int8_t headind;
extern int8_t head_roll;

extern void visrl_init(void);

// extern uint8_t rl_randomize_start(uint8_t waypoint);
extern uint8_t rl_randomize_start(uint8_t waypoint, uint8_t altref_wp);
extern uint8_t rl_smooth_turn(uint8_t targhead_ind);
// extern uint8_t rl_turntoheadroll(void);

extern uint8_t rl_init(void);
extern uint8_t rl_set_cur(void);
extern uint8_t rl_set_nxt(void);

extern uint8_t rl_take_cur_action(void);
extern void rl_action_forward(void);
extern void rl_action_left(void);
extern void rl_action_right(void);


extern uint8_t rl_update_qdict(void);
extern uint8_t rl_check_terminal(void);
extern uint8_t rl_print_qtab(void);

extern uint8_t rl_isterminal;
extern uint16_t episodes_simulated;
extern float episode_rewards;

extern uint8_t rl_eps;
extern float rl_gamma;
extern float rl_alp;

void get_state_ext(char *curstate);
float get_myheading(void);
void update_headind(void);
uint8_t rl_reset_heading(void);

uint8_t pick_action(char *state);
uint8_t rl_get_reward(void);
uint8_t copy_file(char *old_filename, char  *new_filename);

extern md_linkedlist *ll_qdict;

extern uint8_t rl_dec_eps(void);
extern uint8_t rl_inc_eps(void);
extern uint8_t rl_inc_maxepochs(void);


// Declaration of some variables for global tracking

extern uint16_t steps_taken;
extern uint16_t episodes_simulated;
extern float episode_rewards;
extern float sum_dQ;
extern uint32_t total_state_visits;
extern char *act_type;
extern uint8_t cur_act;
extern uint8_t nxt_act;
extern char cur_sta[VISRL_STATESIZE], nxt_sta[VISRL_STATESIZE];

extern int8_t head_roll;

#ifdef VISRL_USEOPTIONS
extern uint8_t start_option; //boolean to check if performing option
extern uint8_t end_option; //boolean to check if performing option
extern char option_start_sta[VISRL_STATESIZE];
#endif

extern int8_t headind;
extern uint8_t headatcompass;

extern uint8_t hitwall;
extern uint8_t rl_isterminal;
extern float cur_rew;
#endif

