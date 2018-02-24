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


// Variables and thresholds for the CV functions
extern uint32_t count_arr[2][3];
extern uint32_t sumcount_arr[2];
extern uint8_t domcol_arr[3];

extern float red_thresh, blue_thresh;
extern uint16_t min_pix_thresh;
extern uint8_t red_goal_reach_thresh;
extern uint8_t blue_goal_reach_thresh;

/* Variables for the RL params */
extern int8_t rl_eps;
extern float rl_gamma;
extern float rl_alp;

/* Counters keeping counts of things */
extern uint16_t steps_taken;
extern uint16_t epinum;
extern float episode_rewards;
extern float sum_dQ;
extern uint32_t total_state_visits;

/* Variables defining state and options */
extern uint8_t goals_visited;

extern char *act_type;
extern uint8_t cur_act;
extern uint8_t nxt_act;
extern char cur_sta[VISRL_STATESIZE], nxt_sta[VISRL_STATESIZE];
extern float step_wait_time;

extern uint8_t init_headind;

#ifdef VISRL_USEOPTIONS
extern uint8_t start_option; //boolean to check if performing option
extern uint8_t end_option; //boolean to check if performing option
extern char option_start_sta[VISRL_STATESIZE];
#endif

extern int8_t headind;

extern uint8_t hitwall;
extern uint8_t rl_isterminal;
extern float cur_rew;

extern void visrl_init(void);

extern int8_t stop_after_episode;


extern uint8_t rl_init_uav(void);
extern uint8_t rl_init_ep(void);

extern uint8_t rl_set_cur(void);
extern uint8_t rl_set_nxt(void);

uint8_t pick_action(char *state);
uint8_t rl_get_reward(void);

/* Functions defining the actions */
extern uint8_t rl_take_cur_action(void);
extern void rl_action_forward(void);
extern void rl_right2(void);
extern void rl_left2(void);
extern uint8_t rl_turn_to_targheadind(void);
extern uint8_t rl_headind_normalize(int8_t inheadind);

extern uint8_t rl_update_qdict(void);
extern uint8_t rl_check_terminal(void);
extern uint8_t rl_print_qtab(void);

extern md_linkedlist *ll_qdict;

void get_state_ext(char *curstate);
float get_myheading(void);
void update_headind(void);

/* Other helper functions */
void rl_get_random_cords(float *cords);

uint8_t copy_file(char *old_filename, char  *new_filename);

// Functions for unentangling the tether
extern int16_t entanglement_count;
extern uint8_t rl_unentangle_tether(void);

extern uint8_t rl_smooth_turn(uint8_t targhead_ind);

//STUFF FOR DQN
extern uint8_t pick_action_random(char *mystate);
extern uint8_t rl_init_ep_dqn(void);
extern uint8_t rl_set_cur_dqn(void);
extern uint8_t rl_set_nxt_dqn(void);
extern float cur_dqn_sta[4];
extern float nxt_dqn_sta[4];

#endif
