#ifndef MYVISRL_H
#define MYVISRL_H

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

#define VISRL_STATESIZE 20

// #define VISRL_USEOPTIONS
#ifdef VISRL_USEOPTIONS
#define VISRL_ACTIONS 4
#else
#define VISRL_ACTIONS 3
#endif
// #define VISRL_USEOPTIONS

// extern GHashTable *myqdict;
extern uint16_t rl_maxepochs;
extern int8_t headind;
extern int8_t head_roll;

extern void visrl_init();


// extern uint8_t rl_randomize_start(uint8_t waypoint);
extern uint8_t rl_randomize_start(uint8_t waypoint, uint8_t altref_wp);
extern uint8_t rl_smooth_turn(uint8_t targhead_ind);
// extern uint8_t rl_turntoheadroll(void);

extern uint8_t rl_init(void);
extern uint8_t rl_set_cur(void);
extern uint8_t rl_set_nxt(void);
extern uint8_t rl_take_cur_action(void);
extern uint8_t rl_update_qdict(void);
extern uint8_t rl_check_terminal(void);
extern uint8_t rl_print_qtab(void);

extern uint8_t rl_isterminal;
extern uint16_t episodes_simulated;
extern uint8_t rl_eps;
extern float rl_gamma;
extern float rl_alp;

extern uint8_t rl_write_episode_log(void);
extern uint8_t rl_write_step_log(void);

extern uint8_t rl_dec_eps(void);
extern uint8_t rl_inc_eps(void);
extern uint8_t rl_inc_maxepochs(void);

extern uint8_t print_qdict(void);
extern uint8_t write_qdict(void);
extern uint8_t load_qdict(void);
extern uint8_t load_qdict_fromtxt(void);

extern uint8_t copy_qdict(void);
extern uint8_t copy_logs(void);


void get_state_ext(char *curstate);
float get_myheading(void);
void update_headind(void);


uint8_t pick_action(char *state);
uint8_t rl_get_reward(void);
uint8_t copy_file(char *old_filename, char  *new_filename);

#endif
