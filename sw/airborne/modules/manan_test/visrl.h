#include <stdint.h>
#include <glib.h>

// #define GetCurX() (stateGetNedToBodyEulers_i()->psi)
// #define GetCurY() (stateGetNedToBodyEulers_i()->psi)

extern uint8_t do_visrl(void);
extern uint8_t get_state(void);

extern float qtab[8][3];
extern GHashTable *myqdict;

extern uint8_t rl_init(void);
extern uint8_t rl_reset_episode(void);
extern uint8_t rl_set_cur(void);
extern uint8_t rl_set_nxt(void);
extern uint8_t rl_take_cur_action(void);
// extern uint8_t rl_update_qtab(void);
extern uint8_t rl_update_qdict(void);
extern uint8_t rl_check_terminal(void);
extern uint8_t rl_print_qtab(void);

extern uint8_t rl_isterminal;
extern uint16_t episodes_simulated;

extern uint8_t rl_read_qtab(void);
extern uint8_t rl_write_qtab(void);
extern uint8_t rl_write_episode_log(void);
extern uint8_t rl_write_step_log(void);

extern uint8_t rl_dec_eps(void);
extern uint8_t rl_inc_eps(void);

extern uint8_t print_qdict(void);
extern uint8_t load_qdict_fromtxt(void);
// wth the 3 grids
// extern uint8_t get_state2(void);

extern uint8_t init_qdict(void);
extern uint8_t size_qdict(void);


extern uint8_t write_qdict(void);
extern uint8_t load_qdict(void);

// extern uint8_t append_qdict(void);
// extern uint8_t print_qdict(void);
//Some testing stuff
//extern uint8_t get_state(void);
//extern uint8_t test_pick_action(void);
//extern uint8_t rl_print_test(void);
//
