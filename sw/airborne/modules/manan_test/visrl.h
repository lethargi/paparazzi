#include <stdint.h>
#include <glib.h>

#define GetSign(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )
// #define GetCurX() (stateGetNedToBodyEulers_i()->psi)
// #define GetCurY() (stateGetNedToBodyEulers_i()->psi)

extern GHashTable *myqdict;
extern uint16_t rl_maxepochs;
extern int8_t headind;
extern int8_t head_roll;


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

extern uint8_t rl_write_episode_log(void);
extern uint8_t rl_write_step_log(void);

extern uint8_t rl_dec_eps(void);
extern uint8_t rl_inc_eps(void);
extern uint8_t rl_inc_maxepochs(void);

extern uint8_t init_qdict(void);
extern uint8_t size_qdict(void);
extern uint8_t print_qdict(void);
extern uint8_t write_qdict(void);
extern uint8_t load_qdict(void);
extern uint8_t load_qdict_fromtxt(void);

extern uint8_t copy_qdict(void);
extern uint8_t copy_logs(void);
