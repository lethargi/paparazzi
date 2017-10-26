#ifndef MYVISRL_SIMSOFT
#define MYVISRL_SIMSOFT
#include <stdint.h>

extern void simsoft_init(void);

extern uint8_t print_qdict(void);
extern uint8_t write_qdict(void);
extern uint8_t load_qdict(void);
extern uint8_t load_qdict_fromtxt(void);

extern uint8_t copy_qdict(void);
extern uint8_t copy_logs(void);

extern uint8_t rl_write_episode_log(void);
extern uint8_t rl_write_step_log(void);


extern uint8_t setup_run_fold(void);
uint8_t setup_sess_fold(void);
extern uint8_t simsoft_cleanup(void);

extern uint8_t runnum;
extern uint8_t rl_maxruns;
extern uint8_t rl_addruncounter(void);

#endif
