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

