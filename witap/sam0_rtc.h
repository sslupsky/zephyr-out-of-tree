/**
 * @brief 
 * 
 */

#include <soc.h>

extern void sys_clock_sync(u32_t ticks);

int rtc_init(void);
void rtc_set_timeout(s32_t ticks, bool idle);
u32_t rtc_elapsed(void);
u32_t rtc_cycle_get_32(void);
u32_t rtc_announce(void);
