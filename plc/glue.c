/* Minimal runtime for the iec2c output, so the compiled ST program can be
   driven from Python (ctypes) exactly as OpenPLC drives it: located %MW words
   in, one scan per call, PLC time advanced by the task interval. */
#include <stdint.h>
#include "iec_std_lib.h"

TIME __CURRENT_TIME;
BOOL __DEBUG;
extern unsigned long long common_ticktime__;
void config_init__(void);
void config_run__(unsigned long tick);

IEC_INT plc_mw[16];
#define __LOCATED_VAR(type, name, loc, size, idx) type* name = (type*)&plc_mw[idx];
#include "LOCATED_VARIABLES.h"
#undef __LOCATED_VAR

static unsigned long tick;

void plc_init(void) {
  __CURRENT_TIME.tv_sec = 0; __CURRENT_TIME.tv_nsec = 0;
  tick = 0;
  for (int i = 0; i < 16; ++i) plc_mw[i] = 0;
  config_init__();
}

/* run one scan of the task, then advance PLC time by one interval */
void plc_scan(void) {
  config_run__(tick++);
  __CURRENT_TIME.tv_nsec += (long)common_ticktime__;
  while (__CURRENT_TIME.tv_nsec >= 1000000000L) { __CURRENT_TIME.tv_nsec -= 1000000000L; __CURRENT_TIME.tv_sec += 1; }
}

int16_t plc_get(int i) { return plc_mw[i]; }
void plc_set(int i, int16_t v) { plc_mw[i] = v; }
unsigned long long plc_scan_ns(void) { return common_ticktime__; }
