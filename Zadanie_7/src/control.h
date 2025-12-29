#pragma once
#include <stddef.h>

typedef enum { MODE_OPEN=0, MODE_CLOSED=1 } mode_t;
typedef enum { ST_INIT=0, ST_IDLE, ST_RUN_OPEN, ST_RUN_CLOSED, ST_SAFE } state_t;

typedef struct {
    // parametry
    float kp, ki;         // PI na start (PID później)
    float u_min, u_max;
    float i_limit;

    // stan regulatora
    float i_acc;
    float e_prev;

    // rejestry / tryby
    mode_t  mode;
    state_t st;
    float   setpoint;
    float   u_cmd;     // OPEN: ustawiane komendą
    float   u;         // faktyczne wyjście na obiekt
    float   y;         // pomiar

    // telemetria
    unsigned ticks;
    unsigned mode_changes;
    unsigned u_sat_count;
    float    overshoot_max;

    // watchdog
    unsigned wd_misses;
    unsigned wd_limit;

    // błędy
    int fault_code;
} control_t;

void control_init(control_t* c);
void control_set_mode(control_t* c, mode_t m);
void control_stop(control_t* c); // SAFE
void control_tick(control_t* c); // PI + limity + AW + telemetria
