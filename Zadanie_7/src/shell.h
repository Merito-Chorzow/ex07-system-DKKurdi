#pragma once
#include "ringbuf.h"
#include "control.h"
#include "plant.h"

typedef struct {
    rb_t rx, tx;
    float setpoint;
    unsigned ticks;
    unsigned broken_lines;

    control_t* ctrl;
    plant_t*   plant;
} shell_t;

void shell_bind(shell_t* sh, control_t* c, plant_t* p);

void shell_init(shell_t* sh);
void shell_rx_bytes(shell_t* sh, const char* s); // wstrzyknięcie wejścia
void shell_tick(shell_t* sh);                    // "SysTick" — przetwarza RX/TX
