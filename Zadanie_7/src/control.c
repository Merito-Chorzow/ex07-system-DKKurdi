#include "control.h"

static float clampf(float x, float lo, float hi){
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void control_init(control_t* c){
    c->kp = 1.0f;
    c->ki = 0.05f;

    c->u_min = -1.0f;
    c->u_max =  1.0f;
    c->i_limit = 1.0f;

    c->i_acc = 0.0f;
    c->e_prev = 0.0f;

    c->mode = MODE_OPEN;
    c->st   = ST_INIT;

    c->setpoint = 0.0f;
    c->u_cmd = 0.0f;
    c->u = 0.0f;
    c->y = 0.0f;

    c->ticks = 0;
    c->mode_changes = 0;
    c->u_sat_count = 0;
    c->overshoot_max = 0.0f;

    c->wd_misses = 0;
    c->wd_limit = 50; // przykładowo: 50 “ticków” bez sterowania => SAFE
    c->fault_code = 0;
    c->st = ST_IDLE;
}

void control_set_mode(control_t* c, mode_t m){
    if (c->mode != m){
        c->mode = m;
        c->mode_changes++;
        // opcjonalnie bumpless: można wyrównać i_acc przy przejściu
    }
    if (c->st != ST_SAFE){
        c->st = (m == MODE_OPEN) ? ST_RUN_OPEN : ST_RUN_CLOSED;
    }
}

void control_stop(control_t* c){
    c->st = ST_SAFE;
    c->u = 0.0f;
}

void control_tick(control_t* c){
    c->ticks++;

    if (c->st == ST_SAFE){
        c->u = 0.0f;
        return;
    }

    if (c->mode == MODE_OPEN){
        c->u = clampf(c->u_cmd, c->u_min, c->u_max);
        if (c->u == c->u_min || c->u == c->u_max) c->u_sat_count++;
        return;
    }

    // CLOSED: PI
    float e = c->setpoint - c->y;

    float p = c->kp * e;
    c->i_acc = clampf(c->i_acc + c->ki * e, -c->i_limit, c->i_limit);

    float u_raw = p + c->i_acc;
    float u_sat = clampf(u_raw, c->u_min, c->u_max);
    if (u_sat != u_raw) c->u_sat_count++;

    c->u = u_sat;

    // overshoot: tylko gdy setpoint > 0 (proste)
    if (c->setpoint > 0.0f){
        float os = c->y - c->setpoint;
        if (os > c->overshoot_max) c->overshoot_max = os;
    }
}
