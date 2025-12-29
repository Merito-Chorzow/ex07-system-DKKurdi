#include <stdio.h>
#include "shell.h"
#include "control.h"
#include "plant.h"

/* ===== pomocnicze ===== */

static float absf(float x){
    return (x < 0.0f) ? -x : x;
}

typedef struct {
    int rise_k;
    float y_max;
    float sse_avg;
} metrics_t;

static void metrics_init(metrics_t* m){
    m->rise_k = -1;
    m->y_max = -1e9f;
    m->sse_avg = 0.0f;
}

/* ===== run_ticks ===== */

static void run_ticks(shell_t* sh, control_t* c, plant_t* p, int n){
    for(int i = 0; i < n; i++){
        control_tick(c);
        plant_step(p, c->u);
        c->y = p->y;
        shell_tick(sh);
    }
}

/* ===== TEST JAKOŚCIOWY ===== */

static metrics_t run_closed_quality_test(float kp, float ki, float alpha){
    shell_t sh;
    control_t ctrl;
    plant_t plant;
    metrics_t m;

    metrics_init(&m);

    shell_init(&sh);
    control_init(&ctrl);
    plant_init(&plant, 0.0f, alpha, -1.0f, 1.0f);
    shell_bind(&sh, &ctrl, &plant);

    ctrl.kp = kp;
    ctrl.ki = ki;

    shell_rx_bytes(&sh, "mode closed\r\n");
    run_ticks(&sh, &ctrl, &plant, 5);

    shell_rx_bytes(&sh, "set 0.50\r\n");
    run_ticks(&sh, &ctrl, &plant, 5);

    float sp = ctrl.setpoint;
    float thr = 0.9f * sp;

    const int N = 600;
    const int SSE_WINDOW = 100;
    float sse_sum = 0.0f;
    int sse_n = 0;

    for(int k = 1; k <= N; k++){
        control_tick(&ctrl);
        plant_step(&plant, ctrl.u);
        ctrl.y = plant.y;

        if (ctrl.y > m.y_max) m.y_max = ctrl.y;
        if (m.rise_k < 0 && ctrl.y >= thr) m.rise_k = k;

        if (k > N - SSE_WINDOW){
            sse_sum += absf(sp - ctrl.y);
            sse_n++;
        }

        if (k % 50 == 0){
            printf("alpha=%.2f kp=%.2f ki=%.2f k=%d y=%.3f u=%.3f\n",
                   alpha, kp, ki, k, ctrl.y, ctrl.u);
        }

        shell_tick(&sh);
    }

    m.sse_avg = (sse_n > 0) ? sse_sum / sse_n : 0.0f;

    float overshoot = (m.y_max > sp) ? (m.y_max - sp) : 0.0f;
    float overshoot_pct = (sp != 0.0f) ? overshoot / sp * 100.0f : 0.0f;

    printf(
        "RESULT alpha=%.2f kp=%.2f ki=%.2f rise_k=%d y_max=%.3f overshoot=%.2f%% sse=%.4f\n",
        alpha, kp, ki, m.rise_k, m.y_max, overshoot_pct, m.sse_avg
    );

    return m;
}

/* ===== main ===== */

int main(void){
    run_closed_quality_test(0.80f, 0.05f, 0.10f);
    run_closed_quality_test(1.40f, 0.08f, 0.10f);
    run_closed_quality_test(0.80f, 0.05f, 0.30f);
    run_closed_quality_test(1.40f, 0.08f, 0.30f);
    return 0;
}
