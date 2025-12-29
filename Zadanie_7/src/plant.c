#include "plant.h"

static float clampf(float x, float lo, float hi){
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void plant_init(plant_t* p, float y0, float alpha, float y_min, float y_max){
    p->y = y0;
    p->alpha = alpha;
    p->y_min = y_min;
    p->y_max = y_max;
}

float plant_step(plant_t* p, float u){
    // y[k+1] = y[k] + alpha * (-y[k] + u[k])
    p->y = p->y + p->alpha * (-p->y + u);
    p->y = clampf(p->y, p->y_min, p->y_max);
    return p->y;
}
