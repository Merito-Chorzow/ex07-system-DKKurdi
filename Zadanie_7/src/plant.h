#pragma once
typedef struct {
    float y;      // pomiar
    float alpha;  // 0<alpha<1
    float y_min, y_max;
} plant_t;

void  plant_init(plant_t* p, float y0, float alpha, float y_min, float y_max);
float plant_step(plant_t* p, float u);
