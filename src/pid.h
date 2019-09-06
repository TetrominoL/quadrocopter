#ifndef PID_H
#define PID_H

/*  Author: Simon Lindhorst <lindhorst.simon@live.de>
*/
#include <stdint.h>

typedef struct {
    float ki;
    float kp;
    float kd;
    double old_error;
    double sum_error;
    int16_t out_min;
    int16_t out_max;
} pid_t;

void pid_init(pid_t *p);

void pid_set_out_min(pid_t *p, int16_t min);

void pid_set_out_max(pid_t *p, int16_t max);

void pid_set_ki(pid_t *p, double ki);

void pid_set_kd(pid_t *p, double kd);

void pid_set_kp(pid_t *p, double kp);

int16_t pid_run(pid_t *p, double target, double be, int16_t offset);

#endif //PID_H