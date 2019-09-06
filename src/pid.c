#include "pid.h"

void pid_init(pid_t *p){
    p->sum_error = 0;
    p->old_error = 0;
}

void pid_set_out_min(pid_t *p, int16_t min){
    p->out_min = min;
}

void pid_set_out_max(pid_t *p, int16_t max){
    p->out_max = max;
}

void pid_set_ki(pid_t *p, double ki){
    p->ki = ki;
}

void pid_set_kd(pid_t *p, double kd){
    p->kd = kd;
}

void pid_set_kp(pid_t *p, double kp){
    p->kp = kp;
}

int16_t pid_run(pid_t *p, double target, double be, int16_t offset){
    int32_t u;
    double error = be -target;

    double ki = p->ki;
    double kd = p->kd;
    double kp = p->kp;

    int16_t ui, ud, up;

    ui = (int16_t) (p->sum_error * ki + 0.5);
    ud = (int16_t) ((error - p->old_error) * kd + 0.5);
    up = (int16_t) (error * kp + 0.5);

    p->old_error = error;
    p->sum_error += error;

    u = ui + ud + up + offset;

    if (u < p->out_min)
        u = p->out_min;
    else if (u > p->out_max)
        u = p->out_max;

    return (int16_t)u;

//   int16_t u;                      // Stellgrösse
//   int16_t Error;                  // Regelabweichung

//   float Ki = 0.02;                // Faktor I
//   float Kd = 0.05;                // Faktor D
//   float Kp = 0.5;                 // Faktor P

//   float KI;                       // Faktor I temporär
//   int16_t Ui;                     // Wert I
//   int16_t Ud;                     // Wert D
//   int16_t Up;                     // Wert P

//   Error=(Soll-Ist);               // Regelabweichung berechnen
//   if (abs(Error) > 100){          // >100 -> Kein I-Anteil
//     KI = 0;
//   }
//   else{                           // sonst I-Faktor 0.02
//     KI = Ki;
//   }
//   sumError += Error;              // Regelabweichung aufsummieren
//   if(Error <=-15) sumError = 0;   // 15 Grad zu Heiß -> I-Summe  = 0

//   if(sumError >=  20000){         // I-Summe begrenzen
//     sumError = 20000;
//   }
//   if(sumError <= 0){              // Untere Grenze 0
//     sumError =0;
//   }

//   Ui = sumError * KI;             // Werte berechnen
//   Ud = (Error - Error_alt) * Kd;
//   Up = Error * Kp;
//   U = Ui + Ud + Up;               // Gesamtwert berechnen

//   if (U > 50) U = 50;             // Gesamtwert begrenzen
//   if (U<0) U = 0;
//   Error_alt = Error;              // Abweichung speichern
// return U;                         // Heizwert in Watt (max 50W)
}