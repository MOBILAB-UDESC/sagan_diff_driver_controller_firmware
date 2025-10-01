#ifndef MOTORSPEEDCONTROL_CPP
#define MOTORSPEEDCONTROL_CPP

#include "motor_speed_control.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

//PID controller constructor
SpeedControl::SpeedControl(float kp, float ki, float kd, float N, float sampling_time, int saturation){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->N = N;
    this->sampling_time = sampling_time;
    this->saturation = saturation;
    this->actualPos_prev = 0;

    this->u = 0.0f;
    this->u_prev = 0.0f;
    this->u_i = 0.0f;
    this->u_i_prev = 0.0f;
    this->u_d = 0.0f;
    this->u_d_prev = 0.0f;
}
//Pi controller constructor
SpeedControl::SpeedControl(float kp, float ki, float sampling_time, int saturation){
    this->kp = kp;
    this->ki = ki;
    this->sampling_time = sampling_time;
    this->saturation = saturation;
    this->actualPos_prev = 0;

    this->u = 0.0f;
    this->u_prev = 0.0f;
    this->u_i = 0.0f;
    this->u_i_prev = 0.0f;
    this->u_d = 0.0f;
    this->u_d_prev = 0.0f;
}
//Ragazzini controller constructor
SpeedControl::SpeedControl(float sampling_time, int saturation){
    this->kp = kp;
    this->ki = ki;
    this->sampling_time = sampling_time;
    this->saturation = saturation;
    this->actualPos_prev = 0;

    //Ragazzini controller variables initialization
    this->u_diff_prev1 = 0;
    this->u_diff_prev2 = 0;
    this->e_diff_prev1 = 0;
    this->e_diff_prev2 = 0;
    this->u = 0.0f;

}

float SpeedControl::controlCalcPI(float targetVel, float actualVel){

    float e_i = 0;
    float e = 0;

    this->u_i_prev = u_i;
    this->u_prev = u;

    if( u_prev >= saturation && targetVel > actualVel){
        e_i = 0;
    } else if(u_prev <= -saturation && targetVel < actualVel) {
        e_i = 0;
    } else {
        e_i = targetVel - actualVel;
    }

    e = targetVel - actualVel;

    this->u_i = u_i_prev + (kp * sampling_time * ki) * e_i;
    float u_p = kp * e;
    this->u = u_p + u_i;

    return this->u;
}

float SpeedControl::controlCalcPD(float targetPos, float actualPos){
    
    float e = targetPos - actualPos;

    this->u_d_prev = u_d;
    this->u_prev = u;

    float Td = 1 / kd;
    this->u_d = (Td / (Td + N * sampling_time) * u_d_prev) - ((kp * N * Td) / (Td + N * sampling_time) * (actualPos - actualPos_prev));
    float u_p = kp * e;
    this->u = u_p + u_d;

    this->actualPos_prev = actualPos;

    return u;
}

float SpeedControl::controlCalcRagazzini(float target, float actual) {
    // Calcula o erro atual e[n]
    float e_current = target - actual;

    // Equação a diferenças:
    // u[n] = 1.7596*u[n-1] - 0.7596*u[n-2] + 2.8596*e[n] - 4.393*e[n-1] + 1.646*e[n-2]
    float u_current = (1.7596f * this->u_diff_prev1) - 
                      (0.7596f * this->u_diff_prev2) + 
                      (2.8596f * e_current) - 
                      (4.393f  * this->e_diff_prev1) + 
                      (1.646f  * this->e_diff_prev2);

    // Aplica a saturação para evitar valores extremos e wind-up
    if (u_current > this->saturation) {
        u_current = this->saturation;
    } else if (u_current < -this->saturation) {
        u_current = -this->saturation;
    }

    // Atualiza as variáveis de estado para a próxima iteração
    // O valor antigo de n-1 se torna o novo valor de n-2
    this->u_diff_prev2 = this->u_diff_prev1;
    this->e_diff_prev2 = this->e_diff_prev1;

    // O valor atual se torna o novo valor de n-1
    this->u_diff_prev1 = u_current;
    this->e_diff_prev1 = e_current;
    
    this->u = u_current; // Atualiza a saída principal do objeto, se necessário
    return this->u;
}

#endif