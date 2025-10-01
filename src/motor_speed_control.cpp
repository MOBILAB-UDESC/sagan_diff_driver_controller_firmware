#ifndef MOTORSPEEDCONTROL_CPP
#define MOTORSPEEDCONTROL_CPP

#include "motor_speed_control.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

SpeedControl::SpeedControl(float kp, float ki, float kd, float N, float sampling_time, int saturation){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->N = N;
    this->sampling_time = sampling_time;
    this->saturation = saturation;
    this->actualPos_prev = 0;

    this->u_diff_prev1 = 0;
    this->u_diff_prev2 = 0;
    this->e_diff_prev1 = 0;
    this->e_diff_prev2 = 0;

    // --- INICIALIZAÇÃO DAS NOVAS VARIÁVEIS ---
    // Garante que o filtro comece em um estado "limpo"
    this->filter_y_prev = 0.0f;
    this->filter_x_prev = 0.0f;

    // --- INICIALIZAÇÃO DAS NOVAS VARIÁVEIS DA MEDIANA ---
    this->median_window_index = 0;
    this->median_sample_count = 0;
    // Zera o buffer da janela para começar limpo
    for (int i = 0; i < MEDIAN_WINDOW_SIZE; ++i) {
        this->median_window[i] = 0.0f;
    }

    this->u = 0.0f;
    this->u_prev = 0.0f;
    this->u_i = 0.0f;
    this->u_i_prev = 0.0f;
    this->u_d = 0.0f;
    this->u_d_prev = 0.0f;
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

/**
 * @brief Aplica um filtro IIR passa-baixa de primeira ordem.
 *
 * Implementa a equação de diferenças:
 * y[n] = -0.99922393 * y[n-1] + 0.10562750 * x[n-1]
 *
 * @param current_input A amostra de entrada atual, x[n].
 * @return A amostra de saída filtrada, y[n].
 */
float SpeedControl::conversorCurrent2Velocity(float current_input) {
    // Coeficientes da equação (usando 'f' para literais float)
    const float coeff_y1 = + 0.999308890685400f;
    const float coeff_x1 = 0.098694278585172f;

    // 1. Calcula a saída atual usando os valores de estado (n-1)
    float current_output = (coeff_y1 * this->filter_y_prev) + 
                           (coeff_x1 * this->filter_x_prev);

    // 2. Atualiza o estado para a próxima iteração
    //    O y[n] atual será o y[n-1] da próxima vez
    this->filter_y_prev = current_output;
    //    O x[n] atual será o x[n-1] da próxima vez
    this->filter_x_prev = current_input;

    // 3. Retorna a saída calculada
    return current_output;
}

float SpeedControl::movingMedian(float current_input) {
    // 1. Adiciona a nova leitura no nosso buffer circular
    this->median_window[this->median_window_index] = current_input;

    // 2. Atualiza o índice para a próxima inserção, de forma circular
    this->median_window_index = (this->median_window_index + 1) % MEDIAN_WINDOW_SIZE;

    // 3. Incrementa o contador de amostras, mas o limita ao tamanho da janela
    if (this->median_sample_count < MEDIAN_WINDOW_SIZE) {
        this->median_sample_count++;
    }

    // 4. Prepara um vetor temporário para ordenação
    //    Usamos um vetor local para não alterar a ordem do nosso buffer circular.
    //    O tamanho do vetor é o número de amostras válidas que temos até agora.
    std::vector<float> sorted_window;
    sorted_window.assign(this->median_window, this->median_window + this->median_sample_count);

    // 5. Ordena o vetor temporário para encontrar a mediana
    std::sort(sorted_window.begin(), sorted_window.end());

    // 6. Retorna o elemento do meio, que é a mediana
    //    O índice do meio é (tamanho / 2)
    return sorted_window[this->median_sample_count / 2];
}


#endif