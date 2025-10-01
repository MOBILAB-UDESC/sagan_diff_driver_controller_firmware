#ifndef MOTORSPEEDCONTROL_HPP
#define MOTORSPEEDCONTROL_HPP

#include <algorithm>
#include <vector>

class SpeedControl
{
    public:

    SpeedControl(float kp, float ki, float kd, float N, float sampling_time, int saturation);

    float controlCalcPI(float targetVel, float actualVel);

    float controlCalcPD(float targetPos, float actualPos);

    float controlCalcRagazzini(float target, float actual);

    float conversorCurrent2Velocity(float current_input);

    float movingMedian(float current_input);

    private:
        static const int MEDIAN_WINDOW_SIZE = 5;

        float kp;
        float ki;
        float kd;
        float N;
        float sampling_time;
        float u_i;
        float u_i_prev;
        float u;
        float u_prev;
        float u_d;
        float u_d_prev;
        float actualPos_prev;
        int saturation;

        //Variáveis para armazenar os valores anteriores para a equação a diferenças
        float u_diff_prev1; // Saída no instante n-1
        float u_diff_prev2; // Saída no instante n-2
        float e_diff_prev1; // Erro no instante n-1
        float e_diff_prev2; // Erro no instante n-2

            // --- NOVAS VARIÁVEIS DE ESTADO ---
        // Variáveis para manter o estado (memória) do novo filtro passa-baixa
        float filter_y_prev; // Armazena y[n-1]
        float filter_x_prev; // Armazena x[n-1]

        // --- NOVAS VARIÁVEIS DE ESTADO PARA A MEDIANA ---
        float median_window[MEDIAN_WINDOW_SIZE]; // Buffer circular para os valores
        int median_window_index;                 // Índice para o próximo valor a ser inserido
        int median_sample_count;                 // Contador de amostras recebidas (útil no início)

};

#endif