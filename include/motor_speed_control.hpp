#ifndef MOTORSPEEDCONTROL_HPP
#define MOTORSPEEDCONTROL_HPP

#include <algorithm>
#include <vector>

class SpeedControl
{
    public:

    SpeedControl(float kp, float ki, float kd, float N, float sampling_time, int saturation);

    SpeedControl(float kp, float ki, float sampling_time, int saturation);

    SpeedControl(float sampling_time, int saturation);

    float controlCalcPI(float targetVel, float actualVel);

    float controlCalcPD(float targetPos, float actualPos);

    float controlCalcRagazzini(float target, float actual);

    private:
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

};

#endif