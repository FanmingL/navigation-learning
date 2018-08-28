//
// Created by erdou on 18-8-28.
//

#include "PID.h"
PID::PID(float kp_, float ki_, float kd_, float inte_lim_, float out_lim_, float inc_hz_, float k_pre_d_,
         float k_inc_d_norm_, float k_ff_) : kp(kp_), ki(ki_), kd(kd_), inte_lim(inte_lim_), out_lim(out_lim_),
         inc_hz(inc_hz_), k_pre_d(k_pre_d_), k_inc_d_norm(k_inc_d_norm_), k_ff(k_ff_)
         {
            reset();
            k_inc_d_norm = LIMIT(k_inc_d_norm, 0, 1);
         }

void PID::reset() {
    err = 0.0f;err_old = 0.0f;
    feedback_old = 0.0f;feedback_d = 0.0f;
    err_d = 0.0f;err_d_lpf = 0.0f;
    err_i = 0.0f;ff = 0.0f;
    pre_d = 0.0f;
}



float PID::step(float expect, float feedback, float T, float in_ff) {
    if (T <= 0.0f)return 0;
    float out, differential;
    feedback_d = (-1.0f) * (feedback - feedback_old) / T;
    err = expect - feedback;
    err_d = (err - err_old) / T;
    differential = (kd * err_d + k_pre_d * feedback_d);
    LPF_1_(inc_hz, T, differential, err_d_lpf);
    err_i += (err + k_pre_d * feedback_d) * T;
    err_i = LIMIT(err_i, -inte_lim, inte_lim);
    out = k_ff * in_ff
            + kp * err
            + k_inc_d_norm * err_d_lpf + (1.0f - k_inc_d_norm) * differential
            + ki * err_i;
    feedback_old = feedback;
    err_old = err;
    out = LIMIT(out, -out_lim, out_lim);
    return out;
}