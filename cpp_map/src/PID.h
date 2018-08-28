//
// Created by erdou on 18-8-28.
//

#ifndef CPP_MAP_PID_H
#define CPP_MAP_PID_H

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
class PID {
public:
    /*
     * @param inte_lim_:    integration limit
     * @param inte_out_:    out limit
     * @param inc_hz_:      differential low pass filter band width
     * @param kd:           differential = 1/T*{kd * (error - error_old) - k_pre_d * (feedback - feedback_old)}
     * @param k_pre_d:      differential = 1/T*{kd * (error - error_old) - k_pre_d * (feedback - feedback_old)}
     * @param k_inc_d_norm  out_d = k_inc_d_norm * differential + (1 - k_inc_d_norm) * LOW_PASS_FILTER(differential)
     * @param k_ff:         first feedback parameter
     */
    PID(float kp_, float ki_, float kd_, float inte_lim_, float out_lim_, float inc_hz_, float k_pre_d_=0.0f,
            float k_inc_d_norm_=1.0f,float k_ff_=0.0f);
    void reset();
    float step(float expect, float feedback, float T=0.02f, float in_ff=0.0f);
private:
    float kp, ki, kd, k_pre_d, inc_hz, k_inc_d_norm, k_ff,
            err, err_old, feedback_old, feedback_d, err_d, err_d_lpf,
            err_i, ff, pre_d, inte_lim, out_lim;

};


#endif //CPP_MAP_PID_H
