//
// Created by Fanming Luo on 2018/9/7.
//

#include "KalmanFilter.h"


void KalmanFilter::correct(const VECTOR_TYPE &measure, const VECTOR_TYPE &control, VECTOR_TYPE &output,
        bool is_velocity_in){
    if (is_velocity_in)
    {
        xPre = transferMatrix * xPost + controlMatrix * control;
    }
    else
    {
        xPre = control;
    }

    preCov = transferMatrix * postCov * transferMatrix.transpose() + controlCov;
    temp1 = measureMatrix * preCov * measureMatrix.transpose() + measureCov;
    temp2 = preCov * measureMatrix.transpose();
    // TODO: simplify below
    kalmanGain = temp2 * temp1.inverse();
    xPost = xPre + kalmanGain * (measure - measureMatrix * xPre);
    postCov = (MATRIX_I - kalmanGain * measureMatrix) * preCov;
    output = xPost;
}

KalmanFilter::KalmanFilter(const MATRIX_TYPE &_measureCov, const MATRIX_TYPE &_controlCov,
        const MATRIX_TYPE &_transferMatrix, const MATRIX_TYPE &_controlMatrix, const MATRIX_TYPE &_measureMatrix) :
        measureCov(_measureCov), controlCov(_controlCov), transferMatrix(_transferMatrix),
        controlMatrix(_controlMatrix), measureMatrix(_measureMatrix)
        {
    init();
}

void KalmanFilter::init() {
    xPost = VECTOR_ZERO;
    xPre = VECTOR_ZERO;
}

void KalmanFilter::setStart(const VECTOR_TYPE &start_measure) {
    xPost = start_measure;
}
