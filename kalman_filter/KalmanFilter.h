//
// Created by Fanming Luo on 2018/9/7.
//

#ifndef KF_KALMANFILTER_H
#define KF_KALMANFILTER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>

#ifndef DIMENSION
#define DIMENSION 4
#endif
#if defined NUM_TYPE
#undef NUM_TYPE
#endif

#if 1
#define NUM_TYPE float
#else
#define NUM_TYPE double
#endif

#define MATRIX_TYPE Eigen::Matrix<NUM_TYPE, DIMENSION, DIMENSION>
#define VECTOR_TYPE Eigen::Matrix<NUM_TYPE, DIMENSION, 1>
#define MATRIX_I Eigen::Matrix<NUM_TYPE, DIMENSION, DIMENSION>::Identity()
#define VECTOR_ZERO Eigen::Matrix<NUM_TYPE, DIMENSION, 1>::Zero()
class KalmanFilter {
public:
    KalmanFilter(const MATRIX_TYPE &_measureCov, const MATRIX_TYPE &_controlCov);
    KalmanFilter(const MATRIX_TYPE &_measureCov, const MATRIX_TYPE &_controlCov,
            const MATRIX_TYPE &_transferMatrix, const MATRIX_TYPE &_controlMatrix, const MATRIX_TYPE &_measureMatrix);
    ~KalmanFilter() = default;
    /*                A                 B               H          */
    MATRIX_TYPE transferMatrix, controlMatrix, measureMatrix, measureCov, controlCov, preCov, postCov, kalmanGain;
    VECTOR_TYPE xPost, xPre, xEstimate;
    void correct(const VECTOR_TYPE &measure, const VECTOR_TYPE &control, VECTOR_TYPE &output);
    void init();
private:
    MATRIX_TYPE temp1;

};


#endif //KF_KALMANFILTER_H
