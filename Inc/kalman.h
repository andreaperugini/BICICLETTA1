#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include "systemDimension.h"



// Kalman filter structure definition
typedef struct {
    arm_matrix_instance_f32 A; // State transition matrix
    arm_matrix_instance_f32 B; // Control matrix
    arm_matrix_instance_f32 H; // Observation matrix
    arm_matrix_instance_f32 Q; // Process noise covariance
    arm_matrix_instance_f32 R; // Measurement noise covariance
    arm_matrix_instance_f32 P; // Error covariance
    arm_matrix_instance_f32 K; // Kalman gain
    arm_matrix_instance_f32 x; // State estimate
} KalmanFilter;

// Function declarations
void kalman_filter_init(KalmanFilter* kf, float32_t* A_data, float32_t* B_data, float32_t* H_data, float32_t* Q_data, float32_t* R_data, float32_t* P_data, float32_t* K_data, float32_t* x_data);
void printMatrix(const arm_matrix_instance_f32 *matrix);
void kalman_predict(KalmanFilter* kf, const arm_matrix_instance_f32* u);
void kalman_update(KalmanFilter* kf, const arm_matrix_instance_f32* z);

#endif // KALMAN_H
