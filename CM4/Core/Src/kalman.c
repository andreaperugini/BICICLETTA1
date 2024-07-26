#include "arm_math.h"
#include "stdlib.h"
#include "stdio.h"
#include "systemDimension.h"


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

//dimensioni del sistema


void kalman_filter_init(KalmanFilter* kf, float32_t* A_data,float32_t* B_data,
		float32_t* H_data, float32_t* Q_data,float32_t* R_data,float32_t* P_data,
		float32_t* K_data, float32_t* x_data) {


    arm_mat_init_f32(&kf->A, state_dim, state_dim, (float32_t *)A_data);
    arm_mat_init_f32(&kf->B, state_dim, control_dim, (float32_t *)B_data);
    arm_mat_init_f32(&kf->H, measure_dim, state_dim, (float32_t *)H_data);
    arm_mat_init_f32(&kf->Q, state_dim, state_dim, (float32_t *)Q_data);
    arm_mat_init_f32(&kf->R, measure_dim, measure_dim, (float32_t *)R_data);
    arm_mat_init_f32(&kf->P, state_dim, state_dim, (float32_t *)P_data);
    arm_mat_init_f32(&kf->x, state_dim, 1, (float32_t *)x_data);

    arm_mat_init_f32(&kf->K, state_dim, measure_dim, (float32_t *)&x_data);

}





void printMatrix(const arm_matrix_instance_f32 *matrix) {


    for (uint16_t i = 0; i < matrix->numRows; i++) {
        for (uint16_t j = 0; j < matrix->numCols; j++) {
            printf("%.3f ", matrix->pData[i * matrix->numCols + j]);
        }
        printf("\r\n");
    }
}



void kalman_predict(KalmanFilter* kf, const arm_matrix_instance_f32* u) {



	arm_matrix_instance_f32 temp1,temp2,temp3,temp4,temp5;

	float32_t temp1_data[state_dim];
	float32_t temp2_data[state_dim];
	float32_t temp3_data[state_dim*state_dim];
	float32_t temp4_data[state_dim*state_dim];
	float32_t temp5_data[state_dim*state_dim];
    arm_mat_init_f32(&temp1, state_dim, 1, (float32_t *)&temp1_data);
    arm_mat_init_f32(&temp2, state_dim, 1, (float32_t *)&temp2_data);
    arm_mat_init_f32(&temp3, state_dim, state_dim, (float32_t *)&temp3_data);
    arm_mat_init_f32(&temp4, state_dim, state_dim, (float32_t *)&temp4_data);
    arm_mat_init_f32(&temp5, state_dim, state_dim, (float32_t *)&temp5_data);

    // x = A * x + B * u
	arm_mat_mult_f32(&kf->A, &kf->x, &temp1);
	arm_mat_mult_f32(&kf->B, u, &temp2);
	arm_mat_add_f32(&temp1, &temp2,  &kf->x);


    // P = A * P * A' + Q
    arm_mat_mult_f32(&kf->A, &kf->P, &temp4);
    arm_mat_trans_f32(&kf->A, &temp5);
    arm_mat_mult_f32(&temp4, &temp5, &temp3);
    arm_mat_add_f32(&temp3, &kf->Q, &kf->P);


}


void kalman_update(KalmanFilter* kf, const arm_matrix_instance_f32* z)
{

    arm_matrix_instance_f32 temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8;
    arm_matrix_instance_f32 temp9, temp10, temp11, temp12, temp13, temp14, temp15, temp16;
    float32_t temp1_data[state_dim * measure_dim];
    float32_t temp2_data[state_dim * measure_dim];
    float32_t temp3_data[measure_dim * measure_dim];
    float32_t temp4_data[measure_dim * measure_dim];
    float32_t temp5_data[measure_dim * measure_dim];
    float32_t temp6_data[state_dim * state_dim];
    float32_t temp7_data[state_dim * state_dim];
    float32_t temp8_data[measure_dim * state_dim];
    float32_t temp9_data[state_dim * measure_dim];
    float32_t temp10_data[state_dim * measure_dim];
    float32_t temp11_data[measure_dim * state_dim];
    float32_t temp12_data[measure_dim * state_dim];
    float32_t temp13_data[state_dim * measure_dim];
    float32_t temp14_data[measure_dim];
    float32_t temp15_data[measure_dim];
    float32_t temp16_data[state_dim];
    arm_mat_init_f32(&temp1, state_dim, measure_dim, temp1_data);
    arm_mat_init_f32(&temp2, state_dim, measure_dim, temp2_data);
    arm_mat_init_f32(&temp3, measure_dim, measure_dim, temp3_data);
    arm_mat_init_f32(&temp4, measure_dim, measure_dim, temp4_data);
    arm_mat_init_f32(&temp5, measure_dim, measure_dim, temp5_data);
    arm_mat_init_f32(&temp6, state_dim, state_dim, temp6_data);
    arm_mat_init_f32(&temp7, state_dim, state_dim, temp7_data);
    arm_mat_init_f32(&temp8, state_dim, state_dim, temp8_data);
    arm_mat_init_f32(&temp9, state_dim, measure_dim, temp9_data);
    arm_mat_init_f32(&temp10, state_dim, measure_dim, temp10_data);
    arm_mat_init_f32(&temp11, measure_dim,state_dim, temp11_data);
    arm_mat_init_f32(&temp12, measure_dim, state_dim, temp12_data);
    arm_mat_init_f32(&temp13, state_dim, measure_dim, temp13_data);
    arm_mat_init_f32(&temp14, measure_dim, 1, temp14_data);
	arm_mat_init_f32(&temp15, measure_dim, 1, temp15_data);
	arm_mat_init_f32(&temp16, state_dim, 1, temp16_data);

    // K = P * H' * (H * P * H' + R)^-1
    arm_mat_trans_f32(&kf->H, &temp1);
    arm_mat_mult_f32(&kf->P, &temp1, &temp2);
    arm_mat_mult_f32(&kf->H, &temp2, &temp3);
    arm_mat_add_f32(&temp3, &kf->R, &temp4);
    arm_mat_inverse_f32(&temp4, &temp5);
    arm_mat_mult_f32(&temp2, &temp5, &kf->K);

    // x = x + K * (z - H * x)
    arm_mat_mult_f32(&kf->H, &kf->x, &temp14);
    arm_mat_sub_f32(z, &temp14, &temp15);
    arm_mat_mult_f32(&kf->K, &temp15, &temp16);
    arm_mat_add_f32(&kf->x, &temp16, &kf->x);

    // Aggiornamento di P
    // P = P - P*H'*K' - K*H*P + K*(H*P*H' + R)*K'

    // Prima parte: P * H'
    arm_mat_trans_f32(&kf->H, &temp9);
    arm_mat_mult_f32(&kf->P, &temp9, &temp10);
    // Seconda parte: P * H' * K'
    arm_mat_trans_f32(&kf->K, &temp11);
    arm_mat_mult_f32(&temp10, &temp11, &temp6);
    // Terza parte: K * H * P
    arm_mat_mult_f32(&kf->H, &kf->P, &temp12);
    arm_mat_mult_f32(&kf->K, &temp12, &temp7);
    // Quarta parte: K * (H * P * H' + R) * K'
    arm_mat_mult_f32(&kf->H, &kf->P, &temp8);
    arm_mat_mult_f32(&temp8, &temp9, &temp3);
    arm_mat_add_f32(&temp3, &kf->R, &temp4);
    arm_mat_mult_f32(&kf->K, &temp4, &temp13);
    arm_mat_mult_f32(&temp13, &temp11, &temp8);
    // Risultato finale: P = P - temp6 - temp7 + temp8
    arm_mat_sub_f32(&kf->P, &temp6, &temp12);
    arm_mat_sub_f32(&temp12, &temp7, &temp6);
    arm_mat_add_f32(&temp6, &temp8, &kf->P);


}
