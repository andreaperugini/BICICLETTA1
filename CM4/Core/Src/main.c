/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "stdio.h"
#include "stdint.h"
#include "PID.h"
#include "DC_motor.h"
#include "arm_math.h"
#include "kalman.h"
#include "systemDimension.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define radius 18 //numero a caso

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIR_LENGHT 3
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//*******************
//PID
struct PID pid_speed;
struct PID pid_yaw_rate;
struct PID pid_roll;
struct PID pid_steering_torque;

const float dt = 0.01; //?????
int flag_BNO055_Data_Ready = 0;
float flag_Tc = 0; //flag che viene settata ad 1 ogni 0.01 secondi dalla funzione di callback del timer2

//VARIABILI PER PID RUOTA POSTERIORE
float speed = 18; //velocita di regime della ruota dietro
float counts = 0; //counts per encoder
float counts_steer = 0; //counts per encoder sterzo
float delta_angle_degree_steer = 0; //delta angolo sterzo
float angle_steer = 0; //angolo sterzo
float delta_angle_degree = 0;
float angle_degree=0;
float speed_degsec = 0;
float speed_rpm = 0;
float desired_speed_metsec = 0;
float desired_speed_rpm = 0;
float u_back_wheel = 0;
float duty_back_wheel = 0;
int direction_back_wheel = 0;
int bno055_calibrated = 0; //0 falso, 1 true
float speed_degsec_steer = 0;
int sys_started = 0;

float corrente_non_filtrata = 0;
float desired_yaw_rate = 0;
float desired_roll = 0;
float desired_torque = 0;
float desired_filtered_torque = 0;
float old_desired_filtered_torque = 0;
float old_desired_torque = 0;
float CountValue = 0;
float volt = 0;
float yaw_rate = 0;
float roll = 0;
int timeout = 10;
float u_front_wheel;
float dir_front_wheel;
float duty_front_wheel;
float filtered_current = 0;
float filtered_current_kalman = 0;
float torque = 0;
float resolution = 65336 - 1;
float Vref = 3.3; //avevano messo 5 non so perche, cosi dava problemi per la lettura dell'adc

const float K = 0.0234;
int n_ref = 0;
//filtro corrente

//V partitore = a*I + b
const float a=0.7937;
const float b=2.4699;
const float lambda = -767528;
float VoltSens = 0;

//dati delle matrici
float32_t A_data[state_dim * state_dim] = { 1 / (1 - dt * lambda), -dt
		* (lambda) * a / (1 - dt * lambda), 0, 1 };

float32_t B_data[state_dim * control_dim] = { 1, 0 };

float32_t H_data[measure_dim * state_dim] = { 1, 0, 0, 1 };

float32_t Q_data[state_dim * state_dim] = { 0.00005, 0, 0, 0.0001 };

float32_t R_data[measure_dim * measure_dim] = { 0.0220*2*2*2*2, 0, 0, 0.4655*2*2*2*2};
float32_t P_data[state_dim * state_dim] = { 100, 0, 0, 100 };

float32_t K_data[measure_dim * state_dim] = { 1, 1, 1, 1 };

KalmanFilter kf;

// Stato iniziale del sistema
float32_t x_data[state_dim] = { 0, 0 };

// Matrici per ingressi e uscite
arm_matrix_instance_f32 u;
arm_matrix_instance_f32 z;

float32_t u_data[control_dim];
float32_t z_data[measure_dim];

//test ingressi sinusoidali
float f=0;
					int periodi=0;
					int stadio = 0;
					int stadio_corrente=0;
					float freq=0;
					int ampiezza=0;



//filtro corrente dsp
//tutti questi parametri si ottengono da FilterDesigner
float32_t fir_coefficients[FIR_LENGHT] =  {0.00615948446257876848,0.00528379517032089019,0.00735631636866262116,0.0097868502666202499,
0.0125619048149247584,0.0156511887932382979,0.0190054015899194864,0.0225582142322212839,
0.0262347998936645968,0.0299451091997236796,0.0335861322649273694,0.0370693657994936865,
0.0402816217367879537,0.0431359037300533127,0.0455361817986052256,0.0474098813406092293,
0.0486930931266708056,0.0493461427791147336,0.0493461427791147336,0.0486930931266708056,
0.0474098813406092293,0.0455361817986052256,0.0431359037300533127,0.0402816217367879537,
0.0370693657994936865,0.0335861322649273694,0.0299451091997236796,0.0262347998936645968,
0.0225582142322212839,0.0190054015899194864,0.0156511887932382979,0.0125619048149247584,
0.0097868502666202499,0.00735631636866262116,0.00528379517032089019,0.00615948446257876848};
int16_t buffer_fir[FIR_LENGHT];
int16_t counter_fir;
float32_t fir_out, fir_out_arm_buffer[FIR_LENGHT];
arm_fir_instance_f32 fir_instance;
float32_t fir_in_arm, fir_out_arm, fir_state[FIR_LENGHT];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define WINDOW_SIZE 30


double calcolaMediaMobile(double nuovoValore) {
    static double valori[WINDOW_SIZE] = {0}; // Array circolare per memorizzare i valori
    static int indice = 0;                   // Indice corrente nell'array circolare
    static int count = 0;                    // Conta quanti valori sono stati inseriti
    double somma = 0.0;

    // Inserisci il nuovo valore nell'array circolare
    valori[indice] = nuovoValore;
    indice = (indice + 1) % WINDOW_SIZE; // Avanza l'indice in modo circolare

    // Incrementa il contatore dei valori, ma non superare WINDOW_SIZE
    if (count < WINDOW_SIZE) {
        count++;
    }

    // Calcola la somma degli ultimi 'count' valori
    for (int i = 0; i < count; i++) {
        somma += valori[i];
    }

    // Restituisci la media mobile
    return somma / count;
}



float voltToAmpere(float Volt)
{
	//float ampere = (Volt-2.47)/0.22;  //a3b RESISTENZA
	//float ampere = Volt*1.25994074 - 3.1119; //a3b MOTORE
	float ampere = (Volt -2.53)/0.8 + 0.095;

	//float ampere = 2.3*Volt - 5.75;   //a4b DA RIVEDERE
	//float ampere = (Volt-2.48)/0.185; //sensore ACS712 05b
	return ampere;
}

//funzione per generazione random di valore, per ingressi a scalino casuali
float random_float(float min, float max) {
					    return ((float) rand() / RAND_MAX) * (max - min) + min;
					}





//******************
//FUNZIONE PER VELOCITA DESIDERATA DELLA RUOTA DIETRO, INIZIALMENTE A RAMPA E POI COSTANTE
float getSpeed(float actual_speed) {
	float ramp_time = 6; //secondi di rampa

	//speed è la velocita di regime (costante)
	if (actual_speed < speed) {
		return actual_speed + speed * dt / ramp_time;
	} else
		return speed;
}
//******************

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
	PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  MX_GPIO_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  arm_fir_init_f32(&fir_instance, FIR_LENGHT, fir_coefficients, fir_state, 1);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim8);

	HAL_GPIO_EXTI_Callback(GPIO_PIN_13);//forse da aggiungere anche quello per il bno


	srand(1233);

	//*************************
	//BNO055
	bno055_assignI2C(&hi2c1);
	bno055_setup(); //il BNO055 viene inizializzato con questa funzione

	bno055_setPowerMode(NORMAL_MODE); //Inserire la modalita da impostare
	PowerMode currentMode = bno055_getPowerMode(); //Metodo per ottenere la modalita impostata
	printf("Modalita energetica attuale: %d\r\n", currentMode);

	//Per angoli di Eulero
	bno055_vector_t eul;
	bno055_setOperationModeNDOF();

	//calibrazione sensore???
	bno055_calibration_state_t cal = bno055_getCalibrationState();
	//printf("GYR : %+2.2d | ACC : %+2.2d | MAG : %+2.2d | %+2.2d\r\n", //questo andrebbe nel while per vedere se ha calibrato
	//		cal.gyro, cal.accel, cal.mag, cal.sys);
	//*************************

	//*************************
	//PID motore ruota dietro
	init_PID(&pid_speed, dt, V_MAX, -V_MAX);
	tune_PID(&pid_speed, 0.001, 0.05, 0);
	//*************************

	//*************************
	//PID angolo roll
	init_PID(&pid_roll, dt, 3*K, -3*K);
	tune_PID(&pid_roll, 40, 9, 0.5);
	//*************************

	//*************************
	//PID coppia manubrio
	init_PID(&pid_steering_torque, dt, V_MAX, -V_MAX);
	tune_PID(&pid_steering_torque, 30000, 20000*5, 0);

	//*************************


	//*************************
	//Filtro di Kalman per corrente
	arm_mat_init_f32(&u, control_dim, 1, (float32_t*) &u_data); // Input di controllo
	arm_mat_init_f32(&z, measure_dim, 1, (float32_t*) &z_data);  // Misurazione
	kalman_filter_init(&kf, &A_data, &B_data, &H_data, &Q_data, &R_data,
			&P_data, &K_data, &x_data);
	 u_data[0]=-dt*lambda*b/(1-dt*lambda);

	//*************************

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (flag_Tc == 1) {
			flag_Tc = 0;
			//#####################################
			//##             BNO055			   ##
			//#####################################
			//*******************************

			flag_BNO055_Data_Ready = 1; //perche non va mai a 1 non ho capito perche
			if (flag_BNO055_Data_Ready == 1) {
				flag_BNO055_Data_Ready = 0;

					bno055_writeData(BNO055_SYS_TRIGGER, 0x40); //reset int
					bno055_calibration_state_t cal = bno055_getCalibrationState();


				if (cal.sys != 3)
				{
					// printf("GYR : %+2.2d | ACC : %+2.2d | MAG : %+2.2d | %+2.2d\r\n",
					// cal.gyro, cal.accel, cal.mag, cal.sys);
					bno055_calibrated = 1;
				}
				else bno055_calibrated=1;

				//bno055_calibrated = 1; //da togliere
				if (bno055_calibrated) {
					eul = bno055_getVectorEuler();
					//stampa angoli eulero

					//printf("Yaw: %+2.2f Roll: %+2.2f Pitch: %+2.2f \r\n", eul.x, eul.y, eul.z);
					roll = eul.y;
					//#####################################
					//##          RUOTA DIETRO		   ##
					//#####################################
					//*******************************
					//Ottengo velocita ruota dietro
					counts = (double) TIM4->CNT - (TIM4->ARR) / 2;
					TIM4->CNT = (TIM4->ARR) / 2;


					delta_angle_degree = (counts * 360) / (13 * 4 * 66);
					angle_degree += delta_angle_degree;
					speed_degsec = delta_angle_degree / dt;
					speed_rpm = -(DegreeSec2RPM(speed_degsec) / 28 * 18); //wtf perche il meno???
					//*******************************

					//******************************+
					//PID ruota dietro
					desired_speed_metsec = getSpeed(desired_speed_metsec); //funzione che crea un riferimento a rampa e poi costante per la velocita della ruota dietro
					desired_speed_rpm = DegreeSec2RPM(
							desired_speed_metsec / radius);

					u_back_wheel = PID_controller(&pid_speed, speed_rpm,
							desired_speed_rpm);
					duty_back_wheel = Voltage2Duty(u_back_wheel);
					direction_back_wheel = Ref2Direction(desired_speed_rpm); //in teoria non serve perche la direzione è sempre in avanti
					set_PWM_and_dir_back_wheel(duty_back_wheel,
							(uint8_t) direction_back_wheel); //non ho capito bene cosa fa.
					//tocca aggiungere il rallentamento nel caso in cui inizi a cadere troppo velocemente
					//******************************

					//test
//					printf("desired_speed_metsec %f.3, speed_rpm %f.3 \r\n", desired_speed_metsec,speed_rpm);

					//fine test

					//#####################################
					//##          PID YAW RATE		   ##
					//#####################################
					//******************************
					desired_yaw_rate = 0;
					desired_roll = PID_controller(&pid_yaw_rate, yaw_rate,
							desired_yaw_rate);
					//******************************

					//#####################################
					//##            PID ROLL			   ##
					//#####################################
					//******************************


					//prima di usare l'encoder

					desired_roll = 0.88;
					//desired_torque = PID_controller(&pid_roll, roll,
							//desired_roll);


					//******************************


					//ora con encoder
					//ottengo i counts dell'encoder
					counts_steer = (double) TIM8->CNT - (TIM8->ARR) / 2; //credo cosi hai sia i conteggi negativi che positivi
					TIM8->CNT = (TIM8->ARR) / 2;


					//calcolo l'angolo dello sterzo
					delta_angle_degree_steer = (counts_steer * 360) / (13 * 4 * 66);
					angle_steer = angle_steer + delta_angle_degree_steer;  //angolo sterzo
					speed_degsec_steer = delta_angle_degree / dt;  //velocita sterzo

					//fase 1 printf("%.3f \r\n", angle_steer);

					//fase 2
					//per trovare il processo tra tensione in ingresso e velocita angolare
					//ingressi a gradino

					/*
					  switch(stadio)
							  {

							  case 0:
								  //niente
								  duty=0;  //cioe tensione

								  if(dt*n_ref >= 8) //dopo 8 secondi
										  {
											  stadio++;
											  n_ref = 0;
										  }

								  break;
							  case 1:
								  //step con 10V;
								  u_front_wheel = 10;
								  break;

							  case 2:
								  u_front_wheel = 5;
								  break;
							  case 3:
								  u_front_wheel = 3;
								  break;
							  case 4:
								  u_front_wheel = 0;
								  break;
							case 5:
								  u_front_wheel = 18;
								  break;
							  case 6:
								  u_front_wheel = 0;
								  break;
							  case 7:
								  //ingresso sinusoidale con periodo = 1 sec
								  u_front_wheel = 8*sin(2*3.14*(n_ref*dt));
							  case 8:
								  u_front_wheel = 0;
							  default:
								  break;
							  }

					  if(dt*n_ref >= 8) //dopo 8 secondi
					  {
						  stadio++;
						  n_ref = 0;
					  }



					  duty_front_wheel = Voltage2Duty(u_front_wheel);
					  dir_front_wheel = Ref2Direction(u_front_wheel);
					  set_PWM_and_dir_front_wheel(duty_front_wheel, dir_front_wheel);

					  printf("%.3f ", u_front_wheel);
					  printf("%.3f \r\n", delta_angle_steer);

					*/











					//******************************

					//#####################################
					//##         LOW-PASS FILTER	   	   ##
					//#####################################
					//******************************
					//filtro passa basso 1/s+1 discretizzato con Matlab
					desired_filtered_torque = 0.99 * old_desired_filtered_torque
							+ 0.00995 * old_desired_torque;
					old_desired_torque = desired_torque;
					old_desired_filtered_torque = desired_filtered_torque;
				//	printf("%.3f ", desired_filtered_torque);
					//******************************

					//#####################################
					//##            STERZO			   ##
					//#####################################
					//******************************
					//Calcolo valore corrente
					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, timeout);
					CountValue = HAL_ADC_GetValue(&hadc1);
					volt = ((float) CountValue) * Vref / (resolution);
					HAL_ADC_Stop(&hadc1);
					VoltSens = volt * 1.524 - 0.1018;
					corrente_non_filtrata = voltToAmpere(VoltSens);

					 fir_in_arm = (float32_t)corrente_non_filtrata;
					 arm_fir_f32(&fir_instance, &fir_in_arm, &fir_out_arm, 1);
					 filtered_current = fir_out_arm;

							 // printf("%.3f ", corrente_non_filtrata);



					//Filtro di Kalman per corrente
					//setta i valori di input e di misura per il filtro di kalman
					z_data[0] = VoltSens; //misura del voltaggio del sensore di corrente
					z_data[1] = corrente_non_filtrata; //per non usare la misura I aggiuntiva, volendo si puo usare la formula V=IR del motore, adesso vedo come metterla
					//z_data[1] = misuracorrente; //qua ci va la misura della corrente. scelta grazie al pwm dalla formula I = V/R
					kalman_predict(&kf, &u);
					kalman_update(&kf, &z);
					filtered_current_kalman = x_data[1];
					//filtered_current = calcolaMediaMobile(corrente_non_filtrata);

					//idea di trovare il modello del processo dello sterzo(quindi modello sterzo+motore) tramite risposta indiciale, poi dare come misura la I ottenuta dalla equazione dinamica


					//calcolo coppia
					torque = filtered_current_kalman * K;


					//PID
					//test

					//desired_torque = K * 0.15*sin((n_ref / 100.0) * 2 * 3.14/3);
					//desired_torque =  K*0.15*sin((n_ref/2000.0)*2*3.14);
					//desired_torque = 0.040;

					//printf("%.3f %.3f %.3f %.3f %.3f \r\n" ,desired_torque,torque,u_front_wheel,dir_front_wheel,pid_steering_torque.Iterm);
					//coppia a scalino
					/*
					if ((n_ref/1000)%2 == 0) desired_torque=K*5;
					else desired_torque=-K*5;
					*/
					/*
					printf("filtr_I %.3f ", filtered_current);
					printf("u_front_wheel: %.3f ", u_front_wheel);
					printf("duty_front_wheel: %.3f \r\n", duty_front_wheel);

					*/



					if(sys_started==0)
					{torque=0;
					desired_torque=0;
					stadio = 1000;
					}

					u_front_wheel = PID_controller(&pid_steering_torque, torque, desired_torque);
					//u_front_wheel = 5.0*18/12/2*(sin(2*3.14/3* dt*n_ref) + 1);
				//	u_front_wheel = 0;



										  switch(stadio)
												  {
												  case 0:
												  case 4:
													  //niente
													  desired_torque=0;  //cioe tensione
													  if(dt*n_ref >= 3*2) //dopo 3 secondi
															  {
																  stadio++;
																  n_ref = 0;
															  }

													  break;
												  case 1:
													  //step con 10V;
													  if(desired_torque >= 0.0025/3) {stadio++;n_ref = 0;}
													  else desired_torque+=0.0066/60*dt;
													  break;

												  case 2:
												  case 6:
												   if(dt*n_ref >=6) {
														  stadio++;
														  n_ref=0;
													  }
													  break;
												  case 3:
													  if(desired_torque <= 0)
													  {stadio++;
													  n_ref = 0;}
													  else desired_torque+=-0.0066/60*dt;
													  break;
												  case 5:
													  if(desired_torque <= -0.0025/3)
													  {stadio++;
													  n_ref = 0;}
													  else desired_torque+=-0.0066/60*dt;
													  break;


												  case 7:
												  if(desired_torque >= 0) {stadio++;n_ref = 0;}
												  else desired_torque+=0.0066/60*dt;
												  break;

												case 8:
													  stadio  =0;
													  break;
												case 1000:
												  default:
													  break;
												  }




					//u_front_wheel = 18;
					//u_front_wheel = 0;
					duty_front_wheel = Voltage2Duty(u_front_wheel);
					dir_front_wheel = Ref2Direction(u_front_wheel);




					set_PWM_and_dir_front_wheel(duty_front_wheel, dir_front_wheel);

//per coppia pid

					printf("%.5f ",corrente_non_filtrata);
					printf("%.5f ",desired_torque);
					printf("%.5f ",torque);
					printf("%.5f ",u_front_wheel/18.0*12);



//per dietro
					/*
					printf("%.5f ",angle_degree);
					printf("%.5f ",desired_torque);
				    printf("%.5f ",torque);
					printf("%.5f ",u_front_wheel/18.0*12);

*/
					printf("\r\n");

					//inizio ingressi casuali a gradino


					/*
						if (stadio==0) periodi=5;

					    while (stadio < 15 && stadio!=stadio_corrente && stadio!=0) {
					        // Randomize the periodi between 2 and 7
					        periodi = (int)random_float(5, 12);

					        // Randomize the input f between -18 and 18
					        f = (int)random_float(4, 18);

					        //frequenza logaritmica
					        freq = random_float(-0.3,1.3);

					        //frequenza normale
					        freq = pow(10,freq);

					        ampiezza = f;



					        stadio_corrente=stadio;
					    }

					    u_front_wheel = f*sin(freq*n_ref/100.0);

					        if (n_ref/100 >= periodi) {
					            stadio++;
					            n_ref = 0;
					        }

					        if(stadio >=15 && stadio!=stadio_corrente) u_front_wheel=0;

					*/
					//fine ingressi casuali a gradino


					//printf("%.3f %.3f %.3f \r\n" , desired_filtered_torque, torque, filtered_current);


					//u_front_wheel = 5*sin((n_ref/100.0)*2*3.14*0.1);

					//u_front_wheel = PID_controller(&pid_steering_torque, torque, desired_filtered_torque);
					//duty_front_wheel = Voltage2Duty(u_front_wheel);
					//dir_front_wheel = Ref2Direction(u_front_wheel);



					//set_PWM_and_dir_front_wheel(duty_front_wheel,
					//		dir_front_wheel);
					//printf("%.3f %.3f %d %.2f %.3f %d \r\n", u_front_wheel, torque, periodi, n_ref/100.0, freq, ampiezza);

					//******************************

				}

			}
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3423-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//******************
//INTERRUPT PA3 SENSORE DATA READY
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {


	if (GPIO_Pin == GPIO_PIN_3) {
		flag_BNO055_Data_Ready = 1;
	}

	//FUNZIONE DI CALLBACK PULSANTE BLU
	if(GPIO_Pin == GPIO_PIN_13)
		{
			sys_started++;

			if(sys_started==2) sys_started=0;

		}
}
//******************

//******************
//FUNZIONE DI CALLBACK PER IL TIMER 2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim2) {
		flag_Tc = 1; //Flag che permette di entrare nel ciclo del while ogni 0.01 secondi
		n_ref++;

		if (n_ref > 100 * 500)
			n_ref = 0;

	}
}
//*******************


//******************
//INPUT E OUTPUT USART
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}
int __io_getchar(void) {
	uint8_t ch;
	__HAL_UART_CLEAR_OREFLAG(&huart3);
	HAL_UART_Receive(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}
//******************

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
