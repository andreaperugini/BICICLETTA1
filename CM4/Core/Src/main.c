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

#define radius 0.18 //in realta è 18.75cm //raggio ruota posteriore bicicletta
#define INTERVALLO_CAMPIONAMENTO 10   // Campionamento ogni 0.01 secondi (10 ms)
#define TEMPO_TRASMISSIONE 3000     // Trasmissione ogni 3 secondi
#define NUM_CAMPIONI (TEMPO_TRASMISSIONE / INTERVALLO_CAMPIONAMENTO)

//parametri encoder
#define ppr 4
#define gear_ratio 64.5222
#define encoder_resolution 500

#define roll_limite 30
#define raggio_deragliatore_anteriore 0.057
#define raggio_deragliatore_posteriore 0.038

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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//*****************
//COMUNICAZIONE ESP32
char bufferDati[NUM_CAMPIONI * 110];
int indiceBuffer = 0;
float tempo = 0;
uint8_t bytesricevuti[12];
uint8_t bytesricevuti1[4];
uint8_t bytesricevuti2[4];
uint8_t bytesricevuti3[4];
float floatricevuto1;
float floatricevuto2;
float floatricevuto3;
int i = 0;

//*******************
//PID
struct PID pid_speed;
struct PID pid_yaw_rate;
struct PID pid_roll;
struct PID pid_steering_torque;

const float dt = 0.01; //?????
float flag_Tc = 0; //flag che viene settata ad 1 ogni 0.01 secondi dalla funzione di callback del timer2

float flag_1ms = 0;
float roll_kp = 0;
float roll_ki = 0;
float roll_kd = 0;
//VARIABILI PER PID RUOTA POSTERIORE
float speed = 1.8*5; //velocita di regime della ruota dietro [m/s] credo //A QUESTA CORRISPONDE 6.7 KM/H, NON SO XK
float counts = 0; //counts per encoder
float counts_steer = 0; //counts per encoder sterzo
float delta_angle_degree_steer = 0; //delta angolo sterzo
float angle_steer = 0; //angolo sterzo
float delta_angle_degree = 0;
float angle_degree = 0;
float speed_degsec = 0;
float speed_rpm = 0;
float desired_speed_metsec = 0;
float speed_metsec = 0;
float desired_speed_rpm = 0;
float u_back_wheel = 0;
float duty_back_wheel = 0;
int direction_back_wheel = 0;
int bno055_calibrated = 0; //0 falso, 1 true
float speed_degsec_steer = 0;
int sys_started = 0;
float acc_steer = 0;
float speed_degsec_back = 0;
float angle_back_wheel = 0;


float volt_D[4]; //misure per settare la costante D
float sommaVolt = 0;
float Kproll;
float tauI;
float tauD;
float tempo_attuale = 0;
float tempo_iniziale = 0;
int tasto_premuto = 0;
int tasto_appena_premuto = 0;

int contatore_esp320 = 0;
float speed_degsec_steer_filtrata = 0;
float speedsteernuovo[30];
float speedsteervecchio[30];
float angolo_sterzo = 0;
float corrente_vecchia[170];
float corrente_nuova[170];

float velocitavecchia[30];
float velocitanuova[30];
float speed_degsec_filtrata = 0;
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
float yaw = 0;
float old_yaw = 0;
int timeout = 10;
float u_front_wheel;
float dir_front_wheel;
float duty_front_wheel;
float filtered_current = 0;
float filtered_current_kalman = 0;
float torque = 0;
//float resolution = 65336 - 1; //NO OVERSAMPLING
//float resolution = 4194304 - 1;//OVERSAMPLING RATIO 64

float resolution = 4194304*2*2*2*2 - 1; //OVERSAMPLING RATIO 1024


float Vref = 3.3; //avevano messo 5 non so perche, cosi dava problemi per la lettura dell'adc

const float K = 0.0234;
int n_ref = 0;
//filtro corrente

//V partitore = a*I + b

//se scheda alimentata da pc
/*
 const float a=0.7937;
 //const float b=2.4699;
 //const float b=2.4699 - 0.025 + 0.01 + 0.002;
 const float b=2.4569;
 */

//se scheda alimentata da batteria;
const float a = 0.7937;
//const float b= 2.4550-0.08-0.08-0.03;
//float b = 2.535;
float b = 2.53;

const float lambda = -967528;
float VoltSens = 0;

typedef struct {

	int velocita;
	double accelerazione;
	double tempo;
} Dati;

typedef struct {

	float angle_steer;
	float desired_filtered_torque;
	float torque;
	float u_front_wheel;
	float roll;
	float desired_speed_metsec;
	float speed_metsec;
	float u_back_wheel;
	float tempo;
	float corrente_non_filtrata;
	float corrente_filtrata;
	float desired_torque;
	float costanteD;
	float Volt_Adc;
	float Volt_sens;
	float Kp;
	float Ki;
	float Kd;

} DatiBici;

// Buffer per la ricezione dei dati dalla UART2
char rx_buffer[1];
char pid_buffer[3];

uint8_t trasmissione_attiva = 0;
Dati dati;
DatiBici datibici;

//dati delle matrici
float32_t A_data[state_dim * state_dim] = { 1 / (1 - dt * lambda), -dt
		* (lambda) * a / (1 - dt * lambda), 0, 1 };

float32_t B_data[state_dim * control_dim] = { 1, 0 };

float32_t H_data[measure_dim * state_dim] = { 1, 0, 0, 1 };

float32_t Q_data[state_dim * state_dim] = { 0.00005, 0, 0, 0.0001 };

float32_t R_data[measure_dim * measure_dim] = { 0.0220 * 2 * 2 * 2 * 2 * 2, 0,
		0, 0.4655 * 2 * 2 * 2 * 2 * 2 };
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
float f = 0;
int periodi = 0;
int stadio = 0;
int stadio_corrente = 0;
float freq = 0;
int ampiezza = 0;
float D = 0; //variabile per aggiustare l'offset dell'adc che cambia ogni volta

//filtro corrente dsp
//tutti questi parametri si ottengono da FilterDesigner
float32_t fir_coefficients[FIR_LENGHT] = { 0.00615948446257876848,
		0.00528379517032089019, 0.00735631636866262116, 0.0097868502666202499,
		0.0125619048149247584, 0.0156511887932382979, 0.0190054015899194864,
		0.0225582142322212839, 0.0262347998936645968, 0.0299451091997236796,
		0.0335861322649273694, 0.0370693657994936865, 0.0402816217367879537,
		0.0431359037300533127, 0.0455361817986052256, 0.0474098813406092293,
		0.0486930931266708056, 0.0493461427791147336, 0.0493461427791147336,
		0.0486930931266708056, 0.0474098813406092293, 0.0455361817986052256,
		0.0431359037300533127, 0.0402816217367879537, 0.0370693657994936865,
		0.0335861322649273694, 0.0299451091997236796, 0.0262347998936645968,
		0.0225582142322212839, 0.0190054015899194864, 0.0156511887932382979,
		0.0125619048149247584, 0.0097868502666202499, 0.00735631636866262116,
		0.00528379517032089019, 0.00615948446257876848 };
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
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define WINDOW_SIZE 30

double calcolaMediaMobile(double nuovoValore) {
	static double valori[WINDOW_SIZE] = { 0 }; // Array circolare per memorizzare i valori
	static int indice = 0;               // Indice corrente nell'array circolare
	static int count = 0;             // Conta quanti valori sono stati inseriti
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

float voltToAmpere(float Volt, float a, float b) {
	//float ampere = (Volt-2.47)/0.22;  //a3b RESISTENZA
	//float ampere = Volt*1.25994074 - 3.1119; //a3b MOTORE
	//float ampere = (Volt -2.53)/0.8 + 0.095 + 0.065 + 0.07 ;
	float ampere = (Volt - b) / a;

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
	float ramp_time = 2; //secondi di rampa

	//speed è la velocita di regime (costante)
	if (actual_speed < speed) {
		return actual_speed + speed * dt / ramp_time;
	} else
		return speed;
}
//******************
// FUNZIONE PER IL FILTRO A MEDIA MOBILE
float filtro_media_mobile(float *vettorenuovo, float *vettorevecchio,
		float nuovamisurazione, int dimensione) {
	vettorenuovo[0] = nuovamisurazione;
	for (int i = 0; i < dimensione - 1; i++) {
		vettorenuovo[i + 1] = vettorevecchio[i]; // dal 2 al n-1 esimo valore si ricopiano i valori vecchi
	}

	float somma = 0;
	float media;
	for (int i = 0; i < dimensione; i++) {
		vettorevecchio[i] = vettorenuovo[i]; //copia il vettore nuovo nel vecchio
		somma += vettorenuovo[i];           //calcola la somma di tutti i valori
		// printf("%f.3 vet ", vettorenuovo[i]);
	}

	media = somma / dimensione;
	return media;
}

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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim8);

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_UART_Receive_IT(&huart2, (uint8_t*) rx_buffer, 1);

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
	tune_PID(&pid_speed, 14, 40, 0);
	//*************************

	init_PID(&pid_yaw_rate, dt, 45, -45);
	//	tune_PID(&pid_roll, 0.00012*3,00012/10/3,00012/10); //prova ad alzare
	tune_PID(&pid_yaw_rate, 1.1, 19, 0.12);
	//*************************
	//PID angolo roll
	init_PID(&pid_roll, dt, 10 * K, -10 * K);
	//tune_PID(&pid_roll, +0.00015*2, 0.00015/50,  -0.00015/10/5); //prova ad alzare -0.00015*100000
	//pid filtro prof tune_PID(&pid_roll, +0.00015*2,0, +0.00015/10/5); //prova ad alzare -0.00015*100000

	//tune_PID(&pid_roll, +0.00015/2, 0.00015/4, 0); //prova ad alzare -0.00015*100000

	//tune_PID(&pid_roll, +0.00015/2, 0.00015/80, +0.00015*10); //prova ad alzare -0.00015*100000
	roll_kp = 0.00015*1.5;
	//roll_ki = 0.00015/100/4;

	roll_ki =0;
	roll_kd=0.00015*2*2;

	tune_PID(&pid_roll, roll_kp, roll_ki, roll_kd);

	//buonotune_PID(&pid_roll, +0.00015/2, 0.00015/3, +0.00015*6); //prova ad alzare -0.00015*100000

	//tune_PID(&pid_roll, 0.00015,0.00012/400,0.00012*225*10*3*5);
	//*************************

	//*************************
	//PID coppia manubrio
	init_PID(&pid_steering_torque, dt, V_MAX, -V_MAX);
	//tune_PID(&pid_steering_torque, 30000*5, 20000 * 1.5, 0);


	//tune_PID(&pid_steering_torque, 30000*3, 20000 * 2, 0);
	tune_PID(&pid_steering_torque, 30000*3.5, 20000 * 1.5, 0); // migliore con roll_pid_attuale
	//16:48 tune_PID(&pid_steering_torque, 30000*4, 20000 * 1.5, 0); // altro migliore

	//tune_PID(&pid_steering_torque, 30000*4, 20000 * 2, 0); // migliore pid che mantiene l'equilibrio
	//prima 30000 20000*5 0
	//*************************

	//*************************
	//Filtro di Kalman per corrente
	arm_mat_init_f32(&u, control_dim, 1, (float32_t*) &u_data); // Input di controllo
	arm_mat_init_f32(&z, measure_dim, 1, (float32_t*) &z_data);  // Misurazione
	kalman_filter_init(&kf, &A_data, &B_data, &H_data, &Q_data, &R_data,
			&P_data, &K_data, &x_data);
	u_data[0] = (-dt * lambda * b / (1 - dt * lambda));
	//u_data[0]=b;
	//*************************

	//SETTO D PER LE MISURE DELL'ADC
	for(int op = 0; op<4; op++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, timeout);
		CountValue = HAL_ADC_GetValue(&hadc1);
		volt_D[op] = ((float) CountValue) * Vref / (resolution);
		HAL_ADC_Stop(&hadc1);
		sommaVolt +=volt_D[op];
	}



	//prendo 4 misurazione e faccio la media
	D = 1.68 - sommaVolt/4;
	sommaVolt=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if(flag_Tc != 0 && flag_1ms ==1)
		{

		}

		if (flag_Tc == 1) {
			flag_Tc = 0;

			if ((tasto_premuto == 1) && (tasto_appena_premuto == 1)) {
				tempo_iniziale = n_ref * dt;
				tasto_appena_premuto = 2;
			}

			if ((tasto_premuto == 1) && (tasto_appena_premuto == 2)) {
				tempo_attuale = n_ref * dt;
				if (tempo_attuale - tempo_iniziale >= 3) {
					sys_started++;

					if (sys_started == 1) {
						stadio = 0;
						n_ref = 0;

					}
					if (sys_started == 2) {
						stadio = 0;
						n_ref = 0;
						pid_roll.e_old=0;
					}
					if (sys_started == 3)
						sys_started = 0;
					tasto_premuto = 0;

				}
			}

			//#####################################
			//##             BNO055			     ##
			//#####################################
			//*******************************

			eul = bno055_getVectorEuler();
			//stampa angoli eulero
			//printf("Yaw: %+2.2f Roll: %+2.2f Pitch: %+2.2f \r\n", eul.x, eul.y, eul.z);

			roll = -eul.y + 1.6; //ottengo angolo di eulero (il sensore è leggermente inclinato rispetto al piano in cui giace la bicicletta)
			yaw = eul.x; //ottengo angolo di yaw

			yaw_rate = (yaw - old_yaw) / dt;
			old_yaw = yaw;

			//controllo angolo di roll, se è troppo grande ferma tutto
			if (roll >= roll_limite || roll <= -1 * roll_limite) {
				sys_started = 0;
			}

			//#####################################
			//##          RUOTA DIETRO		     ##
			//#####################################
			//*******************************
			//Ottengo velocita ruota dietro
			counts = (double) TIM4->CNT - (TIM4->ARR) / 2;
			TIM4->CNT = (TIM4->ARR) / 2;
			//velocita angolare
			//encoder ha risoluzione cpr 500, non 66
			delta_angle_degree = (counts * 360)
					/ (ppr * gear_ratio * encoder_resolution); //del motore(davanti) 18 denti dietro (raggio)3.8cm ,28 denti davanti (raggio) 5.7cm
			speed_degsec = -1 * delta_angle_degree / dt;

			angle_degree += delta_angle_degree;
			//filtro media mobile
			speed_degsec_filtrata = filtro_media_mobile(velocitavecchia,
					velocitanuova, speed_degsec, 30);
			//rapporto ruota posteriore
			speed_degsec_back = speed_degsec_filtrata
					* raggio_deragliatore_anteriore
					/ raggio_deragliatore_posteriore; //rapporto velcoita angolare tra ruota dietro e avanti???

			//velocita ruota dietro
			speed_metsec = speed_degsec_back / 180 * 3.14 * radius;

			//angolo dietro
			angle_back_wheel += speed_degsec_back * dt;
			//*******************************

			//******************************+
			//PID ruota dietro
			desired_speed_metsec = getSpeed(desired_speed_metsec); //funzione che crea un riferimento a rampa e poi costante per la velocita della ruota dietro
			desired_speed_rpm = DegreeSec2RPM(desired_speed_metsec / radius); //inutile per ora
			u_back_wheel = PID_controller(&pid_speed, speed_metsec,	desired_speed_metsec);
			//******************************


			if (sys_started == 0) {
				u_front_wheel = 0;
				u_back_wheel = 0;

			}
			if (sys_started == 1) {

				//setto l'angolo del manubrio a 0
				angle_steer = 0;



				sys_started++;
			}

			//settare duty e pwm driver
			duty_back_wheel = Voltage2Duty(u_back_wheel);
			direction_back_wheel = Ref2Direction(u_back_wheel);
			set_PWM_and_dir_back_wheel(duty_back_wheel,
					(uint8_t) direction_back_wheel);

			//#####################################
			//##          PID YAW RATE		     ##
			//#####################################
			//******************************
			desired_yaw_rate = 0;
			desired_roll = PID_controller(&pid_yaw_rate, yaw_rate,
					desired_yaw_rate);
			//******************************

			//#####################################
			//##            PID ROLL			 ##
			//#####################################

			//******************************
			//prima di usare l'encoder

			desired_roll = 0; //l'angolo di equilibrio sono 2 gradi
			desired_torque = PID_controller(&pid_roll, roll, desired_roll);
			//******************************

			//encoder per ruota anteriore
			//ottengo i counts dell'encoder
			counts_steer = (double) TIM8->CNT - (TIM8->ARR) / 2; //credo cosi hai sia i conteggi negativi che positivi
			TIM8->CNT = (TIM8->ARR) / 2;
			//calcolo l'angolo dello sterzo
			delta_angle_degree_steer = (counts_steer * 360)
					/ (ppr * gear_ratio * encoder_resolution);

			angle_steer = angle_steer + delta_angle_degree_steer; //angolo sterzo
			speed_degsec_steer = delta_angle_degree_steer / dt; //velocita sterzo

			speed_degsec_steer_filtrata = filtro_media_mobile(speedsteernuovo,
					speedsteervecchio, speed_degsec_steer, 30);
			angolo_sterzo += speed_degsec_steer_filtrata * dt;

			//******************************

			//#####################################
			//##         LOW-PASS FILTER	   	   ##
			//#####################################
			//******************************
			//filtro passa basso 1/s+1 discretizzato con Matlab

			desired_filtered_torque = 0.009516 * old_desired_filtered_torque
					+ 0.9048 * old_desired_torque; //questo è quello del prof
			old_desired_torque = desired_torque;
			old_desired_filtered_torque = desired_filtered_torque;


			desired_filtered_torque = desired_torque;

			//desired_filtered_torque = 0.99 * old_desired_filtered_torque
			//+ 0.00995 * old_desired_torque; //questo è quello del prof
			//skippo il filtro per provare
			//desired_filtered_torque  = desired_torque;

			//altro filtro, funziona meglio
/*
			 desired_filtered_torque = 0.9 * old_desired_filtered_torque
			 + 0.1 * old_desired_torque;
			 old_desired_torque = desired_torque;
			 old_desired_filtered_torque = desired_filtered_torque;
*/
			//******************************
			//#####################################
			//##            STERZO			     ##
			//#####################################
			//******************************
			//Calcolo valore corrente
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, timeout);
			CountValue = HAL_ADC_GetValue(&hadc1);
			volt = ((float) CountValue) * Vref / (resolution);
			HAL_ADC_Stop(&hadc1);
			VoltSens = (volt + D) * 1.5059;
			corrente_non_filtrata = voltToAmpere(VoltSens, a, b);

			//filtro non funzionante del gruppo precedente
			/*
			 fir_in_arm = (float32_t)corrente_non_filtrata;
			 arm_fir_f32(&fir_instance, &fir_in_arm, &fir_out_arm, 1);
			 filtered_current = fir_out_arm;
			 */

			//Filtro di Kalman per corrente
			//setta i valori di input e di misura per il filtro di kalman
			z_data[0] = VoltSens; //misura del voltaggio del sensore di corrente
			z_data[1] = corrente_non_filtrata; //per non usare la misura I aggiuntiva, volendo si puo usare la formula V=IR del motore, adesso vedo come metterla
			kalman_predict(&kf, &u);
			kalman_update(&kf, &z);
			filtered_current_kalman = x_data[1];

			//filtered_current_kalman = filtro_media_mobile(corrente_vecchia, corrente_nuova, corrente_non_filtrata, 170);
			//calcolo coppia
			torque = filtered_current_kalman * K;

			//PID
			//test

			//desired_torque = K * 0.15*sin((n_ref / 100.0) * 2 * 3.14/3);

			//per test con coppia trapezoidale
			/*
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
			 if(desired_torque >= 0.0025*2) {stadio++;n_ref = 0;}
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
			 if(desired_torque <= -0.0025*2)
			 {stadio++;
			 n_ref = 0;}
			 else desired_torque+=-0.0066/60*dt;
			 break;


			 case 7:
			 if(desired_torque >= 0) {stadio++;n_ref = 0;}
			 else desired_torque+=0.0066/60*dt;
			 break;

			 case 8:
			 stadio =0;
			 n_ref=0;
			 break;
			 case 1000:


			 default:
			 break;
			 }


			 */

			u_front_wheel = PID_controller(&pid_steering_torque, torque,
					desired_filtered_torque);

			//controllo angolo limite manubrio
			//quando la u è negativa, l'angolo diminuisce

			 if (angle_steer <= -90) {
			 if (u_front_wheel < 0)
			 u_front_wheel = 0;
			 }
			 if (angle_steer >= 90) //se
			 {
			 if (u_front_wheel > 0)
			 u_front_wheel = 0;

			 }



			if (sys_started <= 1) {
				//stadio = 1000; utile se hai l'algoritmo per il segnale trapezoidale
				desired_speed_metsec = 0;
				u_back_wheel = 0;
				u_front_wheel = 0;

			}

			duty_front_wheel = Voltage2Duty(u_front_wheel);
			dir_front_wheel = Ref2Direction(u_front_wheel);
			set_PWM_and_dir_front_wheel(duty_front_wheel, dir_front_wheel);

//per coppia pid
			/*
			 printf("%.5f ",angle_steer);
			 printf("%.5f ",desired_filtered_torque);
			 printf("%.5f ",torque);
			 //printf("%.5f ",);

			 printf("%.5f ",u_front_wheel/18.0*12);
			 */
//per roll
			/*
			 printf("%.5f ",roll);
			 printf("%.5f ",desired_filtered_torque);
			 printf("%.5f ",torque);
			 printf("%.5f ",u_front_wheel);
			 */

//per dietro
			/*

			 printf("%.5f ",angle_degree);
			 printf("%.5f ",desired_speed_metsec);
			 printf("%.5f ",speed_metsec);
			 printf("%.5f ",u_back_wheel/18.0*12);

			 */
			//printf("\r\n");
			//******************************
			//per dati via bluetooth
			//raccolgo dati da mandare per 3 secondi
			dati.velocita = 100;
			dati.accelerazione = 9.82;
			dati.tempo = 23.02;

			//angle_steer  = 100*sin(2*3.14/5*tempo);

			datibici.angle_steer = angle_steer;
			datibici.desired_filtered_torque = desired_filtered_torque;
			datibici.desired_speed_metsec = desired_speed_metsec;
			datibici.roll = roll;
			datibici.speed_metsec = speed_metsec;
			datibici.torque = torque;
			datibici.u_back_wheel = u_back_wheel;
			datibici.u_front_wheel = u_front_wheel;
			datibici.tempo = tempo;
			datibici.corrente_non_filtrata = corrente_non_filtrata;
			datibici.corrente_filtrata = filtered_current_kalman;
			datibici.desired_torque = desired_torque;
			datibici.Volt_Adc = volt;
			datibici.Volt_sens = VoltSens;
			datibici.costanteD = D;

			datibici.Kp=roll_kp;
			datibici.Ki=roll_ki;
			datibici.Kd=roll_kd;





			//printf("%f,%f\r\n", datibici.desired_filtered_torque,
					//datibici.torque);
			//per roba veloce
			/*
			printf("%f,%f\r\n", datibici.desired_speed_metsec,
					datibici.speed_metsec);
			*/



			//printf("%f,%f\r\n", datibici.corrente_non_filtrata, datibici.corrente_filtrata);
			//printf("%f, \r\n" datibici.corrente_non_filtrata);

			 printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
									 datibici.angle_steer,
							 datibici.desired_filtered_torque,
							 datibici.desired_speed_metsec,
							 datibici.roll,
							 datibici.speed_metsec,
							 datibici.torque,
							 datibici.u_back_wheel,
							 datibici.u_front_wheel,
							 datibici.tempo,
							 datibici.corrente_non_filtrata,
							 datibici.corrente_filtrata,
			 	 	 	 	 datibici.desired_torque,
							 datibici.Volt_Adc,
							 datibici.Volt_sens,
							 datibici.costanteD,
							 datibici.Kp,
			 datibici.Ki,
			 datibici.Kd

			 );

			if (trasmissione_attiva == 1) {
				//dati bicicletta

				int bytesWritten = sprintf(&bufferDati[indiceBuffer],
						"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
						datibici.angle_steer, datibici.desired_filtered_torque,
						datibici.desired_speed_metsec, datibici.roll,
						datibici.speed_metsec, datibici.torque,
						datibici.u_back_wheel, datibici.u_front_wheel,
						datibici.tempo, datibici.corrente_non_filtrata,
						datibici.corrente_filtrata);
				indiceBuffer += bytesWritten;












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
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 1024;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 20000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 30000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  htim8.Init.Period = 4000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 200-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//******************
//INTERRUPT PA3 SENSORE DATA READY
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

//FUNZIONE DI CALLBACK PULSANTE BLU
	if (GPIO_Pin == GPIO_PIN_13) {

		tasto_premuto = 1;
		tasto_appena_premuto = 1;

	}
}
//******************

//******************
//FUNZIONE DI CALLBACK PER IL TIMER 2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim2) {
		flag_Tc = 1; //Flag che permette di entrare nel ciclo del while ogni 0.01 secondi
		n_ref++;
		tempo += 0.01;
		if (n_ref > 100 * 500)
			n_ref = 0;

		if (tempo > 3600)
			tempo = 0;

		contatore_esp320++;
		if (contatore_esp320 > 3000)
			contatore_esp320 = 0;

	}

	if (htim == &htim6) {

		// Gestione dell’invio periodico dei dati

		if (trasmissione_attiva == 1) {
			/*
			 char buffer[50];
			 bytesWritten = sprintf(buffer, "%d,%lf,%lf\n", dati.velocita,
			 dati.accelerazione, dati.tempo);
			 printf("Dati trasmessi: %s\r\n", buffer);
			 //Trasmissione_dati(buffer, bytesWritten);
			 * */
			//printf("Dati trasmessi: %s\r\n", bufferDati);
			Trasmissione_dati(bufferDati, indiceBuffer);
			indiceBuffer = 0;
			//in genere indice buffer arriva a sui 15000 ogni 3 secondi
			if (indiceBuffer >= 30000) //se per qualche motivo non avviene l'azzeramento dell'indice
				indiceBuffer = 0;
			memset(bufferDati, 0, sizeof(bufferDati));  // Pulizia buffer

		}

	}

	if (htim == &htim12){
		flag_1ms = 1;
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

// Funzione per trasmettere dati tramite UART2
void Trasmissione_dati(void *data, size_t size) {
	HAL_UART_Transmit(&huart2, (uint8_t*) data, size, HAL_MAX_DELAY);
}

// Callback chiamata quando un byte viene ricevuto sulla UART2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (rx_buffer[0] == 'S') {
			// Interrompi la trasmissione
			printf("Arrivato: %c\r\n", rx_buffer[0]);
			trasmissione_attiva = 0;

		} else if (rx_buffer[0] == 'V') {
			// Avvia la trasmissione
			printf("Arrivato: %c\r\n", rx_buffer[0]);
			trasmissione_attiva = 1;
		} else if (rx_buffer[0] == 'P') {
			//HAL_UART_Receive_IT(&huart2, (uint8_t*) rx_buffer, 1); //
			i = 1;
			// Ricevi 4 byte tramite UART (blocca fino a ricezione)
			HAL_UART_Receive_IT(&huart2, &bytesricevuti, 12);
			rx_buffer[0] = 0;
			// Ricostruisci il float dai 4 byte ricevuti
			bytesricevuti1[0] = bytesricevuti[0];
			bytesricevuti1[1] = bytesricevuti[1];
			bytesricevuti1[2] = bytesricevuti[2];
			bytesricevuti1[3] = bytesricevuti[3];
			bytesricevuti2[0] = bytesricevuti[4];
			bytesricevuti2[1] = bytesricevuti[5];
			bytesricevuti2[2] = bytesricevuti[6];
			bytesricevuti2[3] = bytesricevuti[7];
			bytesricevuti3[0] = bytesricevuti[8];
			bytesricevuti3[1] = bytesricevuti[9];
			bytesricevuti3[2] = bytesricevuti[10];
			bytesricevuti3[3] = bytesricevuti[11];

			//printf("float: %f",floatricevuto);

		}

		HAL_UART_Receive_IT(&huart2, (uint8_t*) rx_buffer, 1); //
		memcpy(&floatricevuto1, &bytesricevuti1, sizeof(float));
		memcpy(&floatricevuto2, &bytesricevuti2, sizeof(float));
		memcpy(&floatricevuto3, &bytesricevuti3, sizeof(float));

		if (i == 1) {
			tune_PID(&pid_roll, floatricevuto1, floatricevuto2, floatricevuto3); //prova ad alzare
			i = 0;

		}

	}
}

// Funzione di scrittura per printf
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}

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
