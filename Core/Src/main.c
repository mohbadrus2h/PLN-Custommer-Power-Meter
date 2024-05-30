/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2cModule.h"
#include <stdio.h>
#include "string.h"
#include <math.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_PORT GPIOD
#define R1_PIN GPIO_PIN_7

#define R2_PORT GPIOD
#define R2_PIN GPIO_PIN_6

#define R3_PORT GPIOD
#define R3_PIN GPIO_PIN_5

#define R4_PORT GPIOD
#define R4_PIN GPIO_PIN_4

#define C1_PORT GPIOD
#define C1_PIN GPIO_PIN_3

#define C2_PORT GPIOD
#define C2_PIN GPIO_PIN_2

#define C3_PORT GPIOD
#define C3_PIN GPIO_PIN_1

#define C4_PORT GPIOD
#define C4_PIN GPIO_PIN_0

#define DS3231_ADDRESS 0xD0

#define avg 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
struct {
	float voltage;
	float current;
	float power;
	float energy;
	float frequency;
	float pf;
	uint16_t alarms;
} currentValues;
struct {
	int seconds;
	int minutes;
	int hour;
	int dayofweek;
	int dayofmonth;
	int month;
	int year;
} time;


uint8_t buf[8] = { 0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64 };
uint8_t rst_buf[4] = { 0xF8, 0x42, 0xC2, 0x41 };
uint8_t res_buf[25];
int ADC_VAL;
int ADC_res;
double adc_min = 0, adc_max = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}
void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

void Get_Time (void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char read_keypads(void)
{
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET);
	
	if (HAL_GPIO_ReadPin(C1_PORT, C1_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN) == GPIO_PIN_RESET)
		{
			return '1';
		}
	}
	if (HAL_GPIO_ReadPin(C2_PORT, C2_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN) == GPIO_PIN_RESET)
		{
			return '2';
		}
	}
	if (HAL_GPIO_ReadPin(C3_PORT, C3_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN) == GPIO_PIN_RESET)
		{
			return '3';
		}
	}
	if (HAL_GPIO_ReadPin(C4_PORT, C4_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C4_PORT, C4_PIN) == GPIO_PIN_RESET)
		{
			return 'a';
		}
	}
	
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET);
	
	if (HAL_GPIO_ReadPin(C1_PORT, C1_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN) == GPIO_PIN_RESET)
		{
			return '4';
		}
	}
	if (HAL_GPIO_ReadPin(C2_PORT, C2_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN) == GPIO_PIN_RESET)
		{
			return '5';
		}
	}
	if (HAL_GPIO_ReadPin(C3_PORT, C3_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN) == GPIO_PIN_RESET)
		{
			return '6';
		}
	}
	if (HAL_GPIO_ReadPin(C4_PORT, C4_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C4_PORT, C4_PIN) == GPIO_PIN_RESET)
		{
			return 'b';
		}
	}
	
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET);
	
	if (HAL_GPIO_ReadPin(C1_PORT, C1_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN) == GPIO_PIN_RESET)
		{
			return '7';
		}
	}
	if (HAL_GPIO_ReadPin(C2_PORT, C2_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN) == GPIO_PIN_RESET)
		{
			return '8';
		}
	}
	if (HAL_GPIO_ReadPin(C3_PORT, C3_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN) == GPIO_PIN_RESET)
		{
			return '9';
		}
	}
	if (HAL_GPIO_ReadPin(C4_PORT, C4_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C4_PORT, C4_PIN) == GPIO_PIN_RESET)
		{
			return 'c';
		}
	}
	
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_RESET);
	
	if (HAL_GPIO_ReadPin(C1_PORT, C1_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN) == GPIO_PIN_RESET)
		{
			return '*';
		}
	}
	if (HAL_GPIO_ReadPin(C2_PORT, C2_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN) == GPIO_PIN_RESET)
		{
			return '0';
		}
	}
	if (HAL_GPIO_ReadPin(C3_PORT, C3_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN) == GPIO_PIN_RESET)
		{
			return '#';
		}
	}
	if (HAL_GPIO_ReadPin(C4_PORT, C4_PIN)==GPIO_PIN_RESET)
	{
		if(HAL_GPIO_ReadPin(C4_PORT, C4_PIN) == GPIO_PIN_RESET)
		{
			return 'd';
		}
	}
	return 0x01;
}

int result;

int ADC_GetData(){
		ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  result = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);	
		return result;
}

double irms;
int sampleI;
int debugg = 555;

#define ADC_COUNTS  (1<<12)
double offsetI = ADC_COUNTS>>1;
double offsetADC = ADC_COUNTS>>1;
double filteredI;
double sqI;
double Irms;
double sumI;
double ICAL = 30;
uint8_t cmd[500]={0};
uint8_t printLen=0;
//--------------------------------------------------------------------------------------
double calcIrms(unsigned int Number_of_Samples)
{
  int SupplyVoltage=3300;

  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    //sampleI = analogRead(inPinI);
    sampleI = ADC_GetData();
		
    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/4096);
		
    // data yang sudah dikurang dengan offset
		filteredI = sampleI - offsetI;
		
    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum (data akumulasi dari keseluruhan sampel)
    sumI += sqI;
		
		// sprintf(cmd,"%d\n",sampleI-2200);
		// HAL_UART_Transmit(&huart6,(uint8_t *)cmd,strlen(cmd),5);
  }
	double ADCrms = sqrt(sumI / Number_of_Samples);
	
	
	// konversi ke satuan (A) * kalibrasi
	double I_RATIO = ICAL * ((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * ADCrms;

  //Reset accumulators
  sumI = 0;
  //-------------------------------------------------------------------------------------
  return Irms;
}

uint8_t key;
int timer_s = 99;
int timer_m = 99;
int timer_h = 99;
long double daya = 0;
long double daya_akumulasi = 0;
long double energy_akumulasi = 0.0;
long double tarif = 0;
long int s_waktu = 0, s_daya = 0, s_psswd = 0;
char s_nopln[13];
int buzz = 0, buzzernotif = 0;
int system_mode = 0;

uint8_t buffer[30] = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	double v, i;
	char rs1[20], rs2[20], rs3[20], rs4[20];
	char buff[16];
	int sms = 1;
	int set_end = 0;
	
	int h_start, m_start, s_start, day_start, month_start, year_start;
	int h_end, m_end, s_end, day_end, month_end, year_end;
	
	// gsm
	char msg[30];
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	LCD_i2cDeviceCheck();		
	LCD_Init();
	LCD_BackLight(LCD_BL_ON);
	HAL_TIM_Base_Start_IT(&htim4);
	LCD_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	
	// set waktu
	// parameter (detik, menit, jam, 0, tanggal, bulan, tahun)
	// Set_Time(00, 13, 22, 0, 2, 1, 23);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
		// read PZEM
		
		HAL_UART_Transmit(&huart2, buf, 8, 20);
		HAL_UART_Receive(&huart2, res_buf, 25, 250);
		currentValues.voltage = ((uint32_t)res_buf[3] << 8 | // Raw voltage in 0.1V
														 (uint32_t)res_buf[4])/10.0;

		currentValues.current = ((uint32_t)res_buf[5] << 8 | // Raw current in 0.001A
														 (uint32_t)res_buf[6] |
														 (uint32_t)res_buf[7] << 24 |
														 (uint32_t)res_buf[8] << 16) / 1000.0;

		currentValues.power =   ((uint32_t)res_buf[9] << 8 | // Raw power in 0.1W
														 (uint32_t)res_buf[10] |
														 (uint32_t)res_buf[11] << 24 |
														 (uint32_t)res_buf[12] << 16) / 10.0;

		currentValues.energy =  ((uint32_t)res_buf[13] << 8 | // Raw Energy in 1Wh
														 (uint32_t)res_buf[14] |
														 (uint32_t)res_buf[15] << 24 |
														 (uint32_t)res_buf[16] << 16) / 1000.0;

		currentValues.frequency=((uint32_t)res_buf[17] << 8 | // Raw Frequency in 0.1Hz
														 (uint32_t)res_buf[18]) / 10.0;

		currentValues.pf =      ((uint32_t)res_buf[19] << 8 | // Raw pf in 0.01
														 (uint32_t)res_buf[20])/100.0;

		currentValues.alarms =  ((uint32_t)res_buf[21] << 8 | // Raw alarm value
														 (uint32_t)res_buf[22]);
		
		// read sct
		irms = calcIrms(10000);
		if(irms < 0.05)
			irms = 0.0;
		else if((irms > 0.05)&&(irms < 0.5))
			irms = irms - 0.05;
		else if(irms > 0.5)
			irms *= 0.92;
		
		if(irms < 0.05)
			irms = 0.0;
		
		// regresi linear
		// double Irms_baru = 0.9706 * irms + 0.03379;
		
		double Irms_baru = irms;
		
		/*
		sprintf(rs1," I=%.2fA", Irms_baru);
		LCD_SetCursor(1,1);
		LCD_Send_String(rs1, STR_NOSLIDE);
		sprintf(rs2," V=%.2fV", currentValues.voltage);
		LCD_SetCursor(2,1);
		LCD_Send_String(rs2, STR_NOSLIDE);
		sprintf(rs3," Cos Phi=%.2f", currentValues.pf);
		LCD_SetCursor(3,1);
		LCD_Send_String(rs3, STR_NOSLIDE);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		*/
		
		
		if (system_mode == 0)
		{
			while(1)
			{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
				LCD_SetCursor(1,1);
				LCD_Send_String("Waktu (menit):", STR_NOSLIDE);
				LCD_SetCursor(2,1);
				LCD_Send_String("> ", STR_NOSLIDE);
				LCD_SetCursor(4,1);
				LCD_Send_String("C: clear | D: submit", STR_NOSLIDE);
				key = read_keypads();
				if(key!=0x01)
				{
					if(key=='d')
					{	
						timer_s = 0;
						if(s_waktu >= 60){
							timer_m = s_waktu%60;
							timer_h = (s_waktu-(s_waktu%60))/60;
						}
						else {
							timer_m = s_waktu;
							timer_h = 0;
						}				
						system_mode = 1;
						LCD_Clear();
						HAL_Delay(200);
						break;
					}
					else if(key == 'c')
					{
						s_waktu = 0;
						LCD_Clear();
						continue;
					}
					else
					{
						LCD_SetCursor(2,3);
						s_waktu = s_waktu * 10 + ((int)key-48);
						sprintf(buff, "%ld", s_waktu);
						LCD_Send_String(buff, STR_NOSLIDE);
						HAL_Delay(200);
					}
				}
			}
		}
		else if(system_mode == 1)
		{
			while(1)
			{
				LCD_SetCursor(1,1);
				LCD_Send_String("Daya (VA) :", STR_NOSLIDE);
				LCD_SetCursor(2,1);
				LCD_Send_String("> ", STR_NOSLIDE);
				LCD_SetCursor(3,1);
				LCD_Send_String("           B: back", STR_NOSLIDE);
				LCD_SetCursor(4,1);
				LCD_Send_String("C: clear | D: submit", STR_NOSLIDE);
				key = read_keypads();
				if(key!=0x01)
				{
					if(key=='d')
					{
						// untuk set daya bebas
						if (1){
							
						// set pilihan daya hanya untuk 2200, 3500, 5500
						//if ((s_daya == 2200)||(s_daya == 3500)||(s_daya == 5500)){
						
							system_mode = 2;							
							LCD_Clear();
							HAL_Delay(200);
							break;
						}
						else {
							LCD_Clear();
							LCD_SetCursor(1,1);
							LCD_Send_String("-Daya Tidak Valid!-", STR_NOSLIDE);
							s_daya = 0;
							HAL_Delay(2000);
							LCD_Clear();
						}
					}
					else if(key == 'c')
					{
						s_daya = 0;
						LCD_Clear();
						continue;
					}
					else if(key == 'b')
					{
						s_daya = 0;
						s_waktu = 0;
						system_mode = 0;
						LCD_Clear();
						break;
					}
					else
					{
						LCD_SetCursor(2,3);
						s_daya = s_daya *10 + ((int)key-48);
						sprintf(buff, "%ld", s_daya);
						LCD_Send_String(buff, STR_NOSLIDE);
						HAL_Delay(200);
					}
				}
			}
		}
		else if(system_mode == 2)
		{
			while(1)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				LCD_SetCursor(1,1);
				LCD_Send_String("No Hp PLN :", STR_NOSLIDE);
				LCD_SetCursor(2,1);
				LCD_Send_String("> ", STR_NOSLIDE);
				LCD_SetCursor(3,1);
				LCD_Send_String("           B: back", STR_NOSLIDE);
				LCD_SetCursor(4,1);
				LCD_Send_String("C: clear | D: submit", STR_NOSLIDE);
				key = read_keypads();
				if(key!=0x01)
				{
					if(key=='d')
					{
						system_mode = 3;
						LCD_Clear();
						HAL_Delay(200);
						break;
					}
					else if(key == 'c')
					{
						int x = 0;
						for(x = 0; x < 13; x++)
							s_nopln[x] = NULL;
						LCD_Clear();
						continue;
					}
					else if(key == 'b')
					{
						for(int x = 0; x < 13; x++)
							s_nopln[x] = NULL;
						s_daya = 0;
						system_mode = 1;
						LCD_Clear();
						break;
					}
					else
					{
						LCD_SetCursor(2,3);
						char inp = key;
						strncat(s_nopln, &inp, 1);
						sprintf(buff, "%s", s_nopln);
						LCD_Send_String(buff, STR_NOSLIDE);
						HAL_Delay(200);
					}
				}
			}	
		}
		else if(system_mode == 3)
		{
			while(1)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				LCD_SetCursor(1,1);
				LCD_Send_String("Password : ", STR_NOSLIDE);
				LCD_SetCursor(2,1);
				LCD_Send_String("> ", STR_NOSLIDE);
				LCD_SetCursor(3,1);
				LCD_Send_String("           B: back", STR_NOSLIDE);
				LCD_SetCursor(4,1);
				LCD_Send_String("C: clear | D: submit", STR_NOSLIDE);
				key = read_keypads();
				if(key!=0x01)
				{
					if(key=='d')
					{
						sms = 1;
						if (s_psswd == 123456){
							system_mode = 5;
							Get_Time();
							h_start = time.hour;
							m_start = time.minutes;
							s_start = time.seconds;
							day_start = time.dayofmonth;
							month_start = time.month;
							year_start = time.year+2000;
						}
						else {
							LCD_Clear();
							LCD_SetCursor(1,1);
							LCD_Send_String("--Password Salah!!--", STR_NOSLIDE);
							s_psswd = 0;
							HAL_Delay(2000);
						}
						LCD_Clear();
						HAL_Delay(200);
						break;
					}
					else if(key == 'c')
					{
						s_psswd = 0;
						LCD_Clear();
						continue;
					}
					else if(key == 'b')
					{
						system_mode = 2;
						s_psswd = 0;
						for(int x = 0; x < 13; x++)
							s_nopln[x] = NULL;
						LCD_Clear();
						break;
					}
					else
					{
						LCD_SetCursor(2,3);
						s_psswd = s_psswd *10 + ((int)key-48);
						sprintf(buff, "%ld", s_psswd);
						LCD_Send_String(buff, STR_NOSLIDE);
						HAL_Delay(200);
					}
				}
			}
		}
		else if(system_mode == 4)
		{
			if(set_end == 0){
				Get_Time();
				h_end = time.hour;
				m_end = time.minutes;
				s_end = time.seconds;
				day_end = time.dayofmonth;
				month_end = time.month;
				year_end = time.year+2000;
				set_end = 1;
			}
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
			LCD_Clear();
			
			sprintf(rs3,"Energy= %.4Lf kWh", energy_akumulasi/3600);
			if(tarif == 0.0)
				sprintf(rs4,"Tarif = Rp. 50000.0");
			else
				sprintf(rs4,"Tarif = Rp.%.2Lf", tarif+50000.0);
			
			LCD_SetCursor(1,1);
			LCD_Send_String("Penggunaan Selesai! ", STR_NOSLIDE);
			LCD_SetCursor(2,1);
			LCD_Send_String(rs3, STR_NOSLIDE);
			LCD_SetCursor(3,1);
			LCD_Send_String(rs4, STR_NOSLIDE);
			LCD_SetCursor(4,1);
			sprintf(buff, "Timer : %d:%d:%d", timer_h, timer_m, timer_s);
			LCD_Send_String(buff, STR_NOSLIDE);
			if(sms) 
			{
				LCD_SetCursor(4,1);
				sprintf(rs1,"Check..");
				LCD_Send_String(rs1, STR_NOSLIDE);
				uint8_t flag=1;
				while(flag){
					printLen = snprintf((char*)cmd, 100, "AT\r\n");
					HAL_UART_Transmit(&huart6,cmd, printLen,1000);
					HAL_UART_Receive(&huart6, buffer, 30, 1000);
					HAL_Delay(500);
								
					if(strstr((char*)buffer, "OK")){
							flag=0;
					}
					HAL_Delay(500);
					memset(buffer,0,sizeof(buffer));
					flag=0;
				}
				sprintf(msg,"Send SMS...");
				LCD_SetCursor(4,1);
				LCD_Send_String(msg, STR_NOSLIDE);
					
				printLen = snprintf((char*)cmd, 100,"AT+CMGF=1\r\n");
				HAL_UART_Transmit(&huart6,cmd,printLen,1000);
				HAL_UART_Receive(&huart6, buffer, 30,1000);
				HAL_Delay(1000);
							
				printLen = snprintf((char*)cmd, 100, "AT+CSQ\r\n");
				HAL_UART_Transmit(&huart6,cmd,printLen,1000);
				HAL_UART_Receive(&huart6, buffer, 30,1000);
				HAL_Delay(1000);
							
				printLen = snprintf((char*)cmd, 100, "AT+CCID\r\n");
				HAL_UART_Transmit(&huart6,cmd,printLen,1000);
				HAL_UART_Receive(&huart6, buffer, 30,1000);
				HAL_Delay(1000);
							
				printLen = snprintf((char*)cmd, 100, "AT+CREG?\r\n");
				HAL_UART_Transmit(&huart6,cmd,printLen,1000);
				HAL_UART_Receive(&huart6, buffer, 30,1000);
				HAL_Delay(1000);
					
				memset(buffer,0,sizeof(buffer));
				
				printLen = snprintf((char*)cmd, 100, "AT+CMGS=\"%s\"\r\n", s_nopln);
				HAL_UART_Transmit(&huart6,cmd,printLen,50);
				HAL_Delay(100);
				printLen = snprintf((char*)cmd, 200, "Penggunaan Selesai!\nWaktu mulai   : %d:%d:%d %d-%d-%d \n"
					,h_start,m_start,s_start,day_start,month_start, year_start);
				HAL_UART_Transmit(&huart6, cmd, printLen, 100);
				HAL_Delay(100);
				printLen = snprintf((char*)cmd, 200, "Waktu selesai : %d:%d:%d %d-%d-%d \n"
					,h_end,m_end,s_end,day_end,month_end, year_end);
				HAL_UART_Transmit(&huart6, cmd, printLen, 100);
				HAL_Delay(100);
				
				printLen = snprintf((char*)cmd, 200, "Energy : %.3LfkW, Tarif : Rp. %.2Lf\n"
					,energy_akumulasi/3600.0, tarif+50000);
				HAL_UART_Transmit(&huart6, cmd, printLen, 100);
				
				printLen = snprintf((char*)cmd, 50, "%c", 26);
				HAL_UART_Transmit(&huart6, cmd, printLen, 50);
				
				HAL_Delay(100);
				HAL_UART_Receive(&huart6, buf, 30,1000);
				HAL_Delay(5000);
				
				LCD_Clear();
				LCD_SetCursor(4,1);
				sprintf(msg,"SMS Terkirim");
				LCD_Send_String(msg, STR_NOSLIDE);
				sms = 0;
			}
			key = read_keypads();
			if(key!=0x01)
				{
					if(key=='d')
					{
						system_mode = 0;
					}
					if(key=='a')
					{
						sms=1;
					}
				}
			HAL_Delay(2000);
		}
		else if (system_mode == 5){
			LCD_Clear();
			// relay
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
			// read current from ADC pin

			v = currentValues.voltage;
			
			// i = currentValues.current;
			i = Irms_baru;
			
			sprintf(rs1,"V=%.2fV", v);
			sprintf(rs2," I=%.2fA", i);
			
			//daya = currentValues.power;
			daya = v*i;
			double limit = s_daya * 0.1;
			if(daya >= s_daya)
				system_mode = 6;
			if(daya >= s_daya-limit)
				buzzernotif = 1;
			if(daya <= s_daya-limit)
				buzzernotif = 0;
			
			if(daya > 100.0){
				long double dayakw = daya / 1000.0;
				sprintf(rs3,"Daya(pakai)=%.2Lf kW", dayakw);
			}
			else {
				sprintf(rs3,"Daya(pakai)=%.2Lf W", daya);
			}
			
			// sprintf(rs3,  "Energy = %.3Lf kWh",energy_akumulasi);
			if(tarif == 0.0)
				sprintf(rs4,"Tarif = Rp. 0.0");
			else if (tarif < 100.0)
				sprintf(rs4,"Tarif = Rp.%.4Lf", tarif);
			else 
				sprintf(rs4,"Tarif = Rp.%.2Lf", tarif);
				
			LCD_SetCursor(1,1);
			LCD_Send_String(rs1, STR_NOSLIDE);
			LCD_SetCursor(1,11);
			LCD_Send_String(rs2, STR_NOSLIDE);
			LCD_SetCursor(2,1);
			LCD_Send_String(rs3, STR_NOSLIDE);
			LCD_SetCursor(3,1);
			LCD_Send_String(rs4, STR_NOSLIDE);
			LCD_SetCursor(4,1);
			sprintf(buff, "%d:%d:%d", timer_h, timer_m, timer_s);
			LCD_Send_String(buff, STR_NOSLIDE);
			LCD_SetCursor(4,11);
			sprintf(buff, "pf=%.2f", currentValues.pf);
			LCD_Send_String(buff, STR_NOSLIDE);
			
			key = read_keypads();
			if(key!=0x01)
			{
				if(key=='d')
				{
					system_mode = 4;
					Get_Time();
					h_end = time.hour;
					m_end = time.minutes;
					s_end = time.seconds;
					day_end = time.dayofmonth;
					month_end = time.month;
					year_end = time.year+2000;
				}
			}
		}
		
		else if(system_mode == 6)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			buzzernotif = 0;
			LCD_Clear();
			while(1)
			{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
				LCD_SetCursor(1,1);
				LCD_Send_String("== Daya Berlebih! ==", STR_NOSLIDE);
				LCD_SetCursor(2,1);
				LCD_Send_String(" kurangi pemakaian! ", STR_NOSLIDE);
				LCD_SetCursor(3,1);
				LCD_Send_String("     (Tekan D)      ", STR_NOSLIDE);
				key = read_keypads();
				if(key!=0x01)
				{
					if(key=='d')
					{
						system_mode = 4;
						Get_Time();
						h_end = time.hour;
						m_end = time.minutes;
						s_end = time.seconds;
						day_end = time.dayofmonth;
						month_end = time.month;
						year_end = time.year+2000;
						LCD_Clear();
						break;
					}
				}
			}
		}
		
		
	}
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE6 PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(system_mode == 5)
	{
		if((timer_h == 0)&&(timer_m == 0)&&(timer_s == 0))
		{
			timer_s = 0;
			if(s_waktu >= 60){
				timer_m = s_waktu%60;
				timer_h = (s_waktu-(s_waktu%60))/60;
			}
			else 
				timer_m = s_waktu;
			system_mode = 4;
		}
		else {
			if(timer_s > 0){
				timer_s -= 1;
			}
			else {
				if(timer_m > 0){
					timer_m -= 1;
					timer_s = 59;
				}
				else {
					if(timer_h>0){
						timer_h -=1;
						timer_m = 59;
						timer_s = 59;
					}
				}
			}
			if ((timer_h == 0)&&(timer_m == 0)) {
				buzzernotif = 1;
			}
			if ((timer_h == 0)&&(timer_m == 0)&&(timer_s == 0)) {
				buzzernotif = 0;
			}
			if(buzzernotif == 1){
				if(buzz == 1){
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
					buzz = 0;
				}
				else {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
					buzz = 1;
				}
			}
			else {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			}
			// harga untuk setpoint daya 2200
			if(s_daya <= 2200)
				daya_akumulasi = daya_akumulasi + (daya/1000.0) * 0.4013056;
			// harga untuk setpoint daya 3500 dan 5500
			else if((s_daya == 3500)||(s_daya == 5500))
				daya_akumulasi = daya_akumulasi + (daya/1000.0) * 0.47209167;
			else 
				daya_akumulasi = daya_akumulasi + (daya/1000.0) * 0.4013056;
			
			energy_akumulasi = energy_akumulasi + (daya/1000.0);
			tarif  = daya_akumulasi;
		}
	}
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
  while (1)
  {
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
