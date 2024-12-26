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
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mk_dht11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t Seconds;
	uint8_t	Minute;
	uint8_t Hour;
	uint8_t Date;
	uint8_t Month;
	uint8_t Year;
}Time;


typedef struct{
	uint8_t Hour;
	uint8_t	Minute;
}Time_A;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS1307_Address 0x68
#define Seconds_Address 0x00
#define	Minute_Address 0x01
#define	Hour_Address	0x02
#define Date_Address 0x04
#define Month_Address 0x05
#define Year_Address 0x06
#define SQW_Address 0x07
#define HourAlarm_1_Address 0x08
#define MinuteAlarm_1_Address 0x09
#define HourAlarm_2_Address 0x10
#define MinuteAlarm_2_Address 0x11
#define HourAlarm_3_Address 0x12
#define MinuteAlarm_3_Address 0x13
#define AlarmState_Address 0x14
#define Value_1HZ 0x10
#define Value_Start 0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
 uint8_t LED_CODE[31] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10, 0xFF,
 0x83/*chu b 21*/, 0xAB/*chu n 22*/, 0x8E/*chu f 23*/, 0xC3/*nua w 24*/, 0xE1/*nua w 25*/, 0xC1/*chu U 26*/, 0x8C/*chu p 27*/, 0xF7/*dau _ 28*/, 0x87/*chu t 29*/, 0xCF/*chu i 30*/};
 uint8_t LED_POS[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
 uint8_t LED_State[8];
 uint8_t T = 0;
 uint8_t Status = 0;
 uint8_t Status_Alarm = 0;
 bool sqw_before, sqw_later;
 bool Buzzer_State;
 uint8_t LED_Pos_Loop;
 dht11_t DHT11;
 Time DS1307;
 Time_A Alarms[3];
 uint8_t Alarm_State[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Ham xu li nap xuat du lieu 74HC595 hien thi LED 7 thanh
void HC_595(uint8_t code,uint8_t pos){
	HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin,0);
	uint8_t i;
	for(i = 0; i < 8; i++){
		if(pos&0x80) HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin,1);
		else HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin,0);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin,1);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin,0);
		pos = pos<<1;
		}
	for(i = 0; i < 8; i++){
		if(code&0x80) HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin,1);
		else HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin,0);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin,1);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin,0);
		code = code<<1;
		}
		HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin,1);
}

//Ham giai ma hex to dec 
uint8_t DS1307_Decode(uint8_t temp){
	return (((temp & 0xf0) >> 4) * 10) + (temp & 0x0f);
}

//Ham ma hoa dec to hex 
uint8_t DS1307_Encode(uint8_t dec) {
  return (dec % 10 + ((dec / 10) << 4));
}

//Ham gi du truyen ghi du lieu vao DS1307
void Write_DS(uint8_t Address, uint8_t Value){
	uint8_t bytes[2] = {Address, Value};
	HAL_I2C_Master_Transmit(&hi2c1, DS1307_Address << 1, bytes, 2, 2000);
}

//Ham doc du lieu tu DS1307
uint8_t Read_DS(uint8_t Address){
	uint8_t Value;
	HAL_I2C_Master_Transmit(&hi2c1, DS1307_Address << 1, &Address, 1, 2000);
	HAL_I2C_Master_Receive(&hi2c1, DS1307_Address << 1, &Value, 1, 2000);
	return Value;
}
//Ham dat thoi gian va ngay thang nam
void Set_Time(uint8_t Seconds, uint8_t Hour, uint8_t Minute, uint8_t Date, uint8_t Month, uint8_t Year){
	Write_DS(Seconds_Address, DS1307_Encode(Seconds&0x7f));
	Write_DS(Hour_Address, DS1307_Encode(Hour&0x3f));
	Write_DS(Minute_Address, DS1307_Encode(Minute));
	Write_DS(Date_Address, DS1307_Encode(Date));
	Write_DS(Month_Address, DS1307_Encode(Month));
	Write_DS(Year_Address, DS1307_Encode(Year));
}

//Ham doc thoi gian

void Read_Time(){
	DS1307.Seconds = DS1307_Decode(Read_DS(Seconds_Address)&0x7f);
	DS1307.Hour = DS1307_Decode(Read_DS(Hour_Address)&0x3f);
	DS1307.Minute = DS1307_Decode(Read_DS(Minute_Address));
}
//Ham doc ngay thang nam 
void Read_Date(){
	DS1307.Date = DS1307_Decode(Read_DS(Date_Address));
	DS1307.Month = DS1307_Decode(Read_DS(Month_Address));
	DS1307.Year = DS1307_Decode(Read_DS(Year_Address));
}
//Ham doc bao thuc
void Read_Alarm(){
	Alarms[0].Hour = DS1307_Decode(Read_DS(HourAlarm_1_Address));
	Alarms[0].Minute = DS1307_Decode(Read_DS(MinuteAlarm_1_Address));
	Alarms[1].Hour = DS1307_Decode(Read_DS(HourAlarm_2_Address));
	Alarms[1].Minute = DS1307_Decode(Read_DS(MinuteAlarm_2_Address));
	Alarms[2].Hour = DS1307_Decode(Read_DS(HourAlarm_3_Address));
	Alarms[2].Minute = DS1307_Decode(Read_DS(MinuteAlarm_3_Address));
	uint8_t Alarm_State_byte = DS1307_Decode(Read_DS(AlarmState_Address));
	for(uint8_t i = 0; i < 3; i++){
		if(Alarm_State_byte>>i&0x01) Alarm_State[i] = 1;
		else Alarm_State[i] = 0;
	}
}
//Ham dat bao thuc
void Set_Alarm(Time_A Alarms[]){
	Write_DS(HourAlarm_1_Address, DS1307_Encode(Alarms[0].Hour));
	Write_DS(MinuteAlarm_1_Address, DS1307_Encode(Alarms[0].Minute));
	Write_DS(HourAlarm_2_Address, DS1307_Encode(Alarms[1].Hour));
	Write_DS(MinuteAlarm_2_Address, DS1307_Encode(Alarms[1].Minute));
	Write_DS(HourAlarm_3_Address, DS1307_Encode(Alarms[2].Hour));
	Write_DS(MinuteAlarm_3_Address, DS1307_Encode(Alarms[2].Minute));
	uint8_t Alarm_State_byte;
	for(uint8_t i = 0; i < 3; i++){
		if(Alarm_State[i]) Alarm_State_byte |= (1 << i);
		else Alarm_State_byte &= (1 << i);
	}
	Write_DS(AlarmState_Address, DS1307_Encode(Alarm_State_byte));
}
//Ham kiem tra bao thuc de bat chuong 188
void Checking_Alarm(){
	for(uint8_t i = 0; i < 3; i++){
		if(Alarm_State[i] == true){
			if(Alarms[i].Hour == DS1307.Hour && Alarms[i].Minute == DS1307.Minute){
				Buzzer_State = true;
				return;
			}
		} 
	}
	Buzzer_State = false;
}

//Ham tinh thoi gian de chuyen trang thai tu thoi gian sang thang va nam.
void Timer_Switch(uint8_t *T){
	sqw_before = sqw_later;
	sqw_later = HAL_GPIO_ReadPin(SQW_GPIO_Port,SQW_Pin);
	if(sqw_before == 1&& sqw_later == 0){
		if(Status == 0){
			if ((*T) == 59 ){
				Read_Time();
				Checking_Alarm();
				(*T) = 0;
				if (DS1307.Hour == 0 && DS1307.Minute == 0) Read_Date();
			}
			else{
				(*T)++;
			}
			readDHT11(&DHT11);
		}
	}
}

//Ham nap cac ki tu hien thi theo tung status day vao xu li 74HC595 Display_1 Hien thi Nhiet do/do am/thoi gian, Display_2  hien thi
void Display_0(){
	if(HAL_GPIO_ReadPin(SQW_GPIO_Port,SQW_Pin) == 0){
		for(uint8_t i = 0; i < 8; i++) LED_State[i]= 20;
	}
	else{
		LED_State[0] = 24;
		LED_State[1] = 25;
		LED_State[2] = 26;
		LED_State[3] = 27;
		LED_State[4] = DS1307.Hour/10 + 10;
		LED_State[5] = DS1307.Hour%10 + 10;
		LED_State[6] = DS1307.Minute/10 + 10;
		LED_State[7] = DS1307.Minute%10 + 10;
	}
	HAL_GPIO_WritePin(STATE_1_GPIO_Port, STATE_1_Pin, 1);
	HAL_GPIO_WritePin(STATE_2_GPIO_Port, STATE_2_Pin, 1);
}
void Display_1(uint8_t Status){
	if(Status == 0){
		LED_State[0] = (uint8_t)DHT11.temperature/10;
		LED_State[1] = (uint8_t)DHT11.temperature%10;
		LED_State[2] = (uint8_t)DHT11.humidty/10;
		LED_State[3] = (uint8_t)DHT11.humidty%10;
	}
	else{
		LED_State[0] = 5;
		LED_State[1] = 28;
		LED_State[2] = 29;
		LED_State[3] = 30;
	}
	if(HAL_GPIO_ReadPin(SQW_GPIO_Port,SQW_Pin) == 0){
		LED_State[4] = DS1307.Hour/10;
		LED_State[5] = DS1307.Hour%10;
		LED_State[6] = DS1307.Minute/10;
		LED_State[7] = DS1307.Minute%10;
	}
	else{
		LED_State[4] = DS1307.Hour/10 + 10; 
		LED_State[5] = DS1307.Hour%10 + 10;
		LED_State[6] = DS1307.Minute/10 + 10;
		LED_State[7] = DS1307.Minute%10 + 10;
	}
	HAL_GPIO_WritePin(STATE_1_GPIO_Port, STATE_1_Pin, 1);
	HAL_GPIO_WritePin(STATE_2_GPIO_Port, STATE_2_Pin, 0);
}

void Display_2(){
	if(DS1307.Date/10 == 0) LED_State[0] = 20;
	else LED_State[0] = DS1307.Date/10;
	LED_State[1] = DS1307.Date%10;
	if(DS1307.Month/10 == 0) LED_State[2] = 20;
	else LED_State[2] = DS1307.Month/10;
	LED_State[3] = DS1307.Month%10;
	LED_State[4] = 2;
	LED_State[5] = 0;
	LED_State[6] = DS1307.Year/10;
	LED_State[7] = DS1307.Year%10;
	HAL_GPIO_WritePin(STATE_1_GPIO_Port, STATE_1_Pin, 0);
	HAL_GPIO_WritePin(STATE_2_GPIO_Port, STATE_2_Pin, 1);
}
void Display_3(uint8_t Status_Alarm, Time_A Alarms[], uint8_t Alarm_State[]){
	LED_State[0] = 21;
	LED_State[1] = Status_Alarm + 1;
	LED_State[2] = 0;
	if(Alarm_State[Status_Alarm]) LED_State[3] = 22;
	else LED_State[3] = 23;
	if(HAL_GPIO_ReadPin(SQW_GPIO_Port,SQW_Pin) == 0){
		LED_State[4] = Alarms[Status_Alarm].Hour/10;
		LED_State[5] = Alarms[Status_Alarm].Hour%10;
		LED_State[6] = Alarms[Status_Alarm].Minute/10;
		LED_State[7] = Alarms[Status_Alarm].Minute%10;
	}
	else{
		LED_State[4] = Alarms[Status_Alarm].Hour/10 + 10;
		LED_State[5] = Alarms[Status_Alarm].Hour%10 + 10;
		LED_State[6] = Alarms[Status_Alarm].Minute/10 + 10;
		LED_State[7] = Alarms[Status_Alarm].Minute%10 + 10;
	}
	HAL_GPIO_WritePin(STATE_1_GPIO_Port, STATE_1_Pin, 0);
	HAL_GPIO_WritePin(STATE_2_GPIO_Port, STATE_2_Pin, 0);
}
void Display_4(uint8_t Status){
	if(Status == 1 || Status == 7 || Status == 10 || Status == 13){
		LED_State[4] = 20;
		LED_State[5] = 20;
	}
	else if(Status == 2 || Status == 8 || Status == 11 || Status == 14){
		LED_State[6] = 20;
		LED_State[7] = 20;
	}
	else if(Status == 3){
		LED_State[4] = 20;
		LED_State[5] = 20;
		LED_State[6] = 20;
		LED_State[7] = 20;
	}
	else if(Status == 4 || Status == 6 || Status == 9 || Status == 12){
		LED_State[2] = 20;
		LED_State[3] = 20;
	}
	else if(Status == 5){
		LED_State[0] = 20;
		LED_State[1] = 20;
	}
}

//Ham hien thi tong
void Show_out(){
	if(Status == 0){
		if(!Buzzer_State){
			if((T%20)<15) Display_1(0);
			else Display_2();	
		}
		else Display_0();
	}
	else if(Status == 1){
		Display_1(Status);
	}
	else if(Status == 2){
		Display_1(Status);
	}
	else if(Status == 3){
		Display_2();
	}
	else if(Status == 4){
		Display_2();
	}
	else if(Status == 5){
		Display_2();
	}
	else if(Status >= 6){
		Display_3(Status_Alarm, Alarms, Alarm_State);
	}
	if(Status != 0) if(HAL_GPIO_ReadPin(SQW_GPIO_Port,SQW_Pin) == 0) Display_4(Status);
	HC_595(LED_CODE[LED_State[LED_Pos_Loop]],LED_POS[LED_Pos_Loop]);
}
//Ham doc trang thai key chuyen doi trang thai
void Switch_Status(uint8_t *Value, uint8_t Max, bool *External){
	if(HAL_GPIO_ReadPin(K1_GPIO_Port,K1_Pin) == 0){
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(K1_GPIO_Port,K1_Pin) == 0) HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,1);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,0);
		if(*External == true) *External = false;
		else{
			if (*Value < Max) (*Value)++;
			else if(*Value == Max) {
				*Value = 0;	
			}
		}
	}
}
//Ham doc trang thai key tang len
void Up(uint8_t *Value, uint8_t Min, uint8_t Max){
	if(HAL_GPIO_ReadPin(K2_GPIO_Port,K2_Pin) == 0){
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(K2_GPIO_Port,K2_Pin) == 0)	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,1);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,0);
		if(*Value < Max) (*Value)++;
		else if(*Value == Max) *Value = Min;
	}
}
//Ham doc trang thai key giam xuong
void Down(uint8_t *Value,uint8_t Min, uint8_t Max){
	if(HAL_GPIO_ReadPin(K3_GPIO_Port,K3_Pin) == 0){
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(K3_GPIO_Port,K3_Pin) == 0) HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,1);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,0);
		if(*Value > Min)(*Value)--;
		else if(*Value == Min) *Value = Max; 
	}
}
//Ham chay chuong bao thuc 
void Run_Buzzer(bool Buzzer_State){
	if(Buzzer_State){
		if(HAL_GPIO_ReadPin(SQW_GPIO_Port,SQW_Pin) == 1) HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,1);
		else HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,0);
	}
	else HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,0);
}
//Ham kiem tra cac phim
void Key_Scan(){
	Switch_Status(&Status, 15, &Buzzer_State);
	switch(Status){
		case 0: break;
		case 1:{
			Up(&DS1307.Hour,0,23);
			Down(&DS1307.Hour,0,23);
			break;
		}
		case 2:{
			Up(&DS1307.Minute,0,59);
			Down(&DS1307.Minute,0,59);
			break;
		}
		case 3:{
			Up(&DS1307.Year,0,99);
			Down(&DS1307.Year,0,99);
			break;
		}
		case 4:{
			Up(&DS1307.Month,1,12);
			Down(&DS1307.Month,1,12);
			break;
		}
		case 5:{
			if(DS1307.Month == 1 || DS1307.Month == 3 || DS1307.Month == 5 || DS1307.Month == 7 || 
				DS1307.Month == 8 || DS1307.Month == 10 || DS1307.Month == 12){
				Up(&DS1307.Date,1,31);
				Down(&DS1307.Date,1,31);
				break;
				}
			else if(DS1307.Month == 4 || DS1307.Month == 6 || DS1307.Month == 9 || DS1307.Month == 11){
				if(DS1307.Date > 30) DS1307.Date = 30;
				Up(&DS1307.Date,1,30);
				Down(&DS1307.Date,1,30);
				break;
			}
			else if (DS1307.Month == 2){
				if(DS1307.Year%4 == 0){
					if(DS1307.Date > 29) DS1307.Date = 29;
					Up(&DS1307.Date,1,29);
					Down(&DS1307.Date,1,29);
					break;
				}
				else{
					if(DS1307.Date > 28) DS1307.Date = 28;
					Up(&DS1307.Date,1,28);
					Down(&DS1307.Date,1,28);
					break;
				}
			}
		}
		case 6:{
			Status_Alarm = 0;
			Up(&Alarm_State[0],0,1);
			Down(&Alarm_State[0],0,1);
			break;
		}
		case 7:{
			Up(&Alarms[0].Hour,0,23);
			Down(&Alarms[0].Hour,0,23);
			break;
		}
		case 8:{
			Up(&Alarms[0].Minute,0,59);
			Down(&Alarms[0].Minute,0,59);
			break;
		}
		case 9:{
			Status_Alarm = 1;
			Up(&Alarm_State[1],0,1);
			Down(&Alarm_State[1],0,1);
			break;
		}
		case 10:{
			Up(&Alarms[1].Hour,0,23);
			Down(&Alarms[1].Hour,0,23);
			break;
		}
		case 11:{
			Up(&Alarms[1].Minute,0,59);
			Down(&Alarms[1].Minute,0,59);
			break;
		}
		case 12:{
			Status_Alarm = 2;
			Up(&Alarm_State[2],0,1);
			Down(&Alarm_State[2],0,1);
			break;
		}
		case 13:{
			Up(&Alarms[2].Hour,0,23);
			Down(&Alarms[2].Hour,0,23);
			break;
		}
		case 14:{
			Up(&Alarms[2].Minute,0,59);
			Down(&Alarms[2].Minute,0,59);
			break;
		}
		case 15:{
			Set_Time(DS1307.Seconds ,DS1307.Hour, DS1307.Minute, DS1307.Date, DS1307.Month, DS1307.Year);
			Set_Alarm(Alarms);
			Status = 0;
			T = 0;
			HAL_Delay(1000);
			break;
		}
	}
}
void Init(){
	HAL_Delay(1000);
	Write_DS(SQW_Address,Value_1HZ);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim4);
	init_dht11(&DHT11, &htim4, DHT11_GPIO_Port, DHT11_Pin);
	Read_Time();
	Read_Date();
	Read_Alarm();
	readDHT11(&DHT11);
	HAL_Delay(1000);
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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Key_Scan();
		Timer_Switch(&T);
		Run_Buzzer(Buzzer_State);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
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
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CLK_Pin|LAT_Pin|DATA_Pin|DHT11_Pin
                          |STATE_1_Pin|STATE_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_Pin K2_Pin K3_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K2_Pin|K3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin LAT_Pin DATA_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|LAT_Pin|DATA_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA8 PA9 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STATE_1_Pin STATE_2_Pin */
  GPIO_InitStruct.Pin = STATE_1_Pin|STATE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SQW_Pin */
  GPIO_InitStruct.Pin = SQW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SQW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
