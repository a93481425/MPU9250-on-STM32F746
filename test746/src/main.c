/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f7xx.h"
#include "stm32f7xx_hal_i2c.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

float inter_test=0 , optime=0;
uint16_t GYRO_X=0,GYRO_Y=0,GYRO_Z=0,ACCEL_X=0,ACCEL_Y=0,ACCEL_Z=0,Temp=0;
float GYRO_XR,GYRO_YR,GYRO_ZR,ACCEL_XR,ACCEL_YR,ACCEL_ZR=-9.8;
float GYRO_XRini,GYRO_YRini,GYRO_ZRini,ACCEL_XRini,ACCEL_YRini,ACCEL_ZRini;
float TempR;
I2C_HandleTypeDef hi2c1;
uint32_t uwPrescalerValue = 0;
int i;

TIM_HandleTypeDef    TimHandle;
#define SLAVE_ADDRESS 0b11010000
#define LCD_FRAME_BUFFER				SDRAM_DEVICE_ADDR
#define RGB565_BYTE_PER_PIXEL			2
#define ARBG8888_BYTE_BYTE_PER_PIXEL	4
char buffer[50];
int i=0;

void SetInitialState();
void I2C_Get_Data();
void Data_Processing();
void zero();
float Translate(uint16_t value,float range ,float privious,int filter);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
void MyI2C_Init(void);
void LCD_9250_Data();
void LCD_Start();

int main(void)
{
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config();

  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;
  /* Set TIMx instance */
  __TIM2_CLK_ENABLE();
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period            = 1000-1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  HAL_TIM_Base_Init(&TimHandle);
  HAL_TIM_Base_Start_IT(&TimHandle);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  //Set LCD

  MyI2C_Init();
  SetInitialState();

  HAL_Delay(10);
  I2C_Get_Data();
  Data_Processing();
  LCD_Start();
  GYRO_XRini=GYRO_XR;
  GYRO_YRini=GYRO_YR;
  GYRO_ZRini=GYRO_ZR;
  ACCEL_XRini=ACCEL_XR;
  ACCEL_YRini=ACCEL_YR;
  ACCEL_ZRini=ACCEL_ZR;

  //ACCEL_YRini=ACCEL_ZR,ACCEL_ZRini=ACCEL_ZR;
  while (1)
  {

	  I2C_Get_Data();
	  Data_Processing();
	  zero();
	  LCD_9250_Data();
  }
}
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

void TIM2_IRQHandler() {
	HAL_TIM_IRQHandler(&TimHandle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	  optime=optime+0.1;
	  inter_test=inter_test+GYRO_ZR*0.1;
	  sprintf(buffer,"  Yaw=%2g        ",inter_test);
	  BSP_LCD_DisplayStringAtLine(10, buffer);

}

void MyI2C_Init(void){
  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	//I2C_SCL config
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	//I2C_SDA config

	__HAL_RCC_I2C1_CLK_ENABLE();
  hi2c1.Instance=I2C1;
  hi2c1.Init.Timing=0x00707CBB;
  hi2c1.Init.OwnAddress1=0;
  hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2=0;
  hi2c1.Init.OwnAddress2Masks=I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  hi2c1.Mode=HAL_I2C_MODE_MASTER;
  HAL_I2C_Init(&hi2c1);

}

void LCD_Start(){
BSP_LCD_Init();
BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER,LCD_FRAME_BUFFER);
BSP_LCD_SetLayerVisible(LTDC_ACTIVE_LAYER,ENABLE);
BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
BSP_LCD_Clear(LCD_COLOR_BLUE);
BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
BSP_LCD_DisplayOn();
}

void SetInitialState(){
	uint32_t ret;
	uint8_t data;
	data=0X07;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 104, 1,&data, 1, 1);//Power management
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 104, 1,&data, 1, 1);//Power management
	}

	data=0X80;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 107, 1,&data, 1, 1);//Power management
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 107, 1,&data, 1, 1);//Power management
	}

	data=0X00;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS,  25, 1,&data, 1, 1);//GYRO Sampling rate
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 25, 1,&data, 1, 1);//Power management
	}

	data=0x03;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS,  27, 1,&data, 1, 1);//GYRO measurement range
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 27, 1,&data, 1, 1);//Power management
	}

	data=0x06;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS,  26, 1,&data, 1, 1);//GYRO Low-pass filter
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 26, 1,&data, 1, 1);//Power management
	}

	data=0x00;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS,  28, 1 , &data , 1, 1);//G-sensor measurement range
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 28, 1,&data, 1, 1);//Power management
	}

	data=0x0e;
	ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS,  29, 1,&data, 1, 1);//G-sensor Low-pass filter
	while (ret != 0) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS, 29, 1,&data, 1, 1);//Power management
	}
}

void I2C_Get_Data(){ //Get mean of five Datas.
	GYRO_X=0;
	GYRO_Y=0;
	GYRO_Z=0;
	ACCEL_X=0;
	ACCEL_Y=0;
	ACCEL_Z=0;
	uint32_t ret;
	uint16_t GYRO_Xtemp=0,GYRO_Ytemp=0,GYRO_Ztemp=0,ACCEL_Xtemp=0,ACCEL_Ytemp=0,ACCEL_Ztemp=0,Temptemp=0;
	uint8_t GYRO_XL=0,GYRO_XH=0,GYRO_YL=0,GYRO_YH=0,GYRO_ZL=0,GYRO_ZH=0;
	uint8_t ACCEL_XL=0,ACCEL_XH=0,ACCEL_YL=0,ACCEL_YH=0,ACCEL_ZL=0,ACCEL_ZH=0,Temp_L=0,Temp_H=0;
	Temp=0;
	for(int i=0;i<5;i++){

		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3b, 1, &ACCEL_XH, 1, 1);
		while (ret != 0) {
			HAL_I2C_DeInit(&hi2c1);
			HAL_I2C_Init(&hi2c1);
			SetInitialState();
			ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3b, 1, &ACCEL_XH, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3c, 1, &ACCEL_XL , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3c, 1, &ACCEL_XL , 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3d, 1, &ACCEL_YH, 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3d, 1, &ACCEL_YH, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3e, 1, &ACCEL_YL , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3e, 1, &ACCEL_YL , 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3f, 1, &ACCEL_ZH, 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x3f, 1, &ACCEL_ZH, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x40, 1, &ACCEL_ZL , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x40, 1, &ACCEL_ZL , 1, 1);
		}

		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x41, 1, &Temp_H, 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x41, 1, &Temp_H, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x42, 1, &Temp_L , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x42, 1, &Temp_L , 1, 1);
		}

		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x43, 1, &GYRO_XH, 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x43, 1, &GYRO_XH, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x44, 1, &GYRO_XL , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x44, 1, &GYRO_XL , 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x45, 1, &GYRO_YH, 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x45, 1, &GYRO_YH, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x46, 1, &GYRO_YL , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x46, 1, &GYRO_YL , 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x47, 1, &GYRO_ZH, 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x47, 1, &GYRO_ZH, 1, 1);
		}
		ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x48, 1, &GYRO_ZL , 1, 1);
		while (ret != 0) {
			 HAL_I2C_DeInit(&hi2c1);
			 HAL_I2C_Init(&hi2c1);
			 SetInitialState();
			 ret=HAL_I2C_Mem_Read(&hi2c1,SLAVE_ADDRESS , 0x48, 1, &GYRO_ZL , 1, 1);
		}


		ACCEL_Xtemp=ACCEL_XH*256+ACCEL_XL;
		ACCEL_Ytemp=ACCEL_YH*256+ACCEL_YL;
		ACCEL_Ztemp=ACCEL_ZH*256+ACCEL_ZL;
		Temptemp=Temp_H*256+Temp_L;
		GYRO_Xtemp=GYRO_XH*256+GYRO_XL;
		GYRO_Ytemp=GYRO_YH*256+GYRO_YL;
		GYRO_Ztemp=GYRO_ZH*256+GYRO_ZL;


		ACCEL_X=ACCEL_X+(ACCEL_Xtemp/5);
		ACCEL_Y=ACCEL_Y+(ACCEL_Ytemp/5);
		ACCEL_Z=ACCEL_Z+(ACCEL_Ztemp/5);
		Temp=Temp+(Temptemp/5);
		GYRO_X=GYRO_X+(GYRO_Xtemp/5);
		GYRO_Y=GYRO_Y+(GYRO_Ytemp/5);
		GYRO_Z=GYRO_Z+(GYRO_Ztemp/5);
		HAL_Delay(10);
	}
}

void Data_Processing(){

	ACCEL_XR=Translate(ACCEL_X,19.6,ACCEL_XR,5);
	ACCEL_YR=Translate(ACCEL_Y,19.6,ACCEL_YR,5);
	ACCEL_ZR=Translate(ACCEL_Z,19.6,ACCEL_ZR,5);
	GYRO_XR=Translate(GYRO_X, 250,GYRO_XR,50);
	GYRO_YR=Translate(GYRO_Y, 250,GYRO_YR,50);
	GYRO_ZR=Translate(GYRO_Z,250,GYRO_ZR,50);
	TempR=21 + ( Temp / 333.87 );
}

void zero(){
	GYRO_XR=GYRO_XR-GYRO_XRini;
	GYRO_YR=GYRO_YR-GYRO_YRini;
	GYRO_ZR=GYRO_ZR-GYRO_ZRini;
	ACCEL_XR=ACCEL_XR-ACCEL_XRini;
	ACCEL_YR=ACCEL_YR-ACCEL_YRini;
	ACCEL_ZR=ACCEL_ZR-ACCEL_ZRini-9.8;
}

float Translate(uint16_t value , float range ,float privious,int filter){
	float a=0;
	if((value)<=32767)
		a=-value*range/32767;
	else
		a=-(value-65535)*range/32767;
	if((a-privious)>filter||(a-privious)<-filter) a=privious;
	return a;
}


static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
	  sprintf(buffer,"errorx%d",i);
	  BSP_LCD_Clear(LCD_COLOR_BLACK);
	  BSP_LCD_DisplayStringAtLine(2, buffer);
	  while(1){

	  }
}

static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

void LCD_9250_Data(){

	  sprintf(buffer,"MPU9250 DATA:     %g",optime);
	  BSP_LCD_DisplayStringAtLine(0, buffer);
	  sprintf(buffer,"  Accelerometer:");
	  BSP_LCD_DisplayStringAtLine(1, buffer);
	  sprintf(buffer,"    ax=%3g(m/s^2)         ",ACCEL_XR);
	  BSP_LCD_DisplayStringAtLine(2, buffer);
	  sprintf(buffer,"    ay=%3g(m/s^2)        ",ACCEL_YR);
	  BSP_LCD_DisplayStringAtLine(3, buffer);
	  sprintf(buffer,"    az=%3g(m/s^2)        ",ACCEL_ZR);
	  BSP_LCD_DisplayStringAtLine(4, buffer);

	  sprintf(buffer,"  GYRO:");
	  BSP_LCD_DisplayStringAtLine(5, buffer);
	  sprintf(buffer,"    Vx=%3g(DPS)        ",GYRO_XR);
	  BSP_LCD_DisplayStringAtLine(6, buffer);
	  sprintf(buffer,"    Vy=%3g(DPS)        ",GYRO_YR);
	  BSP_LCD_DisplayStringAtLine(7, buffer);
	  sprintf(buffer,"    Vz=%3g(DPS)        ",GYRO_ZR);
	  BSP_LCD_DisplayStringAtLine(8, buffer);
	  sprintf(buffer,"  Temp=%2g(C)        ",TempR);
	  BSP_LCD_DisplayStringAtLine(9, buffer);
	  BSP_LCD_DrawRect(380,200, 90, 40);
	  BSP_LCD_DisplayStringAt(385,210, "Next." , LEFT_MODE);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
