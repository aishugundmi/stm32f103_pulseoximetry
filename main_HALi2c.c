/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// FIFO registers
#define MAX30100_FIFO_W_POINTER      0x02
#define MAX30100_OVF_COUNTER         0x03
#define MAX30100_FIFO_R_POINTER      0x04
#define MAX30100_FIFO_DATA_REG       0x05

// configuration registers
#define MAX30100_MODE_CONFIG         0x06
#define MAX30100_SPO2_CONFIG         0x07
#define MAX30100_LED_CONFIG          0x09

// PART ID registers
#define MAX30100_PART_ID             0xFF

// MAX30100 I2C addresses
#define MAX30100_WADDRESS        0xAE  // 8bit address converted to 7bit + W
#define MAX30100_RADDRESS        0xAF  // 8bit address converted to 7bit + R


#define UPPER_TH					40
#define LOWER_TH					10

static const uint8_t MAX_30100_ADDR  = 0x57 << 1; // Use 8-bit address
HAL_StatusTypeDef ret;

uint8_t buf[12];
uint16_t ir_buff[16]  = {0}, red_buff[16] = {0}, ir_lf_buff[16]  = {0}, red_lf_buff[16] = {0}, ir_hf_buff[16]  = {0}, red_hf_buff[16] = {0} ;
uint8_t samples;
uint8_t ser[3];
char hr_buff[30];



void MAX30100_reset(void);
void MAX30100_wakeup(void);
void MAX30100_SetHR (void);
void MAX30100_SetSPO2 (void);
void MAX30100_InitFIFO (void);
void MAX30100_setLEDs(uint8_t ledCurrentRed, uint8_t ledCurrentIr);
void MAX30100_setSR(uint8_t sr);
void MAX30100_setPW (uint8_t pw);

uint8_t MAX30100_read (uint8_t device_register);
void MAX30100_write (uint8_t device_register, uint8_t reg_data);
uint8_t MAX30100_getNumSamp(uint16_t* ir_buff, uint16_t* red_buff);

uint16_t lp_iir_filter( uint16_t input_value );
int16_t hp_iir_filter( int16_t input_value );
int16_t process_sample( int16_t value );

void uprintf(char *str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Resets the MAX30100 IC
void MAX30100_reset(void)
{
   uint8_t reg;
   reg =  MAX30100_read (MAX30100_MODE_CONFIG);
   // RESET bit is B6
   // 0x40 = 0100 0000
   reg = (reg | 0x40); // Set reset bit to 1
   MAX30100_write (MAX30100_MODE_CONFIG, reg);
}

// Sets Heart rate mode

// This means MODE{2:0} = 0b010 (or 0x02 in hexadecimal)
void MAX30100_SetHR (void)
{
   uint8_t reg;
   reg =  MAX30100_read (MAX30100_MODE_CONFIG);
   // RESET bit is B7
   // First we clear bits 2:0
   reg = reg & 0xF8;
   // Then we set bits 2:0 to 0x02
   reg = reg | 0x02;
   MAX30100_write (MAX30100_MODE_CONFIG, reg);
}


// Sets SPO2 rate mode
// This means MODE{2:0} = 0b011 (or 0x03 in hexadecimal)
void MAX30100_SetSPO2 (void)
{
   unsigned short reg;
   reg =  MAX30100_read (MAX30100_MODE_CONFIG);
   // RESET bit is B7
   // First we clear bits 2:0
   reg = reg & 0xF8;
   // Then we set bits 2:0 to 0x03
   reg = reg | 0x03;
   MAX30100_write (MAX30100_MODE_CONFIG, reg);
}


// Wakes up the MAX30100
void MAX30100_wakeup(void)
{
   uint8_t reg;
   reg =  MAX30100_read (MAX30100_MODE_CONFIG);
   reg = reg & 0x7F; // Set SHDN bit to 0
   MAX30100_write (MAX30100_MODE_CONFIG, reg);
}

// Initializes FIFO
// Sets RD and WR pointers to 0
// Clears OVF
void MAX30100_InitFIFO (void)
{
   MAX30100_write (MAX30100_FIFO_W_POINTER, 0x00);
   MAX30100_write (MAX30100_FIFO_R_POINTER, 0x00);
   MAX30100_write (MAX30100_OVF_COUNTER, 0x00);
}
// Sets LED currents
void MAX30100_setLEDs(uint8_t ledCurrentRed, uint8_t ledCurrentIr)
{
   uint8_t reg;
   reg = ( ledCurrentRed << 4 ) | ledCurrentIr;
   MAX30100_write (MAX30100_LED_CONFIG, reg);
}

// Sets sample rate
// sample rate is bits 4:2 of register MAX30100_SPO2_CONFIG
// bitmask is 0xE3
void MAX30100_setSR (uint8_t sr)
{
   uint8_t reg;
   reg =  MAX30100_read (MAX30100_SPO2_CONFIG);
   reg = reg & 0xE3;
   reg = reg | (sr << 2);
   MAX30100_write (MAX30100_SPO2_CONFIG, reg);
}

// Sets pulse width
// sample rate is bits 1:0 of register MAX30100_SPO2_CONFIG
void MAX30100_setPW (uint8_t pw)
{
   uint8_t reg;
   reg =  MAX30100_read (MAX30100_SPO2_CONFIG);
   reg = reg & 0xFC;
   reg = reg | pw;
   MAX30100_write (MAX30100_SPO2_CONFIG, reg);
}

// My I2C read and write functions
uint8_t MAX30100_read (uint8_t device_register )
{
   uint8_t read_data;
   HAL_I2C_Mem_Read(&hi2c1,MAX30100_RADDRESS,device_register,I2C_MEMADD_SIZE_8BIT,&read_data,1,50);
   return read_data;
}

void MAX30100_write (uint8_t device_register, uint8_t reg_data)
{
   HAL_I2C_Mem_Write(&hi2c1,MAX30100_WADDRESS,device_register,I2C_MEMADD_SIZE_8BIT,&reg_data,1,50);
}

void uprintf(char *str)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str), 100);
}

// Gets number of samples in FIFO and read then
uint8_t MAX30100_getNumSamp(uint16_t* ir_buff, uint16_t* red_buff)
{
    uint8_t wreg;
    uint8_t rreg;
    uint8_t sampleNum;
    uint8_t samples[4];
    wreg = MAX30100_read (MAX30100_FIFO_W_POINTER);
    rreg = MAX30100_read (MAX30100_FIFO_R_POINTER);
    sampleNum = (abs( 16 + wreg - rreg ) % 16);

    if(sampleNum > 0)
    {
    	for(int i=0; i<sampleNum; i++)
    	{

       	 HAL_I2C_Mem_Read(&hi2c1,MAX30100_RADDRESS,MAX30100_FIFO_DATA_REG,1,&samples[0],4,1000);
     //  	 HAL_I2C_Mem_Read(&hi2c1,MAX30100_RADDRESS,MAX30100_FIFO_DATA_REG,1,&samples[1],1,50);
    //  	 HAL_I2C_Mem_Read(&hi2c1,MAX30100_RADDRESS,MAX30100_FIFO_DATA_REG,1,&samples[2],1,50);
    //     HAL_I2C_Mem_Read(&hi2c1,MAX30100_RADDRESS,MAX30100_FIFO_DATA_REG,1,&samples[3],1,50);

        *(ir_buff) =  (uint16_t)samples[1];
        *(ir_buff++) |= (uint16_t)samples[0] << 8;
        *(red_buff) = (uint16_t)samples[3];
        *(red_buff++) |=  (uint16_t) samples[2] << 8;

    	}

    }

    return sampleNum;
}


int16_t hp_iir_filter( int16_t input_value )
{

	//IIR Biquad, 100 sampling, High pass filter cut off 1Hz, Q = .707, Gain 6 dB

	static double a0 = 0.956542835577484;  //SR=100, fc=1hz
	static double a1 = -1.913085671154968;
	static double a2 = 0.956542835577484;
	static double b1 = -1.911196288237583;
	static double b2 = 0.914975054072353;

/*	static double a0 = 0.9780302754084559;  //SR=200, fc=1hz
	static double a1 = -1.9560605508169118;
	static double a2 = 0.9780302754084559;
	static double b1 = -1.9555778328194147;
	static double b2 = 0.9565432688144089;   */


	static double z1 = 0;
	static double z2 = 0;


	uint16_t ret;
	double outd;
	double new_seed;

 	new_seed = (double)input_value - z1 * b1 - z2 * b2;
	outd = new_seed* a0 + z1 * a1 + z2 * a2;

	outd = ((-1) * outd );

	z2  = z1 ;
	z1  = new_seed;

	ret = (int16_t) outd;

        return ret;
}


uint16_t lp_iir_filter( uint16_t input_value )
{

	//IIR Biquad, 100 sampling, Low pass filter cut off 8Hz, Q = .707, Gain 6 dB
/*	static double  a0 = 0.005542711916075981;
	static double  a1 = 0.011085423832151962;
	static double  a2 = 0.005542711916075981;
	static double  b1 = -1.7786300789392977;
	static double  b2 = 0.8008009266036016;  */

	static double  a0 = 0.046131689679824825;   // SR=100, fc=8hz
	static double  a1 = 0.09226337935964965;
	static double  a2 = 0.046131689679824825;
	static double  b1 = -1.3072818432709727;
	static double  b2 = 0.49180860199027215;

/*	static double a0 = 0.013359180867844565;  //SR=200, fc=8hz
	static double a1 = 0.02671836173568913;
	static double a2 = 0.013359180867844565;
	static double b1 = -1.6474576182593796;
	static double b2 = 0.7008943417307579;   */



	static double z1 = 0;
	static double z2 = 0;


	uint16_t ret;
	double outd;
	double new_seed;

 	new_seed = (double)input_value - z1 * b1 - z2 * b2;
	outd = new_seed* a0 + z1 * a1 + z2 * a2;

	z2  = z1 ;
	z1  = new_seed;

	ret = (uint16_t) outd;

        return ret;
}

/*
int16_t process_sample( int16_t value )
{

	   static int16_t pre_value;
	   uint8_t i=0;
	   static uint8_t sw_case, cnt[16], count, bpm;

	   count++;
	   switch (sw_case)
	   {

	               case 0:
	            	    if(value > UPPER_TH)
	            		{
	            	   		//uprintf("Inside case:0\n");
	            	   		sw_case++;
	            	   	}
	            	    break;
	               case 1:
	            	    if(value < pre_value)
	            	   	{
	            	    	cnt[++i]=count;
	            	    //	T = (cnt[i]-cnt[i-1]);
	            	    	bpm=((200*60)/(cnt[i]-cnt[i-1]));

	            	    	itoa(bpm,hr_buff,10);

	            	    	//uprintf("Inside case:1\n");
	            	        uprintf(" bpm =     ");
	            	        uprintf(hr_buff);
	            	        uprintf("\n");

	            	    	sw_case++;
	            	   	}
	            	    break;
	               case 2:
	            	    if(value < LOWER_TH)
	            	   	{
	            	    	sw_case=0;
	            	   		//uprintf("Inside case:2\n");
	            	   	}
	                    break;
	               default:
	               { }

	               pre_value = value;

    	}//sw_end
}
*/

int16_t process_sample( int16_t value )
{

  static int16_t pp_value=0, pre_value = 0;
  static uint8_t sw_case = 0;
  static uint16_t bpm = 0;
  static uint32_t cnt[2] = {0,0}, count = 0, arr_cnt[6] = {0, 0, 0, 0, 0, 0};

  count++;
  if((count % 100) == 0)
  {
	  char pdata[20];
	  sprintf(pdata, "sec = %d\n", count);
	//  uprintf(pdata);
  }


  switch (sw_case)
  {

            case 0:
            if(value > UPPER_TH)
            {
              //uprintf("Inside case:0\n");
              sw_case = 1;
            }
            break;
            case 1:
            if((value < pre_value) && (pre_value < pp_value))
            {
             //   cnt[0]=cnt[1];
             //   cnt[1] = count;
             //   bpm=( ((uint32_t)(100*60)) /(cnt[1]-cnt[0]));

            	arr_cnt[0] = arr_cnt[1];
            	arr_cnt[1] = arr_cnt[2];
            	arr_cnt[2] = arr_cnt[3];
            	arr_cnt[3] = arr_cnt[4];
            	arr_cnt[4] = arr_cnt[5];
            	arr_cnt[5] = count;

            	if( arr_cnt[0] != 0)
            	{
            		bpm=( ((uint32_t)(100*60*5)) /(arr_cnt[5]-arr_cnt[0]));
            	}


        /*      itoa(bpm,hr_buff,10);
             // uprintf("Inside case:1\n");
                uprintf(" bpm =     ");
                uprintf(hr_buff);
                uprintf("\n");   */


                char print_buff[40];
                sprintf(print_buff, " bpm = %d\n", (int)bpm);
                uprintf(print_buff);

                sw_case = 2;
              }
              break;
              case 2:
              if(value < LOWER_TH)
              {
                sw_case = 0;
              //uprintf("Inside case:2\n");
              }
              break;
              default:
              {
            	  	;
              }

    }//sw_end

    pp_value = pre_value;
  	pre_value = value;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */
 
  HAL_I2C_Mem_Read(&hi2c1, MAX_30100_ADDR, 0xFF, 1, buf, 1, 50);

    	if(buf[0]==0x11)
    	{
    	    uprintf("buf=0x11\n");
    	}

    	MAX30100_reset();
    	// Set LED current
        MAX30100_setLEDs(0x02, 0x02);
        // Set sample rate
        MAX30100_setSR(0x01);   //100
        // Set pulse width
        MAX30100_setPW(0x03);
        // Set heart rate mode
        MAX30100_SetHR ();
        // We set SpO2 mode and start measuring
        MAX30100_SetSPO2 ();
        // Set RD and WR pointers to 0x00
        MAX30100_InitFIFO();
        // Wake up
        MAX30100_wakeup();


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	            // Gets the number  of samples in the FIFO and reads then
	  	        samples = MAX30100_getNumSamp((uint16_t*)ir_buff, (uint16_t*)red_buff);

	  	        if (samples > 0 )
	  	        {

	  	           for (int i = 0; i < samples; i++)
	  	           {


	  	        	   ir_lf_buff[i] = lp_iir_filter( ir_buff[i]);
	  	        	   ir_hf_buff[i] = hp_iir_filter((int16_t)ir_lf_buff[i]);

					   ser[0]=0xF7;
					   ser[1]=((uint16_t)ir_hf_buff[i]) >> 8;     // high byte
					   ser[2]=((uint16_t)ir_hf_buff[i]) & 0x00FF; // low byte

					   process_sample(ir_hf_buff[i]);
				//	   HAL_UART_Transmit(&huart1, ser, 3, 100);


	  	           }//for

	  	        }//if


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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
