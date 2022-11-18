/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *******************************************************************************/

#include "main.h"
#include "ADC.h"
#include "USART.h"
#include "DM.h"

/* Private variables & Function Prototypes ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
void SystemClock_Config(void);

// Global Variables
#define ADC_ARR_LEN 7500
#define V_TOLERANCE 2
#define MIN_PTP_VAL 50

uint8_t ADC_flag = 0;
uint16_t ADC_value = 0;
uint16_t ADC_Arr[ADC_ARR_LEN];
uint16_t sample_Max;
uint16_t sample_Min;


uint8_t DC_FLAG = 0;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset peripherals, & set system clock */
  HAL_Init();
  SystemClock_Config();

  // Set up data transfer protocol
  USART_init();

  // Set up UI
  GUI_init();

  // Set Up ADC
  ADC_init();
  ADC1->CR |= ADC_CR_ADSTART;	// Starts conversion process

  while (1)
  {
	  uint16_t DC_Offset = 0;
	  uint16_t Vpp = 0;
	  uint16_t samples_Taken = 0;	//counter for number of samples taken
	  uint16_t t_Max = 0;
	  uint16_t t_Min = 0;
	  uint16_t first_Val = 1;

	  while(samples_Taken < ADC_ARR_LEN){ // Sample Lens

		  if(ADC_flag && first_Val){
			  //storing the first case to both max and min
			  t_Max = ADC_Conversion(ADC_value);
			  t_Min = ADC_Conversion(ADC_value);
			  first_Val = 0;
		  }
		  if(ADC_flag){ //takes value every tenth read
			  int analogVal = ADC_Conversion(ADC_value);

			  //Convert Analog to Digital and stores it in Array
			  ADC_Arr[samples_Taken] = analogVal;
			  ADC_flag = 0;	//Reseting conversion flag

			  // Find Absolute Max During Period
			  if(analogVal > t_Max ){
				  //Replacing new max
				  t_Max = analogVal;
				  sample_Max = samples_Taken;
			  }

			  // Find Absolute Min During Period
			  else if(analogVal < t_Min){
				  //Replacing new min
				  t_Min = analogVal;
				  sample_Max = samples_Taken;
			  }

			  if(samples_Taken < ADC_ARR_LEN){
				  //Checking to insure that interrupts don't happen during Avg calculation
				  ADC1->CR |= ADC_CR_ADSTART; //start recording again
			  }
		  }
	  }

	  //Getting peak to peak voltage and DC offset
	  Vpp = t_Max - t_Min;

	  //TO DO: if Peak to Peak is < 0.5V must be DC
	  if(Vpp < MIN_PTP_VAL){
		  DC_FLAG = 1; // Indicates a DC Signal
	  }
	  else{
		  DC_FLAG = 0; // Indicates an AC Signal
	  }

	  // If an AC Signal
	  if(DC_FLAG == 0){
		  DC_Offset = t_Max - (Vpp/2);

		  uint8_t extrema_Flag = 1; // Flag if it reaches a max or min
		  uint16_t index = 0; //index of ADC_Arr
		  uint16_t num_Zeros = 0; //number of zero crossings(index of zero_sample_Num)
		  uint16_t zero_sample_Num[3];

		  while(num_Zeros < 3){ //checking for zero crossings only need 3
			  if(ADC_Arr[index] > (DC_Offset - V_TOLERANCE) &&
				 ADC_Arr[index] < (DC_Offset + V_TOLERANCE) &&
				 (extrema_Flag == 1) ) //checking to insure the zero crossing value is not in the same area as previous read

			  { //finding points where wave crosses zero
				  extrema_Flag = 0;
				  if(index > (zero_sample_Num[num_Zeros] + 74) && num_Zeros != 0) //*******NOTE: CHANGE 74 TO WHATEVER THE ANALOG EQUIVALENT IS 0->330
				  {
					  zero_sample_Num[num_Zeros] = index; //adding zero crossing to table
					  num_Zeros ++;
				  }
			  }
			  else{
				  extrema_Flag = 1;
			  }
		  }
		  int AC_Values[4];
		  Find_AC_Params(ADC_Arr, zero_sample_Num, ADC_ARR_LEN , AC_Values);

		  int vrms = calc_RMS(Vpp);
		  //**********NOTE: These are now analog values 0->330
		  int sample_Freq = zero_sample_Num[2] - zero_sample_Num[0]; //gives the differences between every other crossing
		  sample_Freq = (1/(sample_Freq * 640.5)*48000000); //translation to Frequency(IDK if this works properly)

		  update_AC(vrms, Vpp, sample_Freq);
	  }
	  // If a DC Signal
	  else{
		  //Find min/max/average and store them
		  int Avg_Dig_Vals[3]; // These are saved as integers not doubles

		  ADC_Avg(ADC_Arr, ADC_ARR_LEN ,Avg_Dig_Vals);

		  //Print to Terminal
		  update_DC(Avg_Dig_Vals[0], Avg_Dig_Vals[1], Avg_Dig_Vals[2]);
	  }

	  // Temporary Delay for Easier Value Reading
	  //HAL_Delay(500);

	  ADC1->CR |= ADC_CR_ADSTART; //start recording again
  }
}

int counter = 10;
void ADC1_2_IRQHandler(){
	if(ADC1->ISR & ADC_ISR_EOC){
		ADC1->ISR &= ~(ADC_ISR_EOC);
		ADC_value = ADC1->DR;
		if(counter == 0){
			ADC_flag = 1;
			counter = 11;
		}
		counter--;
	}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

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
