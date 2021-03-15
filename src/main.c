
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);

uint8_t ReadNibble(uint32_t address)
{
  uint16_t val;
  uint16_t row,col;

  uint16_t gpa=GPIOA->ODR&0xF800;

  row=gpa|((address>>11)&0x7FF);
  col=gpa|(address&0x7FF);
  // Out Row
  GPIOA->ODR=row; GPIOB->BSRR=0x00020000; // Ras=0
  // Out Col
  GPIOA->ODR=col; GPIOB->BSRR=0x00010000; // Cas=0
  asm("nop");asm("nop");asm("nop");
  val=GPIOB->IDR;
  GPIOB->BSRR=0x00000003; // Ras/Cas=1
  return (val&0x00F0)>>4;
}

void WriteNibble(uint32_t address, uint8_t data)
{
  uint16_t row,col;
  uint16_t gpa=GPIOA->ODR&0xF800;
  uint32_t moder=GPIOB->MODER;

  row=gpa|((address>>11)&0x7FF);
  col=gpa|(address&0x7FF);
    // Out Row
  GPIOA->ODR=row; GPIOB->BSRR=0x00020000; // Ras=0
  // Out Data
  GPIOB->MODER=moder|0x00005500; // GPIOB4-7 as output
  GPIOB->BSRR=0x00F00000; GPIOB->BSRR=(data<<4);
  // Out Col
  GPIOA->ODR=col; GPIOB->BSRR=0x00050000; // Wr=0+Cas=0 - Data valid on bus
  asm("nop");
  GPIOB->MODER=moder;
  GPIOB->BSRR=0x00F00007;   // CAS/RAS/W = 1 
}


int main(void)
{
  uint32_t prevtick=0;
  uint32_t refresh=0;
  uint16_t i;

  uint32_t raddr=0,prevaddr=0;
  uint8_t testval;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();

  while (1)
  {
    if ((HAL_GetTick()-prevtick)>1000) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      prevtick=HAL_GetTick();
    }

    if ((HAL_GetTick()-refresh)>30)
    {      
      GPIOB->BSRR=0x04;       // WE High - just to be sure
      refresh=HAL_GetTick();
      GPIOB->BSRR=0x00010000;  //  CBR refresh (Cas-Before-Ras)
      for(i=0;i<2048;i++)
      {         
        GPIOB->BSRR=0x00020000;  // ~90ns low (min 60) / 60 ns (min 40) high
        asm("nop");              // 
        asm("nop");              // 
        GPIOB->BSRR=0x00000002;  //
      }
      GPIOB->BSRR=0x00000001; 
    }

   switch ((raddr>>24)&0xF)
   {
     case 0:
        testval=0xF; break;
     case 1:
        testval=0x0; break;
     case 2:
        testval=0xA; break;
     case 3:
        testval=0x5; break;
     case 4:
        testval=0x1; break;
     case 5:
        testval=0x2; break;
     case 6:
        testval=0x4; break;
     case 7:
        testval=0x8; break;
     case 8:
        testval=raddr&0xF; break;
     case 9:
        testval=1<<(raddr&0x3); break;
     case 10:
        testval=(raddr&1)?0xA:0x5; break;
     case 11:
        testval=(raddr&1)?0x0:0xF; break;
     case 12:
        testval=(raddr&0x800)?0xA:0x5; break;
     case 13:
        testval=(raddr&0x800)?0x0:0xF; break;
     default:
        testval=((raddr<<1)+raddr)&0xF;
   }

   //testval=(raddr&0xF)^((raddr>>24)&0xF);
   
   if ((raddr&(1<<22))==0)
    WriteNibble(raddr,testval);
  else
   {
    i=ReadNibble(raddr);
    if (i!=testval)
     {
      char debugstr[80];
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET );
      sprintf(debugstr,"Error ADDR : %08lx - WR : %02x / RD : %02x\n",raddr,testval,i);
      CDC_Transmit_FS((uint8_t*)debugstr,strlen(debugstr));
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
     }
   }
 

   if ((raddr&0x100000)!=prevaddr)
   {
     if (raddr&0xF00000) {
                          if ((raddr&(1<<22))==0) CDC_Transmit_FS((uint8_t*)"w",1);
                                          else    CDC_Transmit_FS((uint8_t*)"r",1);
                         }
                    else {
                            char debugstr[80];
                            sprintf(debugstr,"\n%08lx : w",raddr);
                            CDC_Transmit_FS((uint8_t*)debugstr,strlen(debugstr));
                         }
     prevaddr=raddr&0x100000;
   }

   raddr++;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim4.Init.Prescaler = 0;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4-7 - DATA */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
