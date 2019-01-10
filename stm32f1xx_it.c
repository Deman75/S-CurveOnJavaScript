/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

/*  Описание контактов
PA1,PA2,PA3,PA4 - Пины идущие на входы управления.
PA6 - Шим на пины 0 и 2
PA7 - Шим на пины 1 и 3
PA2 - АЦП для теста.
*/

#define Pin1Set   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define Pin1ReSet HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)
#define Pin2Set   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)
#define Pin2ReSet HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)
#define Pin3Set   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define Pin3ReSet HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)
#define Pin4Set   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define Pin4ReSet HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)

#define PWM1 TIM3->CCR4
#define PWM2 TIM3->CCR3


//-------------5v 20kHz----------
#define s1 650
#define s2 686
#define s3 712
#define s4 733
#define s5 750
#define s6 768
#define s7 785
#define s8 800
#define s9 815
#define s10 830
#define s11 848
#define s12 863
#define s13 882
#define s14 905
#define s15 928
#define s16 1000
//-------------5v 20kHz----------

int16_t Pos=0, NeedPos=0, Upd=0, Delay=20,NeedDelay,DelayStart=0, PID_start=0;
int8_t  Step=1, Pin=1, Dir=1, FirstStart=1, FirstStartStep=0;
int16_t S[16]={s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16};
void PinOut (int8_t pin, int8_t step);
extern void PID (int16_t* pos, int16_t* needpos, int16_t* delay, int8_t* direction, int16_t* needdelay);

extern uint16_t MinSpeed;
extern uint16_t taho;

			
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt.
*/
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

	if (FirstStart==0)
	{
		NeedPos=taho;
		if (NeedPos>5000) NeedPos=5000;
		if (NeedPos==0 || NeedPos==5000) // При возврате в 0 и в максимальную позицию нужно быстрое затухание скорости.
				MinSpeed=1000;
			else
				MinSpeed=1000;		
	}
	else
	 {
		 if (FirstStartStep==0)
		 {
			 NeedDelay = Delay;
			 Pos=1000;
			 MinSpeed=200;
			 FirstStartStep++;
		 }
		 if (FirstStartStep==1)
		 {
			 NeedPos=0;
			 if (Pos==NeedPos) FirstStartStep++;
		 }
		 if (FirstStartStep==2)
		 {
			 NeedPos=5000;
			 if (Pos==NeedPos) FirstStartStep++;
		 }
		 if (FirstStartStep==3)
		 {
			 NeedPos=0;
			 if (Pos==NeedPos) 
			 {
				 FirstStart=0;
				 MinSpeed=1000;
			 }
		 }	 
	 }
	

	 
	PID_start++;
	Upd++;
	
	DelayStart++;
//	if (Upd == NeedDelay)
//		{
//			if (NeedDelay < Delay)
//				{
//					NeedDelay++;
//				}
//			else if (NeedDelay > Delay)
//				{
//					NeedDelay--;
//				}
//			DelayStart=0;	
//		}
	 
	if (Dir==1 && Upd==NeedDelay)
		{
			Step++;
			if (Step>16) 
				{
					Step=1;
					Pin++;
					if (Pin>4) Pin=1;				
				}	
			PinOut(Pin,Step);
			Pos++;	
			Upd=0;
			if (NeedDelay < Delay)
				{
					NeedDelay++;
				}
			else if (NeedDelay > Delay)
				{
					NeedDelay--;
				}
		}
	
		
	if (Dir==-1 && Upd==NeedDelay)
		{
			Step--;
			if (Step<1) 
				{
					Step=16;
					Pin--;
					if (Pin<1) Pin=4;					
				}	
			PinOut(Pin,Step);
			Pos--;
			Upd=0;
			if (NeedDelay < Delay)
				{
					NeedDelay++;
				}
			else if (NeedDelay > Delay)
				{
					NeedDelay--;
				}
		}	
		
	if (Upd>3000)
		{
			Upd=0;
		}
		
	if (PID_start==10)
	{
		PID_start=0;
		PID(&Pos,&NeedPos,&Delay, &Dir, &NeedDelay);
	}	

	
	


  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void PinOut (int8_t pin, int8_t step)
{
	if (pin==1)
		{
			Pin1Set;
			Pin2Set;
			Pin3ReSet;
			Pin4ReSet;
			PWM2=S[step-1];
			PWM1=S[16-step-1];
		}
	if (pin==2)
		{
			Pin2Set;
			Pin3Set;
			Pin4ReSet;
			Pin1ReSet;
			PWM1=S[step-1];
			PWM2=S[16-step-1];
		}
	if (pin==3)
		{
			Pin3Set;
			Pin4Set;
			Pin1ReSet;
			Pin2ReSet;
			PWM2=S[step-1];
			PWM1=S[16-step-1];
		}
	if (pin==4)
		{
			Pin4Set;
			Pin1Set;
			Pin2ReSet;
			Pin3ReSet;
			PWM1=S[step-1];
			PWM2=S[16-step-1];
		}
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
