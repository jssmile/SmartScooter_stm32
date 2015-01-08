/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_DEMO_H
#define __STM32F4_DISCOVERY_DEMO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
//#include "stm32f4_lcd.h"
#include <stdio.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* TIM2 Autoreload and Capture Compare register values */
#define TIM_ARR                          (uint16_t)1999
#define TIM_CCR                          (uint16_t)1000

/* MEMS Microphone SPI Interface */
#define SPI_SCK_PIN                   GPIO_Pin_10
#define SPI_SCK_GPIO_PORT             GPIOB
#define SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                GPIO_PinSource10
#define SPI_SCK_AF                    GPIO_AF_SPI2

#define SPI_MOSI_PIN                  GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT            GPIOC
#define SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE               GPIO_PinSource3
#define SPI_MOSI_AF                   GPIO_AF_SPI2

/* Exported macro ------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x
#define MAX(a,b)       (a < b) ? (b) : a
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
void Fail_Handler(void);

/* Lidar USART */
#define LidarPacketNumber        90
#define LidarPacketSize          22
#define USART_BUFFERSIZE         44
void USARTx_IRQHandler(void);
void USART_Config(void);
void write_packet(void);

static __IO uint8_t ubRxIndex = 0x00;
static __IO uint8_t ubTxIndex = 0x00;
static volatile int current_received_byte = 0; // Counting for how many byte have been received after the start byte
static volatile int started_flag = 0;          // 1:After usart receive start byte "FA"   0:If usart still not receive start byte "FA"
static volatile uint8_t current_packet [LidarPacketSize];
static volatile long writed_packet_num = 1;

#define USARTx                           USART3
#define USARTx_CLK                       RCC_APB1Periph_USART3
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_10                
#define USARTx_TX_GPIO_PORT              GPIOB                       
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define USARTx_TX_SOURCE                 GPIO_PinSource10
#define USARTx_TX_AF                     GPIO_AF_USART3

#define USARTx_RX_PIN                    GPIO_Pin_11                
#define USARTx_RX_GPIO_PORT              GPIOB                    
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define USARTx_RX_SOURCE                 GPIO_PinSource11
#define USARTx_RX_AF                     GPIO_AF_USART3

// usart6
#define USARTy                           USART6
#define USARTy_CLK                       RCC_APB2Periph_USART6
#define USARTy_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTy_IRQn                      USART6_IRQn
#define USARTy_IRQHandler                USART6_IRQHandler

#define USARTy_TX_PIN                    GPIO_Pin_6                
#define USARTy_TX_GPIO_PORT              GPIOC                       
#define USARTy_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTy_TX_SOURCE                 GPIO_PinSource6
#define USARTy_TX_AF                     GPIO_AF_USART6

#define USARTy_RX_PIN                    GPIO_Pin_7                
#define USARTy_RX_GPIO_PORT              GPIOC                    
#define USARTy_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTy_RX_SOURCE                 GPIO_PinSource7
#define USARTy_RX_AF                     GPIO_AF_USART6


/* Direction Suggestion */
void direction_suggestion(void);
//static void min_distance_calculation(void);
//static void distance_remind(void);
//static int find_left_weight(void);
//static int find_right_weight(void);
int front_obstacle(void);
void guide_pointer(int suggested_direction);

/* PWM Motor */
void Motor_PWM_config(void);
void PA2_PWM_config(void);
void PB14_15_PWM_config(void);
void PB4_5_PWM_config(void);
void PC8_9_PWM_config(void);
void PE6_PWM_config(void);
void PB9_PWM_config(void);
void PD12_15_PWM_config(void);
void Configure_PB12(void);

/* ADC Config */
void DMA2_Stream4_IRQHandler(void);

/* Lidar USART Config */
void USARTx_IRQHandler(void);

#endif /* __STM32F4_DISCOVERY_DEMO_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
