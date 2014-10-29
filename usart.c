#include "main.h"
#include "stm32f4xx_conf.h"

void init_USART() 
{

	/* Enable APB2 peripheral clock for USART2
	 * and PA2->TX, PA3->RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);        

	/* Enable APB2 peripheral clock for USART2 */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIOA Configuration for USART2 */
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  ///< PA2(TX) and PA3(RX) are used 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            ///< Configured pins as alternate function 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       ///< IO speed 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          ///< Output type as push-pull mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;           ///< Activates pullup resistor
	GPIO_Init(GPIOA, &GPIO_InitStructure);                  ///< Initial GPIOA

	/* Connect GPIO pins to AF */ 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Configuration for USART2 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;                      ///< Baudrate is set to 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     ///< Standard word length = 8 bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          ///< Standard stop bit = 1 bit
	USART_InitStructure.USART_Parity = USART_Parity_No;             ///< Standard parity bit = NONE
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; ///< No flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; ///< To enable receiver and transmitter
	USART_Init(USART2, &USART_InitStructure);                       ///< Initial USART2

	/* Enable receiver interrupt for USART2 and 
	 * Configuration for NVIC 
	 * */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  ///< Enable USART2 receiver interrupt 

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                        ///< Configure USART2 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       ///< Set the priority group of USART2 interrupt
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                        ///< Set the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                ///< Globally enable USART2 interrupt
	NVIC_Init(&NVIC_InitStructure);                                                                ///< Initial NVIC         

	/* Enable USART2 */
	USART_Cmd(USART2, ENABLE);
}

