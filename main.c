/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "task.h"
//#include "usbd_hid_core.h"
//#include "usbd_usr.h"
//#include "usbd_desc.h"

#include "main.h"
#include "stm32f4xx_conf.h"
#include "usart.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555
#define ADC1_DR_Address   ((uint32_t)0x4001204C)

/* PWM Timer mapping */
#define PWM1 (TIM2 -> CCR3)
#define PWM2 (TIM12 -> CCR1)  // right of the front
#define PWM3 (TIM12 -> CCR2)
#define PWM4 (TIM10 -> CCR1)
#define PWM5 (TIM11 -> CCR1)
#define PWM6 (TIM4 -> CCR1)  // center of the front
#define PWM7 (TIM4 -> CCR2)
#define PWM8 (TIM3 -> CCR3)
#define PWM9 (TIM3 -> CCR4)
#define PWM10 (TIM9 -> CCR2)  // left of the front

__IO uint32_t TimingDelay;
__IO uint8_t UserButtonPressed = 0x00;

/* Lidar (Usart) -------------------------------------------------------------*/
const int front_obstacle_region[90] = {500, 500, 500, 501, 501, 502, 503, 504, 505, 506, 508, 509, 511, 513, 515, 518, 520, 523, 526, 529, 532, 536, 539, 543, 547, 552, 556, 561, 566, 572, 577, 583, 590, 596, 603, 610, 618, 626, 635, 643, 653, 663, 673, 684, 695, 707, 720, 733, 747, 762, 778, 795, 812, 831, 851, 872, 894, 918, 944, 971, 1000, 1031, 1065, 1101, 1141, 1183, 1229, 1280, 1335, 1395, 1462, 1536, 1618, 1710, 1814, 1932, 2067, 2223, 2405, 2620, 2879, 3196, 3593, 4103, 4783, 5737, 7168, 9554, 14327, 28649};
static uint8_t aRxBuffer = 0;
int degree_distance[180] = {0};    /* Use this array to store distance information of front 180 degree */
int min_distance[9] = {0};
int check_sum(void);
int check_index(unsigned int);
void reset_tmp(void);
uint16_t distance_convert(uint8_t data0, uint8_t data1);
struct lidarData {
  uint8_t data[16];     /* Store the data of 4 angles */
  uint64_t time_stamp[2];  /* Use this to check update time */
};
volatile struct lidarData lidarBuffer[LidarPacketNumber];
/* Lidar (Usart) -------------------------------------------------------------*/

/**
  * @brief  Use this task to check wethrer the FreeRTOS scheduling is working.
  * @param  None
  * @retval None
  */
static void LED_task(void *pvParameters)
{
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  
  while(1)
  {    
      /* Toggle LED5 */
      STM_EVAL_LEDToggle(LED3);
      vTaskDelay(100);
      STM_EVAL_LEDToggle(LED4);
      vTaskDelay(100);
      STM_EVAL_LEDToggle(LED5);
      vTaskDelay(100);
  }
}

/**
  * @brief  Use this task to read and check the data sent from Lidar.
  * @param  None
  * @retval None
  */
static void usart_receive_task(void *pvParameters){
  //int tmp_index;
  //LCD_Clear(LCD_COLOR_WHITE);
  //LCD_SetBackColor(LCD_COLOR_WHITE);
  
  while(1){
    /* Enable the Rx interrupt */
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    
    /* Waiting the end of Data transfer  */ 
    while(current_received_byte < LidarPacketSize);
    
    /* disable USART interrupt */
      NVIC_DisableIRQ(USARTx_IRQn);
        
    if(check_index(current_packet[1]) & check_sum()){
      write_packet();
    }
    reset_tmp();
  }
}

/**
  * @brief  This task used to 
  * @param  None
  * @retval None
  */
static void lidar_display_task(void *pvParameters){
  //int tmp_index;
  int changed_count = 0;
  int index_count = 0;
  int tmp_array [200];
  int index_statistic [90] = {0};
  
  while(1){
    while(tmp_array[changed_count-1] == current_packet[1]);
      
    tmp_array[changed_count++] = current_packet[1];
    index_statistic[current_packet[1]-160]++; 
    
    if(changed_count == 199){
      while(index_count < 90){
        //printf("%d,", index_statistic[index_count]);
        index_count++;
      }
      index_count = 0;
      changed_count = 0;
    }
  }
}

static void Draw_LCD_task(void *pvParameters){
  int count = 0;
  uint16_t distance_angle1 = 0;
  uint16_t distance_angle2 = 0;
  uint16_t distance_angle3 = 0;
  uint16_t distance_angle4 = 0;

  while(1){
    for(count = 0; count < 45; count++){
      distance_angle1 = distance_convert(lidarBuffer[count].data[0], lidarBuffer[count].data[1]);
      //printf("A: %d D: %d \n", count * 4, distance_angle1);
      distance_angle2 = distance_convert(lidarBuffer[count].data[4], lidarBuffer[count].data[5]);
      //printf("A: %d D: %d \n", count * 4 + 1, distance_angle2);
      distance_angle3 = distance_convert(lidarBuffer[count].data[8], lidarBuffer[count].data[9]);
      //printf("A: %d D: %d \n", count * 4 + 1, distance_angle3);
      distance_angle4 = distance_convert(lidarBuffer[count].data[12], lidarBuffer[count].data[13]);
      //printf("A: %d D: %d \n", count * 4 + 1, distance_angle4);
      
      /* Write data to distance array */
      degree_distance[count*4] = distance_angle1;
      degree_distance[count*4 + 1] = distance_angle2;
      degree_distance[count*4 + 2] = distance_angle3;
      degree_distance[count*4 + 3] = distance_angle4;
    }
    vTaskDelay(30);
  }
}

/**
  * @brief  Use this function to check index of receive data is in the range of correct index number.
  * @param  None
  * @retval None
  */
int check_index(unsigned int index){
  if(index >= 0xA0 && index < 0xFA){ 
    return 1;
  }
  else{ 
    return 0;
  }
}

/**
  * @brief  Use this function to check the packets are correct.
  * @param  None
  * @retval None
  */
int check_sum(){
  int count;
  int transfer;
  long check32 = 0;
  long checksum;
  long received_checksum;

  for (count = 0; count < 10; count++){
    transfer = current_packet[2 * count] + ((int)current_packet[2 * count + 1] << 8);
    check32 = (check32 << 1) + transfer;
  }

  checksum = (check32 & 0x7FFF) + (check32 >> 15);
  checksum = checksum & 0x7FFF;
  received_checksum = current_packet[20] + ((int)current_packet[21] << 8);
  
  if (checksum == received_checksum){
    return 1;
  }
  else{
    return 0;
  }
}

/**
  * @brief  This function write the correct data to lidarBuffer.
  * @param  None
  * @retval None
  */
void write_packet(){
  int count = 0;
  int index = current_packet[1] - 160/* index start from (A0)16 = (160)(10)*/;
  //lidarBuffer[index].time_stamp[0] = lidarBuffer[index].time_stamp[1];
  //lidarBuffer[index].time_stamp[1] = LocalTime;

  for(; count < 16; count++){
    lidarBuffer[index].data[count] = current_packet[count];
  }
  writed_packet_num ++;
}

/**
  * @brief  This function reset the flags and counts, and reenable the Interrrupt of Usart.
  * @param  None
  * @retval None
  */
void reset_tmp(){
  /* enable USART interrupt */
  NVIC_EnableIRQ(USARTx_IRQn);
  
  current_received_byte = 0;
  started_flag = 0;
}

/**
  * @brief  This function convert the distance data from byte format.
  * @param  None
  * @retval None
  */
uint16_t distance_convert(uint8_t data0, uint8_t data1){
  uint16_t distance_data = ((0x00FF & data1) << 8) | data0;
  
  if ((distance_data & 0x8000) == 0x8000){
    return 0;
  }
  else{
    return distance_data & 0x3FFF;
  }
}

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  if(UserButtonPressed){
    UserButtonPressed=0x00;
  }else
    UserButtonPressed=0x01;

}

void USARTx_IRQHandler(void)
{
  /* USART in Receiver mode */
  if ((USART_GetITStatus(USARTx, USART_IT_RXNE) == SET))
  {
    /* Receive the data */
    aRxBuffer = USART_ReceiveData(USARTx);
    //printf("%d ", aRxBuffer);
    
    if(aRxBuffer == 250/* FA(HEX) = 250(DEC) */){
      started_flag = 1;
    }

    /* after received start byte, storing the data to current_packet[] */
    if(started_flag == 1 && (current_received_byte < LidarPacketSize)){
      current_packet[current_received_byte] = aRxBuffer;
      current_received_byte ++;
    }
  }
}

int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Configure SysTick */
  RCC_GetClocksFreq(&RCC_Clocks);

  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  SystemInit();

  //PSRAM_Init();
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_PASS);
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize LEDs to be managed by GPIO */
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  USART_Config();

  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00;

  xTaskCreate(LED_task,(signed portCHAR *) "Implement LED",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(usart_receive_task,(signed portCHAR *) "Implement USART",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(lidar_display_task,(signed portCHAR *) "Implement Lidar display",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(Draw_LCD_task,(signed portCHAR *) "Implement LCD draw",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  
  /* Start running the tasks. */
  vTaskStartScheduler(); 

  return 0;
}

void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USARTx, ENABLE);  

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */ 
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);
  
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    vTaskDelay(5);
  }
}


void vApplicationTickHook()
{
}
