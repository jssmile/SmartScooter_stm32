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
//#define PWM1 (TIM2 -> CCR3)
#define PWM1 (TIM12 -> CCR1)  // right of the front
#define PWM2 (TIM12 -> CCR2)
#define PWM3 (TIM10 -> CCR1)
#define PWM4 (TIM11 -> CCR1)
#define PWM5 (TIM4 -> CCR1)  // center of the front
#define PWM6 (TIM4 -> CCR2)
#define PWM7 (TIM3 -> CCR3)
#define PWM8 (TIM3 -> CCR4)
#define PWM9 (TIM9 -> CCR2)  // left of the front
#define DegreeShift 0         // Use this value to set the angle of lidar

__IO uint32_t TimingDelay = 0;
__IO uint8_t UserButtonPressed = 0x00;

__IO uint64_t currentTime = 0;
__IO uint64_t oldTime = 0;
__IO uint64_t Timing = 0;
volatile int over_flag = 0;

/* Lidar (Usart) -------------------------------------------------------------*/
//const int front_obstacle_region[90] = {500, 500, 500, 501, 501, 502, 503, 504, 505, 506, 508, 509, 511, 513, 515, 518, 520, 523, 526, 529, 532, 536, 539, 543, 547, 552, 556, 561, 566, 572, 577, 583, 590, 596, 603, 610, 618, 626, 635, 643, 653, 663, 673, 684, 695, 707, 720, 733, 747, 762, 778, 795, 812, 831, 851, 872, 894, 918, 944, 971, 1000, 1031, 1065, 1101, 1141, 1183, 1229, 1280, 1335, 1395, 1462, 1536, 1618, 1710, 1814, 1932, 2067, 2223, 2405, 2620, 2879, 3196, 3593, 4103, 4783, 5737, 7168, 9554, 14327, 28649};
const int sin_array[90] = {0, 17, 35, 53, 71, 89, 107, 124, 142, 160, 177, 195, 212, 230, 247, 265, 282, 299, 316, 333, 350, 366, 383, 400, 416, 432, 448, 464, 480, 496, 511, 527, 542, 557, 572, 587, 601, 616, 630, 644, 658, 671, 685, 698, 711, 724, 736, 748, 760, 772, 784, 795, 806, 817, 828, 838, 848, 858, 868, 877, 886, 895, 904, 912, 920, 928, 935, 942, 949, 955, 962, 968, 973, 979, 984, 989, 993, 997, 1001, 1005, 1008, 1011, 1014, 1016, 1018, 1020, 1021, 1022, 1023, 1023};
static uint8_t aRxBuffer = 0;
int16_t degree_distance[180] = {0};    /* Use this array to store distance information of front 180 degree */
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
int front_region_right[5] = {6000};
int front_region_left[5] = {6000};
int front_region[9] = {6000};
volatile int handler_count = 0; 
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
  * @brief  This task used to calculated how much time the data was changed in each index
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

/**
  * @brief  This task used to write distance data of each degree to the buffer
  * @param  None
  * @retval None
  */
static void Transfer_Distance_task(void *pvParameters){
  int count = 0;
  uint16_t distance_angle1 = 0;
  uint16_t distance_angle2 = 0;
  uint16_t distance_angle3 = 0;
  uint16_t distance_angle4 = 0;

  while(1){
    for(count = 0; count < 45; count++){
      //if(distance_convert(lidarBuffer[count].data[0], lidarBuffer[count].data[1]) != -1){
        distance_angle1 = distance_convert(lidarBuffer[count].data[0], lidarBuffer[count].data[1]);
      //}
      //printf("A: %d D: %d \n", count * 4, distance_angle1);

      //if(distance_convert(lidarBuffer[count].data[4], lidarBuffer[count].data[5]) != -1){
        distance_angle2 = distance_convert(lidarBuffer[count].data[4], lidarBuffer[count].data[5]);
      //}
      //printf("A: %d D: %d \n", count * 4 + 1, distance_angle2);
      
      //if(distance_convert(lidarBuffer[count].data[8], lidarBuffer[count].data[9]) != -1){
        distance_angle3 = distance_convert(lidarBuffer[count].data[8], lidarBuffer[count].data[9]);
      //}
      //printf("A: %d D: %d \n", count * 4 + 1, distance_angle3);
      
      //if(distance_convert(lidarBuffer[count].data[12], lidarBuffer[count].data[13]) != -1){
        distance_angle4 = distance_convert(lidarBuffer[count].data[12], lidarBuffer[count].data[13]);
      //}
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
  * @brief  Use this task to detect the front obstacle
  * @param  None
  * @retval None
  */
static void Front_Obstacle_task(void *pvParameters){
  int count, i;
  int tmp_degree;
  int16_t x_position;
  int valid_distance[9] = {0};

  while(1){

     /* for (i = 0; i < 9; i++){
        front_region[i] = 0;
        valid_distance[9] = 0;
      }*/
/*
      for(count = 0; count < 90; count++){
        if(degree_distance != -1){
          x_position = (degree_distance[count]*sin_array[90 - count]) >> 1000;
          
          if (x_position <= 500){
            front_region[x_position/110] += 6000 - ((degree_distance[count]*sin_array[count]) >> 10);
            valid_distance[x_position/110]++;
          }
        }
      }
      
      for(count = 91; count < 180; count++){
        if(degree_distance != -1){
          tmp_degree = 180 - count;
          x_position = (degree_distance[tmp_degree]*sin_array[90 - tmp_degree]) >> 10;

          if (x_position <= 500){
            front_region[x_position/110+4] += 6000 -((degree_distance[count]*sin_array[count]) >> 10);
            valid_distance[x_position/110+4]++;
          }
        }
      }
*/
        
        long tmp_1 = 0, tmp_2 = 0, tmp_3 = 0, tmp_4 = 0, tmp_5 = 0, tmp_6 = 0, tmp_7 = 0, tmp_8 = 0, tmp_9 = 0;
        int ratio = 19;
        long d_1 = ratio, d_2 = ratio, d_3 = ratio, d_4 = ratio, d_5 = ratio, d_6 = ratio, d_7 = ratio, d_8 = ratio, d_9 = ratio;
        int tmp_count = 0;

        for(tmp_count = 0; tmp_count < ratio; tmp_count++){
          if(degree_distance[tmp_count] >= 0){
            tmp_1 += degree_distance[tmp_count];
          }
          else{
            d_1 --;
          }

          if(degree_distance[tmp_count+ratio] >= 0){
            tmp_2 += degree_distance[tmp_count+ratio];
          }
          else{
            d_2 --;
          }

          if(degree_distance[tmp_count+ratio*2] >= 0){
            tmp_3 += degree_distance[tmp_count+ratio*2];
          }
          else{
            d_3 --;
          }
          
          if(degree_distance[tmp_count+ratio*3] >= 0){
            tmp_4 += degree_distance[tmp_count+ratio*3];
          }
          else{
            d_4 --;
          }
          
          if(degree_distance[tmp_count+ratio*4] >= 0){
            tmp_5 += degree_distance[tmp_count+ratio*4];
          }
          else{
            d_5 --;
          }

          if(degree_distance[tmp_count+ratio*5] >= 0){
            tmp_6 += degree_distance[tmp_count+ratio*5];
          }
          else{
            d_6 --;
          }

          if(degree_distance[tmp_count+ratio*6] >= 0){
            tmp_7 += degree_distance[tmp_count+ratio*6];
          }
          else{
            d_7 --;
          }

          if(degree_distance[tmp_count+ratio*7] >= 0){
            tmp_8 += degree_distance[tmp_count+ratio*7];
          }
          else{
            d_8 --;
          }

          if(degree_distance[tmp_count+ratio*8] >= 0){
            tmp_9 += degree_distance[tmp_count+ratio*8];
          }
          else{
            d_9 --;
          }
        }

        tmp_1 = tmp_1/d_1;
        tmp_2 = tmp_2/d_2;
        tmp_3 = tmp_3/d_3;
        tmp_4 = tmp_4/d_4;
        tmp_5 = tmp_5/d_5;
        tmp_6 = tmp_6/d_6;
        tmp_7 = tmp_7/d_7;
        tmp_8 = tmp_8/d_8;
        tmp_9 = tmp_9/d_9;

        if(tmp_1 < 500){
          PWM1 = 100;
        }
        else{
          PWM1 = 0;
        }
        if(tmp_2 < 1000){
          PWM2 = 100;
        }
        else{
          PWM2 = 0;
        }
        if(tmp_3 < 3000){
          PWM3 = 100;
        }
        else{
          PWM3 = 0;
        }
        if(tmp_4 < 4000){
          PWM4 = 100;
        }
        else{
          PWM4 = 0;
        }
        if(tmp_5 < 5000){
          PWM5 = 100;
        }
        else{
          PWM5 = 0;
        }

        if(tmp_6 < 4000){
          PWM6 = 100;
        }
        else{
          PWM6 = 0;
        }

        if(tmp_7 < 3000){
          PWM7 = 100;
        }
        else{
          PWM7 = 0;
        }

        if(tmp_8 < 2000){
          PWM8 = 100;
        }
        else{
          PWM8 = 0;
        }

        if(tmp_9 < 500){
          PWM9 = 100;
        }
        else{
          PWM9 = 0;
        }

        /*PWM5 = degree_distance[45]/60;
        PWM6 = degree_distance[90]/60;
        PWM7 = degree_distance[135]/60;
        PWM8 = degree_distance[180]/60;*/

        //vTaskDelay(30);
        /*
        if(((front_region[0] / valid_distance[0]) / 60) > 20){
          PWM1 = 100;
        }
        if(((front_region[1] / valid_distance[1]) / 60) > 20){
          PWM2 = 100;
        }
        if(((front_region[2] / valid_distance[2]) / 60) > 20){
          PWM3 = 100;
        }
        if(((front_region[3] / valid_distance[3]) / 60) > 20){
          PWM4 = 100;
        }
        if(((front_region[4] / valid_distance[4]) / 60) > 20){
          PWM5 = 100;
        }
        if(((front_region[5] / valid_distance[5]) / 60) > 20){
          PWM6 = 100;
        }
        if(((front_region[6] / valid_distance[6]) / 60) > 20){
          PWM7 = 100;
        }
        if(((front_region[7] / valid_distance[7]) / 60) > 20){
          PWM8 = 100;
        }
        if(((front_region[8] / valid_distance[8]) / 60) > 20){
          PWM9 = 100;
        }*/
        /*PWM2 = ((front_region[1] / valid_distance[1]) / 60);
        PWM3 = ((front_region[2] / valid_distance[2]) / 60);
        PWM4 = ((front_region[3] / valid_distance[3]) / 60);
        PWM5 = ((front_region[4] / valid_distance[4]) / 60);
        PWM6 = ((front_region[5] / valid_distance[5]) / 60);
        PWM7 = ((front_region[6] / valid_distance[6]) / 60);
        PWM8 = ((front_region[7] / valid_distance[7]) / 60);
        PWM9 = ((front_region[8] / valid_distance[8]) / 60);*/
  }
}

static void external_interrupt_task(void *pvParameters){
  PWM1 = 0;
  while(1){
    if (over_flag == 1)
    {
      //NVIC_DisableIRQ(EXTI15_10_IRQn);
      PWM1 = 99;
      //NVIC_EnableIRQ(EXTI15_10_IRQn);
      //flag = 0;
    }
    else{
      PWM1 = 0;
    }
    //STM_EVAL_LEDToggle(LED5);
    //STM_EVAL_LEDToggle(LED6);

    vTaskDelay(1000);
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
  //PWM5 = (writed_packet_num/100);  // check for receive data
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
    //return 6000;/*if the data is invalid, return the maximum value.*/
    return -1;
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
    UserButtonPressed = 0x00;
  }else
    UserButtonPressed = 0x01;

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
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  STM_EVAL_LEDToggle(LED5);
  STM_EVAL_LEDToggle(LED6);
  USART_Config();

  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00;

  PWM_config();
  Configure_PB12();
  xTaskCreate(external_interrupt_task,(signed portCHAR *) "Implement External Interrupt",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  //xTaskCreate(usart_receive_task,(signed portCHAR *) "Implement USART",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  //xTaskCreate(Transfer_Distance_task,(signed portCHAR *) "Implement Transfer distance.",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  //xTaskCreate(Front_Obstacle_task,(signed portCHAR *) "Implement front obstacle detect.",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  
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

void PWM_config(void){
  PA2_PWM_config();
  PB14_15_PWM_config();
  PC8_9_PWM_config();
  PE6_PWM_config();
  PB4_5_PWM_config();
  PB9_PWM_config();
  PD12_15_PWM_config();
}

void PA2_PWM_config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOA , ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOA, &GPIO_InitStructure );  
      
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 840.;
  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC3Init( TIM2, &TIM_OCInitStruct );
  TIM_OC4Init( TIM2, &TIM_OCInitStruct );

  TIM_Cmd( TIM2, ENABLE );
}

void PB14_15_PWM_config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOB , ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM12, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOB, &GPIO_InitStructure );  
      
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400;
  TIM_TimeBaseInit( TIM12, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC1Init( TIM12, &TIM_OCInitStruct );
  TIM_OC2Init( TIM12, &TIM_OCInitStruct );

  TIM_Cmd( TIM12, ENABLE );
}

void PB4_5_PWM_config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOB , ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5; //| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOB, &GPIO_InitStructure );  
      
  // Let PWM frequency equal 100Hz.
  // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
  // Solving for prescaler gives 240.
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400;
  TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC1Init( TIM3, &TIM_OCInitStruct );
  TIM_OC2Init( TIM3, &TIM_OCInitStruct );

  TIM_Cmd( TIM3, ENABLE );
}

void PB7_8_PWM_config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOB , ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8; //| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOB, &GPIO_InitStructure );  
      
  // Let PWM frequency equal 100Hz.
  // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
  // Solving for prescaler gives 240.
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400;
  TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC3Init( TIM4, &TIM_OCInitStruct );

  TIM_Cmd( TIM4, ENABLE );
}

void PC8_9_PWM_config(void){
    GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOC , ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9; //| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOC, &GPIO_InitStructure );  
      
  // Let PWM frequency equal 100Hz.
  // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
  // Solving for prescaler gives 240.
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400;
  TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC3Init( TIM3, &TIM_OCInitStruct );
  TIM_OC4Init( TIM3, &TIM_OCInitStruct );

  TIM_Cmd( TIM3, ENABLE );
}

void PB9_PWM_config(void){
    GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOB , ENABLE );
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM11, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; //| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOB, &GPIO_InitStructure );  
      
  // Let PWM frequency equal 100Hz.
  // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
  // Solving for prescaler gives 240.
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;   // 0..2999
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400; // Div 240   500  0.00002 s  =  0.2ms
  TIM_TimeBaseInit( TIM11, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC1Init( TIM11, &TIM_OCInitStruct );

  TIM_Cmd( TIM11, ENABLE );
}

void PD12_15_PWM_config(void){
    GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOD , ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13; //| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOD, &GPIO_InitStructure );  
      
  // Let PWM frequency equal 100Hz.
  // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
  // Solving for prescaler gives 240.
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;   // 0..2999
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400; // Div 240   500  0.00002 s  =  0.2ms
  TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC1Init( TIM4, &TIM_OCInitStruct );
  TIM_OC2Init( TIM4, &TIM_OCInitStruct );

  TIM_Cmd( TIM4, ENABLE );
}

void PE6_PWM_config(void){
    GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  
  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOE , ENABLE );
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM9, ENABLE );
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
    
  // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; //| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOE, &GPIO_InitStructure );  
      
  // Let PWM frequency equal 100Hz.
  // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
  // Solving for prescaler gives 240.
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;   // 0..2999
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400; // Div 240   500  0.00002 s  =  0.2ms
  TIM_TimeBaseInit( TIM9, &TIM_TimeBaseInitStruct );

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC2Init( TIM9, &TIM_OCInitStruct );

  TIM_Cmd( TIM9, ENABLE );
}

void Configure_PB12(void) {
  /* Set variables used */
  GPIO_InitTypeDef GPIO_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  /* Enable clock for GPIOB */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Set pin as input */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /* Tell system that you will use PB12 for EXTI_Line12 */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
  
  /* PB12 is connected to EXTI_Line12 */
  EXTI_InitStruct.EXTI_Line = EXTI_Line12;
  /* Enable interrupt */
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  /* Interrupt mode */
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  /* Triggers on rising and falling edge */
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  /* Add to EXTI */
  EXTI_Init(&EXTI_InitStruct);

  /* Add IRQ vector to NVIC */
  /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
  NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
  /* Set priority */
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
  /* Set sub priority */
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
  /* Enable interrupt */
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  /* Add to NVIC */
  NVIC_Init(&NVIC_InitStruct);
}

#if 1
void EXTI15_10_IRQHandler(void) {
  /* Make sure that interrupt flag is set */
  if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
    /* Do your stuff when PD0 is changed */
    
    oldTime = currentTime;
    currentTime = Timing;

    if((currentTime - oldTime) > 100){
      over_flag = 1;
    }
    
    /* Clear interrupt flag */
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
}
#endif

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

/*  Frequency = 116kHz 
    it can be config in FreeRTOSConfig.h
*/
void vApplicationTickHook()
{
    Timing ++;
    STM_EVAL_LEDToggle(LED6);
}
