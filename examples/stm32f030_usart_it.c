/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "bqueue.h"

static byte_queue_t usart_tx_pool[1];
static byte_queue_t usart_rx_pool[1];

b_handle bh_usart_tx = 0UL;
b_handle bh_usart_rx = 0UL;

/* USER CODE END 0 */



/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  static uint8_t tx_pool[64];
  static uint8_t rx_pool[64];

  bh_usart_tx = bq_initialize(&usart_tx_pool[0], tx_pool, sizeof(tx_pool));
  bh_usart_rx = bq_initialize(&usart_rx_pool[0], rx_pool, sizeof(rx_pool));

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA2   ------> USART1_TX
  PA3   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  { 
  }

  LL_USART_EnableIT_ERROR(USART1);
  LL_USART_EnableIT_IDLE(USART1);
  LL_USART_EnableIT_RXNE(USART1);
//   LL_USART_EnableIT_TXE(USART1);
//   LL_USART_EnableIT_TC(USART1);

  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    if(LL_USART_IsEnabledIT_RXNE(USART1) && LL_USART_IsActiveFlag_RXNE(USART1)){
        __IO uint8_t rc = 0;
        rc = LL_USART_ReceiveData8(USART1);
        bq_post(bh_usart_rx, (uint8_t *)&rc, 1);
    }
    if(LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1)){
        uint8_t tc = 0;
        uint16_t tl = 0;
        tl = bq_get(bh_usart_tx, &tc, 1);
        if(tl == 0){
            LL_USART_DisableIT_TXE(USART1);
            // LL_USART_EnableIT_TC(USART1);
        }else{
            LL_USART_TransmitData8(USART1, (uint8_t)(tc));
        }
    }
    if(LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1)){
        LL_USART_ClearFlag_TC(USART1);
        LL_USART_DisableIT_TC(USART1);
    }
    if(LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsEnabledIT_ERROR(USART1)){
        LL_USART_ClearFlag_FE(USART1);
        // LL_USART_DeInit(USART1);
        // MX_USART1_UART_Init();
    }
    if(LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1)){
        LL_USART_ClearFlag_IDLE(USART1);
    }
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

uint16_t usart_write(uint8_t * const data, const uint16_t length, const uint32_t timeout){
    (void)timeout;
    uint16_t rn = length;
    uint16_t bql = 0, bqf = 0;

    bql = bq_length(bh_usart_tx);
    // 待发送队列空
    if(bql == 0){
        LL_USART_TransmitData8(USART1, data[length - rn]);
        LL_USART_EnableIT_TXE(USART1);
        rn -= 1;
    }
    if(rn == 0) return length;
    do{
        // int32_t st = SysTick->VAL;
        bqf = bq_get_free(bh_usart_tx);
        if(bqf == 0){
            // int32_t se = st - (SysTick->LOAD >> 3);
            // if(se < 0){
            //     se = se + SysTick->LOAD;
            // }
            // while(SysTick->VAL != se){

            // }
            
            LL_USART_EnableIT_TXE(USART1);
            continue;
        }
        if(bqf > rn){
            bq_post(bh_usart_tx, &data[length - rn], rn);
            rn = 0;
        }else{
            bq_post(bh_usart_tx, &data[length - rn], bqf);
            rn -= bqf;
        }
    }while(rn);
    return (length - rn);
}

uint16_t usart_read(uint8_t * const data, const uint16_t length, const uint32_t timeout){
    (void)timeout;
    uint16_t rn = length;
    uint16_t bql = 0;
    do{
        bql = bq_length(bh_usart_rx);
        if(bql > rn){
            bq_get(bh_usart_rx, &data[length - rn], rn);
            rn = 0;
        }else{
            bq_get(bh_usart_rx, &data[length - rn], bql);
            rn -= bql;
        }
    }while(rn);

    return 0;
}

const uint16_t usart_available(void){
    return bq_length(&bh_usart_rx);
}




/* USER CODE END 1 */
