/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include <string.h>
#include "queue.h"

#include "dma.h"

#define UART_DATA_BUFFER_SIZE               1024            /// 定义一次性接收最大数据长度
#define UART_DEVICE_BUFFER_SIZE             32

struct usart_device_type{
    USART_TypeDef *uart;
    b_handle bqh_tx;
    b_handle bqh_rx;
    struct {
        struct {
            uint16_t dma : 2;
            uint16_t stream : 3;
            uint16_t channel : 4;
        }tx_dma;
        struct {
            uint16_t dma : 2;
            uint16_t stream : 3;
            uint16_t channel : 4;
        }rx_dma;
    };
    struct {
        uint8_t rx_idle;
        uint8_t rx_tail;
        uint8_t rx_end;
        uint8_t tx_nfree;
        uint8_t rx_buffer[UART_DEVICE_BUFFER_SIZE];
        uint8_t tx_buffer[UART_DEVICE_BUFFER_SIZE];
    };
};


typedef struct usart_device_type usart_device_t;


#define DEF_USART_BUFFER(usart, tr, size)             uint8_t usart##_##tr##_buffer[size] = { 0 }
#define USART_BUFFER_ADDRESS(usart, tr)               (usart##_##tr##_buffer)

#define DEF_USART_DEVICE(usart)                   usart_device_t usart##_device;
#define USART_DEVICE(usart)                       (usart##_device)

#define DEF_USART_QUEUE(usart, tr)                    byte_queue_t q_##usart##_##tr
#define USART_QUEUE(usart, tr)                        (q_##usart##_##tr)

#if defined(USE_USART1)

static DEF_USART_BUFFER(usart1, tx, UART_DATA_BUFFER_SIZE);
static DEF_USART_BUFFER(usart1, rx, UART_DATA_BUFFER_SIZE);
static DEF_USART_QUEUE(usart1, tx);
static DEF_USART_QUEUE(usart1, rx);

static DEF_USART_DEVICE(usart1);


#endif
#if defined(USE_USART2)

#endif
#if defined(USE_USART3)
static DEF_USART_BUFFER(usart3, tx, UART_DATA_BUFFER_SIZE);
static DEF_USART_BUFFER(usart3, rx, UART_DATA_BUFFER_SIZE);
static DEF_USART_QUEUE(usart3, tx);
static DEF_USART_QUEUE(usart3, rx);

static DEF_USART_DEVICE(usart3);

#endif

#if defined(USE_UART4)
static DEF_USART_BUFFER(uart4, tx, UART_DATA_BUFFER_SIZE);
static DEF_USART_BUFFER(uart4, rx, UART_DATA_BUFFER_SIZE);
static DEF_USART_QUEUE(uart4, tx);
static DEF_USART_QUEUE(uart4, rx);

static DEF_USART_DEVICE(uart4);
#endif
#if defined(USE_UART5)

#endif

/* USER CODE END 0 */

static usart_device_t* hdevice[6] = {
#if defined USE_USART1
        &USART_DEVICE(usart1),
#else
        NULL,
#endif
#if defined USE_USART2
        &USART_DEVICE(usart2),
#else
        NULL,
#endif
#if defined USE_USART3
        &USART_DEVICE(usart3),
#else
        NULL,
#endif
#if defined USE_UART4
        &USART_DEVICE(uart4),
#else
        NULL,
#endif
#if defined USE_UART5
        &USART_DEVICE(uart5),
#else
        NULL,
#endif
#if defined USE_USART6
        &USART_DEVICE(usart6),
#else
        NULL,
#endif
};


/* USER CODE BEGIN 1 */
static void uart_dma_config_(USART_TypeDef *USARTx,
                             DMA_TypeDef *DMAx, uint32_t Stream, LL_DMA_InitTypeDef *DMA_InitStruct){

    if(DMAx == DMA1){
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }else if(DMAx == DMA2){
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    }else{
        return;
    }
    LL_DMA_DisableStream(DMAx, Stream);
    LL_DMA_EnableIT_HT(DMAx, Stream);
    LL_DMA_EnableIT_TC(DMAx, Stream);
    LL_DMA_Init(DMAx, Stream, DMA_InitStruct);
    LL_DMA_DisableIT_TE(DMAx, Stream);
}

static void uart_rx_dma_config_(USART_TypeDef *USARTx,
                                DMA_TypeDef *DMAx, uint32_t Stream, LL_DMA_InitTypeDef *DMA_InitStruct){

    LL_USART_Enable(USARTx);

    LL_DMA_DisableStream(DMAx, Stream);
    LL_DMA_Init(DMAx, Stream, DMA_InitStruct);
    LL_DMA_DisableIT_TE(DMAx, Stream);
    LL_DMA_EnableIT_HT(DMAx, Stream);
    LL_DMA_EnableIT_TC(DMAx, Stream);
    LL_DMA_EnableStream(DMAx, Stream);

    LL_USART_EnableDMAReq_RX(USARTx);
    LL_USART_EnableIT_IDLE(USARTx);
    LL_USART_EnableIT_ERROR(USARTx);
}

static void uart_tx_dma_config_(USART_TypeDef *USARTx,
                                DMA_TypeDef *DMAx, uint32_t Stream, LL_DMA_InitTypeDef *DMA_InitStruct){
    LL_USART_Enable(USARTx);

    LL_DMA_DisableStream(DMAx, Stream);
    LL_DMA_Init(DMAx, Stream, DMA_InitStruct);
    LL_DMA_DisableIT_TE(DMAx, Stream);
    LL_DMA_DisableIT_HT(DMAx, Stream);
    LL_DMA_EnableIT_TC(DMAx, Stream);
    LL_DMA_EnableStream(DMAx, Stream);

    LL_USART_EnableDMAReq_TX(USARTx);
    LL_USART_EnableIT_ERROR(USARTx);
}

static void usart_rx_dma_circle_post_(usart_device_t *device, DMA_TypeDef *DMAx, uint32_t Stream){

//    CRIT_SEC_E_;
    const uint8_t rx_head = LL_DMA_GetDataLength(DMAx, Stream);
    const uint8_t rx_end = device->rx_end;
    const uint8_t rx_tail = device->rx_tail;

//    device->rx_tail = rx_head;

    /**
     * @brief 当TC置位时, DMA Data Length 会重新加载
     */

    /// 索引递减类型
    /// new head
//    device->rx_head = rx_head;
    /// post data to user's buffer queue
    if((rx_head == rx_tail)){
        if((rx_head == (rx_end >> 1))){
            /// only enable HT

        }else if((rx_head == rx_end)){
            /// only enalbe TC
        }else{
            goto end_section;
        }
    }else{
        /// enable TC, HT
        uint8_t available = 0;
        if((rx_head > rx_tail)){
            /// 比前一次大, 说明循环从头再来
            available = rx_tail + (rx_end - rx_head);
            bq_post(&device->bqh_rx, &device->rx_buffer[rx_end - rx_tail], rx_tail);
            bq_post(&device->bqh_rx, &device->rx_buffer[0], rx_end - rx_head);
        }else{
            available = rx_tail - rx_head;
            bq_post(&device->bqh_rx, &device->rx_buffer[rx_end - rx_tail], available);
        }
    }

    /// new tail
    device->rx_tail = rx_head;
    end_section:
    return;
//    CRIT_SEC_X_;
}

static void usart_tx_circle_get_(usart_device_t *device, DMA_TypeDef *DMAx, uint32_t Stream){
    (void)DMAx;
    (void)Stream;

    const uint16_t available = bq_length(&device->bqh_tx);
//    CRIT_SEC_E_;
    const uint16_t nsend = available && (available > sizeof(device->tx_buffer)) ? sizeof(device->tx_buffer) : available;
    device->tx_nfree = sizeof(device->tx_buffer) - nsend;
    if(nsend){
        bq_get(&device->bqh_tx, device->tx_buffer, nsend);
    }
    /**
     * @brief nsend = 0, 没有数据要发送, 但是此时最后一帧数据没有发送完成, 用户应该等待 tx_nfree == UINT8_MAX
     */
//    CRIT_SEC_X_;
}

static void usart_isr_handler_(void *ctx){
    usart_device_t *dev = (usart_device_t *)ctx;

    volatile uint32_t sr = dev->uart->SR;
    /// 空闲
    if((sr & USART_SR_IDLE)){
        LL_USART_ClearFlag_IDLE(dev->uart);
        usart_rx_dma_circle_post_(dev, DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream]);
    }
    /// 发送完成
    if(sr & USART_SR_TC){
        LL_USART_ClearFlag_TC(dev->uart);
        LL_USART_DisableIT_TC(dev->uart);

//        usart_tx_circle_get_(dev, DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);

        if(dev->tx_nfree != sizeof(dev->tx_buffer)){
            /// 继续发送
            LL_DMA_InitTypeDef dmaInitTypeDef = {
                    .Channel = DMA_CHANNEL[dev->tx_dma.channel],
                    .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
                    .Mode = LL_DMA_MODE_NORMAL,
                    .Priority = LL_DMA_PRIORITY_HIGH,
                    .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
                    .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
                    .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
                    .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
                    .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
            };
            dmaInitTypeDef.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(dev->uart);
            dmaInitTypeDef.MemoryOrM2MDstAddress = (uint32_t)(dev->tx_buffer);
            dmaInitTypeDef.NbData = sizeof(dev->tx_buffer) - dev->tx_nfree;
            if(dmaInitTypeDef.NbData){
                uart_tx_dma_config_(dev->uart,
                                    DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream], &dmaInitTypeDef);
            }
        }else{
            /// 所有数据发送完成
            dev->tx_nfree = UINT8_MAX;
        }
    }

    if(sr & USART_SR_ORE){    ///
        LL_USART_ClearFlag_ORE(dev->uart);
    }
}

static void usart1_isr_handler_(void *ctx){
#if defined(USE_USART1)
    usart_device_t *dev = (usart_device_t *)ctx;

    volatile uint32_t sr = dev->uart->SR;
    /// 空闲
    if((sr & USART_SR_IDLE)){
        LL_USART_ClearFlag_IDLE(dev->uart);
        usart_rx_dma_circle_post_(dev, DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream]);
    }
    /// 发送完成
    if(sr & USART_SR_TC){
        LL_USART_ClearFlag_TC(dev->uart);
        LL_USART_DisableIT_TC(dev->uart);

//        usart_tx_circle_get_(dev, DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);

        if(dev->tx_nfree != sizeof(dev->tx_buffer)){
            /// 继续发送
            LL_DMA_InitTypeDef dmaInitTypeDef = {
                    .Channel = DMA_CHANNEL[dev->tx_dma.channel],
                    .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
                    .Mode = LL_DMA_MODE_NORMAL,
                    .Priority = LL_DMA_PRIORITY_HIGH,
                    .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
                    .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
                    .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
                    .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
                    .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
            };
            dmaInitTypeDef.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(dev->uart);
            dmaInitTypeDef.MemoryOrM2MDstAddress = (uint32_t)(dev->tx_buffer);
            dmaInitTypeDef.NbData = sizeof(dev->tx_buffer) - dev->tx_nfree;
            if(dmaInitTypeDef.NbData){
                uart_tx_dma_config_(dev->uart,
                                    DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream], &dmaInitTypeDef);
            }
        }else{
            /// 所有数据发送完成
            dev->tx_nfree = UINT8_MAX;
        }
    }

    if(sr & USART_SR_ORE){    ///
        LL_USART_ClearFlag_ORE(dev->uart);
    }
#endif
}

static void usart2_isr_handler_(void *ctx) {
#if defined(USE_USART2)

#endif
}

static void usart3_isr_handler_(void *ctx) {
#if defined(USE_USART3)
#if(0)

    uart_device_t *dev = (uart_device_t *)ctx;

    register uint32_t sr = USART3->SR;
    if((sr & USART_SR_IDLE)){
        uint32_t pos = 0, length = 0;
        LL_USART_ClearFlag_IDLE(USART3);

        pos = LL_DMA_GetDataLength(USART3_RX_DMA, USART3_RX_DMA_STREAM);

        dev->rx_ring_buffer.tail = sizeof(dev->rx_ring_buffer.data) - pos;
        length = dev->rx_ring_buffer.tail - dev->rx_ring_buffer.head;
        if(dev->rx_ring_buffer.tail > dev->rx_ring_buffer.head){   /// 队尾大于队首
            LoopBuffer_Write(UART_USER_RX_BUFFER(uart3), &dev->rx_ring_buffer.data[dev->rx_ring_buffer.head], length);
            dev->rx_ring_buffer.length = length;
        }else
        if(dev->rx_ring_buffer.tail <= dev->rx_ring_buffer.head){   /// 队尾小于队头
            length = sizeof(dev->rx_ring_buffer.data) - dev->rx_ring_buffer.head;
            LoopBuffer_Write(UART_USER_RX_BUFFER(uart3), &dev->rx_ring_buffer.data[dev->rx_ring_buffer.head], length);
            length = dev->rx_ring_buffer.tail;
            LoopBuffer_Write(UART_USER_RX_BUFFER(uart3), &dev->rx_ring_buffer.data[0], length);
            dev->rx_ring_buffer.length = sizeof(dev->rx_ring_buffer.data) + dev->rx_ring_buffer.tail - dev->rx_ring_buffer.head;
        }
        dev->rx_ring_buffer.head = dev->rx_ring_buffer.tail;
        return ;
    }

    if(sr & USART_SR_ORE){    ///
        LL_USART_ClearFlag_ORE(USART3);
    }
#endif
#endif
}

static void uart_tx_dma_isr_handler_(void *ctx){

    usart_device_t *dev = (usart_device_t *)ctx;

    if(LL_DMA_IsEnabledIT_TC(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream])
       && ll_dma_get_sr_state(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream], DMA_LISR_TCIF0)){

        ll_dma_clear_sr_state(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream], DMA_LISR_TCIF0);
        LL_DMA_DisableStream(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);
        LL_DMA_DisableIT_TC(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);

        LL_USART_DisableDMAReq_TX(dev->uart);

        usart_tx_circle_get_(dev, DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);

        LL_USART_EnableIT_TC(dev->uart);
    }
}


static void uart1_tx_dma_isr_handler_(void *ctx){

    usart_device_t *dev = (usart_device_t *)ctx;

    if(LL_DMA_IsEnabledIT_TC(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream])
       && ll_dma_get_sr_state(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream], DMA_LISR_TCIF0)){

        ll_dma_clear_sr_state(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream], DMA_LISR_TCIF0);
        LL_DMA_DisableStream(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);
        LL_DMA_DisableIT_TC(DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);

        LL_USART_DisableDMAReq_TX(dev->uart);

        usart_tx_circle_get_(dev, DMA[dev->tx_dma.dma], DMA_STREAM[dev->tx_dma.stream]);

        LL_USART_EnableIT_TC(dev->uart);
    }
}

static void uart_rx_dma_isr_handler_(void *ctx){
    usart_device_t *dev = (usart_device_t *)ctx;
    if(LL_DMA_IsEnabledIT_HT(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream])
       && ll_dma_get_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_HTIF0)){
        ll_dma_clear_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_HTIF0);
        usart_rx_dma_circle_post_(dev, DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream]);
    }

    if(LL_DMA_IsEnabledIT_TC(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream])
       && ll_dma_get_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_TCIF0)){
        ll_dma_clear_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_TCIF0);
        usart_rx_dma_circle_post_(dev, DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream]);
    }
}

static void uart1_rx_dma_isr_handler_(void *ctx){
    usart_device_t *dev = (usart_device_t *)ctx;
    if(LL_DMA_IsEnabledIT_HT(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream])
       && ll_dma_get_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_HTIF0)){
        ll_dma_clear_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_HTIF0);
        usart_rx_dma_circle_post_(dev, DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream]);
    }

    if(LL_DMA_IsEnabledIT_TC(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream])
       && ll_dma_get_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_TCIF0)){
        ll_dma_clear_sr_state(DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream], DMA_LISR_TCIF0);
        usart_rx_dma_circle_post_(dev, DMA[dev->rx_dma.dma], DMA_STREAM[dev->rx_dma.stream]);
    }
}

static void uart3_dma_isr_handler_(void *ctx){

}


int uart_initialize(const uint8_t index, const uart_initialize_t *initialize) {
     uint32_t gpio[6][4] = {
            {/* TX */ (uint32_t)GPIOB, LL_GPIO_PIN_6, /* RX */ (uint32_t)GPIOB, LL_GPIO_PIN_7 },
            {/* TX */ UINT32_MAX, UINT32_MAX, /* RX */ UINT32_MAX, UINT32_MAX },
            {/* TX */ (uint32_t)GPIOB, LL_GPIO_PIN_10, /* RX */ (uint32_t)GPIOB, LL_GPIO_PIN_11 },
            {/* TX */ (uint32_t)GPIOC, LL_GPIO_PIN_10, /* RX */ (uint32_t)GPIOC, LL_GPIO_PIN_11 },
            {/* TX */ (uint32_t)GPIOC, LL_GPIO_PIN_12, /* RX */ (uint32_t)GPIOD, LL_GPIO_PIN_2 },
            {/* TX */ UINT32_MAX, UINT32_MAX, /* RX */ UINT32_MAX, UINT32_MAX },
    };

    uint32_t af[6][2] = {
            {LL_GPIO_AF_7, LL_GPIO_AF_7},
            {UINT32_MAX, UINT32_MAX},
            {LL_GPIO_AF_7, LL_GPIO_AF_7},
            {LL_GPIO_AF_8, LL_GPIO_AF_8},
            {LL_GPIO_AF_8, LL_GPIO_AF_8},
            {UINT32_MAX, UINT32_MAX},
    };

    uint8_t irqn[6] = {
            USART1_IRQn,
            USART2_IRQn,
            USART3_IRQn,
            UART4_IRQn,
            UART5_IRQn,
            USART6_IRQn,
    };

//    USART_TypeDef *uart[6] = {
//            USART1,
//            USART2,
//            USART3,
//            UART4,
//            UART5,
//            USART6
//    };

//    void (*handler[6])(void *)  = {
//            &usart_isr_handler_,
//            &usart_isr_handler_,
//            &usart_isr_handler_,
//            NULL,
//            NULL,
//            NULL,
//    };

    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    const uint8_t id = index - 1;

    USART_InitStruct.BaudRate = initialize->baud;
    USART_InitStruct.DataWidth = (initialize->data_bit == 9) ? LL_USART_DATAWIDTH_9B : LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = (initialize->stop_bit == 1) ? LL_USART_STOPBITS_1 : LL_USART_STOPBITS_2;
    USART_InitStruct.Parity = (initialize->parity == 0) ? LL_USART_PARITY_NONE : ((initialize->parity == 1) ? LL_USART_PARITY_EVEN : LL_USART_PARITY_ODD);
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;

    if(initialize->hwcontrol.value == 0){
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    }else if(initialize->hwcontrol.cts && initialize->hwcontrol.rts){
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_RTS_CTS;
    }else if(initialize->hwcontrol.cts){
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_CTS;
    }else{
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_RTS;
    }
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

#if defined(USE_USART1)
    if(index == 1){
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

        USART_DEVICE(usart1).bqh_rx = bq_initialize(&USART_QUEUE(usart1, rx),
                                                    USART_BUFFER_ADDRESS(usart1, rx), UART_DATA_BUFFER_SIZE);
        USART_DEVICE(usart1).bqh_tx = bq_initialize(&USART_QUEUE(usart1, tx),
                                                    USART_BUFFER_ADDRESS(usart1, tx), UART_DATA_BUFFER_SIZE);

        USART_DEVICE(usart1).rx_end = sizeof(USART_DEVICE(usart1).rx_buffer);
        USART_DEVICE(usart1).rx_tail = USART_DEVICE(usart1).rx_end;
//        USART_DEVICE(usart1).rx_head = USART_DEVICE(usart1).rx_end;
        USART_DEVICE(usart1).tx_nfree = UINT8_MAX;

        USART_DEVICE(usart1).uart = USART1;
#ifdef USART1_TX_USE_DMA
        USART_DEVICE(usart1).tx_dma.dma = 2 - 1;
        USART_DEVICE(usart1).tx_dma.stream = (uint8_t)7;
        USART_DEVICE(usart1).tx_dma.channel = (uint8_t)4;
#endif
#ifdef USART1_RX_USE_DMA
        USART_DEVICE(usart1).rx_dma.dma = 2 - 1;
        USART_DEVICE(usart1).rx_dma.stream = (uint8_t)2;
        USART_DEVICE(usart1).rx_dma.channel = (uint8_t)4;
#endif
    }
#endif
#if defined (USE_USART2)

#endif
#if defined(USE_USART3)
    if(index == 3){
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

        USART_DEVICE(usart3).bqh_rx = bq_initialize(&USART_QUEUE(usart3, rx),
                                                    USART_BUFFER_ADDRESS(usart3, rx), UART_DATA_BUFFER_SIZE);
        USART_DEVICE(usart3).bqh_tx = bq_initialize(&USART_QUEUE(usart3, tx),
                                                    USART_BUFFER_ADDRESS(usart3, tx), UART_DATA_BUFFER_SIZE);

        USART_DEVICE(usart3).rx_end = sizeof(USART_DEVICE(usart3).rx_buffer);
        USART_DEVICE(usart3).rx_tail = USART_DEVICE(usart3).rx_end;
//        USART_DEVICE(usart3).rx_head = USART_DEVICE(usart3).rx_end;
        USART_DEVICE(usart3).tx_nfree = UINT8_MAX;

        USART_DEVICE(usart3).uart = USART3;
#ifdef USART3_TX_USE_DMA
        USART_DEVICE(usart3).tx_dma.dma = 1 - 1;
        USART_DEVICE(usart3).tx_dma.stream = (uint8_t)3;
        USART_DEVICE(usart3).tx_dma.channel = (uint8_t)4;
#endif
#ifdef USART3_RX_USE_DMA
        USART_DEVICE(usart3).rx_dma.dma = 1 - 1;
        USART_DEVICE(usart3).rx_dma.stream = (uint8_t)1;
        USART_DEVICE(usart3).rx_dma.channel = (uint8_t)4;
#endif
    }
#endif
#if defined(USE_UART4)
    if(index == 4){
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

        USART_DEVICE(uart4).bqh_rx = bq_initialize(&USART_QUEUE(uart4, rx),
                                                    USART_BUFFER_ADDRESS(uart4, rx), UART_DATA_BUFFER_SIZE);
        USART_DEVICE(uart4).bqh_tx = bq_initialize(&USART_QUEUE(uart4, tx),
                                                    USART_BUFFER_ADDRESS(uart4, tx), UART_DATA_BUFFER_SIZE);

        USART_DEVICE(uart4).rx_end = sizeof(USART_DEVICE(uart4).rx_buffer);
        USART_DEVICE(uart4).rx_tail = USART_DEVICE(uart4).rx_end;
//        USART_DEVICE(uart4).rx_head = USART_DEVICE(uart4).rx_end;
        USART_DEVICE(uart4).tx_nfree = UINT8_MAX;

        USART_DEVICE(uart4).uart = UART4;
#ifdef USART3_TX_USE_DMA
        USART_DEVICE(uart4).tx_dma.dma = 1 - 1;
        USART_DEVICE(uart4).tx_dma.stream = (uint8_t)4;
        USART_DEVICE(uart4).tx_dma.channel = (uint8_t)4;
#endif
#ifdef USART3_RX_USE_DMA
        USART_DEVICE(uart4).rx_dma.dma = 1 - 1;
        USART_DEVICE(uart4).rx_dma.stream = (uint8_t)2;
        USART_DEVICE(uart4).rx_dma.channel = (uint8_t)4;
#endif
    }
#endif
    for(int i = 0; i < (sizeof(hdevice) / sizeof(usart_device_t *)); i++){
        if(i != id) continue;
        if(hdevice[i] == NULL) break;

        LL_USART_DeInit(hdevice[id]->uart);
        LL_USART_Init(hdevice[id]->uart, &USART_InitStruct);

        LL_USART_ConfigAsyncMode(hdevice[id]->uart);

        ll_peripheral_isr_install((int)irqn[i], &usart_isr_handler_, (void *)hdevice[id]);
        NVIC_SetPriority(irqn[i], NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));

        NVIC_EnableIRQ(irqn[i]);

        /// TX
        GPIO_InitStruct.Pin = gpio[i][1];;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = af[i][0];
        LL_GPIO_Init((GPIO_TypeDef *)gpio[i][0], &GPIO_InitStruct);
        /// RX
        GPIO_InitStruct.Pin = gpio[i][3];;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = af[i][1];
        LL_GPIO_Init((GPIO_TypeDef *)gpio[i][2], &GPIO_InitStruct);

        break;
    }

    return 0;
}

int uart_deinitialize(const uint8_t index){
    (void)index;
    return 0;
}


void uart_start_receive_async(const uint8_t index){
    LL_DMA_InitTypeDef dmaInitTypeDef = {
            .Channel = LL_DMA_CHANNEL_4,
            .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
            .Mode = LL_DMA_MODE_CIRCULAR,
            .Priority = LL_DMA_PRIORITY_HIGH,
            .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
            .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
            .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
    };
    uint8_t dma_irq[sizeof(DMA) / sizeof(DMA_TypeDef *)][sizeof(DMA_STREAM) / sizeof(uint32_t)] = {
            {DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
             DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn },
            {DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
             DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn },
    };
    const int id = index - 1;
    for(int i = 0; i < (sizeof(hdevice) / sizeof(usart_device_t *)); i++){
        if(i != id) continue;
        if(hdevice[i] == NULL) break;
        dmaInitTypeDef.Channel = DMA_CHANNEL[hdevice[id]->rx_dma.channel];
        dmaInitTypeDef.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(hdevice[id]->uart);
        dmaInitTypeDef.MemoryOrM2MDstAddress = (uint32_t)(hdevice[id]->rx_buffer);
        dmaInitTypeDef.NbData = sizeof(hdevice[id]->rx_buffer);

        ll_peripheral_isr_install(dma_irq[hdevice[id]->rx_dma.dma][hdevice[id]->rx_dma.stream],
                                  &uart_rx_dma_isr_handler_, hdevice[id]);

        uart_rx_dma_config_(hdevice[id]->uart,
                            DMA[hdevice[id]->rx_dma.dma], DMA_STREAM[hdevice[id]->rx_dma.stream],
                            &dmaInitTypeDef);

        NVIC_SetPriority(dma_irq[hdevice[id]->rx_dma.dma][hdevice[id]->rx_dma.stream],
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));

        NVIC_EnableIRQ(dma_irq[hdevice[id]->rx_dma.dma][hdevice[id]->rx_dma.stream]);
        break;
    }
}

void uart_stop_receive_async(const uint8_t index){
    const int id = index - 1;
    for(int i = 0; i < (sizeof(hdevice) / sizeof(usart_device_t *)); i++){
        if(i != id) continue;
        if(hdevice[i] == NULL) break;
        LL_USART_Disable(hdevice[id]->uart);
        LL_USART_DisableIT_IDLE(hdevice[id]->uart);
        LL_USART_DisableDMAReq_RX(hdevice[id]->uart);
        LL_DMA_DisableStream(DMA[hdevice[id]->rx_dma.dma], DMA_STREAM[hdevice[id]->rx_dma.stream]);
        LL_USART_ReceiveData8(hdevice[id]->uart);
        break;
    }
}


uint16_t uart_write_bytes(const uint8_t index, void *data, uint16_t length, uint32_t timeout_ms){
    LL_DMA_InitTypeDef dmaInitTypeDef = {
            .Channel = LL_DMA_CHANNEL_4,
            .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
            .Mode = LL_DMA_MODE_NORMAL,
            .Priority = LL_DMA_PRIORITY_HIGH,
            .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
            .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
            .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
    };
    uint16_t offset = 0;
    uint8_t dma_irq[sizeof(DMA) / sizeof(DMA_TypeDef *)][sizeof(DMA_STREAM) / sizeof(uint32_t)] = {
            {DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
                    DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn },
            {DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
                    DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn },
    };

    const uint8_t id = index - 1;

    for(int i = 0; i < (sizeof(hdevice) / sizeof(usart_device_t *)); i++){
        if(i != id) continue;
        if(hdevice[i] == NULL) break;
        CRIT_SEC_E_;
        const uint8_t tx_nfree = hdevice[id]->tx_nfree;
        const uint16_t nsend = length > sizeof(hdevice[id]->tx_buffer) ? sizeof(hdevice[id]->tx_buffer) : length;

        dmaInitTypeDef.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(hdevice[id]->uart);
        dmaInitTypeDef.MemoryOrM2MDstAddress = (uint32_t)(hdevice[id]->tx_buffer);
        dmaInitTypeDef.Channel = DMA_CHANNEL[hdevice[id]->tx_dma.channel];

        if(tx_nfree == UINT8_MAX){
            /// 串口空闲
            dmaInitTypeDef.NbData = nsend;
            hdevice[id]->tx_nfree = sizeof(hdevice[id]->tx_buffer) - nsend;
            for(offset = 0; offset < nsend; offset++){
                hdevice[id]->tx_buffer[offset] = ((uint8_t *)data)[offset];
            }
        }else if(tx_nfree == sizeof(hdevice[id]->tx_buffer)){
            /// 发送缓冲区空, 但最后一帧未发送完成
            dmaInitTypeDef.NbData = nsend;
            hdevice[id]->tx_nfree = sizeof(hdevice[id]->tx_buffer) - nsend;
            for(offset = 0; offset < nsend; offset++){
                hdevice[id]->tx_buffer[offset] = ((uint8_t *)data)[offset];
            }
        }
        CRIT_SEC_X_;
        /// 开始发送
        if(tx_nfree == UINT8_MAX){
            ll_peripheral_isr_install(dma_irq[hdevice[id]->tx_dma.dma][hdevice[id]->tx_dma.stream],
                                      &uart_tx_dma_isr_handler_, hdevice[id]);
            NVIC_SetPriority(dma_irq[hdevice[id]->tx_dma.dma][hdevice[id]->tx_dma.stream],
                             NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
            NVIC_EnableIRQ(dma_irq[hdevice[id]->tx_dma.dma][hdevice[id]->tx_dma.stream]);
            uart_tx_dma_config_(hdevice[id]->uart,
                                DMA[hdevice[id]->tx_dma.dma], DMA_STREAM[hdevice[id]->tx_dma.stream],
                                &dmaInitTypeDef);
        }
        /// 将数据填入循环缓冲区
        uint16_t remain = length - offset;

        const uint16_t tx_end = bq_full_size(&hdevice[id]->bqh_tx);

        while(remain > 0){
            uint16_t npost = 0;
            npost = bq_get_free(&hdevice[id]->bqh_tx);
            if(timeout_ms && (npost == 0)){
                HAL_Delay(1);
                --timeout_ms;
                if(timeout_ms == 0) break;
                continue;
            }
//            if(!((npost >= (remain >> 1)) || (npost >= (tx_end >> 1)))) continue;

            npost = npost > remain ? remain : npost;
            npost = bq_post(&hdevice[id]->bqh_tx, &data[offset], npost);
            offset += npost;
            remain -= npost;
        }
        break;
    }

    CRIT_SEC_X_;
    return offset;
}


uint16_t uart_available_bytes(const uint8_t index){
    const uint8_t id = index - 1;
    uint16_t al = 0;
    for(int i = 0; i < (sizeof(hdevice) / sizeof(usart_device_t *)); i++){
        if(i != id) continue;
        if(hdevice[i] == NULL) break;
        al = bq_length(&hdevice[id]->bqh_rx);
        break;
    }
    return al;
}

uint16_t uart_read_bytes(const uint8_t index, void *data, uint16_t length, uint32_t timeout_ms){
    uint16_t rl = 0;
    uint16_t remain = length;
    const uint8_t id = index - 1;
    for(int i = 0; i < (sizeof(hdevice) / sizeof(usart_device_t *)); i++){
        if(i != id) continue;
        if(hdevice[i] == NULL) break;
        do{
            read_section:
            rl = bq_get(&hdevice[id]->bqh_rx, &data[length - remain], remain);
            remain -= rl;
            if(remain == 0) break;
            if((rl == 0) && timeout_ms){
                HAL_Delay(1);
                --timeout_ms;
                if((timeout_ms == 0) && bq_length(&hdevice[id]->bqh_rx)){
                    goto read_section;
                }
            }
        }while(timeout_ms);
        break;
    }
    return length - remain;
}

#include <stdio.h>
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    uint32_t timeout = 1000;
    while(LL_USART_IsEnabledDMAReq_TX(USART1) && timeout){
        HAL_Delay(1);
        timeout--;
    }
    while((!LL_USART_IsActiveFlag_TXE(USART1)) && timeout){
        HAL_Delay(1);
        timeout--;
    }
    if(timeout){
        LL_USART_ClearFlag_TC(USART1);
        LL_USART_TransmitData8(USART1, ch);
        while(!LL_USART_IsActiveFlag_TC(USART1)){

        }
        LL_USART_ClearFlag_TC(USART1);
    }

    return ch;
}

#include "printf/printf.h"
void putchar_(char ch){
    uint8_t d = ch;
    uart_write_bytes(1, &d, 1, 0);
}


uint8_t uart_printf(uint8_t index, const char *format, ...){

    return 0;
}



/* USER CODE END 1 */
