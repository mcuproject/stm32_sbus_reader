#include "sbus.h"

static void process_sbus(void);

GPIO_InitTypeDef  GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
NVIC_InitTypeDef NVIC_InitStructure;
DMA_InitTypeDef DMA_InitStruct;

/* Receive buffer for DMA */
#define DMA_RX_BUFFER_SIZE          30
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
uint32_t idle_cnt;

/* extern variables from main.c */
extern uint16_t channels[16];
extern uint8_t lost_frame, failsafe;

void uart_sbus_init(void) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
	
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); /* UART Tx pins */ 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); /* UART Rx pins */
	
    //USART configuration
    USART_InitStruct.USART_BaudRate = 100000;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_Even;
    USART_InitStruct.USART_StopBits = USART_StopBits_2;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART6, &USART_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
		USART_Cmd(USART6, ENABLE);
		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	
	  /* Configure DMA for USART RX, DMA1, Stream1, Channel4 */
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_Channel = DMA_Channel_5;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMA_RX_Buffer;
    DMA_InitStruct.DMA_BufferSize = DMA_RX_BUFFER_SIZE;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Init(DMA2_Stream1, &DMA_InitStruct);
	  DMA_Cmd(DMA2_Stream1, ENABLE);
}

void USART6_IRQHandler(void) {
	  if (USART6->SR & USART_FLAG_IDLE) {         /* We want IDLE flag only */
    DMA_Cmd(DMA2_Stream1, DISABLE);
    volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
    tmp = USART6->SR;                       /* Read status register */
    tmp = USART6->DR;                       /* Read data register */
		idle_cnt++;
		DMA2_Stream1->M0AR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
    DMA2_Stream1->NDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
		DMA_Cmd(DMA2_Stream1, ENABLE);
	  process_sbus();	  
    }	
}

static void process_sbus(void) {
			channels[0]  = (uint16_t)((DMA_RX_Buffer[1] | DMA_RX_Buffer[2] << 8) & 0x07FF);
			channels[1]  = (uint16_t)((DMA_RX_Buffer[2] >> 3 | DMA_RX_Buffer[3] << 5) & 0x07FF);
			channels[2]  = (uint16_t)((DMA_RX_Buffer[3] >> 6 | DMA_RX_Buffer[4] << 2 | DMA_RX_Buffer[5] << 10) & 0x07FF);
			channels[3]  = (uint16_t)((DMA_RX_Buffer[5] >> 1 | DMA_RX_Buffer[6] << 7) & 0x07FF);
			channels[4]  = (uint16_t)((DMA_RX_Buffer[6] >> 4 | DMA_RX_Buffer[7] << 4) & 0x07FF);
			channels[5]  = (uint16_t)((DMA_RX_Buffer[7] >> 7 | DMA_RX_Buffer[8] << 1 | DMA_RX_Buffer[9] << 9) & 0x07FF);
			channels[6]  = (uint16_t)((DMA_RX_Buffer[9] >> 2 | DMA_RX_Buffer[10] << 6) & 0x07FF);
			channels[7]  = (uint16_t)((DMA_RX_Buffer[10] >> 5 | DMA_RX_Buffer[11]<< 3) & 0x07FF);
			channels[8]  = (uint16_t)((DMA_RX_Buffer[12] | DMA_RX_Buffer[13] << 8) & 0x07FF);
			channels[9]  = (uint16_t)((DMA_RX_Buffer[13] >> 3 | DMA_RX_Buffer[14] << 5) & 0x07FF);
			channels[10] = (uint16_t)((DMA_RX_Buffer[14] >> 6 | DMA_RX_Buffer[15] << 2 | DMA_RX_Buffer[16] << 10) & 0x07FF);
			channels[11] = (uint16_t)((DMA_RX_Buffer[16] >> 1 | DMA_RX_Buffer[17] << 7) & 0x07FF);
			channels[12] = (uint16_t)((DMA_RX_Buffer[17] >> 4 | DMA_RX_Buffer[18] << 4) & 0x07FF);
			channels[13] = (uint16_t)((DMA_RX_Buffer[18] >> 7 | DMA_RX_Buffer[19] << 1 | DMA_RX_Buffer[20] <<9 ) & 0x07FF);
			channels[14] = (uint16_t)((DMA_RX_Buffer[20] >> 2 | DMA_RX_Buffer[21] << 6) & 0x07FF);
			channels[15] = (uint16_t)((DMA_RX_Buffer[21] >> 5 | DMA_RX_Buffer[22] << 3) & 0x07FF);
			lost_frame = DMA_RX_Buffer[23] && (1<<2);
			failsafe = DMA_RX_Buffer[23] && (1<<3);
}
