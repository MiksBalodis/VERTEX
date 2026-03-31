#include "gps.h"
#include "main.h"

#define GPS_RX_BUF_SIZE 512
uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];

extern UART_HandleTypeDef huart1;

void GPS_Init(){
    HAL_UART_Receive_DMA(&huart1, gps_rx_buf, GPS_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    HAL_UART_Transmit_DMA(&huart1, configUBX,
			sizeof(configUBX) / sizeof(uint8_t));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(&huart1, setNMEA410,
			sizeof(setNMEA410) / sizeof(uint8_t));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(&huart1, setGNSS,
			sizeof(setGNSS) / sizeof(uint8_t));
	HAL_Delay(250);
}

void GPS_Process_Data(uint16_t data_len){
    if(gps_rx_buf[0] == 0xB5 && gps_rx_buf[1] == 0x62){
        if(gps_rx_buf[2] == 0x27 && gps_rx_buf[3] == 0x03){
            
        }
    }
}

void GPS_GetUniqID(){
	HAL_UART_Transmit_DMA(&huart1, getDeviceID,
			sizeof(getDeviceID) / sizeof(uint8_t));
}

void GPS_UART_IdleCallback(void){
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        uint16_t len = GPS_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        GPS_Process_Data(len);

        // Restart DMA
        HAL_UART_AbortReceive(&huart1);
        HAL_UART_Receive_DMA(&huart1, gps_rx_buf, GPS_RX_BUF_SIZE);
    }
}