/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
 extern CAN_RxHeaderTypeDef RxHeader;
 extern CAN_TxHeaderTypeDef TxHeader;
 extern uint8_t RxData[8];
 extern uint8_t TxData[8];
 extern uint32_t TxMailbox;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Start_IT(CAN_HandleTypeDef *hcan)//can.h里面要声明
{

  CAN_FilterTypeDef CAN_Filter;

  if(hcan==&hcan1)
    CAN_Filter.FilterBank=0,CAN_Filter.SlaveStartFilterBank=14;
  else 
	CAN_Filter.FilterBank=14,CAN_Filter.SlaveStartFilterBank=14;
  
  CAN_Filter.FilterActivation=CAN_FILTER_ENABLE;
  CAN_Filter.FilterIdHigh=0;
  CAN_Filter.FilterIdLow=0;
  CAN_Filter.FilterMaskIdHigh=0;
  CAN_Filter.FilterMaskIdLow=0;
  CAN_Filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  CAN_Filter.FilterMode=CAN_FILTERMODE_IDMASK;
  CAN_Filter.FilterScale=CAN_FILTERSCALE_32BIT;
  
  HAL_CAN_ConfigFilter(hcan,&CAN_Filter);
	
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	RxHeader.StdId = 0200;//作为标识符，通信双方保持一致，回环模式不作要求
  RxHeader.RTR = CAN_RTR_DATA;//发送数据帧   选择数据帧或遥控帧
  RxHeader.IDE = CAN_ID_STD;
  RxHeader.DLC = 8;//数据长度
  RxHeader.FilterMatchIndex = DISABLE;

//  TxHeader.StdId = 0x200;//作为标识符，通信双方保持一致，回环模式不作要求
//  TxHeader.RTR = CAN_RTR_REMOTE;//发送数据帧   选择数据帧或遥控帧
//  TxHeader.IDE = CAN_ID_STD;
//  TxHeader.DLC = 8;//数据长度
//  TxHeader.TransmitGlobalTime = DISABLE;
//	
}

typedef struct
{
	uint16_t angle;
	
	int16_t speed;
	
	int16_t current;
	
	int8_t temper;

} motor;
motor Motor;

void CAN_get_message(void)
{

  Motor.angle=(uint16_t)RxData[0]<<8|RxData[1];
	Motor.speed=(int16_t)RxData[2]<<8|RxData[3];
	Motor.current=(int16_t)RxData[4]<<8|RxData[5];
	Motor.temper=(int8_t)RxData[6];
	
//	Motor.angle=RxData[0];Motor.angle<<=8;Motor.angle|=RxData[1];
//	

}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
