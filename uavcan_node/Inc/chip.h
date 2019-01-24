#pragma once

//#include "stm32f1xx_hal_def.h"
//#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_hal.h"
//#include "stm32f407xb.h"
#include "stm32f1xx.h"

#define STM32F1XX
#define STM32_PCLK1           (32000000ul)          // 42 MHz
#define STM32_TIMCLK1         (64000000ul)          // 84 MHz
#define CAN1_TX_IRQHandler    CAN1_TX_IRQHandler
#define CAN1_TX_IRQHandler    USB_HP_CAN1_TX_IRQHandler
#define CAN1_RX0_IRQHandler   CAN1_RX0_IRQHandler
#define CAN1_RX0_IRQHandler   USB_LP_IRQHandler
#define CAN1_RX1_IRQHandler   CAN1_RX1_IRQHandler