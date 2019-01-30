#pragma once

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include "stdbool.h"

/// <summary>
/// This function is executed in case of error occurrence.
/// </summary>
void ErrorHandler(const char * file, int32_t line);

/// <summary>
/// This function is executed in case of error occurrence(Macro definition under ErrorHandler(char * file, int line))
/// </summary>
#define Error_Handler() ErrorHandler(__FILE__, __LINE__)

#ifdef __cplusplus
}
#endif

#endif
