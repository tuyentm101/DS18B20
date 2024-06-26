/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "led.hpp"
#include "logger.hpp"
#include "ds18b20.hpp"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

static Gpio g_d2LedGpio(BSP_GPIO_PORT_A, BSP_GPIO_PIN_5);
static Gpio g_d3LedGpio(BSP_GPIO_PORT_A, BSP_GPIO_PIN_1);
static Led g_d2Led(&g_d2LedGpio);
static Led g_d3Led(&g_d3LedGpio);
static Gpio ds18b20Gpio(BSP_GPIO_PORT_E, BSP_GPIO_PIN_0);
DS18B20 g_ds18b20Inst(&ds18b20Gpio);

void blinkLed()
{
    static uint32_t currentTick = 0;
    static uint32_t startTick = BSP::getTick();

    currentTick = BSP::getTick();
    if (currentTick - startTick > 500UL) {
        startTick = currentTick;
        g_d2Led.toggle();
        g_d3Led.toggle();
        LOG_PRINT("Temperature: %.4f\r\n", g_ds18b20Inst.value());
    }
}


int main(void)
{
	BSP::init();

	LOG_PRINT("SystemCoreClock: %lu\r\n", BSP::getSystemClock());

	while (1) {
		blinkLed();
	}
}
