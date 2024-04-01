/*
 * bsp.hpp
 *
 * Author: Manh Tuyen Ta
 */

#ifndef BSP_HPP_
#define BSP_HPP_

#include <cstdint>
#include <cstdio>

/************************ BSP GPIO ***************/
#define BSP_GPIO_PORT_A			(0U)
#define BSP_GPIO_PORT_B			(1U)
#define BSP_GPIO_PORT_C			(2U)
#define BSP_GPIO_PORT_D			(3U)
#define BSP_GPIO_PORT_E			(4U)
#define BSP_GPIO_PORT_F			(5U)
#define BSP_GPIO_PORT_G			(6U)
#define BSP_GPIO_PORT_H			(7U)
#define BSP_GPIO_PORT_I			(8U)

#define BSP_GPIO_PIN_0			(0U)
#define BSP_GPIO_PIN_1			(1U)
#define BSP_GPIO_PIN_2			(2U)
#define BSP_GPIO_PIN_3			(3U)
#define BSP_GPIO_PIN_4			(4U)
#define BSP_GPIO_PIN_5			(5U)
#define BSP_GPIO_PIN_6			(6U)
#define BSP_GPIO_PIN_7			(7U)
#define BSP_GPIO_PIN_8			(8U)
#define BSP_GPIO_PIN_9			(9U)
#define BSP_GPIO_PIN_10			(10U)
#define BSP_GPIO_PIN_11			(11U)
#define BSP_GPIO_PIN_12			(12U)
#define BSP_GPIO_PIN_13			(13U)
#define BSP_GPIO_PIN_14			(14U)
#define BSP_GPIO_PIN_15			(15U)

/************************ BSP UART *************/
#define N_USART 			(6)
#define BSP_USART1			(0UL)
#define BSP_USART2			(1UL)
#define BSP_USART3			(2UL)
#define BSP_UART4			(3UL)
#define BSP_UART5			(4UL)
#define BSP_USART6			(5UL)

namespace BSP
{
	void init(void);
	uint32_t getSystemClock();
	void deInit(void);
	uint32_t getTick(void);
	void delayMs(uint32_t nTimeMs);
	void GPIO_InitOutputOpenDrain(uint8_t nPort, uint8_t nPin);
	void GPIO_InitOutputPushPull(uint8_t nPort, uint8_t nPin);
	void GPIO_InitInputFloating(uint8_t nPort, uint8_t nPin);
	void GPIO_InitInputPullUp(uint8_t nPort, uint8_t nPin);
	void GPIO_InitInputPullDown(uint8_t nPort, uint8_t nPin);
	void GPIO_ToggleOutputPin(uint8_t nPort, uint8_t nPin);
	void GPIO_WriteOutputPin(uint8_t nPort, uint8_t nPin, uint8_t value);
	uint8_t GPIO_ReadOutputPin(uint8_t nPort, uint8_t nPin);
	uint8_t GPIO_ReadInputPin(uint8_t nPort, uint8_t nPin);
	int USART_Send(uint8_t nUart, uint8_t data);
	int USART_SendString(uint8_t nUart, const char *buf);
	int USART_SendBuf(uint8_t nUart, const char *buf, size_t nBuf);
	int USART_Receive(uint8_t nUart, uint8_t *const p_data);
}

#endif /* BSP_HPP_ */
