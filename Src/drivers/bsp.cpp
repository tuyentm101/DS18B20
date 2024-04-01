/*
 * bsp.cpp
 *
 * Author: Manh Tuyen Ta
 */

#include "bsp.hpp"
#include "fifo.h"
#include "ds18b20.hpp"

#include <inttypes.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32_assert.h"

#define USART_BUFFER_SIZE (4096)

/*------------------------- GPIO --------------------------------*/
typedef struct {
		int port;
		int pin;
} BspGpio_t;

static GPIO_TypeDef* g_gpioPorts[] = {
		GPIOA,
		GPIOB,
		GPIOC,
		GPIOD,
		GPIOE,
		GPIOF,
		GPIOG,
		GPIOH,
		GPIOI
};
static uint32_t g_gpioPins[] = {
		LL_GPIO_PIN_0,
		LL_GPIO_PIN_1,
		LL_GPIO_PIN_2,
		LL_GPIO_PIN_3,
		LL_GPIO_PIN_4,
		LL_GPIO_PIN_5,
		LL_GPIO_PIN_6,
		LL_GPIO_PIN_7,
		LL_GPIO_PIN_8,
		LL_GPIO_PIN_9,
		LL_GPIO_PIN_10,
		LL_GPIO_PIN_11,
		LL_GPIO_PIN_12,
		LL_GPIO_PIN_13,
		LL_GPIO_PIN_14,
		LL_GPIO_PIN_15
};
/*------------------------- USART --------------------------------*/
static USART_TypeDef* g_usarts[] = {
		USART1,
		USART2,
		USART3,
		UART4,
		UART5,
		USART6
};
static FifoIndex_t g_usartRxFifoIndex[N_USART];
static uint8_t g_usartRxBuffer[N_USART][USART_BUFFER_SIZE];
static uint8_t g_usartTxBuffer[N_USART][USART_BUFFER_SIZE];
static FifoIndex_t g_usartTxFifoIndex[N_USART];

static volatile uint32_t g_nSysTicks = 0;
static volatile uint32_t g_nTimer1Ticks = 0;

extern DS18B20 g_ds18b20Inst;

/*!
 * @fn void SystemClock_Init(void)
 * @brief System Clock Configuration
 *
 * f_VCO = PLL clock input x (PLLN / PLLM)
 * PLL general clock output = f_VCO / PLLP = PLL clock input x (PLLN / PLLM) / PLLP
 * USB OTG FS, SDIO, RNG clock output = f_VCO / PLLQ
 * System Clock Source = PLL (HSE source)
 * SYSCLK = PLLCLK = 168 MHz
 *
 */
static void SystemClock_Init(void)
{
	LL_RCC_HSE_Enable();
	while (LL_RCC_HSE_IsReady() == 0);

	LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() == 0);

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1); // maxinum frequency of AHB domain = 168 MHz
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4); // maximum frequency of ABP1 (low - speed) domain = 42 MHz
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2); // maximum frequency of ABP2 (high - speed) domain = 84 MHz

	/* Timers clocks prescalers selection, Timer1 clock = 2 * PCLK2 */
//	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);

	SystemCoreClockUpdate(); // Update CMSIS variable
}

static void SysTick_Init(void)
{
	SysTick_Config(SystemCoreClock / 1000U); // set systick to 1ms
}

/*!
 * @fn void GPIO_InitAF(GPIO_TypeDef*, uint32_t, uint32_t, uint32_t, uint32_t)
 * @brief
 *
 * @param gpioPort
 * @param gpioPin
 * @param af
 * @param outType
 * @param pull
 */
static void GPIO_InitAF(GPIO_TypeDef *gpioPort, uint32_t gpioPin, uint32_t af)
{
	LL_GPIO_InitTypeDef gpioInit;
	LL_GPIO_StructInit(&gpioInit);
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.Alternate = af;
	gpioInit.Pin = gpioPin;
	LL_GPIO_Init(gpioPort, &gpioInit);
}

/**
 * @fn void GPIO_InitInputPullUp(GPIO_TypeDef*, uint32_t)
 * @brief Input pullup
 *
 * @param gpioPort
 * @param gpioPin
 */
static void GPIO_InitInputPullUp(GPIO_TypeDef *gpioPort, uint32_t gpioPin)
{
	LL_GPIO_InitTypeDef gpioInit;
	LL_GPIO_StructInit(&gpioInit);
	gpioInit.Mode = LL_GPIO_MODE_INPUT;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.Pull = LL_GPIO_PULL_UP;
	gpioInit.Pin = gpioPin;
	LL_GPIO_Init(gpioPort, &gpioInit);
}

static void GPIO_InitInputPullDown(GPIO_TypeDef *gpioPort, uint32_t gpioPin)
{
	LL_GPIO_InitTypeDef gpioInit;
	LL_GPIO_StructInit(&gpioInit);
	gpioInit.Mode = LL_GPIO_MODE_INPUT;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.Pull = LL_GPIO_PULL_DOWN;
	gpioInit.Pin = gpioPin;
	LL_GPIO_Init(gpioPort, &gpioInit);
}

static void GPIO_InitInputFloating(GPIO_TypeDef *gpioPort, uint32_t gpioPin)
{
	LL_GPIO_InitTypeDef gpioInit;
	LL_GPIO_StructInit(&gpioInit);
	gpioInit.Mode = LL_GPIO_MODE_INPUT;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.Pin = gpioPin;
	LL_GPIO_Init(gpioPort, &gpioInit);
}

/**
 * @fn void GPIO_InitOutputPushPull(GPIO_TypeDef*, uint32_t)
 * @brief Output push-pull with pull-up capability
 *
 * @param gpioPort
 * @param gpioPin
 */
static void GPIO_InitOutputPushPull(GPIO_TypeDef *gpioPort, uint32_t gpioPin)
{
	LL_GPIO_InitTypeDef gpioInit;
	LL_GPIO_StructInit(&gpioInit);
	gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpioInit.Pull = LL_GPIO_PULL_UP;
	gpioInit.Pin = gpioPin;
	LL_GPIO_Init(gpioPort, &gpioInit);
}

/**
 * @brief Ouput open - drain
 *
 * @param gpioPort
 * @param gpioPin
 */
static void GPIO_InitOutputOpenDrain(GPIO_TypeDef *gpioPort, uint32_t gpioPin)
{
	LL_GPIO_InitTypeDef gpioInit;
	LL_GPIO_StructInit(&gpioInit);
	gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	gpioInit.Pull = LL_GPIO_PULL_UP;
	gpioInit.Pin = gpioPin;
	LL_GPIO_Init(gpioPort, &gpioInit);
}

static void GPIO_Init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
	GPIO_InitOutputPushPull(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_5);
}

/**
 * @fn void Timer_Init(void)
 * @brief
 *  **** Timebase configuration:
 *	    Counter mode: up
 *		Timer1/8 input clock (CK_INT, CK_PSC) = 2 * PCLK2 = 2 * 84MHz = 168MHz
 *		Timer1/8 counter frequency (CK_CNT)
 *		= Timer1/8 input clock / (PSC + 1) = 168 MHz / (7 + 1) = 21 MHz
 *		Timer1 period = ARR = Timer1 counter frequency / output frequency - 1
 *		= 21 MHz / 1 MHz - 1 = 20 (1us)
 */
static void Timer_Init(void)
{
	LL_TIM_InitTypeDef timebaseInit;

    /* Enable peripheral clock of Timer1 */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Timebase configuration of timer1 */
    LL_TIM_StructInit(&timebaseInit);
    timebaseInit.CounterMode = LL_TIM_COUNTERMODE_UP;
    timebaseInit.Prescaler = 7;
    timebaseInit.Autoreload = 20;
    timebaseInit.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &timebaseInit);
    LL_TIM_DisableARRPreload(TIM1);

    /* Enable Update Event interrupt */
    LL_TIM_EnableIT_UPDATE(TIM1);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    /* Enable Timer1 */
    LL_TIM_EnableCounter(TIM1);
}

static void USART_Init(void)
{
    LL_USART_InitTypeDef uartInit;

    /* UART1 TX PA9 */
    GPIO_InitAF(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7);

    /* UART1 RX PA10 */
    GPIO_InitAF(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7);

    /* Enable USART1 peripheral clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    /* USART1 config */
    LL_USART_StructInit(&uartInit);
    uartInit.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    uartInit.TransferDirection = LL_USART_DIRECTION_TX_RX;
    uartInit.Parity = LL_USART_PARITY_NONE;
    uartInit.StopBits = LL_USART_STOPBITS_1;
    uartInit.DataWidth = LL_USART_DATAWIDTH_8B;
    uartInit.OverSampling = LL_USART_OVERSAMPLING_8;
    uartInit.BaudRate = 115200;
    LL_USART_Init(USART1, &uartInit);

    /* Enable UART RXNE interrupt */
    LL_USART_EnableIT_RXNE(USART1);

    /* Set priority for USARTx_IRQn */
    NVIC_SetPriority(USART1_IRQn, 2);

    /* Enable USARTx_IRQn */
    NVIC_EnableIRQ(USART1_IRQn);

    /* Enable USART */
    LL_USART_Enable(USART1);
}

static void USART_Callback(uint8_t nUart)
{
    if (LL_USART_IsActiveFlag_RXNE(g_usarts[nUart]) == 1) {
        uint8_t rxData = 0;
        rxData = LL_USART_ReceiveData8(g_usarts[nUart]);
        fifoPushOne(g_usartRxFifoIndex[nUart], &rxData);
    }

    if (LL_USART_IsActiveFlag_TXE(g_usarts[nUart]) == 1) {
        uint8_t txData = 0;
        if (fifoPopOne(g_usartTxFifoIndex[nUart], &txData) < 0) {
            LL_USART_DisableIT_TXE(g_usarts[nUart]);
            return;
        }
        LL_USART_TransmitData8(g_usarts[nUart], txData);
    }
}

extern "C" {
	/**
	 * @brief   This function handles NMI exception.
	 * @param  None
	 * @retval None
	 */
	void NMI_Handler(void) {}

	/**
	 * @brief  This function handles Hard Fault exception.
	 * @param  None
	 * @retval None
	 */
	void HardFault_Handler(void)
	{
	    /* Go to infinite loop when Hard Fault exception occurs */
	    while (1) {
	    }
	}

	/**
	 * @brief  This function handles Memory Manage exception.
	 * @param  None
	 * @retval None
	 */
	void MemManage_Handler(void)
	{
	    /* Go to infinite loop when Memory Manage exception occurs */
	    while (1) {
	    }
	}

	/**
	 * @brief  This function handles Bus Fault exception.
	 * @param  None
	 * @retval None
	 */
	void BusFault_Handler(void)
	{
	    /* Go to infinite loop when Bus Fault exception occurs */
	    while (1) {
	    }
	}

	/**
	 * @brief  This function handles Usage Fault exception.
	 * @param  None
	 * @retval None
	 */
	void UsageFault_Handler(void)
	{
		/* Go to infinite loop when Usage Fault exception occurs */
		while (1) {
		}
	}

	/**
	 * @brief  This function handles SVCall exception.
	 * @param  None
	 * @retval None
	 */
	void SVC_Handler(void) {}

	/**
	 * @brief  This function handles Debug Monitor exception.
	 * @param  None
	 * @retval None
	 */
	void DebugMon_Handler(void) {}

	/**
	 * @brief  This function handles PendSVC exception.
	 * @param  None
	 * @retval None
	 */
	void PendSV_Handler(void) {}

	/**
	 * @brief  This function handles SysTick Handler.
	 * @param  None
	 * @retval None
	 */
	void SysTick_Handler(void)
	{
		++g_nSysTicks;
	}

	void USART1_IRQHandler(void)
	{
		USART_Callback(BSP_USART1);
	}

	void TIM1_UP_TIM10_IRQHandler(void)
	{
		if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
			LL_TIM_ClearFlag_UPDATE(TIM1);

			g_ds18b20Inst.process();
		}
	}
} /* extern "C" */

namespace BSP
{
	void init(void)
	{
	    g_usartTxFifoIndex[BSP_USART1] = fifoInit(1, USART_BUFFER_SIZE, g_usartTxBuffer[BSP_USART1]);
	    g_usartRxFifoIndex[BSP_USART1] = fifoInit(1, USART_BUFFER_SIZE, g_usartRxBuffer[BSP_USART1]);

		SystemClock_Init();
		SysTick_Init();
		Timer_Init();
		GPIO_Init();
		USART_Init();
	}

	uint32_t getSystemClock()
	{
		return SystemCoreClock;
	}

	uint32_t getTick(void)
	{
	    return g_nSysTicks;
	}

	void deInit(void)
	{
		LL_RCC_DeInit();
	}

	void delayMs(uint32_t nTimeMs)
	{
	    uint32_t delay = g_nSysTicks + nTimeMs;
	    while (delay > g_nSysTicks) {
	    }
	}

	void GPIO_InitOutputOpenDrain(uint8_t nPort, uint8_t nPin)
	{
		GPIO_InitOutputOpenDrain(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	void GPIO_InitOutputPushPull(uint8_t nPort, uint8_t nPin)
	{
		GPIO_InitOutputPushPull(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	void GPIO_InitInputFloating(uint8_t nPort, uint8_t nPin)
	{
		GPIO_InitInputFloating(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	void GPIO_InitInputPullUp(uint8_t nPort, uint8_t nPin)
	{
		GPIO_InitInputPullUp(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	void GPIO_InitInputPullDown(uint8_t nPort, uint8_t nPin)
	{
		GPIO_InitInputPullDown(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	void GPIO_ToggleOutputPin(uint8_t nPort, uint8_t nPin)
	{
		LL_GPIO_TogglePin(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	void GPIO_WriteOutputPin(uint8_t nPort, uint8_t nPin, uint8_t value)
	{
	    if (value) {
	        LL_GPIO_SetOutputPin(g_gpioPorts[nPort], g_gpioPins[nPin]);
	    }
	    else {
	        LL_GPIO_ResetOutputPin(g_gpioPorts[nPort], g_gpioPins[nPin]);
	    }
	}

	uint8_t GPIO_ReadOutputPin(uint8_t nPort, uint8_t nPin)
	{
	    return (uint8_t)LL_GPIO_IsOutputPinSet(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	uint8_t GPIO_ReadInputPin(uint8_t nPort, uint8_t nPin)
	{
	    return (uint8_t)LL_GPIO_IsInputPinSet(g_gpioPorts[nPort], g_gpioPins[nPin]);
	}

	int USART_Send(uint8_t nUart, uint8_t data)
	{
	    int ret = fifoPushOne(g_usartTxFifoIndex[nUart], &data);
	    if (LL_USART_IsEnabledIT_TXE(g_usarts[nUart]) == 0) {
	        LL_USART_EnableIT_TXE(g_usarts[nUart]);
	    }
	    return ret;
	}

	int USART_SendString(uint8_t nUart, const char *buf)
	{
	    if (!buf)
	        return -1;
	    while (*buf) {
	        int ret = USART_Send(nUart, *(buf++));
	        if (ret < 0)
	            return -1;
	    }
	    return 0;
	}

	int USART_SendBuf(uint8_t nUart, const char *buf, size_t nBuf)
	{
	    for (unsigned int i = 0; i < nBuf; ++i) {
	        USART_Send(nUart, (uint8_t)buf[i]);
	    }
	    return 0;
	}

	int USART_Receive(uint8_t nUart, uint8_t *const p_data)
	{
		return fifoPopOne(g_usartRxFifoIndex[nUart], p_data);
	}
}
