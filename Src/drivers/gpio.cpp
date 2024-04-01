/*
 * gpio.cpp
 *
 * Author: Manh Tuyen Ta
 */

#include "gpio.hpp"

Gpio::Gpio(uint8_t nPort, uint8_t nPin)
: m_nPort(nPort),
  m_nPin(nPin)
{
}

Gpio::~Gpio()
{
}

void Gpio::initInputFloating()
{
	BSP::GPIO_InitInputFloating(m_nPort, m_nPin);
}

void Gpio::initInputPullUp()
{
	BSP::GPIO_InitInputPullUp(m_nPort, m_nPin);
}

void Gpio::initInputPullDown()
{
	BSP::GPIO_InitInputPullDown(m_nPort, m_nPin);
}

void Gpio::initOutputOpenDrain()
{
	BSP::GPIO_InitOutputOpenDrain(m_nPort, m_nPin);
}

void Gpio::initOutputPushPull()
{
	BSP::GPIO_InitOutputPushPull(m_nPort, m_nPin);
}

void Gpio::toggleOutputPin()
{
	BSP::GPIO_ToggleOutputPin(m_nPort, m_nPin);
}

void Gpio::writeOutputPin(uint8_t value)
{
	BSP::GPIO_WriteOutputPin(m_nPort, m_nPin, value);
}

uint8_t Gpio::readOutputPin() const
{
	return BSP::GPIO_ReadOutputPin(m_nPort, m_nPin);
}

uint8_t Gpio::readInputPin() const
{
	return BSP::GPIO_ReadInputPin(m_nPort, m_nPin);
}
