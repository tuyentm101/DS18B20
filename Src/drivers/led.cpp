/*
 * led.cpp
 *
 * Author: Manh Tuyen Ta
 */

#include "led.hpp"

Led::Led(Gpio* const p_gpio, bool isActiveLow)
: m_pGpio(p_gpio),
  m_activeLevel(isActiveLow ? 0 : 1)
{
}

Led::~Led()
{
}

void Led::toggle()
{
	m_pGpio->toggleOutputPin();
}

void Led::turnOn()
{
	m_pGpio->writeOutputPin(m_activeLevel);
}

void Led::turnOff()
{
	m_pGpio->writeOutputPin(m_activeLevel);
}

