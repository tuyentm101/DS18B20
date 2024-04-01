/*
 * usart.cpp
 *
 * Author: Manh Tuyen Ta
 */

#include "usart.hpp"

Usart::Usart(const char *name, uint8_t nUsart)
: m_name(name),
  m_nUsart(nUsart)
{
}

Usart::~Usart()
{

}

bool Usart::send(uint8_t data)
{
	return (BSP::USART_Send(m_nUsart, data) == 0 ? true : false);
}

bool Usart::send(const char *msg)
{
	return ((BSP::USART_SendString(m_nUsart, msg) < 0) ? false : true);
}

bool Usart::read(uint8_t &data)
{
	return (BSP::USART_Receive(m_nUsart, &data) == 0 ? true : false);
}
