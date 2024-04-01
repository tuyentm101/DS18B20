/*
 * usart.hpp
 *
 * Author: Manh Tuyen Ta
 */

#ifndef USART_H_
#define USART_H_

#include "bsp.hpp"

class Usart
{
	private:
		const char *const m_name;
		uint8_t m_nUsart;

	public:
		Usart(const char *name, uint8_t nUsart);
		virtual ~Usart();

		bool send(uint8_t data);
		bool send(const char* msg);
		bool read(uint8_t &data);
};

#endif /* USART_H_ */
