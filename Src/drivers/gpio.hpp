/*
 * gpio.h
 *
 * Author: Manh Tuyen Ta
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "bsp.hpp"

class Gpio
{
	private:
		uint8_t m_nPort;
		uint8_t m_nPin;

	public:
		Gpio(uint8_t nPort, uint8_t nPin);
		virtual ~Gpio();

		void initInputFloating();
		void initInputPullUp();
		void initInputPullDown();
		void initOutputOpenDrain();
		void initOutputPushPull();
		void toggleOutputPin();
		void writeOutputPin(uint8_t value);
		uint8_t readOutputPin() const;
		uint8_t readInputPin() const;
};

#endif /* GPIO_H_ */
