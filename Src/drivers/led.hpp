/*
 * led.hpp
 *
 * Author: Manh Tuyen Ta
 */

#ifndef LED_H_
#define LED_H_

#include "gpio.hpp"

class Led
{
	private:
		Gpio* const m_pGpio;
		uint8_t m_activeLevel;

	public:
		Led(Gpio* const p_gpio, bool isActiveLow = true);
		virtual ~Led();

		void toggle();
		void turnOn();
		void turnOff();
};

#endif /* LED_H_ */
