/*
 * ds18b20.hpp
 *
 * Author: Manh Tuyen Ta
 */

#ifndef DS18B20_HPP_
#define DS18B20_HPP_

#include "gpio.hpp"

class DS18B20
{
	private:
		Gpio *const m_pDqGpio;
		uint16_t m_adcValue;
		uint8_t m_bitIndex;
		uint8_t m_txData;
		uint64_t m_rxData;
		uint32_t m_1usTicks;
		int m_state;
		int m_step;
		uint8_t m_readPinVal;

	public:
		DS18B20() = delete;
		DS18B20(const DS18B20&) = delete;
		DS18B20(const DS18B20&&) = delete;
		DS18B20& operator=(const DS18B20&) = delete;
		DS18B20& operator=(const DS18B20&&) = delete;

		explicit DS18B20(Gpio *const p_DqGpio);
		virtual ~DS18B20();
		void process();
		uint16_t adcValue() const;
		float value() const;
};

#endif /* DS18B20_HPP_ */
