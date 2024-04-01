/*
 * ds18b20.cpp
 *
 * Author: Manh Tuyen Ta
 */

#include "ds18b20.hpp"

#define DS18B20_SKIP_ROM_CMD			(0xCC)
#define DS18B20_CONVERT_CMD				(0x44)
#define DS18B20_READ_SCRATCHPAD_CMD		(0xBE)

DS18B20::DS18B20(Gpio *const p_DqGpio)
: m_pDqGpio(p_DqGpio),
  m_1usTicks(0),
  m_state(0),
  m_step(2)
{
}

DS18B20::~DS18B20()
{
}

void DS18B20::process()
{
	++m_1usTicks;

	switch (m_state) {

		case 0:
			m_pDqGpio->initOutputOpenDrain();
			m_pDqGpio->writeOutputPin(0);
			m_1usTicks = 0;
			m_state = 1;
		break;

		case 1:
			if (m_1usTicks >= 480UL) {
				m_pDqGpio->initInputFloating();
				m_readPinVal = m_pDqGpio->readInputPin();
				m_1usTicks = 0;
				m_state = 2;
			}
		break;

		case 2:
			if (m_1usTicks <= 240UL) {
				if ((m_1usTicks >= 80UL) && (m_readPinVal == 1)) {
					m_readPinVal = m_pDqGpio->readInputPin();
				}
			}
			else if (m_1usTicks >= 480UL) {
				if (m_readPinVal == 0) {
					m_state = 3;
				}
				else {
					m_state = 0;
				}
			}
		break;

		case 3:
			m_txData = DS18B20_SKIP_ROM_CMD;
			m_state = 6;
			m_bitIndex = 0;
		break;

		case 4:
			m_txData = DS18B20_CONVERT_CMD;
			m_state = 6;
			m_step = 3;
			m_bitIndex = 0;
		break;

		case 5:
			m_txData = DS18B20_READ_SCRATCHPAD_CMD;
			m_state = 6;
			m_step = 5;
			m_bitIndex = 0;
		break;

		case 6: // start write time slot
			m_pDqGpio->initOutputOpenDrain();
			m_pDqGpio->writeOutputPin(0);
			m_1usTicks = 0;
			m_state = 7;
		break;

		case 7:
			if ((m_txData & (1U << m_bitIndex)) != 0U) {
				m_pDqGpio->initInputFloating();
			}
			m_state = 8;
		break;

		case 8: // 60us minimum interval of write time slot
			if (m_1usTicks >= 60UL) {
				m_pDqGpio->initInputFloating();
				if ((++m_bitIndex) < 8U) {
					m_state = 6;
				}
				else if (m_step == 2) { // finish send skip rom cmd, continue to send convert t cmd
					m_state = 4;
				}
				else if (m_step == 3) { // finish send convert t cmd
					m_step = 4;
					m_state = 0;
				}
				else if (m_step == 4) { // finish send skip rom cmd, continue to send read scratch pad cmd
					m_state = 5;
				}
				else { // finish send scratch pad cmd
					m_bitIndex = 0;
					m_rxData = 0;
					m_state = 9;
				}
			}
		break;

		case 9: // start read time slot
			m_pDqGpio->initOutputOpenDrain();
			m_pDqGpio->writeOutputPin(0);
			m_1usTicks = 0;
			m_state = 10;
		break;

		case 10:
			m_pDqGpio->initInputFloating();
			m_state = 11;
		break;

		case 11:
			m_readPinVal = m_pDqGpio->readInputPin();
			m_rxData |= m_readPinVal << m_bitIndex;
			m_state = 12;
		break;

		case 12: // 60us minimum interval of read time slot
			if (m_1usTicks >= 60UL) {
				if ((++m_bitIndex) >= 31U) {
					m_adcValue = (uint8_t)(m_rxData >> 8);
					m_adcValue <<= 8;
					m_adcValue |= (uint8_t)m_rxData;
					m_step = 2;
					m_state = 0;
				}
				else {
					m_state = 9;
				}
			}
		break;
	}
}

uint16_t DS18B20::adcValue() const
{
	return m_adcValue;
}

float DS18B20::value() const
{
	return (float)m_adcValue * 0.0625f;
}

