/*
 * logger.hpp
 *
 * Author: Manh Tuyen Ta
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "bsp.hpp"

#define LOG_INFO(m,...)		 Logger::instance(BSP_USART1).write("INFO", m, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_ERROR(m,...)	 Logger::instance(BSP_USART1).write("ERROR", m, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_PRINT(m,...)	 Logger::instance(BSP_USART1).write(m, ##__VA_ARGS__)

class Logger
{
	private:
		uint8_t m_nUart;
		Logger(const uint8_t &nUart);

	public:
		Logger() = delete;
		Logger(const Logger&) = delete;
		Logger(const Logger&&) = delete;
		Logger& operator=(const Logger&) = delete;
		Logger& operator=(const Logger&&) = delete;
		virtual ~Logger();

		static Logger& instance(const uint8_t &nUart);

		void write(const char *prefix, const char *text, const char *file, int line, ...);
		void write(const char *msg, ...);

		int readLine(char *const msgBuf, unsigned int num);
};

#endif /* LOGGER_H_ */
