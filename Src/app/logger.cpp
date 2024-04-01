/*
 * logger.cpp
 *
 * Author: Manh Tuyen Ta
 */


#include "logger.hpp"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

Logger::Logger(const uint8_t &nUart)
: m_nUart(nUart)
{
}

Logger::~Logger()
{

}

Logger& Logger::instance(const uint8_t &nUart)
{
	static Logger instance(nUart);
	return instance;
}

void Logger::write(const char *prefix, const char *text, const char *file, int line, ...)
{
	static char buf[1024] = {0};
	static char msgBuf[1024] = {0};
	va_list args;
	int ret;

	sprintf(buf, "%s - [%s:%u] - ", prefix, file, line);

	va_start(args, line);
	ret = vsnprintf(msgBuf, sizeof(msgBuf), text, args);
	va_end(args);

	if (ret < 0) {
		return;
	}
	else {
		strncat(buf, msgBuf, strlen(msgBuf));
		BSP::USART_SendString(m_nUart, buf);
	}
	BSP::USART_SendString(m_nUart, "\r\n");
}

void Logger::write(const char *msg, ...)
{
	static char msgBuf[2048] = {0};
	va_list args;
	int ret;

	va_start(args, msg);
	ret = vsnprintf(msgBuf, sizeof(msgBuf), msg, args);
	va_end(args);

	if (ret < 0) {
		return;
	}

	BSP::USART_SendString(m_nUart, msgBuf);
}

int Logger::readLine(char *const msgBuf, unsigned int num)
{
	int ret = -1;
	uint8_t readData = 0;
	static char readBuffer[64];
	static unsigned int readIdx = 0;

	if (BSP::USART_Receive(m_nUart, &readData) == 0) {
		readBuffer[readIdx++] = readData;
		if (readData == '\n') {
			if (readIdx < num) {
				ret = 0;
				memcpy(msgBuf, readBuffer, readIdx);
				msgBuf[readIdx] = 0;
			}
			readIdx = 0;
		}
		if (readIdx >= sizeof(readBuffer)) {
			readIdx = 0;
		}
	}

	return ret;
}
