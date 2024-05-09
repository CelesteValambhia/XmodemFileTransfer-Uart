/**
* @file		Log.hpp
* @author	Celeste Valambhia
* @brief	Logging module for Error, Warning, Info and Debug Prints
*
*/
#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <string.h>

class Log {
public:
	const int LogLevelError = 0;
	const int LogLevelWarning = 1;
	const int LogLevelInfo = 2;
	const int LogLevelDebug = 3;

private:
	int m_LogLevel = LogLevelInfo;

	template <typename T, typename... Args>
	void Helper(const T& firstArg, const Args&... args) {
		std::cout << firstArg;
		Helper(args...);
	}

	void Helper() {
		std::cout << std::endl; // End of the Debug message
	}

public:
	void SetLevel(int level)
	{
		m_LogLevel = level;
	}

	int GetLevel()
	{
		return m_LogLevel;
	}

	template <typename T, typename... Args>
	void Error(const T& firstArg, const Args&... args)
	{
		if (m_LogLevel >= LogLevelError) {
			std::cout << "[ERROR]: " << firstArg;
			Helper(args...);
		}
	}

	template <typename T, typename... Args>
	void Warn(const T& firstArg, const Args&... args)
	{
		if (m_LogLevel >= LogLevelWarning) {
			std::cout << "[WARN]: " << firstArg;
			Helper(args...);
		}
	}

	template <typename T, typename... Args>
	void Info(const T& firstArg, const Args&... args)
	{
		if (m_LogLevel >= LogLevelInfo) {
			std::cout << "[INFO]: " << firstArg;
			Helper(args...);
		}
	}

	template <typename T, typename... Args>
	void Debug(const T& firstArg, const Args&... args) {
		if (m_LogLevel >= LogLevelDebug) {
			std::cout << "[DEBUG]: " << firstArg;
			Helper(args...);
		}
	}
	
};

#endif //LOG_H
