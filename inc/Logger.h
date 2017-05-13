/*! @brief
 *  记录数据到文件中。文件名将以创建时间开头，每份数据会附带时间戳
 * */

#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>
#include <stdarg.h>
#include <sys/time.h>

class Logger {

public:

	Logger(std::string name);
	~Logger();

	int operator()(const char *fmt, ...);

private:

	timeval tv;
	std::ofstream ofs;

	int startS;
	int startUs;

	// helpers
	static std::string getTime();

};

#endif  // LOGGER_H
