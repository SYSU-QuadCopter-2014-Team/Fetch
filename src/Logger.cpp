#include "Logger.h"

Logger::Logger(std::string name)
	: ofs((getTime() + "_" + name).c_str()) {
	gettimeofday(&tv, NULL);
	startS = tv.tv_sec;
	startUs = tv.tv_usec;
}

Logger::~Logger() {
	ofs.close();
}

int Logger::operator()(const char *fmt, ...) {
	timeval tv;
	gettimeofday(&tv, NULL);
	int s = tv.tv_sec - startS, us = tv.tv_usec - startUs;
	if (us < 0) {
		us += 1000000;
		s--;
	}
	char time[20];
	sprintf(time, "%3d.%06d", s, us);

	char buffer[1000];
	va_list vArgList;
	int counter;

	va_start(vArgList, fmt);
	counter = vsprintf(buffer, fmt, vArgList);
	va_end(vArgList);

	ofs << time << buffer << std::endl;
	return counter;
}

std::string Logger::getTime() {
	struct tm *t;
	time_t tt;

	time(&tt);
	t = localtime(&tt);

	char s[100];
	sprintf(s, "%4d%02d%02d_%02d_%02d_%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	return std::string(s);
}
