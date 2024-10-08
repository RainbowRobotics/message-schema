#ifndef __H_MTIME_H__
#define __H_MTIME_H__

#include <chrono>
#include <unistd.h>
#include <time.h>

enum TIME_PRECISION
{
    TIME_NANOSECOND = 0,
    TIME_MICROSECOND,
    TIME_MILLISECOND,
	TIME_SECOND,
	TIME_MINUTE,
	TIME_HOUR
};

extern std::chrono::time_point<std::chrono::steady_clock> time_start;
inline void timestart()
{
	time_start = std::chrono::steady_clock::now();
}
inline int64_t timeused()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
}
inline int64_t timeused(std::chrono::time_point<std::chrono::steady_clock>& time_last)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_last).count();
}

void sleep_ms(int ms);
struct tm localdate(int time_zone = 8);
int64_t current_times(int precision = TIME_MILLISECOND);
char *time_str(const char *fmt = "%F %T");

#endif
