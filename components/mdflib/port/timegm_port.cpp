// components/mdflib/port/timegm_port.cpp
#include <time.h>
#include "timegm_port.h"

static long long days_from_civil(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int era  = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (long long)era * 146097 + (long long)doe - 719468; // days since 1970-01-01
}

extern "C" time_t mdf_timegm(struct tm* tm) {
    if (!tm) return (time_t)-1;
    const long long days = days_from_civil(tm->tm_year + 1900,
                                           (unsigned)(tm->tm_mon + 1),
                                           (unsigned)tm->tm_mday);
    const long long secs = days * 86400LL
                         + tm->tm_hour * 3600LL
                         + tm->tm_min  * 60LL
                         + tm->tm_sec;
    return (time_t)secs;     // ignores leap seconds, like timegm()
}
