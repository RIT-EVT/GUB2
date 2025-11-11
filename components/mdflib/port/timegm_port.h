#pragma once
#include <time.h>
#ifdef __cplusplus
extern "C" {
#endif
    time_t timegm(struct tm* tm);   // declare real 'timegm'
#ifdef __cplusplus
}
#endif
