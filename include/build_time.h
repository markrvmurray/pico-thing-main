//
// Created by Mark Murray on 01/11/2025.
//

#ifndef PICO_EXAMPLES_BUILD_TIME_H
#define PICO_EXAMPLES_BUILD_TIME_H

#include <time.h>           // struct tm
#include <stdint.h>

/* Extract numeric fields from __DATE__ and __TIME__ at compile time */
#define BUILD_YEAR  ( \
(__DATE__[7]-'0') * 1000 + \
(__DATE__[8]-'0') * 100  + \
(__DATE__[9]-'0') * 10   + \
(__DATE__[10]-'0') \
)

#define BUILD_MONTH ( \
__DATE__[0]=='J' && __DATE__[1]=='a' ? 1 : \
__DATE__[0]=='F'                     ? 2 : \
__DATE__[0]=='M' && __DATE__[2]=='r' ? 3 : \
__DATE__[0]=='A' && __DATE__[1]=='p' ? 4 : \
__DATE__[0]=='M' && __DATE__[2]=='y' ? 5 : \
__DATE__[0]=='J' && __DATE__[2]=='n' ? 6 : \
__DATE__[0]=='J' && __DATE__[2]=='l' ? 7 : \
__DATE__[0]=='A' && __DATE__[1]=='u' ? 8 : \
__DATE__[0]=='S'                     ? 9 : \
__DATE__[0]=='O'                     ? 10 : \
__DATE__[0]=='N'                     ? 11 : \
__DATE__[0]=='D'                     ? 12 : 0 \
)

#define BUILD_DAY ( \
((__DATE__[4]==' ' ? '0' : __DATE__[4]) - '0') * 10 + \
(__DATE__[5] - '0') \
)

#define BUILD_HOUR ((__TIME__[0]-'0')*10 + (__TIME__[1]-'0'))
#define BUILD_MIN  ((__TIME__[3]-'0')*10 + (__TIME__[4]-'0'))
#define BUILD_SEC  ((__TIME__[6]-'0')*10 + (__TIME__[7]-'0'))

/* Return a struct tm (calendar) filled from build time.
   Note: struct tm months are 0-11, tm_year is years since 1900. */
static inline struct tm build_time_tm(void) {
	struct tm tm = {0};
	tm.tm_year = BUILD_YEAR - 1900;   /* years since 1900 */
	tm.tm_mon  = BUILD_MONTH - 1;     /* 0..11 */
	tm.tm_mday = BUILD_DAY;           /* 1..31 */
	tm.tm_hour = BUILD_HOUR;
	tm.tm_min  = BUILD_MIN;
	tm.tm_sec  = BUILD_SEC;
	tm.tm_isdst = -1; /* unknown / don't apply DST */
	return tm;
}

#endif //PICO_EXAMPLES_BUILD_TIME_H
