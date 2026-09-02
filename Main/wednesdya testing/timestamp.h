#pragma once

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

static inline void get_timestamp(char *buf, size_t len)
{
    struct timeval tv;
    struct tm timeinfo = {0};

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &timeinfo);

    if (timeinfo.tm_year > (2020 - 1900)) {
        char base[24];
        strftime(base, sizeof(base), "%Y-%m-%dT%H:%M:%S", &timeinfo);
        snprintf(buf, len, "%s.%03ld", base, tv.tv_usec / 1000);
    } else {
        snprintf(buf, len, "unknown");
    }
}