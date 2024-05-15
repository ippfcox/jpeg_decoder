#include <stdio.h>
#include <time.h>

// #define log_(fmt, ...)                                                                            \
//     do                                                                                            \
//     {                                                                                             \
//         time_t t = time(NULL);                                                                    \
//         char ts[32] = {0};                                                                        \
//         strftime(ts, 32, "%Y-%m-%d %H:%M:%S", localtime(&t));                                     \
//         fprintf(stderr, "[%s] %s:%d %s() " fmt, ts, __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
//     }                                                                                             \
//     while (0)

#define log_(fmt, ...)                       \
    do                                       \
    {                                        \
        fprintf(stderr, fmt, ##__VA_ARGS__); \
    }                                        \
    while (0)
