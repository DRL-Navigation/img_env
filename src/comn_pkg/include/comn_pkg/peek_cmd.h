/* Macros below are used for peeking commands at console.
Add other macros as you want.
*/

#ifndef __PEEK_CMD__
#define __PEEK_CMD__

#include <stdlib.h>
#include <cstdio>

#define PEEK_CMD(cmd, s) \
        (strcmp(cmd, s) == 0)
#define PEEK_CMD_N(cmd, s, n) \
        (strncmp(cmd, s, n) == 0)
#define SCAN_CMD_D(cmd, d) \
        (sscanf(cmd, "%d", &d))
#define SCAN_CMD_DD(cmd, d1, d2) \
        (sscanf(cmd, "%d %d", &d1, &d2))
#define SCAN_CMD_DF(cmd, d, f) \
    (sscanf(cmd, "%d %f", &d, &f))
#define SCAN_CMD_DDF(cmd, d1, d2, f) \
        (sscanf(cmd, "%d %d %f", &d1, &d2, &f))
#define PEEK_CMD_FF(cmd, s, n, f1, f2) \
        (strncmp(cmd,s,n) == 0 && sscanf(cmd + n, "%f %f", &f1, &f2) == 2)
#define PEEK_CMD_FFF(cmd, s, n, f1, f2, f3) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f", &f1, &f2, &f3) == 3)
#define PEEK_CMD_D(cmd, s, n, d) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d", &d) == 1)
#define PEEK_CMD_DD(cmd, s, n, d1, d2)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d", &d1, &d2) == 2)
#define PEEK_CMD_DDD(cmd, s, n, d1, d2, d3)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d %d", &d1, &d2, &d3) == 3)
#define PEEK_CMD_DDDD(cmd, s, n, d1, d2, d3, d4)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d %d %d", &d1, &d2, &d3, &d4) == 4)
#define PEEK_CMD_DDDDD(cmd, s, n, d1, d2, d3, d4, d5)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d %d %d %d", &d1, &d2, &d3, &d4, &d5) == 5)
#define PEEK_CMD_FFFD(cmd, s, n, d1, d2, d3, d)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %d", &d1, &d2, &d3, &d) == 4)
#define PEEK_CMD_FFFF(cmd, s, n, d1, d2, d3, d4)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %f", &d1, &d2, &d3, &d4) == 4)
#define PEEK_CMD_FFFFF(cmd, s, n, d1, d2, d3, d4, d5)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %f %f", &d1, &d2, &d3, &d4, &d5) == 5)
#define PEEK_CMD_FFFFD(cmd, s, n, d1, d2, d3, d4, d5)\
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %f %d", &d1, &d2, &d3, &d4, &d5) == 5)
#define PEEK_CMD_S(cmd, s, n, str) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s", str) == 1)
#define PEEK_CMD_SS(cmd, s, n, str1, str2) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s %s", str1, str2) == 2)
#define PEEK_CMD_SD(cmd, s, n, str1, d) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s %d", str1, &d) == 2)
#define PEEK_CMD_SDD(cmd, s, n, str1, d1, d2) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s %d %d", str1, &d1, &d2) == 3)
#define PEEK_CMD_DS(cmd, s, n, d, str) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %s", &d, str) == 2)
#define PEEK_CMD_FS(cmd, s, n, f, str) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %s", &f, str) == 2)
#define PEEK_CMD_F(cmd, s, n, f) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f", &f) == 1)

#endif

