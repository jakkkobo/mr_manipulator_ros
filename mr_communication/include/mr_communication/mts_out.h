/*******************************************************************************
 *                                                                             *
 * Author: Matias        Project: MT           Date:23-04-2015     Version:1.0 *
 * Lab: Autonomous Systems Laboratory                         CROB-INESC PORTO *
 *                                                                             *
 *******************************************************************************/

#ifndef MTS_OUT_H
#define MTS_OUT_H

#define MTS_ANSI_COLOR_RED     "\x1b[31m"
#define MTS_ANSI_COLOR_GREEN   "\x1b[32m"
#define MTS_ANSI_COLOR_YELLOW  "\x1b[33m"
#define MTS_ANSI_COLOR_BLUE    "\x1b[34m"
#define MTS_ANSI_COLOR_MAGENTA "\x1b[35m"
#define MTS_ANSI_COLOR_CYAN    "\x1b[36m"
#define MTS_ANSI_COLOR_RESET   "\x1b[0m"

typedef enum
{
    OUT_INFO=0,
    OUT_INFO_RED,
    OUT_INFO_GREEN,
    OUT_INFO_YELLOW,
    OUT_INFO_BLUE,
    OUT_INFO_MAGENTA,
    OUT_INFO_CYAN,
    OUT_WARNING,
    OUT_ERROR,
    OUT_FATAL,
    OUT_INFO_N,
    OUT_INFO_RED_N,
    OUT_INFO_GREEN_N,
    OUT_INFO_YELLOW_N,
    OUT_INFO_BLUE_N,
    OUT_INFO_MAGENTA_N,
    OUT_INFO_CYAN_N,
    OUT_WARNING_N,
    OUT_ERROR_N,
    OUT_FATAL_N,
    OUT_INFO_D,     //DEBUG
    OUT_INFO_RED_D,
    OUT_INFO_GREEN_D,
    OUT_INFO_YELLOW_D,
    OUT_INFO_BLUE_D,
    OUT_INFO_MAGENTA_D,
    OUT_INFO_CYAN_D,
    OUT_INFO_V,     //DEBUG
    OUT_INFO_RED_V,
    OUT_INFO_GREEN_V,
    OUT_INFO_YELLOW_V,
    OUT_INFO_BLUE_V,
    OUT_INFO_MAGENTA_V,
    OUT_INFO_CYAN_V,
}Out_e;

typedef int (*Out)(Out_e type, const char *format, ...);

extern Out OUT;

typedef struct{
    int (*OutInfo)(const char*, ...);
    int (*OuttWarn)(const char*, ...);
    int (*OutError)(const char*, ...);
}Out_t;

int outMts(Out_e type, const char *format, ...);

#endif // MTS_OUT_H
