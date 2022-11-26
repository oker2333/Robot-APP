#ifndef PRINT_H_
#define PRINT_H_

#include <stdio.h>
#include "app_config.h"

#define print_info(format,...) robot_print("%s"format,PowerOn_Time(),##__VA_ARGS__)
int robot_print(const char *const fmt, ...);

void print_logs(void);

#endif
