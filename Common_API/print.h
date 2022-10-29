#ifndef PRINT_H_
#define PRINT_H_

#include <stdio.h>
#include "app_config.h"

#define print_info(format,...) printf("%s"format,PowerOn_Time(),##__VA_ARGS__)

void print_logs(void);

#endif
