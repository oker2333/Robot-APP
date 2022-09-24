#ifndef PRINT_H_
#define PRINT_H_

extern char* PowerOn_Time(void);

#define print_info(format,...) printf("%s"format,PowerOn_Time(),##__VA_ARGS__)

void print_logs(void);

#endif
