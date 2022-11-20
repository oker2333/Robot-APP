#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "console.h"
#include "motor.h"

uint8_t console(uint8_t argc, char **argv)
{
    if (argc == 1)
    {
        goto help;
    }
    else if (argc == 2)
		{
		   if (strcmp("-h", argv[1]) == 0)
			 {
				  help:
				    printf("robot -h\n\tshow help.\n");
				    printf("robot -v velocity_l velocity_r.\n\tcontrol robot motion.\n");
            printf("robot -m -i\n\tshow mpu6050 chip and driver information.\n");
            printf("robot -m -p\n\tshow mpu6050 pin connections of the current board.\n");
            printf("robot -m -t reg -a (0 | 1)\n\trun mpu6050 register test.\n");
            printf("robot -m -t read <times> -a (0 | 1)\n\trun mpu6050 read test.times means the test times.\n");
            printf("robot -m -t fifo <times> -a (0 | 1)\n\trun mpu6050 fifo test.times means the test times.\n");
            printf("robot -m -t dmp <times> -a (0 | 1)\n\trun mpu6050 dmp test.times means the test times.\n");
            printf("robot -m -t motion -a (0 | 1)\n\trun mpu6050 motion test.\n");
            printf("robot -m -t pedometer <times> -a (0 | 1)\n\trun mpu6050 pedometer test.times means the test times.\n");
            printf("robot -m -c read <times> -a (0 | 1)\n\trun mpu6050 read function.times means the read times.\n");
            printf("robot -m -c fifo <times> -a (0 | 1)\n\trun mpu6050 fifo function. times means the read times.\n");
            printf("robot -m -c dmp <times> -a (0 | 1)\n\trun mpu6050 dmp function.times means the read times.\n");
            printf("robot -m -c motion -a (0 | 1)\n\trun mpu6050 motion function.\n");
            printf("robot -m -c pedometer <times> -a (0 | 1)\n\trun mpu6050 pedometer function.times means the read times.\n");
			 }
		}
		else if(argc == 4)
		{
		    if (strcmp("-v", argv[1]) == 0)
				{
					  int left_target = atoi(argv[2]);
					  int right_target = atoi(argv[3]);
					  
				    pid_motor_control(left_target,right_target);
				}
		}
}
