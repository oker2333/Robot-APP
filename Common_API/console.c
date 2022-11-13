#include <string.h>
#include <stdio.h>

#include "console.h"

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
						printf("robot -h\n\tshow robot help.\n");
			 }
		}
}
