/*******************************************************************************
Copyright � 2014, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED. 
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/
/*
 * $Date: 2015-06-11 12:21:44 +0200 (Thu, 11 Jun 2015) $
 * $Revision: 2396 $
 */
 
/**
 * @file vl6180x_range_interrupt.c
 *
 * @brief alternate range and als polling
 */

#include "vl6180x_api.h"
#include "vl6180x_sample_plat.h" /* contain all device/platform specific code */

VL6180xDev_t theVL6180xDev;        // global unique device in uses

void Sample_Interrupt(void) {

    MyDev_Init(theVL6180xDev); // your code to create/init your device specific struct ptr
    MyDev_SetChipEnable(theVL6180xDev); // your code to enable vl6180x device chip enable
    MyDev_uSleep(1000);             // your code to sleep 2ms to let device boot

    VL6180x_InitData(theVL6180xDev);
    VL6180x_FilterSetState(theVL6180xDev, 0); //disable filering as not effective in continuous mode

    VL6180x_Prepare(theVL6180xDev);     // default vl6180x init

		/*
			Upscale support - Extended range
			* Upscale factor = 1, VL6180 measures distances up to 20 cm with a granularity of 1 mm.
			* Upscale factor = 2, VL6180 measures distances up to 40 cm with a granularity of 2 mm.
			* Upscale factor = 3, VL6180 measures distances up to 60 cm with a granularity of 3 mm.
		*/
    VL6180x_UpscaleSetScaling(theVL6180xDev, 1);
		VL6180x_SetGroupParamHold(theVL6180xDev, 1);
    // if slow reaction is enough then set a high time like 100 ms (up to 2550 msec)
    // if fastest reaction is required then set 0  that will set minimal possible
    VL6180x_RangeSetInterMeasPeriod(theVL6180xDev, 50);

    // we don't care of high threshold as we don't use it , group hold is managed externaly
    VL6180x_RangeSetThresholds(theVL6180xDev, 100, 00, 0);

    VL6180x_ClearAllInterrupt(theVL6180xDev);
    VL6180x_RangeStartContinuousMode(theVL6180xDev);

    VL6180x_SetGroupParamHold(theVL6180xDev, 0);
    VL6180x_RangeClearInterrupt(theVL6180xDev); // clear flags
}
