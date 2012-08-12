/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#include "stm32f10x.h"
#include "ch.h"
#include "hal.h"


#include "fc_ext.h"
#include "fc_spi.h"
#include "fc_threads.h"
#include "fc_nrf24l01.h"

//#include <stdio.h>
#include <string.h>
//#include <math.h>

/*
 * Dummy abort symbol, need for newlib
 * Added this to support the official ARM toolchain
 * https://launchpad.net/gcc-arm-embedded
 */
void abort(void){while(1==1){}}

/*
 * Toggle LED 2
 */
static void ledtoggle2(void){
	  palTogglePad(IOPORT2, GPIOB_LED2);
}


/*
 * NRF24L01 test
 */
void NRFtest(void)
{
	uint8_t data_out[33];

	/*
	 * Test sending packet
	 */
	memcpy(data_out, "Test 123\r\n", strlen("Test 123\r\n"));
	NRFSendData(data_out);

	/*
	 * Sleep and show we are still alive
	 */
	ledtoggle2();
	chThdSleepMilliseconds(100);
}

/*
 * Application entry point.
 */
int main(void) {

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	/*
	 * Initialize peripherals
	 */
	EXTInit();
	SPIInit();

	/*
	 * Device initialization
	 */
	NRFInit();

	/*
	 * Starting threads
	 */
	startThreads();


	/*
	 * Normal main() thread activity, in this demo it does nothing.
	 */
	while (TRUE) {
		/*
		 * Run NRF24L01 test
		 */
		NRFtest();
	}

	// return 0;
}
