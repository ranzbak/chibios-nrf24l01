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

/*
 * Define IRQ an CE PORT/PAD
 * Make sure to setup the IRQ pad with the EXT driver as well
 */
#define NRF_PORT_CE_IRQ	IOPORT3
#define NRF_PORT_IRQ		4
#define NRF_PORT_CE			5

/*
 * Initialize the NRF24L01 Wifi chip.
 */
void NRFInit(void);

/*
 * Set channel 
 * Frequency is F0= 2400 + RF_CH [MHz]
 */
void NRFSetChannel(uint8_t chan);

/*
 * Handle the IRQ signal, unlock the Semaphores or set flags. 
 */
void NRFHandleIrq(void);

/*
 * NRF24L01 test
 */
void NRFtest(void);

/*
 * Function to send data
 * This functions blocks until data is available
 * The send output buffer needs to be NRF_FIFO_BYTES(32) bytes wide
 */
void NRFSendData(uint8_t *inBuf);

/*
 * Routine to unlock IRQ handling.
 */
void NRFReportIRQ(void);

/*
 * Function to receive data from FIFO
 * This functions blocks until data is available
 * The output buffer needs to be NRF_FIFO_BYTES(32) bytes wide
 */
void NRFReceiveData(uint8_t *pipeNr, uint8_t *outBuf);
