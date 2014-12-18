/*
 * Copyright (c) <YEAR(S)>, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Firmware for running a VelociRoACH robot.
 *
 * by Andrew Pullin, Duncan Haldane, Austin Buchan, et al 
 *
 * v.0.2
 *
 * Revisions:
 *  dhaldance     ?/?/2014      Initial version brought into mainstream
 *
 * Notes:
 *  
 * Usage:
 *  
 */

// XC compiler include
#include <xc.h>

// imageproc-lib includes
#include "utils.h"          // CSP and BSP
#include "init_default.h"   // port and clocks
#include "sclock.h"         // microsecond timer

// Project settings
#include "settings.h"

// module includes
#include "radio.h"
#include "tih.h"
#include "ams-enc.h"
#include "dfmem.h"
#include "telem.h"
#include "mpu6000.h"
#include "spi_controller.h"
#include "pid-ip2.5.h"
#include "adc_pid.h"
#include "cmd.h"
#include "uart_driver.h"
#include "ppool.h"
#include "carray.h"

// Transional includes ; therese are expected to be changed or deprecated
#include "tests.h"

#include <stdlib.h>

static Payload rx_payload;
static MacPacket rx_packet;
static test_function rx_function;

volatile MacPacket uart_tx_packet;
volatile unsigned char uart_tx_flag;

volatile CircArray fun_queue;

int main() {

    // Processor Initialization
    SetupClock();
    SwitchClocks();
    SetupPorts();
    sclockSetup();

    LED_1 = 1;
    LED_2 = 1;
    LED_3 = 1;

    // Message Passing
    fun_queue = carrayCreate(FUN_Q_LEN);
    cmdSetup();

    // Radio setup
    radioInit(RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcAddr(RADIO_SRC_ADDR);
    radioSetSrcPanID(RADIO_PAN_ID);

    //TODO: Move to UART module, or UART init function.
    uart_tx_packet = NULL;
    uart_tx_flag = 0;
    uartInit(&cmdPushFunc);

    // Need delay for encoders to be ready
    delay_ms(100);
    amsEncoderSetup();
    mpuSetup();
    tiHSetup();
    dfmemSetup();
    telemSetup();
    adcSetup();
    pidSetup();


    LED_1 = 0;
    LED_3 = 1;
    while(1){
        // Send outgoing radio packets
        radioProcess();

        //TODO: Everything below this line needs cleanup (apullin, abuchan)
        // Send outgoing uart packets
//        if(uart_tx_flag) {
//            uartSendPacket(uart_tx_packet);
//            uart_tx_flag = 0;
//        }


        // move received packets to function queue
        while (!radioRxQueueEmpty()) {
            // Check for unprocessed packet
            rx_packet = radioDequeueRxPacket();
            if(rx_packet != NULL) {
                cmdPushFunc(rx_packet);
            }
        }

        // process commands from function queue
        while(!carrayIsEmpty(fun_queue)) {
            rx_packet = carrayPopHead(fun_queue);
            if(rx_packet != NULL) {
               rx_payload = macGetPayload(rx_packet);
               if(rx_payload != NULL) {
                   rx_function = (test_function)(rx_payload->test);
                   if(rx_function != NULL) {
                       LED_2 = ~LED_2;
                       (rx_function)(payGetType(rx_payload), payGetStatus(rx_payload), payGetDataLength(rx_payload), payGetData(rx_payload));
                   }
               }
               ppoolReturnFullPacket(rx_packet);
            }
        }
    }
    return 0;
}
