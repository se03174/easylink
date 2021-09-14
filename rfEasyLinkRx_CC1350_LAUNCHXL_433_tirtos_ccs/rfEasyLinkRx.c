/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== rfEasyLinkRx.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"


#define RFEASYLINKRX_TASK_STACK_SIZE    1024
#define RFEASYLINKRX_TASK_PRIORITY      2

#define TX_BOTTON_ON                    1

EasyLink_RxPacket rxPacket = {0};

/* clock driver handle */
static Clock_Struct clk_Struct;
static Clock_Handle rxLedtimeoutClockHandle;
static Clock_Params clkParams;

static uint8_t nodeRfChannel = 20;
static uint8_t nodeAddress[2] = {0, 230};
static uint8_t concId = 230;


const float rf_channel[] =
{
 0,
 /* 1 ~ 4 */
 447.2625, 447.2750, 447.2875, 447.3000,
 /* 5 ~ 12 */
 447.3125, 447.3250, 447.3375, 447.3500, 447.3625, 447.3750, 447.3875, 447.4000,
 /* 13 ~ 20 */
 447.4125, 447.4250, 447.4375, 447.4500, 447.4625, 447.4750, 447.4875, 447.5000,
 /* 21 ~ 25 */
 447.5125, 447.5250, 447.5375, 447.5500, 447.5625,
};

/* Pin driver handle */
static PIN_Handle rxledPinHandle;
static PIN_State rxledPinState;


PIN_Config ledpinTable[] = {
      IOID_21 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      IOID_22 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      PIN_TERMINATE
};


/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RFEASYLINKRX_TASK_STACK_SIZE];

static Void rxledclkFxn(UArg arg0);
static Void rfEasyLinkRxFnx(UArg arg0, UArg arg1);
static Void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status);


void rxTask_init(void) {

    PIN_init(ledpinTable);

    /* Open LED pins */
    rxledPinHandle = PIN_open(&rxledPinState, ledpinTable);



    /* clock init */
    Clock_Params_init(&clkParams);

    clkParams.period = 1000*1000/Clock_tickPeriod; //1ms
    clkParams.startFlag = false;

    /* Construct a periodic Clock Instance */
    Clock_construct(&clk_Struct, (Clock_FuncPtr)rxledclkFxn,
                    1000*1000/Clock_tickPeriod, &clkParams);

    rxLedtimeoutClockHandle = Clock_handle(&clk_Struct);

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RFEASYLINKRX_TASK_STACK_SIZE;
    rxTaskParams.priority = RFEASYLINKRX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;

    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions */
    Board_initGeneral();

    rxTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
    // Initialize the EasyLink parameters to their default values
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);


    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_init failed");
    }

    EasyLink_setFrequency( rf_channel[nodeRfChannel] * 1000000 );

    /* Set the filter to the generated random address */
    EasyLink_enableRxAddrFilter(nodeAddress, 1, 2);



    while(1)
    {

        EasyLink_Status result = EasyLink_receive(&rxPacket);
        rxDoneCb(&rxPacket , result);



//        /* Wait for an event */
//        uint32_t events = Event_pend(rxtransmitEventHandle, 0, COMM_EVENT_SAME, BIOS_WAIT_FOREVER);
//       if (EasyLink_enableRxAddrFilter(nodeAddress, 1, 2) != EasyLink_Status_Success)
//           {
//             System_abort("EasyLink_enableRxAddrFilter failed");
//           }

//       if(rxPacket.dstAddr[1] == concId)
//       {
           if( rxPacket.payload[2] == TX_BOTTON_ON )
           {
               Clock_start(rxLedtimeoutClockHandle);
               //printf("button on \n");

           }
//
//       }


    }
}

/***** Function definitions *****/
void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        printf("success \n");
       // Clock_start(rxLedtimeoutClockHandle);
    }
    else if(status == EasyLink_Status_Aborted)
    {
        printf("aborted \n");

    }
    else if(status == EasyLink_Status_Busy_Error)
    {
        printf("busy error \n");
    }
    else
    {
        printf("error \n");
    }

}

/* ======= clkFxn =======*/
void rxledclkFxn(UArg arg0)
{

    PIN_setOutputValue(rxledPinHandle, IOID_21,!PIN_getOutputValue(IOID_21));
    PIN_setOutputValue(rxledPinHandle, IOID_22,!PIN_getOutputValue(IOID_22));
    //printf("success_clk \n");

}
