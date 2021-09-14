/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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
 *  ======== rfEasyLinkTx.c ========
 */
 /* Standard C Libraries */
#include <stdlib.h>
#include <rfEasyLinkTx.h>

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
#include <ti/drivers/GPIO.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/* Application header files */
#include "smartrf_settings/smartrf_settings.h"

/* Undefine to not use async mode */
#define RFEASYLINKTX_TASK_STACK_SIZE    1024
#define RFEASYLINKTX_TASK_PRIORITY      2

#define RFEASYLINKTXPAYLOAD_LENGTH      30


static uint8_t nodeRfChannel = 20;
static uint8_t nodeAddress[2] = {0,1 };
static uint8_t concId = 230;
uint8_t buttonState = 0;

/* clock driver handle */
static Clock_Struct clk_Struct;
static Clock_Handle txLedtimeoutClockHandle;
static Clock_Params clkParams;

EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };
EasyLink_Status result;

static Void txledclkFxn(UArg arg0);
static Void rfEasyLinkTxFnx(void);
static Void txDoneCb(EasyLink_Status status);
void gpioButtonFxn0(uint_least16_t index);

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

Task_Struct txTask;    /* not static so you can see in ROV */
static Task_Params txTaskParams;
static uint8_t txTaskStack[RFEASYLINKTX_TASK_STACK_SIZE];

Event_Struct TxTimeoutEvent;
static Event_Handle TxTimeoutEventHandle;

/* Pin driver handle */
static PIN_Handle pinHandle;
static PIN_State pinState ;
//buttonHandle; buttonState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

////button table
//static PIN_Config buttonPinTable[] =
//{
//     Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP  | PIN_IRQ_NEGEDGE,
//     PIN_TERMINATE
//};


void txTask_init(void) {

    GPIO_init();
    /* button init */
   // GPIO_setConfig(Board_GPIO_BUTTON0, PIN_INPUT_EN | GPIO_CFG_IN_INT_FALLING | PIN_PULLUP );
    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

    GPIO_enableInt(Board_GPIO_BUTTON0);

   // PIN_setConfig(handle, updateMask, pinCfg)
   // PIN_registerIntCb(pinHandle, callbackFxn)


   // buttonHandle =  PIN_open(&buttonState, buttonPinTable );

    /* install button callback */
    //PIN_registerIntCb(buttonHandle, &gpioButtonFxn0);
   // PIN_setInterrupt(buttonHandle, Board_PIN_BUTTON0 | PIN_IRQ_NEGEDGE);


    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);


    /* Enable interrupts */



    /* clock init */
    Clock_Params_init(&clkParams);

    clkParams.period = 1000*1000/Clock_tickPeriod; //1ms
    clkParams.startFlag = false;

    /* Construct a periodic Clock Instance */
    Clock_construct(&clk_Struct, (Clock_FuncPtr)txledclkFxn,
                    1000*1000/Clock_tickPeriod, &clkParams);

    txLedtimeoutClockHandle = Clock_handle(&clk_Struct);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&TxTimeoutEvent, &eventParam);
    TxTimeoutEventHandle = Event_handle(&TxTimeoutEvent);

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = RFEASYLINKTX_TASK_STACK_SIZE;
    txTaskParams.priority = RFEASYLINKTX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;

    Task_construct(&txTask, rfEasyLinkTxFnx, &txTaskParams, NULL);

}


void rfEasyLinkTxFnx(void)
{

    // Initialize the EasyLink parameters to their default values
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);


    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_init failed");
    }

    EasyLink_setFrequency( rf_channel[nodeRfChannel] * 1000000 );


    while(1)
    {

        /* Wait for an event */
        uint32_t events = Event_pend(TxTimeoutEventHandle, 0, COMM_EVENT_ALL, BIOS_WAIT_FOREVER);

        if( events & COMM_BOUTTON_ON)
        {
            buttonState = 1;
            txPacket.payload[2] = buttonState;
            EasyLink_transmit(&txPacket);
        }


        if( events & COMM_TX_TIMEOUT )
        {

            txPacket.dstAddr[0] = concId;
            txPacket.payload[0] = nodeAddress[1];
            txPacket.payload[1] = nodeRfChannel;
            txPacket.payload[2] = buttonState;
            //txPacket.payload[2] = GPIO_read(Board_GPIO_BUTTON0);
            uint8_t i;
            for(i = 2; i < RFEASYLINKTXPAYLOAD_LENGTH ; i++)
            txPacket.payload[1+i] = rand();

            txPacket.len = RFEASYLINKTXPAYLOAD_LENGTH;

            result = EasyLink_transmit(&txPacket);

            txDoneCb(result);


            buttonState = 0;

        }

    }
}
void txDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
      printf("EasyLink_Status_Success \n");


    }
    else if(status == EasyLink_Status_Aborted)
    {
        printf("EasyLink_Status_Aborted \n");
    }
    else
    {
       printf("EasyLink_Status_Error \n");
    }
}

/* === button cb === */

void gpioButtonFxn0(uint_least16_t index)
{
    CPUdelay(300 * 1000 * 48 / 4);      // delay 300ms debouncing

    Event_post(TxTimeoutEventHandle, COMM_BOUTTON_ON );

    Clock_start(txLedtimeoutClockHandle);


}


void txledclkFxn(UArg arg0)
{
    //led1 toggle
    PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
}

void rfEasyLinkTx_postEvent(uint32_t event)
{
    Event_post(TxTimeoutEventHandle, event);

}
