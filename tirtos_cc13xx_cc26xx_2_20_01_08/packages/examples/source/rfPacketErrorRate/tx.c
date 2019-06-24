/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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


/***** Includes *****/

/* XDCtools Header files */
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* Application specific Header files */
#include <stdlib.h>
#include "menu.h"
#include "RFQueue.h"
#include "rf_application_settings.h"


/***** Defines *****/
#define PAYLOAD_LENGTH          30  /* Length of the packet to send */
#define DATA_ENTRY_HEADER_SIZE  8   /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH              125 /* Set the length of the data entry */
#define NUM_DATA_ENTRIES        1
#define NUM_APPENDED_BYTES      0

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;
uint32_t packetCounter = 0;
uint32_t packetInterval;

#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (txDataEntryBuffer, 4);
    static uint8_t txDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
    static uint8_t txDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                     MAX_LENGTH,
                                                                     NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
   static uint8_t txDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
            MAX_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif

/* Tx queue or RF Core to read data from */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t *pPacket;


/* Runs the receiving part of the test application and returns a result. */
TestResult tx_runTxTest(const ApplicationConfig* config)
{
    Assert_isTrue(config != NULL, NULL);
    uint32_t time;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    if( RFQueue_defineQueue(&dataQueue,
                            txDataEntryBuffer,
                            sizeof(txDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(true);
    }

    RF_cmdTxHS.pQueue = &dataQueue;
    RF_cmdTxHS.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdTxHS.startTrigger.pastTrig = 1;
    RF_cmdTxHS.startTime = 0;

    currentDataEntry = (rfc_dataEntryGeneral_t*)&txDataEntryBuffer;
    currentDataEntry->length = PAYLOAD_LENGTH;
    pPacket = &currentDataEntry->data;

    /* Request access to the radio based on test case*/
    switch (config->rfSetup)
    {
        case RfSetup_Fsk:

            RF_cmdPropRadioDivSetup_fsk.centerFreq = config_frequencyTable[config->frequency].frequency;
            rfHandle = RF_open(&rfObject, &RF_prop_fsk, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_fsk, &rfParams);
            packetInterval = (uint32_t)(4000000*0.05f); /* Set packet interval to 50 ms */

            break;

        case RfSetup_Lrm:

            RF_cmdPropRadioDivSetup_lrm.centerFreq = config_frequencyTable[config->frequency].frequency;
            rfHandle = RF_open(&rfObject, &RF_prop_lrm, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_lrm, &rfParams);
            packetInterval = (uint32_t)(4000000*0.6f); /* Set packet interval to 600 ms */
            break;

        case RfSetup_Ook:

            RF_cmdPropRadioDivSetup_ook.centerFreq = config_frequencyTable[config->frequency].frequency;
             rfHandle = RF_open(&rfObject, &RF_prop_ook, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_ook, &rfParams);
            packetInterval = (uint32_t)(4000000*0.1f); /* Set packet interval to 100 ms */
            break;

        case RfSetup_Hsm:

            rfHandle = RF_open(&rfObject, &RF_prop_hsm, (RF_RadioSetup*)&RF_cmdRadioSetup_hsm, &rfParams);
            packetInterval = (uint32_t)(4000000*0.05f); /* Set packet interval to 10 ms */
            break;

        default:

            break;
    }

    /* Set the frequency */
    RF_cmdFs.frequency = config_frequencyTable[config->frequency].frequency;
    RF_cmdFs.fractFreq = config_frequencyTable[config->frequency].fractFreq;
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */
    time = RF_getCurrentTime();
    while (++packetCounter <= config->packetCount)
    {
        /* Create packet with incrementing sequence number and random payload */
        pPacket[0] = packet[0] = (uint8_t)(seqNumber >> 8);
        pPacket[1] = packet[1] = (uint8_t)(seqNumber++);
        uint8_t i;
        for (i = 2; i < PAYLOAD_LENGTH; i++)
        {
            pPacket[i] = packet[i] = rand();
        }

        /* Set absolute TX time to utilize automatic power management */
        time += packetInterval;
        RF_cmdPropTx.startTime = time;
        RF_cmdTxHS.startTime = time;

        /* Send packet */
        switch (config->rfSetup)
        {
            case RfSetup_Hsm:
                {
                    RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdTxHS, RF_PriorityNormal, NULL, 0);

                    if (result != RF_EventLastCmdDone)
                    {
                        /* Error */
                        while(true);
                    }
                    break;
                }

            default:

                {
                    RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
                    if (!(result & RF_EventLastCmdDone))
                    {
                        /* Error */
                        while(true);
                    }
                    break;
                }
        }

        /* Update the display */
        menu_updateTxScreen(packetCounter);

        /* Check, whether a button has been pressed */
        if (menu_isButtonPressed())
        {
            packetCounter = 0;
            seqNumber = 0;
            RF_close(rfHandle);
            return TestResult_Aborted;
        }
    }

    packetCounter = 0;
    seqNumber = 0;
    RF_close(rfHandle);
    return TestResult_Finished;
}
