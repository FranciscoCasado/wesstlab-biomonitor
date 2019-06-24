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

/* Application specific Header files */
#include "config.h"

/*
The values have been generated
with RF studio and are used to overwrite the CMD_FS
from smartrf_settings.c
*/
const FrequencyTableEntry config_frequencyTable[] =
{
    { "868.0", 0x0364, 0x0000 },
    { "915.0", 0x0393, 0x0000 },
};

const char* const config_opmodeLabels[] =
{
    "Rx",
    "Tx"
};

const uint32_t config_packetCountTable[] =
{
    10,
    100,
    1000,
    10000
};

const char* const config_rfSetupLabels[] =
{
    "2-GFSK ",
    "LR Mode",
    "OOK    ",
    "HS Mode"
};

const uint8_t config_NrOfFrequencies = (sizeof(config_frequencyTable)/sizeof(FrequencyTableEntry));
const uint8_t config_NrOfPacketCounts = (sizeof(config_packetCountTable)/sizeof(uint32_t));
