/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

/** ============================================================================
 *  @file       SensorIcm42670p.h
 *
 *  @brief      Driver for the InvenSense Icm42670p Motion Processing Unit.
 *
 *  ============================================================================
 */
#ifndef SENSOR_Icm42670p_H
#define SENSOR_Icm42670p_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                          Includes
 * -----------------------------------------------------------------------------
 */
#include "stdint.h"
#include "stdbool.h"

/* -----------------------------------------------------------------------------
 *                                          Constants
 * -----------------------------------------------------------------------------
 */
// Accelerometer ranges
#define ACC_RANGE_2G      0
#define ACC_RANGE_4G      1
#define ACC_RANGE_8G      2
#define ACC_RANGE_16G     3
#define ACC_RANGE_INVALID 0xFF

// Axis bitmaps
#define MPU_AX_GYR        0x07
#define MPU_AX_ACC        0x38
#define MPU_AX_MAG        0x40
#define MPU_AX_ALL        0x7F

// Interrupt status bit
#define MPU_DATA_READY    0x01
#define MPU_MOVEMENT      0x40

// Magnetometer status
#define MAG_STATUS_OK     0x00
#define MAG_READ_ST_ERR   0x01
#define MAG_DATA_NOT_RDY  0x02
#define MAG_OVERFLOW      0x03
#define MAG_READ_DATA_ERR 0x04
#define MAG_BYPASS_FAIL   0x05
#define MAG_NO_POWER      0x06

/* ----------------------------------------------------------------------------
 *                                           Typedefs
 * -----------------------------------------------------------------------------
*/
typedef void (*SensorIcm42670pCallbackFn_t)(void);

/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */


bool SensorIcm42670p_MPU_frqconfig(uint8_t frq);
bool SensorIcm42670p_Acel_frqconfig(uint8_t frq);
bool SensorIcm42670p_Gyro_frqconfig(uint8_t frq);



bool SensorIcm42670p_init(void);
bool SensorIcm42670p_reset(void);
void SensorIcm42670p_registerCallback(SensorIcm42670pCallbackFn_t);
bool SensorIcm42670p_test(void);
void SensorIcm42670p_powerOn(void);
void SensorIcm42670p_powerOff(void);
bool SensorIcm42670p_powerIsOn(void);

void SensorIcm42670p_enable(uint16_t config);
bool SensorIcm42670p_enableWom(uint8_t threshold);
uint8_t SensorIcm42670p_irqStatus(void);

bool SensorIcm42670p_accSetRange(uint8_t range);
uint8_t SensorIcm42670p_accReadRange(void);
bool SensorIcm42670p_accRead(uint16_t *rawData);
float SensorIcm42670p_accConvert(int16_t rawValue);

bool SensorIcm42670p_gyroRead(uint16_t *rawData);
float SensorIcm42670p_gyroConvert(int16_t rawValue);

/*******************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif
