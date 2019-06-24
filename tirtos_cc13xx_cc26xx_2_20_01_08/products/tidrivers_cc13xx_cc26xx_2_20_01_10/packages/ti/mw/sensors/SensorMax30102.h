/** ============================================================================
 *  @file       SensorMax30102.c
 *
 *  @brief      Driver for the Maxim Max30102 Hr and SpO2 Sensor.
 *  ============================================================================
 */

#ifndef SENSOR_Max30102_H
#define SENSOR_Max30102_H

#ifdef __cplusplus
extern "C"
{
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

// Device addresses
#define _dev_addr 0x57
#define _WRITE_ADDR 0xAE
#define _READ_ADDR 0xAF

//register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03

#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_LED3_PA 0x0E
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

#define MAX30102_OK 0x00
#define MAX30102_ERROR 0x01

#define MAX30102_NO_ERROR 0x00
#define MAX30102_READ_ERROR 0x01

/* ----------------------------------------------------------------------------
 *                                           Typedefs
 * -----------------------------------------------------------------------------
*/
typedef void (*SensorMax30102CallbackFn_t)(void);

/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */
bool SensorMax30102_init(void);

bool SensorMax30102_readReg(uint8_t, int num, uint8_t*);
bool SensorMax30102_writeReg(uint8_t,uint8_t);
bool SensorMax30102_writeCheck(uint8_t ,uint8_t);


bool SensorMax30102_readFifo(uint8_t *);


bool SensorMax30102_reset();
bool SensorMax30102_enable(bool);


// Just used in baby monitor
bool SensorMax30102_SampleRate(uint8_t);
bool SensorMax30102_LedMode(uint8_t);
bool SensorMax30102_LedI(uint8_t);



//bool SensorMax30102_read(uint8_t*);
bool SensorMax30102_test(void);

void SensorMax30102_Auxfn(bool);
void SensorMax30102_InterruptEnable(bool);
void SensorMax30102_registerCallback(SensorMax30102CallbackFn_t);

#ifdef __cplusplus
}

#endif

#endif
