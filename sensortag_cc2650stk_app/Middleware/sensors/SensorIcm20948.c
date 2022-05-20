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
 *  @file       SensorIcm20948.c
 *
 *  @brief      Driver for the InvenSense ICM20948 Motion Processing Unit.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "SensorIcm20948.h"
#include "SensorOpt3001.h" // For reset of I2C bus
#include "SensorUtil.h"
#include "SensorI2C.h"

/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/
// Registers
#define REG_BANK_SEL                  0x7F // R/W
#define USER_BANK_0                   0x00 // R/W
#define USER_BANK_1                   0x10 // R/W
#define USER_BANK_2                   0x20 // R/W
#define USER_BANK_3                   0x30 // R/W

// USER BANK 0
#define WHO_AM_I                      0x00 // R/W
#define USER_CTRL                     0x03 // R/W
#define PWR_MGMT_1                    0x06 // R/W
#define PWR_MGMT_2                    0x07 // R/W
#define INT_PIN_CFG                   0x0F // R/W
#define INT_ENABLE                    0x10 // R/W
#define INT_ENABLE_1                  0x11 // R/W
#define INT_ENABLE_2                  0x12 // R/W
#define INT_ENABLE_3                  0x13 // R/W
//      INT_MST_STATUS                0x19 // R/C
#define INT_STATUS                    0x19 // R
#define ACCEL_XOUT_H                  0x2D // R
#define ACCEL_XOUT_L                  0x2E // R
#define ACCEL_YOUT_H                  0x2F // R
#define ACCEL_YOUT_L                  0x30 // R
#define ACCEL_ZOUT_H                  0x31 // R
#define ACCEL_ZOUT_L                  0x32 // R
#define GYRO_XOUT_H                   0x33 // R
#define GYRO_XOUT_L                   0x34 // R
#define GYRO_YOUT_H                   0x35 // R
#define GYRO_YOUT_L                   0x36 // R
#define GYRO_ZOUT_H                   0x37 // R
#define GYRO_ZOUT_L                   0x38 // R
#define TEMP_OUT_H                    0x39 // R
#define TEMP_OUT_L                    0x3A // R
#define FIFO_EN_1                     0x66 // R/W
#define FIFO_EN_2                     0x67 // R/W
#define FIFO_RST                      0x68 // R/W
#define FIFO_MODE                     0x69 // R/W
#define FIFO_COUNT_H                  0x70 // R
#define FIFO_COUNT_L                  0x71 // R
#define FIFO_R_W                      0x72 // R/W
//#define SIGNAL_PATH_RESET             0x68 // R/W NOT PRESENT

// USER BANK 1
#define SELF_TEST_X_GYRO              0x02 // R/W
#define SELF_TEST_Y_GYRO              0x03 // R/W
#define SELF_TEST_Z_GYRO              0x04 // R/W
#define SELF_TEST_X_ACCEL             0x0E // R/W
#define SELF_TEST_Z_ACCEL             0x0F // R/W
#define SELF_TEST_Y_ACCEL             0x10 // R/W
#define XA_OFFSET_H                   0x14 // R/W
#define XA_OFFSET_L                   0x15 // R/W
#define YA_OFFSET_H                   0x17 // R/W
#define YA_OFFSET_L                   0x18 // R/W
#define ZA_OFFSET_H                   0x1A // R/W
#define ZA_OFFSET_L                   0x1B // R/W

// USER BANK 2
#define GYRO_SMPLRT_DIV               0x00 // R/W
#define GYRO_CONFIG_1                 0x01 // R/W
#define GYRO_CONFIG_2                 0x02 // R/W
#define XG_OFFSET_H                   0x03 // R/W
#define XG_OFFSET_L                   0x04 // R/W
#define YG_OFFSET_H                   0x05 // R/W
#define YG_OFFSET_L                   0x06 // R/W
#define ZG_OFFSET_H                   0x07 // R/W
#define ZG_OFFSET_L                   0x08 // R/W
#define ACCEL_SMPLRT_DIV_1            0x10 // R/W
#define ACCEL_SMPLRT_DIV_2            0x11 // R/W
#define ACCEL_INTEL_CTRL              0x12 // R/W
#define ACCEL_WOM_THR                 0x13 // R/W
#define ACCEL_CONFIG                  0x14 // R/W
#define ACCEL_CONFIG_2                0x15 // R/W

// #define CONFIG                        0x1A // R/W TODO
// #define LP_ACCEL_ODR                  0x1E // R/W REDEFINED AS ACCEL_SMPLRT
// #define FIFO_EN                       0x23 // R/W TODO


// Masks is mpuConfig variable
#define ACC_CONFIG_MASK               0x38  // --XX X---
#define GYRO_CONFIG_MASK              0x07  // ---- -XXX

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6

// Output data rates DETERMINED BY ACCEL_SMPLRT REGISTERS
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x08
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20

// Magnetometer registers
#define MAG_WHO_AM_I                  0x01  // Should return 0x48
// #define MAG_INFO                      0x01
#define MAG_ST1                       0x10  // Data ready status: bit 0
#define MAG_XOUT_L                    0x11  // Data array
#define MAG_XOUT_H                    0x12
#define MAG_YOUT_L                    0x13
#define MAG_YOUT_H                    0x14
#define MAG_ZOUT_L                    0x15
#define MAG_ZOUT_H                    0x16
#define MAG_ST2                       0x18  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1                     0x31  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2                     0x32  // System reset, bit 0

#define MAG_DEVICE_ID                 0x48

// Mode
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02
#define MAG_MODE_CONT2                0x06
#define MAG_MODE_FUSE                 0x0F

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1,Board_ICM20948_ADDR)
#define SENSOR_SELECT_MAG()           SensorI2C_select(SENSOR_I2C_1,Board_ICM20948_MAG_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()

/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorMpuSleep(void);
static void sensorIcm20948WakeUp(void);
static void sensorIcm20948SelectAxes(void);
static void SensorIcm20948_Callback(PIN_Handle handle, PIN_Id pinId);
static void sensorMagInit(void);
static void sensorMagEnable(bool);
static bool sensorIcm20948SetBypass(void);

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/

//Debugging

uint8_t MSen_d[5];


static uint8_t mpuConfig;
static uint8_t magStatus;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t val;

// Magnetometer calibration
static int16_t calX;
static int16_t calY;
static int16_t calZ;

// Magnetometer control
static uint8_t scale = MFS_16BITS;      // 16 bit resolution
static uint8_t mode = MAG_MODE_SINGLE;  // Operating mode

// Pins that are used by the ICM20948
static PIN_Config MpuPinTable[] =
{
    Board_MPU_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
   // Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};
static PIN_State pinGpioState;
static PIN_Handle hMpuPin;

// The application may register a callback to handle interrupts
static SensorIcm20948CallbackFn_t isrCallbackFn = NULL;

/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/

void SensorIcm20948_selectBank(uint8_t bank){
    val = bank;
    ST_ASSERT(SensorI2C_writeReg(REG_BANK_SEL, &val, 1));
}


bool SensorIcm20948_MPU_frqconfig(uint8_t frq){

    // TODO
    if (!SENSOR_SELECT())
      {
          return false;
      }

    // ** USER BANK 2 **
    SensorIcm20948_selectBank(USER_BANK_2);
    
    // Set SRDiveder to zero
    val = 0x00;
    ST_ASSERT(SensorI2C_writeReg(GYRO_SMPLRT_DIV, &val, 1));
    ST_ASSERT(SensorI2C_writeReg(ACCEL_SMPLRT_DIV_1, &val, 1));
    ST_ASSERT(SensorI2C_writeReg(ACCEL_SMPLRT_DIV_2, &val, 1));

    // Set Accel LPF setting to 92 Hz Bandwidth
    
    val = 0x10;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &val, 1));

    // Set Gyro LPF setting to 92 Hz Bandwidth and FS Sel to 500 DPS
    val = 0x12;
    ST_ASSERT(SensorI2C_writeReg(GYRO_CONFIG_1, &val, 1));
    
    // Enable Accel Hardware Intelligence/ Accel interrupt control-->Register 105
    val = 0x03;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));


    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    
    // Set FIFO MODE to Stream (0x00)
    val = 0x00;
    ST_ASSERT(SensorI2C_writeReg(FIFO_MODE, &val, 1));

    // Make sure accelerometer is running (auto select Clock source) and disable temperature sensor
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Enable accelerometer, and gyro
    val = 0x00;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));

    // Enable raw data ready interrupt from any sensor to propagate to interrupt pin 1
    val = BIT_RAW_RDY_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE_1, &val, 1));

    // Clear interrupt
     SensorI2C_readReg(INT_STATUS,&val,1);

    SENSOR_DESELECT();
    MSen_d[3]++;
    return  true;
}


//bool SensorIcm20948_Acel_frqconfig(uint8_t frq){}

//bool SensorIcm20948_Gyro_frqconfig(uint8_t frq){}


/*******************************************************************************
* @fn          SensorIcm20948_powerOn
*
* @brief       This function turns on the power supply to Icm20948
*
* @return      none
*/
void SensorIcm20948_powerOn(void)
{
    // Turn on power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    DELAY_MS(100);
    SensorIcm20948_reset();
}

/*******************************************************************************
* @fn          SensorIcm20948_powerOff
*
* @brief       This function turns off the power supply to Icm20948
*
* @return      none
*/
void SensorIcm20948_powerOff(void)
{
    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    // Turn off power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

    // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
   // SensorOpt3001_test();
}

/*******************************************************************************
* @fn          SensorIcm20948_powerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool SensorIcm20948_powerIsOn(void)
{
    return PIN_getOutputValue(Board_MPU_POWER) == Board_MPU_POWER_ON;
}

/*******************************************************************************
* @fn          SensorIcm20948_registerCallback
*
* @brief       Register a call-back for interrupt processing
*
* @return      none
*/
void SensorIcm20948_registerCallback(SensorIcm20948CallbackFn_t pfn)
{
    isrCallbackFn = pfn;
}

/*******************************************************************************
* @fn          SensorIcm20948_init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool SensorIcm20948_init(void)
{
    // Pins used by MPU
    hMpuPin = PIN_open(&pinGpioState, MpuPinTable);

    // Register MPU interrupt
    PIN_registerIntCb(hMpuPin, SensorIcm20948_Callback);

    // Application callback initially NULL
    isrCallbackFn = NULL;

    return SensorIcm20948_reset();
}


/*******************************************************************************
* @fn          SensorIcm20948_reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool SensorIcm20948_reset(void)
{
    bool ret;

    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    accRange = ACC_RANGE_INVALID;
    mpuConfig = 0;   // All axes off
    magStatus = 0;

    if (!SENSOR_SELECT())
    {
        return false;
    }
    
    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);

    // Device reset
    val = 0x80;
    SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    SENSOR_DESELECT();

    DELAY_MS(100);

    ret = SensorIcm20948_test();
    if (ret)
    {
        // Initial configuration
        SensorIcm20948_accSetRange(ACC_RANGE_8G);
        sensorMagInit();

        // Power save
        sensorMpuSleep();
    }

    return ret;
}


/*******************************************************************************
* @fn          SensorIcm20948_enableWom
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool SensorIcm20948_enableWom(uint8_t threshold)
{
    // TODO
    ST_ASSERT(SensorIcm20948_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }
    
    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);

    // Make sure accelerometer is running
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Enable accelerometer, disable gyro
    val = 0x07;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));
    
    // Enable Motion Interrupt
    val = BIT_WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));

    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));


    // ** USER BANK 2 **
    SensorIcm20948_selectBank(USER_BANK_2);

    // Set Accel LPF setting to 246 Hz Bandwidth and +-2g Full Scale
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &val, 1));

    // Enable Accel Hardware Intelligence
    val = 0x01;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));

    // Set Motion Threshold
    val = threshold;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_WOM_THR, &val, 1));


    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    
    // Clear interrupt
    SensorI2C_readReg(INT_STATUS,&val,1);

    SENSOR_DESELECT();

    mpuConfig = 0;

    // Enable pin for wake-on-motion interrupt
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

    return true;
}

/*******************************************************************************
* @fn          SensorIcm209480_irqStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t SensorIcm20948_irqStatus(void)
{
    // TODO
    uint8_t intStatus;

    intStatus = 0;
    ST_ASSERT(SensorIcm20948_powerIsOn());

    if (!SENSOR_SELECT()) return intStatus;
    
    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    if (!SensorI2C_readReg(INT_STATUS,&intStatus,1))
    {
        intStatus = 0;
    }
    SENSOR_DESELECT();

    return intStatus;
}

/*******************************************************************************
* @fn          SensorIcm20948_enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
* @                  Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
*                    MPU  bit [6], all axes
*
* @return      None
*/
void SensorIcm20948_enable(uint16_t axes)
{
    ST_ASSERT_V(SensorIcm20948_powerIsOn());

    if (mpuConfig == 0 && axes != 0)
    {
        MSen_d[0]++;
        // Wake up the sensor if it was off
        sensorIcm20948WakeUp();
    }

    mpuConfig = axes;

    if (mpuConfig != 0)
    {   MSen_d[1]++;
        // Enable gyro + accelerometer + magnetometer readout
        sensorIcm20948SelectAxes();
    }
    else if (mpuConfig == 0)
    {
        MSen_d[2]++;
        sensorMpuSleep();
    }
}


/*******************************************************************************
* @fn          SensorIcm20948_accSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool SensorIcm20948_accSetRange(uint8_t newRange)
{
        // TODO
    bool success;

    if (newRange == accRange)
    {
        return true;
    }

    ST_ASSERT(SensorIcm20948_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    accRangeReg = (newRange << 1);
    
    
    // ** USER BANK 2 **
    SensorIcm20948_selectBank(USER_BANK_2);

    // Apply the range
    success = SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    if (success)
    {
        accRange = newRange;
    }

    return success;
}

/*******************************************************************************
* @fn          SensorIcm20948_accReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t SensorIcm20948_accReadRange(void)
{
    // TODO
    ST_ASSERT(SensorIcm20948_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }


    // ** USER BANK 2 **
    SensorIcm20948_selectBank(USER_BANK_2);
    
    // Apply the range
    SensorI2C_readReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    accRange = (accRangeReg>>1) & 3;

    return accRange;
}


/*******************************************************************************
* @fn          SensorIcm20948_accRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool SensorIcm20948_accRead(uint16_t *data )
{
    // TODO
    bool success;
    ST_ASSERT(SensorIcm20948_powerIsOn());

    // Burst read of all accelerometer values
    if (!SENSOR_SELECT())
    {
        return false;
    }
    
    
    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);

    success = SensorI2C_readReg(ACCEL_XOUT_H, (uint8_t*)data, DATA_SIZE);

    SENSOR_DESELECT();

    if (success)
    {
        SensorUtil_convertToLe((uint8_t*)data,DATA_SIZE);
    }

    return success;
}

/*******************************************************************************
* @fn          SensorIcm20948_gyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool SensorIcm20948_gyroRead(uint16_t *data )
{
    // TODO
    bool success;

    ST_ASSERT(SensorIcm20948_powerIsOn());

    // Select this sensor
    if (!SENSOR_SELECT())
    {
        return false;
    }
    
    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);

    // Burst read of all gyroscope values
    success = SensorI2C_readReg(GYRO_XOUT_H, (uint8_t*)data, DATA_SIZE);

    SENSOR_DESELECT();

    if (success)
    {
        SensorUtil_convertToLe((uint8_t*)data,DATA_SIZE);
    }

    return success;
}

/******************************************************
 * *************************
 * @fn          SensorIcm20948_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorIcm20948_test(void)
{
    // TODO
    static bool first = true;

    ST_ASSERT(SensorIcm20948_powerIsOn());

    // Select Gyro/Accelerometer
    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure power is ramped up
    if (first)
    {
        DELAY_MS(100);
        first = false;
    }

    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    
    // Check the WHO AM I register
    ST_ASSERT(SensorI2C_readReg(WHO_AM_I, &val, 1));
    ST_ASSERT(val == 0x71);

    SENSOR_DESELECT();

    return true;
}


/*******************************************************************************
 * @fn          SensorIcm20948_accConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float SensorIcm20948_accConvert(int16_t rawData)
{
    float v;

    switch (accRange)
    {
    case ACC_RANGE_2G:
        //-- calculate acceleration, unit G, range -2, +2
        v = (rawData * 1.0) / (32768/2);
        break;

    case ACC_RANGE_4G:
        //-- calculate acceleration, unit G, range -4, +4
        v = (rawData * 1.0) / (32768/4);
        break;

    case ACC_RANGE_8G:
        //-- calculate acceleration, unit G, range -8, +8
        v = (rawData * 1.0) / (32768/8);
        break;

    case ACC_RANGE_16G:
        //-- calculate acceleration, unit G, range -16, +16
        v = (rawData * 1.0) / (32768/16);
        break;
    }

    return v;
}

/*******************************************************************************
 * @fn          SensorIcm20948_gyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float SensorIcm20948_gyroConvert(int16_t data)
{
    //-- calculate rotation, unit deg/s, range -250, +250
    return (data * 1.0) / (65536 / 500);
}

/*******************************************************************************
* @fn          sensorMpuSleep
*
* @brief       Place the MPU in low power mode
*
* @return
*/
static void sensorMpuSleep(void)
{
    // TODO
    bool success;

    ST_ASSERT_V(SensorIcm20948_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);

    val = ALL_AXES;
    success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
    if (!success) return;
    
    val = MPU_SLEEP;
    success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    

    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorIcm20948WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/
static void sensorIcm20948WakeUp(void)
{
    // TODO
    bool success;

    ST_ASSERT_V(SensorIcm20948_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    
    val = MPU_WAKE_UP;
    success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);

    if (success)
    {
        // All axis initially disabled
        val = ALL_AXES;
        success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
        mpuConfig = 0;
    }

    if (!success) return;
    // ** USER BANK 2 **
    SensorIcm20948_selectBank(USER_BANK_2);
    // Restore the range
    success = SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1);

    if (!success) return;
    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    // Clear interrupts
    success = SensorI2C_readReg(INT_STATUS,&val,1);

    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorIcm20948SelectAxes
*
* @brief       Select gyro, accelerometer, magnetometer
*
* @return      none
*/
static void sensorIcm20948SelectAxes(void)
{
    // TODO
    if (!SENSOR_SELECT())
    {
        return;
    }

    // ** USER BANK 0 **
    SensorIcm20948_selectBank(USER_BANK_0);
    // Select gyro and accelerometer (3+3 axes, one bit each)
    val = ~mpuConfig;
    SensorI2C_writeReg(PWR_MGMT_2, &val, 1);

    SENSOR_DESELECT();

    // Select magnetometer (all axes at once)
    sensorMagEnable(!!(mpuConfig & MPU_AX_MAG));
}

/*******************************************************************************
* @fn          sensorIcm20948SetBypass
*
* @brief       Allow the I2C bus to control the magnetomoter
*
* @return      true if success
*/
static bool sensorIcm20948SetBypass(void)
{
    // TODO
    bool success;

    if (SENSOR_SELECT())
    {
        // ** USER BANK 0 **
        SensorIcm20948_selectBank(USER_BANK_0);
        
        val = BIT_BYPASS_EN | BIT_LATCH_EN;
        success = SensorI2C_writeReg(INT_PIN_CFG, &val, 1);
        DELAY_MS(10);

        SENSOR_DESELECT();
    }
    else
    {
        success = false;
    }

    return success;
}

/*******************************************************************************
* @fn          sensorMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorMagInit(void)
{
    // TODO
    ST_ASSERT_V(SensorIcm20948_powerIsOn());

    if (!sensorIcm20948SetBypass())
    {
        return;
    }

    if (SENSOR_SELECT_MAG())
    {
        static uint8_t rawData[3];

        // Enter Fuse ROM access mode
        val = MAG_MODE_FUSE;
        SensorI2C_writeReg(MAG_CNTL1, &val, 1);
        DELAY_MS(10);

        // Get calibration data
        //if (SensorI2C_readReg(MAG_ASAX, &rawData[0], 3))
        //{
            // Return x-axis sensitivity adjustment values, etc.
        //    calX =  (int16_t)rawData[0] + 128;
        //    calY =  (int16_t)rawData[1] + 128;
        //    calZ =  (int16_t)rawData[2] + 128;
        //}

        // Turn off the sensor by doing a reset
        val = 0x01;
        SensorI2C_writeReg(MAG_CNTL2, &val, 1);

        SENSOR_DESELECT();
    }
}

/*******************************************************************************
 * @fn          SensorIcm20948_magReset
 *
 * @brief       Reset the magnetometer
 *
 * @return      none
 */
void SensorIcm20948_magReset(void)
{
    // TODO
    ST_ASSERT_V(SensorIcm20948_powerIsOn());

    if (sensorIcm20948SetBypass())
    {
        if (SENSOR_SELECT_MAG())
        {
            // Turn off the sensor by doing a reset
            val = 0x01;
            SensorI2C_writeReg(MAG_CNTL2, &val, 1);
            DELAY_MS(10);

            // Re-enable if already active
            if (mpuConfig & MPU_AX_MAG)
            {
                // Set magnetometer data resolution and sample ODR
                val = (scale << 4) | mode;
                SensorI2C_writeReg(MAG_CNTL1, &val, 1);
            }
            SENSOR_DESELECT();
        }
    }
}

/*******************************************************************************
 * @fn          sensorMagEnable
 *
 * @brief       Enable or disable the compass part of the Icm20948
 *
 * @return      none
 */
static void sensorMagEnable(bool enable)
{
    // TODO
    uint8_t val;

    ST_ASSERT_V(SensorIcm20948_powerIsOn());

    if (!sensorIcm20948SetBypass())
    {
        return;
    }

    if (SENSOR_SELECT_MAG())
    {
        if (enable)
        {
            // Set magnetometer data resolution and sample ODR
            val = (scale << 4) | mode;
        }
        else
        {
            // Power down magnetometer
            val = 0x00;
        }
        SensorI2C_writeReg(MAG_CNTL1, &val, 1);

        SENSOR_DESELECT();
    }
}

/*******************************************************************************
 * @fn          SensorIcm20948_magTest
 *
 * @brief       Run a magnetometer self test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorIcm20948_magTest(void)
{
    // TODO
    ST_ASSERT(SensorIcm20948_powerIsOn());

    // Connect magnetometer internally in Icm20948
    sensorIcm20948SetBypass();

    // Select magnetometer
    SENSOR_SELECT_MAG();

    // Check the WHO AM I register
    val = 0xFF;
    ST_ASSERT(SensorI2C_readReg(MAG_WHO_AM_I, &val, 1));
    ST_ASSERT(val == MAG_DEVICE_ID);

    SENSOR_DESELECT();

    return true;
}

/*******************************************************************************
* @fn          SensorIcm20948_magRead
*
* @brief       Read data from the compass - X, Y, Z - 3 words
*
* @return      Magnetometer status
*/
uint8_t SensorIcm20948_magRead(int16_t *data)
{
    // TODO
    uint8_t val;
    uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
    // must read ST2 at end of data acquisition
    magStatus = MAG_NO_POWER;
    ST_ASSERT(SensorIcm20948_powerIsOn());

    magStatus = MAG_STATUS_OK;

    // Connect magnetometer internally in Icm20948
    SENSOR_SELECT();
    val = BIT_BYPASS_EN | BIT_LATCH_EN;
    if (!SensorI2C_writeReg(INT_PIN_CFG, &val, 1))
    {
        magStatus = MAG_BYPASS_FAIL;
    }
    SENSOR_DESELECT();

    if (magStatus != MAG_STATUS_OK)
    {
        return false;
    }

    // Select this sensor
    SENSOR_SELECT_MAG();

    if (SensorI2C_readReg(MAG_ST1,&val,1))
    {
        // Check magnetometer data ready bit
        if (val & 0x01)
        {
            // Burst read of all compass values + ST2 register
            if (SensorI2C_readReg(MAG_XOUT_L, &rawData[0],7))
            {
                val = rawData[6]; // ST2 register

                // Check if magnetic sensor overflow set, if not report data
                if(!(val & 0x08))
                {
                    // Turn the MSB and LSB into a signed 16-bit value,
                    // data stored as little Endian
                    data[0] = ((int16_t)rawData[1] << 8) | rawData[0];
                    data[1] = ((int16_t)rawData[3] << 8) | rawData[2];
                    data[2] = ((int16_t)rawData[5] << 8) | rawData[4];

                    // Sensitivity adjustment
                    data[0] = data[0] * calX >> 8;
                    data[1] = data[1] * calY >> 8;
                    data[2] = data[2] * calZ >> 8;
                }
                else
                {
                    magStatus = MAG_OVERFLOW;
                }
            }
            else
            {
                magStatus = MAG_READ_DATA_ERR;
            }
        }
        else
        {
            magStatus = MAG_DATA_NOT_RDY;
        }
    }
    else
    {
        magStatus = MAG_READ_ST_ERR;
    }

    // Set magnetometer data resolution and sample ODR
    // Start new conversion
    val = (scale << 4) | mode;
    SensorI2C_writeReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();

    return magStatus;
}

/*******************************************************************************
* @fn          SensorIcm20948_magStatus
*
* @brief       Return the magnetometer status
*
* @return      mag status
*/
uint8_t SensorIcm20948_magStatus(void)
{
    return magStatus;
}

/*******************************************************************************
 *  @fn         SensorIcm20948_Callback
 *
 *  Interrupt service routine for the MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void SensorIcm20948_Callback(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == Board_MPU_INT)
    {
        if (isrCallbackFn != NULL)
        {
            isrCallbackFn();
        }
    }
}
