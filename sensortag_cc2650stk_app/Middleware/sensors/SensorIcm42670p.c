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
 *  @file       SensorIcm42670p.c
 *
 *  @brief      Driver for the InvenSense Icm42670p Motion Processing Unit.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "SensorIcm42670p.h"
#include "SensorOpt3001.h" // For reset of I2C bus
#include "SensorUtil.h"
#include "SensorI2C.h"

/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/

//Registers

#define MCLK_RDY                0x00    //  R
#define INT_CONFIG              0x06    //  R/W
#define ACCEL_DATA_X1           0x0B    //  R
#define ACCEL_DATA_X0           0x0C    //  R
#define ACCEL_DATA_Y1           0x0D    //  R
#define ACCEL_DATA_Y0           0x0E    //  R
#define ACCEL_DATA_Z1           0x0F    //  R
#define ACCEL_DATA_Z0           0x10    //  R
#define GYRO_DATA_X1            0x11    //  R
#define GYRO_DATA_X0            0x12    //  R
#define GYRO_DATA_Y1            0x13    //  R
#define GYRO_DATA_Y0            0x14    //  R
#define GYRO_DATA_Z1            0x15    //  R
#define GYRO_DATA_Z0            0x16    //  R
#define PWR_MGMT0               0x1F    //  R/W
#define GYRO_CONFIG0            0x20    //  R/W
#define ACCELS_CONFIG0          0x21    //  R/W
#define GYRO_CONFIG1            0x23    //  R/W
#define ACCELS_CONFIG1          0x24    //  R/W
#define APEX_CONFIG0            0x25    //  R/W
#define APEX_CONFIG1            0x26    //  R/W
#define WOM_CONFIG              0x27    //  R/W
#define FIFO_CONFIG1            0x28    //  R/W
#define INT_SOURCE0             0x2B    //  R/W
#define INT_STATUS_DRDY         0x39    //  R/C
#define WHO_AM_I                0x75    //  R

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6

// Output data rates
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
#define BIT_WOM_EN                    0x40
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



// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1,Board_Icm42670p_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()

/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorMpuSleep(void);
static void sensorIcm42670pWakeUp(void);
static void sensorIcm42670pSelectAxes(void);
static void SensorIcm42670p_Callback(PIN_Handle handle, PIN_Id pinId);

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/

//Debugging

uint8_t MSen_d[5];

static uint8_t mpuConfig;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t val;

// Pins that are used by the Icm42670p
static PIN_Config MpuPinTable[] =
{
    Board_MPU_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
   // Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};
static PIN_State pinGpioState;
static PIN_Handle hMpuPin;

// The application may register a callback to handle interrupts
static SensorIcm42670pCallbackFn_t isrCallbackFn = NULL;

/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/

bool SensorIcm42670p_MPU_frqconfig(uint8_t frq){

    if (!SENSOR_SELECT())
      {
          return false;
      }

    // Set SRDiveder to zero
    val = 0x00;
    ST_ASSERT(SensorI2C_writeReg(SMPLRT_DIV, &val, 1));

    // Set Accel LPF setting to 92 Hz Bandwidth
    val = 0x02;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG_2, &val, 1));

    // Set FIfo: 0, and DLPF_CFG to set the gyroscope to 92 Hz Bandwidth
    val = 0x02;
    ST_ASSERT(SensorI2C_writeReg(CONFIG, &val, 1));

    // Set the Gyro FS Sel to 500 dps
    val = 0x08;
    ST_ASSERT(SensorI2C_writeReg(GYRO_CONFIG, &val, 1));


    // Make sure accelerometer is running
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Enable accelerometer, and gyro
    val = 0x00;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));


    //  RAW Ready Enable
    // Wat is raw data function?
    // wake on motion seems for usefull
    val = BIT_RAW_RDY_EN; //BIT_WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));


   // Enable Accel Hardware Intelligence/ Accel interrupt control-->Register 105
    val = 0xC0;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));


    // Set Motion Threshold
//       val = threshold;
//       ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));


    /*
    // Set Frequency of Wake-up
       val = INV_LPA_20HZ;
       ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));
     */

    // Clear interrupt
     SensorI2C_readReg(INT_STATUS,&val,1);




    SENSOR_DESELECT();
    MSen_d[3]++;
    return  true;
}


//bool SensorIcm42670p_Acel_frqconfig(uint8_t frq){}

//bool SensorIcm42670p_Gyro_frqconfig(uint8_t frq){}


/*******************************************************************************
* @fn          SensorIcm42670p_powerOn
*
* @brief       This function turns on the power supply to Icm42670p
*
* @return      none
*/
void SensorIcm42670p_powerOn(void)
{
    // Turn on power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    DELAY_MS(100);
    SensorIcm42670p_reset();
}

/*******************************************************************************
* @fn          SensorIcm42670p_powerOff
*
* @brief       This function turns off the power supply to Icm42670p
*
* @return      none
*/
void SensorIcm42670p_powerOff(void)
{
    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    // Turn off power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

    // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
   // SensorOpt3001_test();
}

/*******************************************************************************
* @fn          SensorIcm42670p_powerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool SensorIcm42670p_powerIsOn(void)
{
    return PIN_getOutputValue(Board_MPU_POWER) == Board_MPU_POWER_ON;
}

/*******************************************************************************
* @fn          SensorIcm42670p_registerCallback
*
* @brief       Register a call-back for interrupt processing
*
* @return      none
*/
void SensorIcm42670p_registerCallback(SensorIcm42670pCallbackFn_t pfn)
{
    isrCallbackFn = pfn;
}

/*******************************************************************************
* @fn          SensorIcm42670p_init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool SensorIcm42670p_init(void)
{
    // Pins used by MPU
    hMpuPin = PIN_open(&pinGpioState, MpuPinTable);

    // Register MPU interrupt
    PIN_registerIntCb(hMpuPin, SensorIcm42670p_Callback);

    // Application callback initially NULL
    isrCallbackFn = NULL;

    return SensorIcm42670p_reset();
}


/*******************************************************************************
* @fn          SensorIcm42670p_reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool SensorIcm42670p_reset(void)
{
    bool ret;

    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    accRange = ACC_RANGE_INVALID;
    mpuConfig = 0;   // All axes off

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Device reset
    val = 0x80;
    SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    SENSOR_DESELECT();

    DELAY_MS(100);

    ret = SensorIcm42670p_test();
    if (ret)
    {
        // Initial configuration
        SensorIcm42670p_accSetRange(ACC_RANGE_8G);

        // Power save
        sensorMpuSleep();
    }

    return ret;
}


/*******************************************************************************
* @fn          SensorIcm42670p_enableWom
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool SensorIcm42670p_enableWom(uint8_t threshold)
{
    ST_ASSERT(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure accelerometer is running
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Enable accelerometer, disable gyro
    val = 0x07;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));

    // Set Accel LPF setting to 184 Hz Bandwidth
    val = 0x01;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG_2, &val, 1));

    // Enable Motion Interrupt
    val = BIT_WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));

    // Enable Accel Hardware Intelligence
    val = 0xC0;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));

    // Set Motion Threshold
    val = threshold;
    ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));

    // Set Frequency of Wake-up
    val = INV_LPA_20HZ;
    ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));

    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Select the current range
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1));

    // Clear interrupt
    SensorI2C_readReg(INT_STATUS,&val,1);

    SENSOR_DESELECT();

    mpuConfig = 0;

    // Enable pin for wake-on-motion interrupt
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

    return true;
}

/*******************************************************************************
* @fn          SensorIcm42670p_irqStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t SensorIcm42670p_irqStatus(void)
{
    uint8_t intStatus;

    intStatus = 0;
    ST_ASSERT(SensorIcm42670p_powerIsOn());

    if (SENSOR_SELECT())
    {
        if (!SensorI2C_readReg(INT_STATUS,&intStatus,1))
        {
            intStatus = 0;
        }
        SENSOR_DESELECT();
    }

    return intStatus;
}

/*******************************************************************************
* @fn          SensorIcm42670p_enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
* @                  Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
*                    MPU  bit [6], all axes
*
* @return      None
*/
void SensorIcm42670p_enable(uint16_t axes)
{
    ST_ASSERT_V(SensorIcm42670p_powerIsOn());

    if (mpuConfig == 0 && axes != 0)
    {
        MSen_d[0]++;
        // Wake up the sensor if it was off
        sensorIcm42670pWakeUp();
    }

    mpuConfig = axes;

    if (mpuConfig != 0)
    {   MSen_d[1]++;
        // Enable gyro + accelerometer
        sensorIcm42670pSelectAxes();
    }
    else if (mpuConfig == 0)
    {
        MSen_d[2]++;
        sensorMpuSleep();
    }
}


/*******************************************************************************
* @fn          SensorIcm42670p_accSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool SensorIcm42670p_accSetRange(uint8_t newRange)
{
    bool success;

    if (newRange == accRange)
    {
        return true;
    }

    ST_ASSERT(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    accRangeReg = (newRange << 3);

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
* @fn          SensorIcm42670p_accReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t SensorIcm42670p_accReadRange(void)
{
    ST_ASSERT(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Apply the range
    SensorI2C_readReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    accRange = (accRangeReg>>3) & 3;

    return accRange;
}


/*******************************************************************************
* @fn          SensorIcm42670p_accRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool SensorIcm42670p_accRead(uint16_t *data )
{
    bool success;

    ST_ASSERT(SensorIcm42670p_powerIsOn());

    // Burst read of all accelerometer values
    if (!SENSOR_SELECT())
    {
        return false;
    }

    success = SensorI2C_readReg(ACCEL_XOUT_H, (uint8_t*)data, DATA_SIZE);
    SENSOR_DESELECT();

    if (success)
    {
        SensorUtil_convertToLe((uint8_t*)data,DATA_SIZE);
    }

    return success;
}

/*******************************************************************************
* @fn          SensorIcm42670p_gyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool SensorIcm42670p_gyroRead(uint16_t *data )
{
    bool success;

    ST_ASSERT(SensorIcm42670p_powerIsOn());

    // Select this sensor
    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Burst read of all gyroscope values
    success = SensorI2C_readReg(GYRO_XOUT_H, (uint8_t*)data, DATA_SIZE);

    SENSOR_DESELECT();

    if (success)
    {
        SensorUtil_convertToLe((uint8_t*)data,DATA_SIZE);
    }

    return success;
}

/*******************************************************************************
 * @fn          SensorIcm42670p_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorIcm42670p_test(void)
{
    static bool first = true;

    ST_ASSERT(SensorIcm42670p_powerIsOn());

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

    // Check the WHO AM I register
    ST_ASSERT(SensorI2C_readReg(WHO_AM_I, &val, 1));
    ST_ASSERT(val == 0x71);

    SENSOR_DESELECT();

    return true;
}


/*******************************************************************************
 * @fn          SensorIcm42670p_accConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float SensorIcm42670p_accConvert(int16_t rawData)
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
 * @fn          SensorIcm42670p_gyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float SensorIcm42670p_gyroConvert(int16_t data)
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
    bool success;

    ST_ASSERT_V(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    val = ALL_AXES;
    success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
    if (success)
    {
        val = MPU_SLEEP;
        success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    }

    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorIcm42670pWakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/
static void sensorIcm42670pWakeUp(void)
{
    bool success;

    ST_ASSERT_V(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    val = MPU_WAKE_UP;
    success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);

    if (success)
    {
        // All axis initially disabled
        val = ALL_AXES;
        success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
        mpuConfig = 0;
    }

    if (success)
    {
        // Restore the range
        success = SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1);

        if (success)
        {
            // Clear interrupts
            success = SensorI2C_readReg(INT_STATUS,&val,1);
        }
    }

    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorIcm42670pSelectAxes
*
* @brief       Select gyro, accelerometer
*
* @return      none
*/
static void sensorIcm42670pSelectAxes(void)
{
    if (!SENSOR_SELECT())
    {
        return;
    }

    // Select gyro and accelerometer (3+3 axes, one bit each)
    val = ~mpuConfig;
    SensorI2C_writeReg(PWR_MGMT_2, &val, 1);

    SENSOR_DESELECT();
}
