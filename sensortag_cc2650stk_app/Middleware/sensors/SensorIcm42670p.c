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
#define ACCEL_CONFIG0           0x21    //  R/W
#define GYRO_CONFIG1            0x23    //  R/W
#define ACCEL_CONFIG1           0x24    //  R/W
#define APEX_CONFIG0            0x25    //  R/W
#define APEX_CONFIG1            0x26    //  R/W
#define WOM_CONFIG              0x27    //  R/W
#define FIFO_CONFIG1            0x28    //  R/W
#define INT_SOURCE0             0x2B    //  R/W
#define INT_SOURCE1             0x2C    //  R/W
#define INT_STATUS_DRDY         0x39    //  R/C
#define INT_STATUS              0x3A    //  R/C
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


// Parameters
#define ACCEL_UI_FS_SEL             0x60    // +-2g
#define ACCEL_ODR                   0x0B    // 25 Hz

#define ACCEL_UI_AVG                0x00    // 2x average
#define ACCEL_UI_FILT_BW            0x00    // bypass LP filter

#define ACCEL_LP_CLK_SEL            0x00    // LP mode
#define IDLE                        0x00    // RC oscillator turns off when Gyro & Accel are off
#define GYRO_MODE                   0x00    // Gyro off
#define ACCEL_MODE                  0x02    // Accel LP mode

#define WOM_INT_DUR                 0x00    // WoM int asserted at first event
#define WOM_INT_MODE                0x00    // WoM int on the OR of all enabled accel thresholds
#define WOM_MODE                    0x02    // Compare current sample to previous
#define WOM_EN                      0x01    // WoM enabled

#define WOM_Z_INT1_EN               0x04    // WoM Z routed to INT1
#define WOM_Y_INT1_EN               0x02    // WoM Y routed to INT1
#define WOM_X_INT1_EN               0x01    // WoM X routed to INT1

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1,Board_ICM20948_ADDR)
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

    // Set FullScale to +-2g and Output Datarate
    
    val = ACCEL_UI_FS_SEL | ACCEL_ODR ;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG0, &val, 1));

    val = ACCEL_UI_AVG | ACCEL_UI_FILT_BW; // = 0x00
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG1, &val, 1));

    // Make sure accelerometer is running
    val = ACCEL_LP_CLK_SEL | IDLE | GYRO_MODE | ACCEL_MODE;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT0, &val, 1));




    //  RAW Ready Enable
    // Wat is raw data function?
    // wake on motion seems for usefull
    // val = BIT_RAW_RDY_EN; //BIT_WOM_EN;
    // ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));


   // Enable Accel Hardware Intelligence/ Accel interrupt control-->Register 105
    // val = 0xC0;
    // ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));


    // Set Motion Threshold
//       val = threshold;
//       ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));


    /*
    // Set Frequency of Wake-up
       val = INV_LPA_20HZ;
       ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));
     */

    // Clear interrupt
     SensorI2C_readReg(INT_STATUS_DRDY,&val,1);
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
    return true;
    bool ret;

    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    accRange = ACC_RANGE_INVALID;
    mpuConfig = 0;   // All axes off

    if (!SENSOR_SELECT())
    {
        return false;
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

    // Enable Wake on Motion
    
    val = WOM_INT_DUR | WOM_INT_MODE | WOM_MODE | WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(WOM_CONFIG, &val, 1));
    
    // Enable WOM interrupts
    val = WOM_Z_INT1_EN | WOM_Y_INT1_EN | WOM_X_INT1_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_SOURCE1, &val, 1));


    // // Set Motion Threshold
    // val = threshold;
    // ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));

    // // Set Frequency of Wake-up
    // val = INV_LPA_20HZ;
    // ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));

    // Clear interrupt
     SensorI2C_readReg(INT_STATUS_DRDY,&val,1);
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
    return true;
    bool success;

    if (newRange == accRange)
    {
        return true;
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

    SENSOR_DESELECT();

    accRange = (accRangeReg>>3) & 3;

    return 0;
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

    success = SensorI2C_readReg(ACCEL_DATA_X1, (uint8_t*)data, DATA_SIZE);
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
    success = SensorI2C_readReg(GYRO_DATA_X1, (uint8_t*)data, DATA_SIZE);

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
    v = (rawData * 1.0) / (32768/2);
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
    return true;

    ST_ASSERT_V(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
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
    return true;

    ST_ASSERT_V(SensorIcm42670p_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }



    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorIcm42670p_Callback
*
* Interrupt service routine for the MPU
*
* @param       handle PIN_Handle connected to the callback
*
* @param       pinId  PIN_Id of the DIO triggering the callback
*
* @return      none
*/

static void SensorIcm42670p_Callback(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == Board_MPU_INT)
    {
        if (isrCallbackFn != NULL)
        {
            isrCallbackFn();
        }
    }
}
