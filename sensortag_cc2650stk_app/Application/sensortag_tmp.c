/******************************************************************************

 @file  sensortag_tmp.c

 @brief This file contains the Sensor Tag sample application,
        IR Temperature part, for use with the TI Bluetooth Low
        Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2015-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

#ifndef EXCLUDE_TMP
/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"

#include "irtempservice.h"
#include "sensortag_tmp.h"
#include "SensorTmp007.h"
#include "EDAService.h"
#include "SensorTagTest.h"
#include "SensorUtil.h"
#include "board.h"

#include "string.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   1000

// Delay from sensor enable to reading measurement
// (allow for 250 ms conversion time)
#define TEMP_MEAS_DELAY         275

// Length of the data for this sensor
#define SENSOR_DATA_LEN         IRTEMPERATURE_DATA_LEN

// Event flag for this sensor
#define SENSOR_EVT              ST_IRTEMPERATURE_SENSOR_EVT

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  450//600


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;

// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;

// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
static uint8_t sensorConfig;
static uint16_t sensorPeriod;


// Debug Variables
uint16_t dTemp[6];
uint8_t Tmp_valueCharChange=0;
uint8_t e=0;

//uint16_t stackSi[4];



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTaskFxn(UArg a0, UArg a1);
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagTmp_createTask
 *
 * @brief   Task creation function for the SensorTag
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTmp_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

/*********************************************************************
 * @fn      SensorTagTmp_processCharChangeEvt
 *
 * @brief   SensorTag IR temperature event handling
 *
 * @param   paramID - identifies the characteristic that was changed
 *
 * @return  none
 *
 */
void SensorTagTmp_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  dTemp[0]++;
  switch (paramID)
  {
  case SENSOR_CONF:
    if ((SensorTag_testResult() & SENSOR_TMP_TEST_BM) == 0)
    {
      sensorConfig = ST_CFG_ERROR;
    }

    if (sensorConfig != ST_CFG_ERROR)
    {
      IRTemp_getParameter(SENSOR_CONF, &newValue);
      Tmp_valueCharChange=newValue;
      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Reset characteristics
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);

        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);
        dTemp[2]++;
      }
      else
      {
        Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
        dTemp[1]++;
      }

      sensorConfig = newValue;
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
    }

    // Make sure sensor is disabled
    SensorTmp007_enable(false);
    break;

  case SENSOR_PERI:
      dTemp[1]++;
    IRTemp_getParameter(SENSOR_PERI, &newValue);

    if(newValue<50){newValue=50;}
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION;

    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagTmp_init
 *
 * @brief   Initialize the module
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTmp_init(void)
{

   // Add service
   IRTemp_addService();

    // Register callbacks with profile
  IRTemp_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;

  // Initialize characteristics and sensor driver
  SensorTagTmp_reset();
  initCharacteristicValue(SENSOR_PERI,
                          SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                          sizeof(uint8_t));

 }

/*********************************************************************
 * @fn      SensorTagTmp_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTmp_reset(void)
{
  dTemp[5]++;
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));


  EDAService_SetParameter(EDASERVICE_DATA_CONF,EDASERVICE_DATA_CONF_LEN,0);
  EDAService_SetParameter(EDASERVICE_DATA_CHAR,EDASERVICE_DATA_CHAR_LEN,0);

  // Initialize the driver
  //SensorTmp007_init();
}


/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @param   a0 (not used)
 *
 * @param   a1 (not used)
 *
 * @return  none
 */
static void sensorTaskFxn(UArg a0, UArg a1)
{
  typedef union
  {
    struct
    {
      uint16_t tempTarget, tempLocal;
    } v;
    uint16_t a[2];
  } Data_t;


//  Task_Stat statbuf;

  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  Data_t data;
  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);

e=1;

  // Task loop
  while (true)
  {
      dTemp[4]++;
   if (sensorConfig == ST_CFG_SENSOR_ENABLE)
    {
     //Data_t data;

//      Task_Stat statbuf;
//      // Stack Size debug
//      Task_stat(Task_self(),&statbuf);
//      stackSi[0]=statbuf.stackSize;
//      stackSi[1]=statbuf.used;

      // Read data
      SensorTmp007_enable(true);
      DELAY_MS(TEMP_MEAS_DELAY);
      SensorTmp007_read(&data.v.tempLocal, &data.v.tempTarget);
      SensorTmp007_enable(false);


      dTemp[0]=data.v.tempLocal;
      dTemp[1]=data.v.tempTarget;

//      if(e==1){
//          data.v.tempLocal=33+e;
//          data.v.tempTarget=34+e;
//          e=-1;
//      }
//      else{
//          data.v.tempLocal=33+e;
//          data.v.tempTarget=34+e;
//          e=1;
//      }

            // Update GATT
      IRTemp_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, data.a);

      // Next cycle
      DELAY_MS(sensorPeriod - TEMP_MEAS_DELAY);
    }
    else
    {
      DELAY_MS(SENSOR_DEFAULT_PERIOD);
    }
  }
}


/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from IR Temperature Service indicating a value change
 *
 * @param   paramID - identifies the characteristic that was changed
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_TMP, paramID);
}


/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value to be initialized
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  IRTemp_setParameter(paramID, paramLen, data);
}

#endif // EXCLUDE_TMP

/*********************************************************************
*********************************************************************/

