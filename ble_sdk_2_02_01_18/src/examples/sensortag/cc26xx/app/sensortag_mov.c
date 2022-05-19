/******************************************************************************

 @file  sensortag_mov.c

 @brief This file contains the Movement Processor sub-application. It uses the
        MPU-9250 Wake-on-movement feature to allow the
        MPU to turn off the gyroscope and magnetometer when no activity is
        detected.

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

#ifndef EXCLUDE_MOV
/*********************************************************************
 * INCLUDES
 */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "gatt.h"
#include "gattservapp.h"

#include "board.h"
#include "movementservice.h"
#include "sensortag_mov.h"
#include "SensorIcm20948.h"
#include "SensorTagTest.h"
#include "SensorUtil.h"
#include "util.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */
#define MOVEMENT_INACT_CYCLES   (MOVEMENT_INACT_TIMEOUT * \
                                (1000/sensorPeriod) / 10)
                            // = 10*(10.000/sensorPeriod/10)
/*********************************************************************
 * CONSTANTS and MACROS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     100

// Length of the data for this sensor
#define SENSOR_DATA_LEN           MOVEMENT_DATA_LEN

// Event flag for this sensor
#define SENSOR_EVT                ST_GYROSCOPE_SENSOR_EVT

// Movement task states
#define APP_STATE_ERROR           0xFF
#define APP_STATE_OFF             0
#define APP_STATE_IDLE            1
#define APP_STATE_ACTIVE          2

// Movement task configuration
#define MOVEMENT_INACT_TIMEOUT    10     // 10 seconds
#define GYR_SHAKE_THR             10.0
#define WOM_THR                   10

// Configuration bit-masks (bits 0-6 defined in sensor_Icm20948.h)
#define MOV_WOM_ENABLE            0x0080
#define MOV_MASK_WOM_THRESHOLD    0x3C00 // TBD
#define MOV_MASK_INACT_TIMEOUT    0xC000 // TBD


// Task configuration
#define SENSOR_TASK_PRIORITY    1

//#define SENSOR_TASK_STACK_SIZE  600
#define SENSOR_TASK_STACK_SIZE    600
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
static Clock_Struct periodicClock;
static uint16_t sensorPeriod;
static volatile bool sensorReadScheduled;
static uint8_t sensorData[SENSOR_DATA_LEN];

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;

// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;


// Parameters
static uint8_t sensorConfig;


//Task Setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Main loop Semaphore  -- task semaphore
Semaphore_Struct MovSem;
Semaphore_Handle hMovSem;



// Application state variables

// MPU config:
// bit 0-2:   accelerometer enable(z,y,x)
// bit 3-5:   gyroscope enable (z,y,x)
// bit 6:     magnetometer enable
// bit 7:     WOM enable
// bit 8-9:   accelerometer range (2,4,8,16)

uint16_t mpuConfig;
//static uint16_t mpuConfig;

static uint8_t appState;
static volatile bool mpuDataRdy;
static uint32_t nActivity;
static uint8_t movThreshold;
static uint8_t mpuIntStatus;
static bool shakeDetected;
static uint8_t nMotions;

// Debug Var

uint8_t mov_flag[10];
uint8_t Mov_valueCharChange=0;
uint8_t newValueF;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static void SensorTagMov_clockHandler(UArg arg);
static void appStateSet(uint8_t newState);
static void SensorTagMov_processInterrupt(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


// Create Task
void SensorTagMov_createTask(void ){
    Task_Params taskParames;

    // Create the task for the state machine
    Task_Params_init(&taskParames);
    taskParames.stack = sensorTaskStack;
    taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
    taskParames.priority = SENSOR_TASK_PRIORITY;
    Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}


/*********************************************************************
 * @fn      SensorTagMov_init
 *
 * @brief   Initialization function for the SensorTag movement sub-application
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_init(void)
{
#ifdef BIOMONITOR
    // Init Semaphore
     Sem_Mov_init();

       // Add service
     Movement_addService();

     // Register callbacks with profile
     Movement_registerAppCBs(&sensorCallbacks);

     // Initialize the module state variables
     mpuConfig = ST_CFG_SENSOR_DISABLE;
     sensorPeriod = SENSOR_DEFAULT_PERIOD;
     sensorReadScheduled = false;

     appState = APP_STATE_OFF;
     nMotions = 0;

     if (SensorIcm20948_init())
     {
       SensorTagMov_reset();
       SensorIcm20948_registerCallback(SensorTagMov_processInterrupt);
     }

     // Initialize characteristics
     initCharacteristicValue(SENSOR_PERI,
                             SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                             sizeof(uint8_t));

     // Create continuous clock for internal periodic events.
     Util_constructClock(&periodicClock, SensorTagMov_clockHandler,10, 10, false, 0);
     //Util_constructClock(&periodicClock, SensorTagMov_clockHandler,1000, sensorPeriod, false, 0);


#else  // BabyMonitor
     // Init Semaphore
       Sem_Mov_init();

         // Add service
       Movement_addService();

       // Register callbacks with profile
       Movement_registerAppCBs(&sensorCallbacks);

       // Initialize the module state variables
       mpuConfig = ST_CFG_SENSOR_DISABLE;
       sensorPeriod = SENSOR_DEFAULT_PERIOD;
       sensorReadScheduled = false;

       appState = APP_STATE_OFF;
       nMotions = 0;

       if (SensorIcm20948_init())
       {
         SensorTagMov_reset();
        // SensorIcm20948_registerCallback(SensorTagMov_processInterrupt);
       }

       // Initialize characteristics
       initCharacteristicValue(SENSOR_PERI,
                               SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                               sizeof(uint8_t));

       // Create continuous clock for internal periodic events.
       Util_constructClock(&periodicClock, SensorTagMov_clockHandler,10, 10, false, 0);

       //Util_constructClock(&periodicClock, SensorTagMov_clockHandler,1000, sensorPeriod, false, 0);

#endif

}

/*********************************************************************
 * @fn      SensorTagMov_processSensorEvent
 *
 * @brief   SensorTag Movement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_processSensorEvent(void){

    //if clock or MPU_int have call for it.
if (sensorReadScheduled){

    uint8_t axes;
    axes = mpuConfig & MPU_AX_ALL;

    if ((axes != ST_CFG_SENSOR_DISABLE) && (axes != ST_CFG_ERROR)) {
      // Get interrupt status (clears interrupt)
      mpuIntStatus = SensorIcm20948_irqStatus();

      // Process gyro and accelerometer
      if (mpuDataRdy || appState == APP_STATE_ACTIVE) {

        if (mpuIntStatus & MPU_MOVEMENT){

            // Motion detected (small filter)
            nMotions++;
            if (nMotions == 2) {
                nActivity = MOVEMENT_INACT_CYCLES;
            }
        }
        else if (mpuIntStatus & MPU_DATA_READY)
        {
          // Read gyro data
          SensorIcm20948_gyroRead((uint16_t*)sensorData);

          // Read accelerometer data
          SensorIcm20948_accRead((uint16_t*)&sensorData[6]);
          mov_flag[0]++;

          // What is this for???
          if (shakeDetected)
          {
            // Motion detected by gyro
            nActivity = MOVEMENT_INACT_CYCLES;
            shakeDetected = false;
            mov_flag[1]++;
          }
        }

        mpuDataRdy = false;

        if (appState == APP_STATE_ACTIVE && !!(mpuConfig & MPU_AX_MAG))
        {
          uint8_t status;

          status = SensorIcm20948_magRead((int16_t*)&sensorData[12]);

          // Always measure magnetometer (not interrupt driven)
          if (status == MAG_BYPASS_FAIL)
          {
            // Idle on error
            nActivity = 0;
            appState = APP_STATE_ERROR;
          }
          else if (status != MAG_STATUS_OK)
          {
            SensorIcm20948_magReset();
          }
        }


    //    Movement_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, sensorData);
    //    mov_flag[2]++;
      }

      if (nActivity>0)
      {
        if (appState != APP_STATE_ACTIVE)
        {
          // Transition to active state
          appState = APP_STATE_ACTIVE;
          nMotions = 0;
          if (SensorIcm20948_reset())
          {
            SensorIcm20948_enable(axes);
          }
        }

        if (mpuConfig & MOV_WOM_ENABLE)
        {
          nActivity--;
        }

        // Send data
        Movement_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, sensorData);
      }
      else
      {
        if (appState != APP_STATE_IDLE)
        {
          // Transition from active to idle state
          nMotions = 0;
          appState = APP_STATE_IDLE;
          if (SensorIcm20948_reset())
          {
            SensorIcm20948_enableWom(movThreshold);
          }
        }
      }
    }

    sensorReadScheduled = false;
  }
}

/*********************************************************************
 * @fn      SensorTagMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 * @param   paramID - identifies which characteristic has changed
 *
 * @return  none
 */
void SensorTagMov_processCharChangeEvt(uint8_t paramID)
{
  uint16_t newCfg;
  //uint8_t newValueF;
  mov_flag[0]++;

  switch (paramID)
  {
  case SENSOR_CONF:

   if ((SensorTag_testResult() & SENSOR_MOV_TEST_BM) == 0)
    {
      mpuConfig = ST_CFG_ERROR;
    }

    if (mpuConfig != ST_CFG_ERROR)
    {
      Movement_getParameter(SENSOR_CONF, &newCfg);
      Mov_valueCharChange=newCfg;

      if ((newCfg & MPU_AX_ALL) == ST_CFG_SENSOR_DISABLE){
          mov_flag[1]++;
/*       //Desactivar sensor y lectura de data

      // reset characteristic and turn off MAX30102
      sensorConfig = ST_CFG_SENSOR_DISABLE;
     // Erase data from BLE Max service
      initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
     // 1 when the task is running( empty task), 0 when is stoped.
      MAX_state=0;
      // Desactiva lectura de interrupcion generada por Sensor
      SensorMax30102_InterruptEnable(false);
      // Apaga Sensor
      SensorMax30102_enable(false);
      Debuger_max++;
      // Deactivate task
      Task_setPri(Task_handle(&sensorTask), -1);

      SensorMax30102_Auxfn(false);
*/

       // All axes off, turn off device power
        sensorConfig = ST_CFG_SENSOR_DISABLE;
        mpuConfig = newCfg;
        appStateSet(APP_STATE_OFF);

       // initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
        Task_setPri(Task_handle(&sensorTask), -1);
        // Erase data from BLE Max service
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);

      }
      else {

    /*
        if (newValue == ST_CFG_SENSOR_ENABLE){

    dflag[7]++;
    initCharacteristicValue(SENSOR_DATA, 12, SENSOR_DATA_LEN);
    sensorConfig = ST_CFG_SENSOR_ENABLE;

    SensorMax30102_InterruptEnable(true);
    DELAY_MS(10);
    dflag[8]++;
    //Enable SnMax task

    Task_setPri(Task_handle(&sensorTask),SENSOR_TASK_PRIORITY);
    DELAY_MS(10);

    // Enable the sensor to make readings
    //  MaxDataReady=1;
    BoolMax=SensorMax30102_enable(true);
    dflag[9]++;

    SensorMax30102_Auxfn(false);// Debug
    DP3s=true;
    }
      */
              //    if (newCfg == ST_CFG_SENSOR_ENABLE){
          mov_flag[2]++;
          sensorConfig = ST_CFG_SENSOR_ENABLE;

          Task_setPri(Task_handle(&sensorTask),SENSOR_TASK_PRIORITY);
          DELAY_MS(10);

          // Some axes on; power up and activate MPU
          mpuConfig = newCfg;
          appStateSet(APP_STATE_ACTIVE);

          if (SensorIcm20948_powerIsOn()){
              DELAY_MS(25);
              mpuConfig = newCfg | (SensorIcm20948_accReadRange() << 8);
            //}
         }

      }

      Movement_setParameter(SENSOR_CONF, sizeof(mpuConfig), (uint8_t*)&mpuConfig);
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, mpuConfig, sizeof(mpuConfig));
    }

    // Data initially zero
    initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
    break;

  case SENSOR_PERI:
    Movement_getParameter(SENSOR_PERI, &newValueF);
   // sensorPeriod = newValue8 * SENSOR_PERIOD_RESOLUTION;
    if( newValueF<10){
         newValueF=10;}

    if( newValueF>240){
         newValueF=240;}

    sensorPeriod = newValueF * 1;

    Util_rescheduleClock(&periodicClock,sensorPeriod);

    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagMov_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_reset(void)
{
  mov_flag[5]++;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  mpuConfig = ST_CFG_SENSOR_DISABLE | (ACC_RANGE_8G << 8);
  Movement_setParameter(SENSOR_CONF, sizeof(mpuConfig), (uint8_t*)&mpuConfig);

  // Remove power from the MPU
  appStateSet(APP_STATE_OFF);
}


/*********************************************************************
* Private functions
*/


static void sensorTaskFxn(UArg a0, UArg a1){

  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

    // Deactivate task (active only when measurement is enabled)

   Task_setPri(Task_handle(&sensorTask), -1);

  while(true){
     if( sensorConfig == ST_CFG_SENSOR_ENABLE ){
        mov_flag[8]++;

         Semaphore_pend(hMovSem,BIOS_WAIT_FOREVER);

         //if(mpuDataRdy){}
         // Get interrupt status (clears interrupt)
         //mpuIntStatus = SensorIcm20948_irqStatus();

         // Read accelerometer and gyro data
         SensorIcm20948_gyroRead((uint16_t*)sensorData);
         SensorIcm20948_accRead((uint16_t*)&sensorData[6]);

         mov_flag[7]++;
        // sensorData[0]=mov_flag[7];
        Movement_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, sensorData);

/*       if (mpuIntStatus & MPU_DATA_READY){
         // Read gyro data
          SensorIcm20948_gyroRead((uint16_t*)sensorData);

          // Read accelerometer data
          SensorIcm20948_accRead((uint16_t*)&sensorData[6]);
          mov_flag[0]++;
        }
  */

     }
    else{
        mov_flag[9]++;
        DELAY_MS(8);  }
  }
}


// Semaphore initialization
static void Sem_Mov_init(){

   Semaphore_Params semParams;
   Semaphore_Params_init(&semParams);
   Semaphore_construct(&MovSem, 0, &semParams);
   hMovSem = Semaphore_handle(&MovSem);
}

/*********************************************************************
 * @fn      SensorTagMov_processInterrupt
 *
 * @brief   Interrupt handler for MPU
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagMov_processInterrupt(void)
{
  // Wake up the application thread
   mpuDataRdy = true;
  sensorReadScheduled = true;
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SensorTagMov_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - not used
 *
 * @return  none
 */
static void SensorTagMov_clockHandler(UArg arg)
{
  mov_flag[6]++;
  // Schedule readout periodically
  sensorReadScheduled = true;
  mpuDataRdy = true;
  //Semaphore_post(sem);
  Semaphore_post(hMovSem);
}


/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from Movement Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_MOV, paramID);
}


/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
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
  memset(sensorData,value,paramLen);
  Movement_setParameter(paramID, paramLen, sensorData);
}

/*******************************************************************************
 * @fn      appStateSet
 *
 * @brief   Set the application state
 *
 */
static void appStateSet(uint8_t newState)
{
  if (newState == APP_STATE_OFF)
  {
    appState = APP_STATE_OFF;

    SensorIcm20948_enable(0);
   // SensorIcm20948_powerOff();

    // Stop scheduled data measurements
    Util_stopClock(&periodicClock);
  }

  if (newState == APP_STATE_ACTIVE || newState == APP_STATE_IDLE)
  {
    appState = APP_STATE_ACTIVE;
    nActivity = MOVEMENT_INACT_CYCLES;
    movThreshold = WOM_THR;
    mpuIntStatus = 0;
    shakeDetected = false;
    mpuDataRdy = false;

    SensorIcm20948_powerOn();
    SensorIcm20948_enable(mpuConfig & 0xFF);

    //Set frequency of the gyro and the Accel
    SensorIcm20948_MPU_frqconfig(0);


    if (newState == APP_STATE_ACTIVE)
    {
      // Start scheduled data measurements
      Util_startClock(&periodicClock);
    }
    else
    {
      // Stop scheduled data measurements
      Util_stopClock(&periodicClock);
    }
  }
}
#endif // EXCLUDE_MOV

/*********************************************************************
*********************************************************************/

