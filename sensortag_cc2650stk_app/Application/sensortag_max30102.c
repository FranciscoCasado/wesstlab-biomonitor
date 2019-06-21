/*
 * sensortag_max30102.c
 *
 *  Created on: 17-08-2016
 *      Author: JC
 */



#ifndef EXCLUDE_MAX30102
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


#include "Max30102Service.h"
#include "sensortag_max30102.h"
#include "SensorMax30102.h"
#include "SensorTagTest.h"
#include "SensorUtil.h"
#include "util.h"
#include "string.h"
#include "board.h"

/*********************************************************************
 * MACROS
 */

//move task
#define MOVEMENT_INACT_CYCLES   (MOVEMENT_INACT_TIMEOUT * (1000/sensorPeriod))

/*********************************************************************
 * CONSTANTS and MACROS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     60
#define SENSOR_SLEEP_PERIOD       20

// Delay form sensor enable to reading measurement
//
#define TEMP_MEAS_DELAY	  5

// Length of the data for this sensor
#define SENSOR_DATA_LEN           MAX30102SERVICE_MAX_DATA_CHAR_LEN// MOVEMENT_DATA_LEN

// Event flag for this sensor
#define SENSOR_EVT              ST_MAX30102_SENSOR_EVT

// Task configuration
#define SENSOR_TASK_PRIORITY    1

//#define SENSOR_TASK_STACK_SIZE  600
#define SENSOR_TASK_STACK_SIZE  500
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


// Clock for executing the ppg readings in a configurable way
static Clock_Struct periodicClock;


// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;


// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;

// Main loop Semaphore  -- task semaphore
Semaphore_Struct semMainLoop;
Semaphore_Handle hSemMainLoop;


//Task Setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
static uint8_t sensorConfig;
static uint16_t sensorPeriod;
static uint8_t  sensorData[MAX30102SERVICE_MAX_DATA_CHAR_LEN];


// OLD DATA buffes
//uint32_t RData[1];
//uint32_t IRData[1];
//Sofware interrups structurs
Swi_Struct swiMax30102Alert;
Swi_Handle hSwiMax30102Alert;




// Debug variables
uint8_t LedM;
uint8_t LedI;

uint16_t Debuger_max=0;
bool DP3s=false;
uint8_t	baux[3];
//uint16_t Mflag;

// buffers for read, write and send data to BLE service
bool 	 MaxDataReady;
uint8_t  dataDumy[6];
uint8_t  rbuf1[1];

uint8_t  rbuf2[1];
uint8_t  wbuf[1];
uint8_t dflag[10];
uint8_t  Max_valueCharChange=0;
uint8_t  Max_CMD=0;
uint8_t newValueMax=0;


bool   b1m=false;
bool   b2m=false;
//bool 	 b3m=false;

bool 	 BoolMax=false;
//bool 	 BoolMax1=false;
bool 	 BoolMax2=false;

//uint8_t  auxM[MAX30102SERVICE_MAX_DATA_CHAR_LEN];

//uint16_t stackSi[4];


/*********************************************************************
 * LOCAL FUNCTIONS
 */


//static void SWI_init(void);
static void Sem_Max_init(void);
//static void swiMax30102Fxn(UArg a0, UArg a1);
static void SensorTagMax30102_clockHandler(UArg arg);


static void sensorTaskFxn(UArg a0, UArg a1);
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen);
static void SensorTagMax30102_processInterrupt(void);

/***************************************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
		sensorConfigChangeCB,		// Characteristic value change callback
};

/***************************************************************************************
 * Public FUNCTIONS
 */


// Create Task
void SensorTagMax30102_createTask(void ){
	Task_Params taskParames;

	// Create the task for the state machine
	Task_Params_init(&taskParames);
	taskParames.stack = sensorTaskStack;
	taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
	taskParames.priority = SENSOR_TASK_PRIORITY;
	Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

//Event handler
// param ID identifies the characteristics that was changed

void SensorTagMax30102_processCharChangeEvt(uint8_t paramID){

    dflag[9]++;
	uint8_t newValue;
	//uint8_t LedM;
	//uint8_t LedI;

	  switch (paramID)
	  {
	  case SENSOR_CONF: // Parameto BLE de Enable Measurements
	    if ((SensorTag_testResult() & SENSOR_MAX_TEST_BM) == 0)
	    {
	      sensorConfig = ST_CFG_ERROR;
	    }

	    if (sensorConfig != ST_CFG_ERROR)// no hay algun tipo de error
	    {
	    	Max30102Service_GetParameter(SENSOR_CONF, &newValue);
	    	Max_CMD=newValue;
	    	newValue=(Max_CMD & 0x01);
	    	//Max_valueCharChange=newValue;


	      if (newValue == ST_CFG_SENSOR_DISABLE){
            #ifdef BIOMONITOR
	             dflag[6]++;
                //Desactivar sensor y lectura de data

                // reset characteristic and turn off MAX30102
                sensorConfig = ST_CFG_SENSOR_DISABLE;

                // Reinicia y Apaga Sensor. Desactiva la task.
                SensorTagMax30102_reset();

                // reset characteristic and turn off MAX30102
                sensorConfig = ST_CFG_SENSOR_DISABLE;
                DELAY_MS(100);
                dflag[6]++;
            #else  // BabyMonitor
                //Desactivar sensor y lectura de data
              dflag[1]++;

            // Reinicia y Apaga Sensor. Desactiva la task.
             SensorTagMax30102_reset();

             // reset characteristic and turn off MAX30102
              sensorConfig = ST_CFG_SENSOR_DISABLE;
          //    Debuger_max++;

            #endif



	       }
	      else{
	    	  // activar lectura y TASK  para adquisicion de datos.
	    	if (newValue == ST_CFG_SENSOR_ENABLE){

                #ifdef BIOMONITOR
	    	      dflag[2]++;
                  sensorConfig = ST_CFG_SENSOR_ENABLE;
                  // Se elimino el puntero del isrcallback NULL en la inicializacion, ya que en SensorTagMaxINit
                  // se le entrega el callback al ISR
                  BoolMax2=SensorMax30102_init();
                  DELAY_MS(300);


                  LedM=(Max_CMD & 0x06)>>1;
                  SensorMax30102_LedMode(LedM);
                  DELAY_MS(100);


                  //Led Current configuration
                  LedI=(Max_CMD & 0xF8)>>3;
                  SensorMax30102_LedI(LedI);
                  DELAY_MS(100);

                  // why does it need to init the Data characteristic?
               //   initCharacteristicValue(SENSOR_DATA, 12, SENSOR_DATA_LEN);
                  SensorMax30102_InterruptEnable(true);
                  DELAY_MS(50);


                  //Enable SnMax task
                  Task_setPri(Task_handle(&sensorTask),SENSOR_TASK_PRIORITY);
                  DELAY_MS(100);

                  // Enable the sensor to make readings
                  //  MaxDataReady=1;
                  BoolMax=SensorMax30102_enable(true);
                  dflag[3]++;


                #else  // BabyMonitor

                   dflag[2]++;
                   sensorConfig = ST_CFG_SENSOR_ENABLE;
                   // Se elimino el puntero del isrcallback NULL en la inicializacion, ya que en SensorTagMaxINit
                   // se le entrega el callback al ISR
                   BoolMax2=SensorMax30102_init();
                   DELAY_MS(300);


                   LedM=(Max_CMD &0x06)>>1;
                   SensorMax30102_LedMode(LedM);
                   DELAY_MS(100);


                   //Led Current configuration
                   LedI=(Max_CMD & 0xF8)>>3;
                   SensorMax30102_LedI(LedI);
                   DELAY_MS(100);

                   // why does it need to init the Data characteristic?
                   //   initCharacteristicValue(SENSOR_DATA, 12, SENSOR_DATA_LEN);
                   SensorMax30102_InterruptEnable(true);
                   DELAY_MS(50);


                   //Enable SnMax task
                   Task_setPri(Task_handle(&sensorTask),SENSOR_TASK_PRIORITY);
                   DELAY_MS(100);

                   // Enable the sensor to make readings
                   //  MaxDataReady=1;
                   BoolMax=SensorMax30102_enable(true);
                   dflag[3]++;

                #endif
	    	}
	      }
	    }
	    else
	    {
	      // Make sure the previous characteristics value is restored
	      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
	    }
	    // Make sure sensor is disabled
	    // SensorMax30102_enable(false);
	     Debuger_max++;
	    break;

	  case SENSOR_PERI:


        #ifdef BIOMONITOR

          Max30102Service_GetParameter(SENSOR_PERI, &newValue);
          newValueMax=newValue;
          if(newValueMax==20){ SensorMax30102_SampleRate(50);sensorPeriod = newValueMax * 1;}
          if(newValueMax==10){ SensorMax30102_SampleRate(100);sensorPeriod = newValueMax * 1;}
          if(newValueMax==5){ SensorMax30102_SampleRate(200);sensorPeriod = newValueMax * 1;}
          dflag[4]++;
//	        Max30102Service_GetParameter(SENSOR_PERI, &newValue);
//
//	        newValueMax=newValue;
//	        dflag[2]++;
//
//	        if( newValueMax<10){   newValueMax=10;}
//	        if( newValueMax>250){  newValueMax=250;}
//	        sensorPeriod = newValueMax * 1;
//
//	        Util_rescheduleClock(&periodicClock,sensorPeriod);



        #else  // BabyMonitor
	        Max30102Service_GetParameter(SENSOR_PERI, &newValue);
            newValueMax=newValue;
    //      if( newValueMax<10){   newValueMax=10;}
    //      if( newValueMax>250){  newValueMax=250;}
    //      sensorPeriod = newValueMax * 1;
            if(newValueMax==20){ SensorMax30102_SampleRate(50);sensorPeriod = newValueMax * 1;}
            if(newValueMax==10){ SensorMax30102_SampleRate(100);sensorPeriod = newValueMax * 1;}
            if(newValueMax==5){ SensorMax30102_SampleRate(200);sensorPeriod = newValueMax * 1;}
            dflag[4]++;
        #endif



	    break;

	  default:
	    // Should not get here
	    break;
	  }

}

/**************************************************************
 *  fn 		SensorTagMax30102_processInterrupt
 *  brief 	Interrupt habldler for max/MPU?
 *  param 	none
 *  return 	none
 *
 */
static void SensorTagMax30102_processInterrupt(void){

    //MaxDataReady=true;
    dflag[4]++;
    Semaphore_post(hSemMainLoop);
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

static void SensorTagMax30102_clockHandler(UArg arg){
  // Schedule readout periodically
    dflag[4]++;
    Semaphore_post(hSemMainLoop);
}


// Init
void SensorTagMax30102_init(void){
#ifdef BIOMONITOR
        dflag[7]++;
        // Add service
        Max30102Service_AddService();

        // Register Callbacks with profile
        Max30102Service_RegisterAppCBs(&sensorCallbacks);
        sensorPeriod = SENSOR_DEFAULT_PERIOD;
        dflag[7]++;

        // Initialize BLE profile, and configure the Max30102
        SensorTagMax30102_reset();
        dflag[7]++;
        // Lee el registro de interrupt para limpiar la interrupcion de data ready.
        //BoolMax2=SensorMax30102_init();

        //Conecta la interrupcion configurada en SensorMax30102, con su callback
        SensorMax30102_registerCallback(SensorTagMax30102_processInterrupt);

        //SWI connected to HWI to process it: aquire data and update characteristic in BLE profile
        //SWI_init();
        Sem_Max_init();

        initCharacteristicValue(SENSOR_PERI, SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof(uint8_t));
        dflag[7]++;
        // Create continuous clock for internal periodic events.
       // Util_constructClock(&periodicClock, SensorTagMax30102_clockHandler,20, 20, false, 0);


        // Initialise turn OFF the VLed 5V
        //   SensorMax30102_Auxfn(false);
        //    DP3s=false;


#else  // BabyMonitor

        // Add service
            Max30102Service_AddService();
            // Register Callbacks with profile
            Max30102Service_RegisterAppCBs(&sensorCallbacks);

            sensorPeriod = SENSOR_DEFAULT_PERIOD;
            // Initialize BLE profile, and configure the Max30102
            SensorTagMax30102_reset();

            // Lee el registro de interrupt para limpiar la interrupcion de data ready.
            //BoolMax2=SensorMax30102_init();

            //Conecta la interrupcion configurada en SensorMax30102, con su callback
            SensorMax30102_registerCallback(SensorTagMax30102_processInterrupt);

            //SWI connected to HWI to process it: aquire data and update characteristic in BLE profile
            //SWI_init();
            Sem_Max_init();

            initCharacteristicValue(SENSOR_PERI, SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof(uint8_t));


#endif

}

/*

Swi_Struct swiMax30102Alert;
Swi_Handle hSwiMax30102Alert;

*/


/*
static void SWI_init(){
 // SWI Initialization
	Swi_Params swiParams;
	Swi_Params_init(&swiParams);
	swiParams.priority = 4; // Must be bigger than 1, which is the main task pri
	Swi_construct(&swiMax30102Alert, swiMax30102Fxn, &swiParams, NULL);
	hSwiMax30102Alert = Swi_handle(&swiMax30102Alert);
}
*/


static void Sem_Max_init(){
// Semaphore initialization
   Semaphore_Params semParams;
   Semaphore_Params_init(&semParams);
   Semaphore_construct(&semMainLoop, 0, &semParams);
   hSemMainLoop = Semaphore_handle(&semMainLoop);
}



/*	*******************************************************************
 * SWI fxn
 * Read fifo Data, and translates its to
 * ***********************************************************/

/*
static void swiMax30102Fxn(UArg a0, UArg a1){
	Debuger_max++;

	b1m=SensorMax30102_readReg(REG_INTR_STATUS_1,1,rbuf1);
	b2m=SensorMax30102_readReg(REG_INTR_STATUS_2,1,rbuf2);

	BoolMax1=SensorMax30102_readFifo(sensorData);

	Max30102Service_SetParameter(MAX30102SERVICE_MAX_DATA_CHAR, MAX30102SERVICE_MAX_DATA_CHAR_LEN, sensorData);
	//
    //	BoolMax2=SensorMax30102_readReg(REG_INTR_STATUS_1,1,rbuf);

}

*/

/* EvenProcesor is used for sensor with no task associated.
 Sensor event processor  void SensorTagMax30102_processSensorEvent(void){}*/

// en tmp-> se inician las characteristicas: Sensor Data y Sensor_CNF. Ademas se SensorTMP007_init().
// reset characteristics and disable Sensor!!!=O
//
void SensorTagMax30102_reset(void){

    dflag[5]++;

    sensorConfig =ST_CFG_SENSOR_DISABLE;
	//Reinicia el sensor
	SensorMax30102_reset();
	// Apaga Sensor
	SensorMax30102_enable(false);

	dflag[5]++;
	// Deactivate task
	DELAY_MS(50);
	Task_setPri(Task_handle(&sensorTask), -1);

	initCharacteristicValue(MAX30102SERVICE_MAX_DATA_CHAR,0,SENSOR_DATA_LEN);
	initCharacteristicValue(MAX30102SERVICE_MAX_CONF_CHAR,sensorConfig,sizeof(uint8_t));

}



//SensorMax30102_writeReg(uint8_t addr,uint8_t data)

/**********************************************************************
 * PRIVATE's FUNCTIONS
 */

static void sensorTaskFxn(UArg a0, UArg a1){

	//uint8_t  aux[MAX30102SERVICE_MAX_DATA_CHAR_LEN];
	// inicializa buffers... no es necesario, solo se uso durante el desarrollo y debug

  rbuf1[0]=4;
  wbuf[0]=4;

  // Turn off sensor as an start condition--> Not nesseray?
  //SensorMax30102_Auxfn(false);

  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);
  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);

  while(true){

             dflag[0]++;
      if( sensorConfig == ST_CFG_SENSOR_ENABLE ){
          dflag[1]++;
          Semaphore_pend(hSemMainLoop,BIOS_WAIT_FOREVER);

        #ifdef BIOMONITOR
               b1m=SensorMax30102_readReg(REG_INTR_STATUS_1,1,rbuf1);
               b2m=SensorMax30102_readReg(REG_INTR_STATUS_2,1,rbuf2);
               BoolMax=SensorMax30102_readFifo(sensorData);
               dflag[0]++;
               Max30102Service_SetParameter(MAX30102SERVICE_MAX_DATA_CHAR, MAX30102SERVICE_MAX_DATA_CHAR_LEN, sensorData);
             //         Dummy data
             //          dataDumy[0]++;
             //          sensorData[0]=dataDumy[0];
             //          sensorData[1]=0;
             //          sensorData[2]=0;
             //           Max30102Service_SetParameter(MAX30102SERVICE_MAX_DATA_CHAR, MAX30102SERVICE_MAX_DATA_CHAR_LEN, sensorData);


               //  DELAY_MS(20);
        #else  // BabyMonitor
            BoolMax=SensorMax30102_readFifo(sensorData);
            Max30102Service_SetParameter(MAX30102SERVICE_MAX_DATA_CHAR, MAX30102SERVICE_MAX_DATA_CHAR_LEN, sensorData);

        #endif

	}
	else{
	      //DELAY_MS(sensorPeriod);
	        dflag[3]++;
	        DELAY_MS(50);
	}
	}
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
  Max30102Service_SetParameter(paramID, paramLen, data);
}


// Callback from Max30102 service indicating a value change
//paramID  identifies the characteristic that was changed

static void sensorConfigChangeCB(uint8_t paramID){

	// Wake up the App Thread
	SensorTag_charValueChangeCB(SERVICE_ID_MAX, paramID);

}


#endif // EXCLUDE_MAX30102
