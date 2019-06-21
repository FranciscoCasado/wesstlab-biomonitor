// sensortag_EDA_SCS.c
//C:\ti\simplelink\ble_sdk_2_02_01_18\src\examples\sensortag\cc26xx\app




/*********************************************************************
 * INCLUDES


 */
#include "gatt.h"
#include "gattservapp.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>



#include "SensorUtil.h"
#include "SensorTagTest.h"
#include <string.h>

#include "sensortag_EDA.h"
// Ble service
#include "EDAService.h"

// SCS
#include "scif.h"
#define BV(x)    (1 << (x))
#include "board.h"

/*
#include "Max30102Service.h" 				//ok
#include "sensortag_max30102.h"				//ok
#include "SensorMax30102.h"					//Not used: scif.h
#include "SensorTagTest.h"
#include "SensorUtil.h"
#include "util.h"
 */

#include "string.h"
#include "st_util.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   10

// Constants for two-stage reading
#define SENSOR_FSM_PERIOD       80

// Length of the data for this sensor
//#define SENSOR_DATA_LEN         4
//#define SENSOR_DATA_LEN         20
#define SENSOR_DATA_LEN         EDASERVICE_DATA_CHAR_LEN

// Event flag for this sensor
#define SENSOR_EVT              12

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  500//600



 /*********************************************************************
 * TYPEDEFS
 */

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
	APP_MSG_SC_CTRL_READY,		/*SC CTRL ready msg*/
  	APP_MSG_SC_TASK_ALERT,		/*SC task Alert msg*/
    APP_MSG_ADC_AVAILABLE,
} app_msg_types_t;



// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;



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
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;


// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;


// Main loop Semaphore  -- task semaphore
static Semaphore_Struct semMainLoop;
static Semaphore_Handle hSemMainLoop;


// SWI structure
Swi_Struct swiEDAAlert;
Swi_Handle hSwiEDAAlert;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;


// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

// two buffer read and store test
//#define data_buffer_size 256//256
// Structures
//static ADCBuf_Handle ADCBuf_J_Handle;
//static ADCBuf_Conversion ADCBuf_J_Conversion;
//static ADCBuf_FxnTable ADCBuf_J_fxnTable;

uint8_t dflag_EDA[10];
uint8_t daux2=10;

uint8_t  EDA_Data[SENSOR_DATA_LEN];
uint16 Vo=0;
uint16 Vb=0;
uint16 temp=0;
float TempA=0;



/*********************************************************************
 * LOCAL FUNCTIONS
 */


static void SWI_init(void);
static void SCS_init(void);

// SCS  functions
static void scCtrlReadyCallback(void);
static void scTaskAlertCallback(void);

void swiEDAFxn(UArg a0, UArg a1);

void SensorTagEDA_reset(void);
static void sensorTaskFxn(UArg a0, UArg a1);
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static void processTaskAlert(void);


//static void Audio_processApplicationMessage(app_msg_t *pMsg);
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
		sensorConfigChangeCB,		// Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      SensorTagBar_createTask
 *
 * @brief   Task creation function for barometer sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagEDA_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

// Semaphore Init
 void Sem_EDA_init(void){
   Semaphore_Params semParams;
   Semaphore_Params_init(&semParams);
   Semaphore_construct(&semMainLoop, 0, &semParams);
   hSemMainLoop = Semaphore_handle(&semMainLoop);
}

// Function for initialize SWI
static void SWI_init(void){
	Swi_Params swiParams;
	Swi_Params_init(&swiParams);
	swiParams.priority = 4; // Must be bigger than 1, which is the main task pri
	Swi_construct(&swiEDAAlert, swiEDAFxn, &swiParams, NULL);
	hSwiEDAAlert = Swi_handle(&swiEDAAlert);
}

// SCS Init
static void SCS_init(void){

    // Initialize the Sensor Controller
   scifOsalInit();

   // Create response to HWI
   scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
   scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
   // not yet?
   //daux2=scifInit(&scifDriverSetup);

	// Set the Sensor Controller task tick interval to 125 ms
    // uint32_t rtc_Hz = 1;  // 1Hz RTC

   // 1kHz Samples per second-> 0x42=dx66
  // scifStartRtcTicksNow(0x00000042/ rtc_Hz);

   // 1 sample per  15 ms-> 67 Hz
   // scifStartRtcTicksNow(0x000003E8/ rtc_Hz);


   // 1 sample per second
   //scifStartRtcTicksNow(0x00010000/ rtc_Hz);


  Vo=0;
  Vb=0;
  // Configure Sensor Controller tasks
   dflag_EDA[7]++;

  // Start Sensor Controller task
 //  scifStartTasksNbl(BV(SCIF_DUSK2DAWN_TASK_ID));
  // scifStopTasksNbl(BV(SCIF_DUSK2DAWN_TASK_ID));
}

/*********************************************************************
 * @fn      SensorTagBar_init
 *
 * @brief   Initialization function for the SensorTag barometer
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagEDA_init(void){

   	//Add Service
	EDAService_AddService();
	//Register callbacks with profile
	EDAService_RegisterAppCBs(&sensorCallbacks);

	//Initialize the module state variables
	SensorTagEDA_reset();

	//Semaphore Initialization
	 Sem_EDA_init();

	// SWI Initialization
	 SWI_init();
	// SCS Initiliazation
	 SCS_init();

	 //ExternalFlash_Init structures--> moved to characteristics change, for debugin reasons only.
	//  FLbuff_init();

	 // Initialize queue for application messages.
   // Note: Used to transfer control to application thread from e.g. interrupts.

//	Queue_construct(&applicationMsgQ, NULL);
//	hApplicationMsgQ = Queue_handle(&applicationMsgQ);

	initCharacteristicValue(SENSOR_PERI, SENSOR_DEFAULT_PERIOD/ SENSOR_PERIOD_RESOLUTION, sizeof(uint8_t));

 	dflag_EDA[8]++;
//    Audio_flash_buff[2]=45;
//    Audio_flash_buff[3]=33;
}

static void scCtrlReadyCallback(void)
{
  // Do we need this shit?
  // Notify application `Control READY` is active
   // user_enqueueRawAppMsg(APP_MSG_SC_CTRL_READY, NULL, 0);
    dflag_EDA[6]++;
} // scCtrlReadyCallback


// SCI callbacl<-> task Alert Callback
static void scTaskAlertCallback(void)
{
  // Notify application `Task ALERT` is active
	dflag_EDA[5]++;
	Swi_post(hSwiEDAAlert);
 } // scTaskAlertCallback

// SWI Callback
void swiEDAFxn(UArg a0, UArg a1){
    dflag_EDA[6]++;
    processTaskAlert();
}

// Get data from SCS and :1) enqueRawMSG 2) SetBleParameter
// try both
//
//static void processTaskAlert(void){
//  // Clear the ALERT interrupt source
//  dflagA[6]++;
//  scifClearAlertIntSource();
//
//  // Get 'state.dawn', and set dawnStr to appropriate string
//  //dawn = scifTaskData.dusk2dawn.state.dawn;
//
//  adcV = scifTaskData.dusk2dawn.output.adcValue;
//  ADC_VAL adc ={.adcValue = adcV};
//
//  // Add data to 20 bytes buffer, and send the message to make a copy of the the buffer, and send by ble
//  // later: must save data in restricted flash memory block, and send msg about the address
//  //to the main task to read from there
//
//  // Implement MSG queu Structure
//   user_enqueueRawAppMsg(APP_MSG_ADC_AVAILABLE,(uint8_t *)&adc, sizeof(adcV));
//   dflagA[6]++;
//
//
// // Acknowledge the ALERT event
//  scifAckAlertEvents();
//} // processTaskAlert


/************************************************************************************************/

static void processTaskAlert(void){
  // Clear the ALERT interrupt source
    scifClearAlertIntSource();
//    dflag_EDA[7]++;

#ifdef BIOMONITOR
    Vb=scifTaskData.adceda.output.Vb;
    Vo=scifTaskData.adceda.output.Vo;
    temp=scifTaskData.adceda.output.TempA;
    Semaphore_post(hSemMainLoop);

    #else  // BabyMonitor


    #endif

  // Acknowledge the ALERT event
  dflag_EDA[7]++;
  scifAckAlertEvents();
} // processTaskAlert2

/************************************************************************************************/

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the humidity readout task
 *
 * @return  none
 */

static void sensorTaskFxn(UArg a0, UArg a1)
{

  dflag_EDA[0]++;
  //Task_Stat statbuf;

  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);

  // Task loop  // Application main loop

  while (true){


	   // Waits for a signal to the semaphore associated with the calling thread.
	   // Note that the semaphore associated with a thread is signaled when a
	   // message is queued to the message receive queue of the thread or when
	   // ICall_signal() function is called onto the semaphore.
	   //if (sensorConfig == ST_CFG_SENSOR_ENABLE)   {}
       //dflag_EDA[4]++;
		Semaphore_pend(hSemMainLoop, BIOS_WAIT_FOREVER);
		//dflag_EDA[1]++;
		// Calculate Rskin?
		// Update the Vo and the Vb Values in the BLE profile
		dflag_EDA[4]++;
        #ifdef BIOMONITOR
		        EDA_Data[0]=(uint8_t) (0x00FF & Vo);
		        EDA_Data[1]=(uint8_t) ((0xFF00 & Vo)>>8);

		        EDA_Data[2]=(uint8_t) (0x00FF & Vb);
		        EDA_Data[3]=(uint8_t) ((0xFF00 & Vb)>>8);

		        EDA_Data[4]=(uint8_t) (0x00FF & temp);
		        EDA_Data[5]=(uint8_t) ((0xFF00 & temp)>>8);

		        TempA=( ((float)temp)*(-1)*0.00481)+46.505;

		        EDAService_SetParameter(EDASERVICE_DATA_CHAR,EDASERVICE_DATA_CHAR_LEN,EDA_Data);



		#else  // BabyMonitor
		        Vo++;


		        EDAService_SetParameter(EDASERVICE_DATA_CHAR,EDASERVICE_DATA_CHAR_LEN,EDA_Data);

        #endif




  }
 }





/*********************************************************************
 * @fn      SensorTagBar_processCharChangeEvt
 *
 * @brief   SensorTag Barometer event handling
 *
 */

void SensorTagEDA_processCharChangeEvt(uint8_t paramID){

  uint8_t newValue;
  dflag_EDA[0]++;
  switch (paramID){

  case SENSOR_CONF:
    if ((SensorTag_testResult() & SENSOR_EDA_TEST_BM) == 0){
    	dflag_EDA[0]=dflag_EDA[0]+10;
    	sensorConfig = ST_CFG_ERROR;
    }

    if (sensorConfig != ST_CFG_ERROR){

      EDAService_GetParameter(SENSOR_CONF, &newValue);
      if (newValue == ST_CFG_SENSOR_DISABLE) {

        // STOP SCS
        scifStopTasksNbl(BV(SCIF_ADCEDA_TASK_ID));
        DELAY_MS(200);
        scifResetTaskStructs(BV(SCIF_ADCEDA_TASK_ID),0xF);
        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);


        sensorConfig = ST_CFG_SENSOR_DISABLE;
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
        dflag_EDA[3]++;
       }

      else{

         if (newValue == ST_CFG_SENSOR_ENABLE) {
            dflag_EDA[2]++;
            sensorConfig = ST_CFG_SENSOR_ENABLE;
            // Activate task-- suspended for flash memory debuging
            Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
            DELAY_MS(10);
            dflag_EDA[2]++;



            //SCS  init?
            DELAY_MS(10);
            daux2=scifInit(&scifDriverSetup);// AQUI SE QUEDA PEGADO!!!
            dflag_EDA[2]++;

            //  0x10 => 4kHz
            //scifStartRtcTicksNow(0x00000010/ 1);
            // 0x3332 Dx13106 -> 0,199[s]->5 sample per second 5 Hz
            DELAY_MS(10);
            scifStartRtcTicksNow(0x00003332);
            dflag_EDA[2]++;

            //Start SCS task
            scifStartTasksNbl(BV(SCIF_ADCEDA_TASK_ID));
            dflag_EDA[2]++;

         }

       }
    }
    else {
        dflag_EDA[0]=33;
        // Make sure the previous characteristics value is restored
        initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t)); }

    break;

  case SENSOR_PERI:
	 //   Barometer_getParameter(SENSOR_PERI, &newValue);
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION;
  //  dflag_EDA[4]++;
     // have to reset the frequency of the SCS .. .
    //turn off the module and restart it?
    break;

  default:
    // Should not get here
    break;
  }
  dflag_EDA[0]++;
}

/*********************************************************************
 * @fn      SensorTagBar_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagEDA_reset(void)
{
    sensorConfig = ST_CFG_SENSOR_DISABLE;
    initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
    initCharacteristicValue(SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof(uint8_t));


    scifStopTasksNbl(BV(SCIF_ADCEDA_TASK_ID));
    DELAY_MS(200);
    scifResetTaskStructs(BV(SCIF_ADCEDA_TASK_ID),0xF);
    // Deactivate task
    Task_setPri(Task_handle(&sensorTask), -1);

}

/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      sensorConfigChangeCB
 *
 * @brief   Callback from Barometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
    dflag_EDA[9]++;
    // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_EDA, paramID);
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
  uint8_t data[SENSOR_DATA_LEN];

  memset(data, value, paramLen);
  EDAService_SetParameter(paramID, paramLen, data);
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(hSemMainLoop);
  }
}

//static void Audio_processApplicationMessage(app_msg_t *pMsg)
//{
////  char_data_t *pCharData = (char_data_t *)pMsg->pdu;
////  dflagA[13]++;
//
//  switch (pMsg->type)
//  {
//  //  APP_MSG_SC_CTRL_READY,		/*SC CTRL ready msg*/
//  //  APP_MSG_SC_TASK_ALERT,		/*SC task Alert msg*/
//
//    case APP_MSG_ADC_AVAILABLE:
//    //	dflagA[14]++;
//    	ADC_VAL *adcV = (ADC_VAL *)pMsg->pdu;
//    	audio_data[0]=adcV->adcValue;
//
//		Audio_flash_buff[0]=(uint8_t)(audio_data[0]&0x00FF);
//		Audio_flash_buff[1]=(uint8_t)((audio_data[0]&0xFF00)>>8);
//
//        //	Audio_flash_buff[2]++;
//        //	Audio_flash_buff[3]=Audio_flash_buff[3]+3;
//        //  AudioService_SetParameter(SENSOR_DATA, SENSOR_DATA_LEN, Audio_flash_buff);
//		FLbuff_saveSample(Audio_flash_buff,Audio_flash_size);
//
//
//
//    	break;
//
//    default:
//    	break;
//  }
//
//
//}

/*********************************************************************
*********************************************************************/
