/** ============================================================================
 *  @file       SensorMax30102.c
 *
 *  @brief      Driver for the Maxim Max30102 Hr and SpO2 Sensor.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "SensorMax30102.h"
#include "SensorUtil.h"
#include "SensorI2C.h"


/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/


 //    /* I2C *///    // Max30102 I2C bus
// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_0,Board_MAX30102_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()

#define REGISTER_LENGTH					1

/* -----------------------------------------------------------------------------
*  Type Definitions
* ------------------------------------------------------------------------------
*/


/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void SensorMax30102_Callback(PIN_Handle, PIN_Id);

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/
uint8_t _dev_address=0;

// Pins that are used by the MAX30102
// DP0-> Sensor interrupt
// DP2-> ¿?-- output for what?.. debug
	// Max30101 Interrupt Pin
	    // Board_DP0    | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS | PIN_HYSTERESIS,
	    // Power Sensor CTRL
	    //Board_DP3   | PIN_INPUT_DIS|PIN_NOPULL | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX ,

static PIN_Config MaxPinTable[]=
{
    #ifdef BIOMONITOR
       // OLD BioMonitor
         //Board_DP0    | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_HYSTERESIS,
         // Board_DP0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS | PIN_HYSTERESIS,
         // Board_DP3   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
     Board_DP0    | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_HYSTERESIS,
     // DP3 was used in baby monitor as power control for max30101 sesor. Now it's controlled in the start secuence of the BioMonitor
     //Board_DP3    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,


    #else  // BabyMonitor
        Board_DP0    | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_HYSTERESIS,
        Board_DP3    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    #endif


        PIN_TERMINATE
};
//		Negative EDGE IRQ    PIN_IRQ_NEGEDGE
//
static PIN_State  pinGpioState;
static PIN_Handle hMaxPin;

// The application may register a callback to handle interrupts
static SensorMax30102CallbackFn_t isrCallbackFn = NULL;
uint8_t PinConfig=0;




// Debuger variables
uint8_t ledPA1=0;
uint8_t LedMode=0x37;
uint8_t maxSensorD[6];

uint8_t flagSensor;
uint8_t flagSensor2;

uint16_t InitBool=false;
uint16_t OnSensor=false;
bool InEnBool=false;
uint8_t IntEn=0;
uint8_t IntOpen=12;
uint8_t Mval;
uint8_t rbuf_max[1];
uint8_t C_buff[1];
uint8_t SenMaxCB;
uint8_t Max_DB[8];


bool fifoBool=false;
/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/



/*******************************************************************************
 * @fn          SensorMax30102_init
 *
 * @brief       Initialize the sensor
 *
 * @return      true if success
 */

bool SensorMax30102_init(void){

	bool ret= true;
	rbuf_max[0]=0;

	if(!PinConfig){
	    hMaxPin = PIN_open(&pinGpioState, MaxPinTable);
	    DELAY_MS(200);

	    // Register MAX30102 interrupt
        IntOpen=PIN_registerIntCb(hMaxPin, SensorMax30102_Callback);
        SensorMax30102_InterruptEnable(false);


        //Clear startUp Interrupt
        DELAY_MS(500);
        PinConfig=1;
        }

    // Turn ON Vled 5V
	//SensorMax30102_Auxfn(true);


    DELAY_MS(1500);
    // TURN ON SENSOR, AND CLEAR PWR UP INTR
    InitBool=SensorMax30102_readReg(REG_INTR_STATUS_1,1,rbuf_max);


    flagSensor=20;
    flagSensor2=1;
    DELAY_MS(10);

    if(!SensorMax30102_writeCheck(REG_INTR_ENABLE_1,0xC0)){ // C0-> A_Full_EN and PPG_RDY_EN Interruptions configuration. 0x00-> all off
    	  	return false;   	}



    if(!SensorMax30102_writeCheck(REG_INTR_ENABLE_2,0x00))  // wtf
		return false;



	if(!SensorMax30102_writeCheck(REG_FIFO_WR_PTR,0x00))   //FIFO_WR_PTR[4:0]
		return false;

	if(!SensorMax30102_writeCheck(REG_OVF_COUNTER,0x00))   //OVF_COUNTER[4:0]
		return false;

	if(!SensorMax30102_writeCheck(REG_FIFO_RD_PTR,0x00))   //FIFO_RD_PTR[4:0]
		return false;

	if(!SensorMax30102_writeCheck(REG_FIFO_CONFIG,0x0f))   //sample avg = 1, fifo rollover=false, fifo almost full = 17
		return false;

	if(!SensorMax30102_writeCheck(REG_MODE_CONFIG,0x87))  	 //0xX2 for Red only, 0xX3 for SpO2 mode 0xX7 multimode LED
	    return false;				 						// 0x8X to shut down mode(sleep)

	if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x25)) 	 // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz) 001 for 100 Hz,SpO2 200Hz-> 010,
		return false;                                       // LED pulseWidth (400us)-> 11, 118us->01

	if(!SensorMax30102_writeCheck(REG_LED1_PA,0x32))  		 //Choose value for ~ 10mA for LED1
		return false;

	if(!SensorMax30102_writeCheck(REG_LED2_PA,0x32))  	 	//Choose value for ~ 10mA for LED2
		return false;

	if(!SensorMax30102_writeCheck(REG_LED3_PA,0x64))    // 32-> 10mA, 64-> 20mA, 96-> 30mA, C8->40mA        //Choose value for ~ 10mA for LED3
	        return false;


	if(!SensorMax30102_writeCheck(REG_PILOT_PA,0x32))    // Choose value for ~ 10mA for Pilot LED
	        return false;


//	if(!SensorMax30102_writeCheck(REG_PILOT_PA,0x7f))  	 // Choose value for ~ 25mA for Pilot LED
//		return false;

	if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x23))    // Control multi Led operation, Slot 1 and 2
	        return false;

	if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL2,0x00))    //  Control multi Led operation Slot 3 and 4
	        return false;

	//REG_MULTI_LED_CTRL1 0x11
	//REG_MULTI_LED_CTRL2 0x12

	if(!SensorMax30102_writeCheck(REG_PROX_INT_THRESH,0xFF))    // Proximity MOde: 0xFF: disable
	            return false;


    flagSensor++;


    return ret;
}

#ifdef BIOMONITOR

/*******************************************************************************
 * @fn          Max30102_SampleRate
 *
 * @brief       Configure the sample rate acquisition of the max3010X
 *
 * @param       Fr(Hz)
 *
 * @re
*/
bool SensorMax30102_SampleRate(uint8_t Fref){

    SensorMax30102_InterruptEnable(false);
    DELAY_MS(60);
    SensorMax30102_enable(false);


    switch(Fref){
        case 50:
            maxSensorD[0]++;
            if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x21))     // SPO2_ADC range = 4096nA,
                return false;                                        //SPO2 sample rate 000 for 50Hz, 001 for 100Hz,SpO2 200Hz-> 010,
                                                                     // LED pulseWidth (400us)-> 11, 118us->01
            break;
        case 100:
            maxSensorD[1]++;
            if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x25))     // SPO2_ADC range = 4096nA,
                      return false;                                        //SPO2 sample rate 000 for 50Hz, 001 for 100Hz,SpO2 200Hz-> 010,
                                                                          // LED pulseWidth (400us)-> 11, 118us->01
            break;
        case 200:
            maxSensorD[2]++;
            if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x29))     // SPO2_ADC range = 4096nA,
                           return false;                                        //SPO2 sample rate 000 for 50Hz, 001 for 100Hz,SpO2 200Hz-> 010,
                                                                              // LED pulseWidth (400us)-> 11, 118us->01
            break;
    }

    SensorMax30102_InterruptEnable(true);
    DELAY_MS(100);
    SensorMax30102_enable(true);
    return true;

}




/*******************************************************************************
 * @fn          Max30102_LedMode
 *
 * @brief       Controls wich colors led are working.
 *
 * @param
 *
 * @re
*/
bool SensorMax30102_LedMode(uint8_t CMD){



    switch(CMD){
           case 1: // 1 Slot 1: Green; Slot 2: IR
               maxSensorD[3]++;

               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x23))    // Multi Led operation,
                          return false;
               break;
           case 2:// 2 Slot 1: RED; Slot 2: IR
               maxSensorD[4]++;

               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x21))    // Multi Led operation,
                          return false;
               break;
           case 3:// 3 Slot 1: Green; Slot 2: Red
               maxSensorD[5]++;

               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x13))    // Multi Led operation,
                          return false;
               break;
           default:
               maxSensorD[3]++;
               maxSensorD[4]++;
               maxSensorD[5]++;
               // case 0 Slot 1: Green; Slot 2: IR

               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x21))    // Multi Led operation,
               return false;
               break;

    }
    SensorMax30102_writeCheck(REG_MULTI_LED_CTRL2,0x00);
    return true;
}
/*******************************************************************************
 * @fn          Max30102_LedI
 *
 * @brief       Control current in each led
 *
 * @param       enable - flag to turn the sensor on/off
 *
 * @re
*/
bool SensorMax30102_LedI(uint8_t CMD){

    if (CMD!=0){
        ledPA1=(uint8_t)CMD*255/50;}
    else{
        ledPA1=0x32;  }

// ledPA2 y 3 se definen si es que se requiere en futuro configurar la corriente en forma individual
//    uint8_t ledPA2=0;
//    uint8_t ledPA3=0;
//    ledPA2=ledPA1;
//    ledPA3=ledPA1;


    if(!SensorMax30102_writeCheck(REG_LED1_PA,ledPA1))
                     return false;

    if(!SensorMax30102_writeCheck(REG_LED2_PA,ledPA1))
                    return false;

    if(!SensorMax30102_writeCheck(REG_LED3_PA,ledPA1))
                     return false;


return true;

}

#else  // BabyMonitor
/*******************************************************************************
 * @fn          Max30102_SampleRate
 *
 * @brief       Configure the sample rate acquisition of the max3010X
 *
 * @param       Fr(Hz)
 *
 * @re
*/
bool SensorMax30102_SampleRate(uint8_t Fref){

    SensorMax30102_InterruptEnable(false);
    DELAY_MS(60);
    SensorMax30102_enable(false);


    switch(Fref){
        case 50:
            maxSensorD[0]++;
            if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x21))     // SPO2_ADC range = 4096nA,
                return false;                                        //SPO2 sample rate 000 for 50Hz, 001 for 100Hz,SpO2 200Hz-> 010,
                                                                     // LED pulseWidth (400us)-> 11, 118us->01
            break;
        case 100:
            maxSensorD[1]++;
            if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x25))     // SPO2_ADC range = 4096nA,
                      return false;                                        //SPO2 sample rate 000 for 50Hz, 001 for 100Hz,SpO2 200Hz-> 010,
                                                                          // LED pulseWidth (400us)-> 11, 118us->01
            break;
        case 200:
            maxSensorD[2]++;
            if(!SensorMax30102_writeCheck(REG_SPO2_CONFIG,0x29))     // SPO2_ADC range = 4096nA,
                           return false;                                        //SPO2 sample rate 000 for 50Hz, 001 for 100Hz,SpO2 200Hz-> 010,
                                                                              // LED pulseWidth (400us)-> 11, 118us->01
            break;
    }

    SensorMax30102_InterruptEnable(true);
    DELAY_MS(100);
    SensorMax30102_enable(true);
    return true;

}




/*******************************************************************************
 * @fn          Max30102_LedMode
 *
 * @brief       Controls wich colors led are working.
 *
 * @param
 *
 * @re
*/
bool SensorMax30102_LedMode(uint8_t CMD){


    switch(CMD){
           case 1: // 1 Slot 1: Green; Slot 2: IR
               maxSensorD[3]++;
               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x23))    // Multi Led operation,
                          return false;
               break;
           case 2:// 2 Slot 1: RED; Slot 2: IR
               maxSensorD[4]++;
               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x21))    // Multi Led operation,
                          return false;
               break;
           case 3:// 3 Slot 1: Green; Slot 2: Red
               maxSensorD[5]++;
               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x13))    // Multi Led operation,
                          return false;
               break;
           default:
               maxSensorD[3]++;
               maxSensorD[4]++;
               maxSensorD[5]++;
               // case 0 Slot 1: Green; Slot 2: IR
               if(!SensorMax30102_writeCheck(REG_MULTI_LED_CTRL1,0x23))    // Multi Led operation,
               return false;
               break;

    }


}
/*******************************************************************************
 * @fn          Max30102_LedI
 *
 * @brief       Control current in each led
 *
 * @param       enable - flag to turn the sensor on/off
 *
 * @re
*/
bool SensorMax30102_LedI(uint8_t CMD){

    if (CMD!=0){
        ledPA1=(uint8_t)CMD*255/50;}
    else{
        ledPA1=0x4A;  }

// ledPA2 y 3 se definen si es que se requiere en futuro configurar la corriente en forma individual
//    uint8_t ledPA2=0;
//    uint8_t ledPA3=0;
//    ledPA2=ledPA1;
//    ledPA3=ledPA1;


    if(!SensorMax30102_writeCheck(REG_LED1_PA,ledPA1))
                     return false;

    if(!SensorMax30102_writeCheck(REG_LED2_PA,ledPA1))
                    return false;

    if(!SensorMax30102_writeCheck(REG_LED3_PA,ledPA1))
                     return false;


return true;

}
#endif

/*******************************************************************************
 * @fn          Max30102_enable
 *
 * @brief       Enable/disable measurements
 *
 * @param       enable - flag to turn the sensor on/off
 *
 * @return      none
 */
bool SensorMax30102_enable(bool enable)
{
    uint8_t val;
    bool success;

    if (enable) {
        //turn on the max3010X
    	SensorMax30102_readReg(REG_MODE_CONFIG,1,&val);
    	//val=val & 0x7F;
    	//val=val & 0x37;
    	val=val & LedMode;
    	OnSensor=50;
        }
    else  {
    	// Make sure pin interrupt is disabled.
    	SensorMax30102_readReg(REG_MODE_CONFIG,1,&val);
        PIN_setInterrupt(hMaxPin,PIN_ID(Board_DP0)|PIN_IRQ_DIS );
    	//Turn off the Max30102
    	val=val | 0x80;
    	OnSensor=15;
    }

	success=SensorMax30102_writeCheck(REG_MODE_CONFIG,val);

	return success;

}

/*******************************************************************************
 * @fn          SensorBmp280_read
 *
 * @brief       Read temperature and pressure data
 *
 * @param       data - buffer for temperature and pressure (6 bytes)
 *
 * @return      TRUE if valid data
 */
//bool SensorMax30102_readFifo(uint32_t *R_data, uint32_t *IR_data)

bool SensorMax30102_readFifo(uint8_t *Max_data)
{
    bool success;

    //uint8_t aux_by;
	//read and clear status register
	//SensorMax30102_readReg(REG_INTR_STATUS_1,1,&aux_by);
    // SensorMax30102_readReg(REG_INTR_STATUS_2,1,&aux_by);

    // Read data register.
	success=SensorMax30102_readReg(REG_FIFO_DATA,6,Max_data);
	// fifoBool=success;
/*
	uint8_t data_buff[6];
	*IR_data=0;
	*R_data=0;

	if(success){

		ul=(unsigned char)data_buff[0];
		ul=ul<<16;
		*R_data += ul;

		ul=(unsigned char)data_buff[1];
		ul=ul<<8;
		*R_data += ul;

		ul=(unsigned char)data_buff[2];
		*R_data += ul;


		ul=(unsigned char)data_buff[3];
		ul=ul<<16;
		*IR_data += ul;

		ul=(unsigned char)data_buff[4];
		ul=ul<<8;
		*IR_data += ul;

		ul=(unsigned char)data_buff[5];
		*IR_data += ul;


		*R_data  &= 0x03FFFF;  //Mask MSB [23:18]
		*IR_data &= 0x03FFFF;  //Mask MSB [23:18]

		}

*/


	return success;
}

/*******************************************************************************
 * @fn          SensorMax30102_readRed
 *
 * @brief       read num bytes in register addr. Bytes are saved in _buff
 *
 * @return      true if passed
 */

bool SensorMax30102_readReg(uint8_t addr, int num, uint8_t* _buff){

	 bool success;
	 Max_DB[3]++;
	 if (!SENSOR_SELECT()) {
	     Max_DB[4]++;
		 return false;
	    }

	 success = SensorI2C_readReg(addr,_buff, num);

	 Max_DB[3]++;
	 //flagSensor++;

	 SENSOR_DESELECT();

	 return success;

}

/*******************************************************************************
 * @fn          SensorMax30102_writeRed
 *
 * @brief       Write data in the register "addr".
 *
 * @return      true if passed
 */


bool SensorMax30102_writeReg(uint8_t addr,uint8_t data){

	 bool success;
	 Mval=data;
	 Max_DB[0]++;
	 if (!SENSOR_SELECT())
	     {Max_DB[1]++;
			return false;
		}

	 success=SensorI2C_writeReg(addr,&Mval ,REGISTER_LENGTH);

	 SENSOR_DESELECT();
	 Max_DB[2]++;
	 return success;

}
/*******************************************************************************
 * @fn          SensorMax30102_writeCheck
 *
 * @brief       Write data in addr and read it to check if the write operation was succesfull
 *
 * @return      true if passed
 */

bool SensorMax30102_writeCheck(uint8_t addr,uint8_t data){
	bool success = true;

	success=SensorMax30102_writeReg(addr, data);

	if(!success){
		 flagSensor2= flagSensor2+10;
		 //SENSOR_DESELECT();
		 return false;	}
	else {
		DELAY_MS(5);
		success=SensorMax30102_readReg(addr,1,C_buff);
		}


	if (success && C_buff[0]==data){
		return success;	}
	else {
		 flagSensor2= flagSensor2+100;
		return false;}

	//return success;
}
/*******************************************************************************
 * @fn          SensorMax30102_reset()
 *
 * @brief       Restart sensor
 *
 * @return      true if passed
 */

bool SensorMax30102_reset(){
	bool success=false;

	SensorMax30102_InterruptEnable(false);
	success=SensorMax30102_writeReg(REG_MODE_CONFIG,0x40);

	return success;

}


/*******************************************************************************
 * @fn          SensorMax30102_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      true if passed
 */

bool SensorMax30102_test(void)
{
	return true;
}


// Enable Interrupt from Max30102 sensro

void SensorMax30102_InterruptEnable(bool bo){

	if(bo){
		InEnBool=PIN_setInterrupt(hMaxPin,PIN_ID(Board_DP0)|PIN_IRQ_NEGEDGE);
		IntEn=1;
	}else{
		InEnBool=PIN_setInterrupt(hMaxPin,PIN_ID(Board_DP0)|PIN_IRQ_DIS);
		IntEn=0;
	}

}

/*******************************************************************************
 * @fn          SensorMax30102_registerCallback
 *
 * @brief       Register a call back for interrupt processing
 *
 * @return      none
 */
 void SensorMax30102_registerCallback(SensorMax30102CallbackFn_t pfn){
	isrCallbackFn = pfn;
 }

/*******************************************************************************
 *  @fn         SensorMax30102_Callback
 *
 *  Interrupt service routine for the MAX30102
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void SensorMax30102_Callback(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == Board_DP0)
        {
        if (isrCallbackFn != NULL)
        {
        	//SenMaxCB++;
        	isrCallbackFn();
        }
    }
}

// aux function to test things
void SensorMax30102_Auxfn(bool BOO){
    #ifdef BIOMONITOR
        if(BOO){
            Max_DB[7]=6;
            PIN_setOutputValue(hMaxPin,Board_DP3,Sensor_PWR_ON);}
        else{
            Max_DB[7]=1;
            PIN_setOutputValue(hMaxPin,Board_DP3,Sensor_PWR_OFF);}

    #else  // BabyMonitor
        if(!BOO){
                PIN_setOutputValue(hMaxPin,Board_DP3,0);
            }else{
                PIN_setOutputValue(hMaxPin,Board_DP3,1);
            }

    #endif


}
