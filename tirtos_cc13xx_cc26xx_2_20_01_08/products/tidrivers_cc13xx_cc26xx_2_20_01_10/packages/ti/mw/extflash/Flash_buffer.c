/*
 	Flash_buffer.c
	Flash buffer library, for buffering data.
	CC2650
	january 2017
	Jose Caceres

*/ 



/*******************************************************************************
 * INCLUDES
 */

#include <string.h>
#include "hal_board.h"
#include "oad_target.h"
#include <ti/mw/extflash/ExtFlash.h>
#include "ext_flash_layout.h"
#include "Flash_buffer.h"

#include "ExtFlash.h"
//#include "AudioService.h"

#include "SensorUtil.h"
#include "board.h"

/*******************************************************************************
 * Constants and macros
 */
#define PROG_BUF_SIZE             16
#define PAGE_0                    0
#define PAGE_1                    1
#define PAGE_31                   31

// JC
#define PAGE_SIZE				  4096 // In bytes
#define DataSample_SIZE			  1	   // In words :4 bytes
#define MIN_PAGE				  90   // 0 if Oad Is not required
#define MAX_PAGE                  127//127   // 0...127 -> 128 pages of 4K bytes each

// ¿?
#define APP_IMAGE_START           0x1000
#define BOOT_LOADER_START         0x1F000

#define MAX_BLOCKS                (EFL_SIZE_IMAGE_APP / OAD_BLOCK_SIZE)


/*******************************************************************************
 * PRIVATE VARIABLES
 */
static bool isOpen = false;

//  static uint32_t lastW_addr;
//  static uint32_t oldestW_addr;

FLbuff_Instant_t new;
FLbuff_Instant_t old;

uint8_t OldWritenData=0;
uint8_t LastPage_flag=0;



//uint8_t auxBuff[4];

uint8_t FL_dflag[3];
bool    b_FL_dflag=false;
uint8_t WR_flag =1;
uint8_t initFlashFlag=0;

bool PinFlag= FALSE;

//static ExtImageInfo_t imgInfo;

///
// Pins that are used by the MAX30102
// DP2-> Sensor interrupt
// BUZZER->
static PIN_Config MicPinTable[]=
{
 //| PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_HYSTERESIS,
    Board_AUDIODO    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

static PIN_State  pinGpioState;
static PIN_Handle hMicPin;



/*******************************************************************************
 * PRIVATE FUNCTIONS
 */

/*******************************************************************************
 * FUNCTIONS
 */

// aux function to test things
void AMic_Auxfn2(bool BOO1){
    if(!BOO1){
        PIN_setOutputValue(hMicPin,Board_AUDIODO,0);
    }else{
        PIN_setOutputValue(hMicPin,Board_AUDIODO,1);
    }
}


void AMic_AuxfnB(bool BOO2){
    if(!BOO2){
        PIN_setOutputValue(hMicPin,Board_BUZZER,0);
    }else{
        PIN_setOutputValue(hMicPin,Board_BUZZER,1);
    }
}

void FLbuff_init(){

	// Initialize now marker
	// initalial page: 0x60 or 96??
	// Pointer for new Data
    new.offset=0;
	new.page=MIN_PAGE;

	// Pointer for reading data
	old.offset=0;
    old.page=MIN_PAGE;

    //Flag to move the old pointer if new data is been writen
    OldWritenData=0;
    //Flag to stop reading data when all data has been read.
    LastPage_flag=0;


    hMicPin = PIN_open(&pinGpioState, MicPinTable);
    if(hMicPin != NULL){      PinFlag=true;   }

    AMic_Auxfn2(FALSE);
    AMic_AuxfnB(FALSE);
    // Register MAX30102 interrupt
    //IntOpen=PIN_registerIntCb(hMaxPin, SensorMax30102_Callback);


}

void FLbuff_reset(){
    // Erase page?
    FLbuff_eraseFlash(new.page);
}


/*******************************************************************************
 * @fn      FLbuff_open
 *
 * @brief
 */
uint8_t FLbuff_open(void){
    FL_dflag[2]=10;

    if (!isOpen)
    {
        isOpen = ExtFlash_open();


        if(isOpen){
            FL_dflag[2]++; }
    }
    return FL_dflag[2];
//    return isOpen ? TRUE : FALSE;
}


/*******************************************************************************
 * @fn      FLbuff_close
 *
 * @brief
 */
void FLbuff_close(void)
{
  if (isOpen)
  {

    isOpen = false;
    ExtFlash_close();
    FL_dflag[1]++;
  }
}

// Mutex??? or High level signaling to interrupt audio writign while reading?


//void FLbuff_loadSamples(uint8_t *pData, FLbuff_Instant_t start, FLbuff_Instant_t end){

void FLbuff_loadSamples(uint8_t *pData){

    FLbuff_readFlash(new.page,new.offset-4,pData,4);
}


bool FLbuff_LastPage(void){
    if( old.page== new.page){
        LastPage_flag=1;
        return true;
    }

return false;
}


//void FLbuff_loadSamples(uint8_t *pData, FLbuff_Instant_t start, FLbuff_Instant_t end){

bool FLbuff_loadSamples_2(uint8_t *pData, uint8_t len){

    FLbuff_readFlash(old.page,old.offset,pData,len);
    old.offset=old.offset+len;

    if( old.offset >= PAGE_SIZE){

        if(!LastPage_flag){
            // Cambia de pagina
            if(old.page+1>MAX_PAGE){       //  el -1 es solo de maña. despues no deberia ir.
                old.page=MIN_PAGE;}
            else{
                old.page++;   }
            old.offset=0;

            if( old.page == new.page){
                LastPage_flag=1;}
       }
    else{
        return true;}
    }

    return false;
}

/*******************************************************************************
 * @fn      FLbuff_readFlash
 *
 * @param   page   - page to read from in flash
 * @param   offset - offset into flash page to begin reading
 * @param   pBuf   - pointer to buffer into which data is read.
 * @param   len    - length of data to read in bytes.
 */
bool FLbuff_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf,uint16_t len){
       // Check that the flash page is in the limits of free space
 return	ExtFlash_read(FLASH_ADDRESS(page,offset), len, pBuf);
}


void FLbuff_saveSample_2(uint8_t  *pData, uint16_t len){

    b_FL_dflag=FLbuff_writeFlash(new.page,new.offset,pData,len);
    new.offset=new.offset+len;

   if(new.offset >= PAGE_SIZE){
       // Cambia de pagina
       if(new.page+1>MAX_PAGE){       //  el -1 es solo de maña. despues no deberia ir.

            new.page=MIN_PAGE;
            if(!OldWritenData){
                OldWritenData=1;}
        } else{
           new.page++;   }

       new.offset=0;

      // AMic_AuxfnB(FALSE);
       FLbuff_eraseFlash(new.page);
      // AMic_AuxfnB(TRUE);

       if(OldWritenData){
           if(new.page+1>MAX_PAGE){old.page=MIN_PAGE;}
           else{old.page=new.page+1;}
       }
   }
}



void FLbuff_saveSample(uint8_t  *pData, uint16_t len){
//   if(WR_flag){
//       now.offset=now.offset+4;
//       b_FL_dflag[0]=FLbuff_writeFlash(now.page,now.offset,pData,len);
//       WR_flag=0;
//    }
//    else{
//        b_FL_dflag[1]=FLbuff_readFlash(now.page,now.offset,auxBuff,len);
//       //AudioService_SetParameter(0, 4,auxBuff);
//        WR_flag=1;
//        DELAY_MS(50);
//    }

    b_FL_dflag=FLbuff_writeFlash(new.page,new.offset,pData,len);
    new.offset=new.offset+len;

   if(new.offset >= PAGE_SIZE){
       // Cambia de pagina
       if(new.page+1>MAX_PAGE-1){       //  el -1 es solo de maña. despues no deberia ir.

	        new.page=MIN_PAGE;
	        if(!OldWritenData){
	            OldWritenData=1;}
	    } else{
	       new.page++;   }

       new.offset=0;
       FLbuff_eraseFlash(new.page);
       if(OldWritenData){old.page=new.page+1;}
   }
}





/*******************************************************************************
 * @fn      FLbuff_writeFlash
 *
 * @brief
 */
bool FLbuff_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len)
{
    FL_dflag[0]++;
	return ExtFlash_write(FLASH_ADDRESS(page,offset), len, pBuf);
}

/*********************************************************************
 * @fn      FLbuff_eraseFlash
 *
 * @brief   Erase selected flash page.
 *
 * @param   page - the page to erase.
 *
 * @return  None.
 */
void FLbuff_eraseFlash(uint8_t page)
{
  ExtFlash_erase(FLASH_ADDRESS(page,0), HAL_FLASH_PAGE_SIZE);
}

/*********************************************************************
 * @fn      OADTarget_systemReset
 *
 * @brief   Prepare system for a reset and trigger a reset to the boot
 *          image manager. If the image contains a new BIM, copy it to
 *          page '0' before resetting. This is a critical operation.
 *
 * @param   None.
 *
 * @return  None.
 */
void FLbuff_systemReset(void)
{
  // Reset to the bootloader.
  HAL_SYSTEM_RESET();
}
