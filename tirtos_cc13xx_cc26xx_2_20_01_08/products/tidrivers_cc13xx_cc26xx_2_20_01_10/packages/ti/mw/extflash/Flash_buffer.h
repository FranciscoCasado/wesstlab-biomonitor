/*
	Flash buffer library, for buffering data.
	CC2650
	january 2017
	José Càceres
*/ 




/*********************************************************************
 * INCLUDES
 */

#include <ti/sysbios/knl/Queue.h>

/*********************************************************************
 * CONSTANTS
 */
// The Image is transported in 16-byte blocks in order to avoid using blob operations.
#define OAD_BLOCK_SIZE         16
#define OAD_BLOCKS_PER_PAGE    (HAL_FLASH_PAGE_SIZE / OAD_BLOCK_SIZE)
#define OAD_BLOCK_MAX          (OAD_BLOCKS_PER_PAGE * OAD_IMG_D_AREA)



/*********************************************************************
 * MACROS
 */

#define FLASH_ADDRESS(page, offset) (((page) << 12) + (offset))

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * TYPEDEFS
 */



// Just in case
//typedef struct
//{
//  Queue_Elem _elem;
//  uint8_t  event;
//  uint16_t connHandle;
//  uint8_t  *pData;
//} FLbuff_TargetWrite_t;



// Needs to be a queue Element? or is used just as markers?
typedef struct
{
  //Queue_Elem _elem;
  uint8_t  page;
  uint16_t  offset;
//  uint8_t  *pData;
} FLbuff_Instant_t;


/*********************************************************************
 * FUNCTIONS
 */


void AMic_AuxfnB(bool BOO2);
void AMic_Auxfn2(bool BOO1);


void FLbuff_init();
void FLbuff_reset();

void FLbuff_loadSamples(uint8_t *pData);
bool FLbuff_loadSamples_2(uint8_t *pData, uint8_t len);

void FLbuff_saveSample(uint8_t  *pData, uint16_t len);
void FLbuff_saveSample_2(uint8_t  *pData, uint16_t len);

bool FLbuff_LastPage(void);

uint8_t FLbuff_open(void);

void FLbuff_close(void);

bool FLbuff_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len);
bool FLbuff_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len);

void FLbuff_eraseFlash(uint8_t page);

void FLbuff_systemReset(void);
