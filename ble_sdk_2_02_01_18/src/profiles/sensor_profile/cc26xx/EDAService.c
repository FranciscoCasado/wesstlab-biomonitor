/**********************************************************************************************
 * Filename:       EDAService.c
 *
 * Description:    This file contains the implementation of the service.
 *
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
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "EDAService.h"
#include "st_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// EDAService Service UUID
CONST uint8_t EDAServiceUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(EDASERVICE_SERV_UUID)
};

// DATA_CHAR UUID
CONST uint8_t EDAService_DATA_CHARUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(EDASERVICE_DATA_CHAR_UUID)
};

// DATA_CONF UUID
CONST uint8_t EDAService_DATA_CONFUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(EDASERVICE_DATA_CONF_UUID)
};

// DATA_PERI UUID
CONST uint8_t EDAService_DATA_PERIUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(EDASERVICE_DATA_PERI_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// debug
uint8_t EDA_S [10];


//static EDAServiceCBs_t *pAppCBs = NULL;

// new  CBs---TI standar
static sensorCBs_t      *sensor__AppCBs   = NULL;


/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t EDAServiceDecl = { ATT_UUID_SIZE, EDAServiceUUID };

// Characteristic "DATA_CHAR" Properties (for declaration)
static uint8_t EDAService_DATA_CHARProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "DATA_CHAR" Value variable
static uint8_t EDAService_DATA_CHARVal[EDASERVICE_DATA_CHAR_LEN] = {0};

// Characteristic "DATA_CHAR" CCCD
static gattCharCfg_t *EDAService_DATA_CHARConfig;


// Characteristic "DATA_CONF" Properties (for declaration)
static uint8_t EDAService_DATA_CONFProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "DATA_CONF" Value variable
static uint8_t EDAService_DATA_CONFVal[EDASERVICE_DATA_CONF_LEN] = {0};
// Characteristic "DATA_PERI" Properties (for declaration)
static uint8_t EDAService_DATA_PERIProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "DATA_PERI" Value variable
static uint8_t EDAService_DATA_PERIVal[EDASERVICE_DATA_PERI_LEN] = {0};

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t EDAServiceAttrTbl[] =
{
  // EDAService Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&EDAServiceDecl
  },
    // DATA_CHAR Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &EDAService_DATA_CHARProps
    },
      // DATA_CHAR Characteristic Value
      {
        { ATT_UUID_SIZE, EDAService_DATA_CHARUUID },
        GATT_PERMIT_READ,
        0,
        EDAService_DATA_CHARVal
      },
      // DATA_CHAR CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&EDAService_DATA_CHARConfig
      },
    // DATA_CONF Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &EDAService_DATA_CONFProps
    },
      // DATA_CONF Characteristic Value
      {
        { ATT_UUID_SIZE, EDAService_DATA_CONFUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        EDAService_DATA_CONFVal
      },
    // DATA_PERI Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &EDAService_DATA_PERIProps
    },
      // DATA_PERI Characteristic Value
      {
        { ATT_UUID_SIZE, EDAService_DATA_PERIUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        EDAService_DATA_PERIVal
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t EDAService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t EDAService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t EDAServiceCBs =
{
  EDAService_ReadAttrCB,  // Read callback function pointer
  EDAService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * EDAService_AddService- Initializes the EDAService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t EDAService_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  EDAService_DATA_CHARConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( EDAService_DATA_CHARConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, EDAService_DATA_CHARConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( EDAServiceAttrTbl,
                                        GATT_NUM_ATTRS( EDAServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &EDAServiceCBs );

  return ( status );
}

/*
 * EDAService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
//bStatus_t EDAService_RegisterAppCBs( EDAServiceCBs_t *appCallbacks )
bStatus_t EDAService_RegisterAppCBs( sensorCBs_t *appCallbacks )
{
	EDA_S[0]++;
	if (sensor__AppCBs == NULL) {
	  if ( appCallbacks != NULL) {

	  // pAppCBs = appCallbacks;
		  EDA_S[1]++;
		  sensor__AppCBs = appCallbacks;

		return ( SUCCESS );
	  }
	 }

	EDA_S[2]++;
	return ( bleAlreadyInRequestedMode );

}

/*
 * EDAService_SetParameter - Set a EDAService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t EDAService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case EDASERVICE_DATA_CHAR:
      if ( len == EDASERVICE_DATA_CHAR_LEN )
      {
        memcpy(EDAService_DATA_CHARVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( EDAService_DATA_CHARConfig, (uint8_t *)&EDAService_DATA_CHARVal, FALSE,
                                    EDAServiceAttrTbl, GATT_NUM_ATTRS( EDAServiceAttrTbl ),
                                    INVALID_TASK_ID,  EDAService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EDASERVICE_DATA_CONF:
      if ( len == EDASERVICE_DATA_CONF_LEN )
      {
        memcpy(EDAService_DATA_CONFVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EDASERVICE_DATA_PERI:
      if ( len == EDASERVICE_DATA_PERI_LEN )
      {
        memcpy(EDAService_DATA_PERIVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * EDAService_GetParameter - Get a EDAService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t EDAService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case EDASERVICE_DATA_CONF:
      memcpy(value, EDAService_DATA_CONFVal, EDASERVICE_DATA_CONF_LEN);
      break;

    case EDASERVICE_DATA_PERI:
      memcpy(value, EDAService_DATA_PERIVal, EDASERVICE_DATA_PERI_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          EDAService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t EDAService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the DATA_CHAR Characteristic Value
if ( ! memcmp(pAttr->type.uuid, EDAService_DATA_CHARUUID, pAttr->type.len) )
  {
    if ( offset > EDASERVICE_DATA_CHAR_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, EDASERVICE_DATA_CHAR_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the DATA_CONF Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, EDAService_DATA_CONFUUID, pAttr->type.len) )
  {
    if ( offset > EDASERVICE_DATA_CONF_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, EDASERVICE_DATA_CONF_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the DATA_PERI Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, EDAService_DATA_PERIUUID, pAttr->type.len) )
  {
    if ( offset > EDASERVICE_DATA_PERI_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, EDASERVICE_DATA_PERI_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      EDAService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t EDAService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the DATA_CONF Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, EDAService_DATA_CONFUUID, pAttr->type.len) )
  {
    if ( offset + len > EDASERVICE_DATA_CONF_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
    	EDA_S[5]++;
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == EDASERVICE_DATA_CONF_LEN)
        paramID = EDASERVICE_DATA_CONF;
      	EDA_S[4]++;
    }
  }
  // See if request is regarding the DATA_PERI Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, EDAService_DATA_PERIUUID, pAttr->type.len) )
  {
    if ( offset + len > EDASERVICE_DATA_PERI_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == EDASERVICE_DATA_PERI_LEN)
        paramID = EDASERVICE_DATA_PERI;
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF){
	  EDA_S[7]++;
      if ( sensor__AppCBs && sensor__AppCBs->pfnSensorChange ){
    	  EDA_S[6]++;
    	  sensor__AppCBs->pfnSensorChange( paramID ); // Call app function from stack task context.
      }
  }
  return status;
}


//if ( pAppCBs && pAppCBs->pfnChangeCb )
//     pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.

