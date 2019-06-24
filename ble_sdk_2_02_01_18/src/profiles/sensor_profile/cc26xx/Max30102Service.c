/**********************************************************************************************
 * Filename:       Max30102Service.c
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

#include "Max30102Service.h"
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

// Max30102Service Service UUID
CONST uint8_t Max30102ServiceUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MAX30102SERVICE_SERV_UUID)
};

// MAX_Data_Char UUID
CONST uint8_t Max30102Service_MAX_Data_CharUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MAX30102SERVICE_MAX_DATA_CHAR_UUID)
};
// MAX_CONF_Char UUID
CONST uint8_t Max30102Service_MAX_CONF_CharUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MAX30102SERVICE_MAX_CONF_CHAR_UUID)
};
// MAX_PERI_Char UUID
CONST uint8_t Max30102Service_MAX_PERI_CharUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MAX30102SERVICE_MAX_PERI_CHAR_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */
// old CBs---default from builder.
//static Max30102ServiceCBs_t *pAppCBs 		= NULL;

// new  CBs---TI standar
static sensorCBs_t 			*sensorAppCBs 	= NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t Max30102ServiceDecl = { ATT_UUID_SIZE, Max30102ServiceUUID };

// Characteristic "MAX_Data_Char" Properties (for declaration)
static uint8_t Max30102Service_MAX_Data_CharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "MAX_Data_Char" Value variable
static uint8_t Max30102Service_MAX_Data_CharVal[MAX30102SERVICE_MAX_DATA_CHAR_LEN] = {0};

// Characteristic "MAX_Data_Char" CCCD
static gattCharCfg_t *Max30102Service_MAX_Data_CharConfig;


// Characteristic "MAX_CONF_Char" Properties (for declaration)
static uint8_t Max30102Service_MAX_CONF_CharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "MAX_CONF_Char" Value variable
static uint8_t Max30102Service_MAX_CONF_CharVal[MAX30102SERVICE_MAX_CONF_CHAR_LEN] = {0};
// Characteristic "MAX_PERI_Char" Properties (for declaration)
static uint8_t Max30102Service_MAX_PERI_CharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "MAX_PERI_Char" Value variable
static uint8_t Max30102Service_MAX_PERI_CharVal[MAX30102SERVICE_MAX_PERI_CHAR_LEN] = {0};

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t Max30102ServiceAttrTbl[] =
{
  // Max30102Service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&Max30102ServiceDecl
  },
    // MAX_Data_Char Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Max30102Service_MAX_Data_CharProps
    },
      // MAX_Data_Char Characteristic Value
      {
        { ATT_UUID_SIZE, Max30102Service_MAX_Data_CharUUID },
        GATT_PERMIT_READ,
        0,
        Max30102Service_MAX_Data_CharVal
      },
      // MAX_Data_Char CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&Max30102Service_MAX_Data_CharConfig
      },
    // MAX_CONF_Char Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Max30102Service_MAX_CONF_CharProps
    },
      // MAX_CONF_Char Characteristic Value
      {
        { ATT_UUID_SIZE, Max30102Service_MAX_CONF_CharUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Max30102Service_MAX_CONF_CharVal
      },
    // MAX_PERI_Char Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Max30102Service_MAX_PERI_CharProps
    },
      // MAX_PERI_Char Characteristic Value
      {
        { ATT_UUID_SIZE, Max30102Service_MAX_PERI_CharUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Max30102Service_MAX_PERI_CharVal
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Max30102Service_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t Max30102Service_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Max30102ServiceCBs =
{
  Max30102Service_ReadAttrCB,  // Read callback function pointer
  Max30102Service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Max30102Service_AddService- Initializes the Max30102Service service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t Max30102Service_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  Max30102Service_MAX_Data_CharConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( Max30102Service_MAX_Data_CharConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, Max30102Service_MAX_Data_CharConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( Max30102ServiceAttrTbl,
                                        GATT_NUM_ATTRS( Max30102ServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &Max30102ServiceCBs );

  return ( status );
}

/*
 * Max30102Service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Max30102Service_RegisterAppCBs( sensorCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
	 sensorAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * Max30102Service_SetParameter - Set a Max30102Service parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Max30102Service_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MAX30102SERVICE_MAX_DATA_CHAR:
      if ( len == MAX30102SERVICE_MAX_DATA_CHAR_LEN )
      {
        memcpy(Max30102Service_MAX_Data_CharVal, value, len);

        // Try to send notification.
        ret =GATTServApp_ProcessCharCfg( Max30102Service_MAX_Data_CharConfig, (uint8_t *)&Max30102Service_MAX_Data_CharVal, FALSE,
                                    Max30102ServiceAttrTbl, GATT_NUM_ATTRS( Max30102ServiceAttrTbl ),
                                    INVALID_TASK_ID,  Max30102Service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MAX30102SERVICE_MAX_CONF_CHAR:
      if ( len == MAX30102SERVICE_MAX_CONF_CHAR_LEN )
      {
        memcpy(Max30102Service_MAX_CONF_CharVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MAX30102SERVICE_MAX_PERI_CHAR:
      if ( len == MAX30102SERVICE_MAX_PERI_CHAR_LEN )
      {
        memcpy(Max30102Service_MAX_PERI_CharVal, value, len);
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
 * Max30102Service_GetParameter - Get a Max30102Service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Max30102Service_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MAX30102SERVICE_MAX_DATA_CHAR:
      memcpy(value, Max30102Service_MAX_Data_CharVal, MAX30102SERVICE_MAX_DATA_CHAR_LEN);
      break;

    case MAX30102SERVICE_MAX_CONF_CHAR:
      memcpy(value, Max30102Service_MAX_CONF_CharVal, MAX30102SERVICE_MAX_CONF_CHAR_LEN);
      break;

    case MAX30102SERVICE_MAX_PERI_CHAR:
      memcpy(value, Max30102Service_MAX_PERI_CharVal, MAX30102SERVICE_MAX_PERI_CHAR_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          Max30102Service_ReadAttrCB
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
static bStatus_t Max30102Service_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the MAX_Data_Char Characteristic Value
if ( ! memcmp(pAttr->type.uuid, Max30102Service_MAX_Data_CharUUID, pAttr->type.len) )
  {
    if ( offset > MAX30102SERVICE_MAX_DATA_CHAR_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, MAX30102SERVICE_MAX_DATA_CHAR_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the MAX_CONF_Char Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, Max30102Service_MAX_CONF_CharUUID, pAttr->type.len) )
  {
    if ( offset > MAX30102SERVICE_MAX_CONF_CHAR_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, MAX30102SERVICE_MAX_CONF_CHAR_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the MAX_PERI_Char Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, Max30102Service_MAX_PERI_CharUUID, pAttr->type.len) )
  {
    if ( offset > MAX30102SERVICE_MAX_PERI_CHAR_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, MAX30102SERVICE_MAX_PERI_CHAR_LEN - offset);  // Transmit as much as possible
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
 * @fn      Max30102Service_WriteAttrCB
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
static bStatus_t Max30102Service_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
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
  // See if request is regarding the MAX_CONF_Char Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, Max30102Service_MAX_CONF_CharUUID, pAttr->type.len) )
  {
    if ( offset + len > MAX30102SERVICE_MAX_CONF_CHAR_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == MAX30102SERVICE_MAX_CONF_CHAR_LEN)
        paramID = MAX30102SERVICE_MAX_CONF_CHAR;
    }
  }
  // See if request is regarding the MAX_PERI_Char Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, Max30102Service_MAX_PERI_CharUUID, pAttr->type.len) )
  {
    if ( offset + len > MAX30102SERVICE_MAX_PERI_CHAR_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == MAX30102SERVICE_MAX_PERI_CHAR_LEN)
        paramID = MAX30102SERVICE_MAX_PERI_CHAR;
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
  if (paramID != 0xFF)
    if ( sensorAppCBs && sensorAppCBs->pfnSensorChange )
    	sensorAppCBs->pfnSensorChange( paramID ); // Call app function from stack task context.

  return status;
}
