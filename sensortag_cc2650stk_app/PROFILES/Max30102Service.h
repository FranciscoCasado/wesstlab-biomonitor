/**********************************************************************************************
 * Filename:       Max30102Service.h
 *
 * Description:    This file contains the Max30102Service service definitions and
 *                 prototypes.
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


#ifndef _MAX30102SERVICE_H_
#define _MAX30102SERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define MAX30102SERVICE_SERV_UUID 0xAA90

//  Characteristic defines
#define MAX30102SERVICE_MAX_DATA_CHAR      0
#define MAX30102SERVICE_MAX_DATA_CHAR_UUID 0xAA91
#define MAX30102SERVICE_MAX_DATA_CHAR_LEN  6

//  Characteristic defines
#define MAX30102SERVICE_MAX_CONF_CHAR      1
#define MAX30102SERVICE_MAX_CONF_CHAR_UUID 0xAA92
#define MAX30102SERVICE_MAX_CONF_CHAR_LEN  1

//  Characteristic defines
#define MAX30102SERVICE_MAX_PERI_CHAR      2
#define MAX30102SERVICE_MAX_PERI_CHAR_UUID 0xAA93
#define MAX30102SERVICE_MAX_PERI_CHAR_LEN  1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*Max30102ServiceChange_t)( uint8 paramID );

typedef struct
{
  Max30102ServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} Max30102ServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Max30102Service_AddService- Initializes the Max30102Service service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Max30102Service_AddService( void );

/*
 * Max30102Service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Max30102Service_RegisterAppCBs( sensorCBs_t *appCallbacks );

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
extern bStatus_t Max30102Service_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Max30102Service_GetParameter - Get a Max30102Service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Max30102Service_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _MAX30102SERVICE_H_ */
