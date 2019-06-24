/*
 * sensortag_max30102.h
 *
 *  Created on: 17-08-2016
 *      Author: JC
 */

#ifndef SENSORTAGMAX30102_H
#define SENSORTAGMAX30102_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "sensortag.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

#ifndef EXCLUDE_MAX
/* Create the Max30102 sensor task
*/
extern void SensorTagMax30102_createTask(void);
/*
 * Initialize HR and SpO2 sensor module
 */
extern void SensorTagMax30102_init(void);

/*
 * Task Event Processor for HR and SpO2 sensor module
 */
//extern void SensorTagMax30102_processSensorEvent(void);

/*
 * Task Event Processor for characteristic changes
 */
extern void SensorTagMax30102_processCharChangeEvt(uint8_t paramID);

/*
 * Reset HR and SpO2 sensor module
 */
extern void SensorTagMax30102_reset(void);

#else

/* max30102 module not included   ¿¿¿¿¿'???????? */
//
//#define SensorTagMax30102_createTask()
//#define SensorTagMax30102_init()
//#define SensorTagMax30102_processCharChangeEvt(paramID)
//#define SensorTagMax30102_processSensorEvent()
//#define SensorTagMax30102_reset()

#endif // EXCLUDE_MAX

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAGMAX30102_H*/
