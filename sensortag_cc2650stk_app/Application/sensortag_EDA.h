// sensortag_EDA.h


/*********************************************************************
 * INCLUDES
 */

#ifndef SENSORTAGEDA_H
#define SENSORTAGEDA_H


#ifdef __cplusplus
extern "C"
{
#endif

#include "sensortag.h"

#include "stdint.h"
#include "stdbool.h"

/*
 * FUNCTIONS
 */

//void Sem_EDA_init(void);
//void SensorTagEDA_reset(void);

void SensorTagEDA_createTask(void);
void SensorTagEDA_init(void);
void SensorTagEDA_processCharChangeEvt(uint8_t paramID);
void SensorTagEDA_reset(void);


#ifdef __cplusplus
}
#endif


#endif /* SENSORTAGANALOGMIC_H*/
