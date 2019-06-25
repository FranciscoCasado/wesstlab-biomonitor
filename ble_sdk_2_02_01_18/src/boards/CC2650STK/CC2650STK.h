/*
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
 */
/** ============================================================================
 *  @file       CC2650STK.h
 *
 *  @brief      CC2650SENSORTAG Board Specific header file.
 *
 *  NB! This is the board file for PCB versions 1.2 and 1.3
 *
 *  ============================================================================
 */

#define BIOMONITOR
#define bio_m_v1

#ifndef __CC2650STK_SENSORTAG_BOARD_H__
#define __CC2650STK_SENSORTAG_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

//// BIO MONITOR PCB
#ifdef BIOMONITOR

/* RF Configuration definition */                   /* Launchpad or Senrsotag NOT Same RF Configuration as 7x7 EM */
//#define CC2650EM_7ID  /* Default RF config for CC2650 LaunchPad */ // JCL

//MODA guide Board config
#define CC2650EM_5XD /* RF Configuration for CC2650 MODA Module */

//Board.h best fit alternative
//#define CC2650DK_5XD /* RF Configuration for CC2650 MODA Module */


///* Mapping of pins to board signals using general board aliases
// *      <board signal alias>        <pin mapping>
// */
//
///* Discrete outputs */
//#define Board_STK_LED1              IOID_10
//#define Board_STK_LED2              IOID_15
//#define Board_BUZZER                IOID_21
//#define Board_LED_ON                1
//#define Board_LED_OFF               0
//#define Board_BUZZER_ON             1
//#define Board_BUZZER_OFF            0
//
///* Discrete inputs */
//#define Board_KEY_LEFT              IOID_0
//#define Board_KEY_RIGHT             IOID_4
//#define Board_RELAY                 IOID_3
//
///* Sensor outputs */
//#define Board_MPU_INT               IOID_7
//#define Board_TMP_RDY               IOID_1
//
///* I2C */
//#define Board_I2C0_SDA0             IOID_5
//#define Board_I2C0_SCL0             IOID_6
//#define Board_I2C0_SDA1             IOID_8
//#define Board_I2C0_SCL1             IOID_9
//
///* SPI */
//#define Board_SPI_FLASH_CS          IOID_14
//#define Board_SPI_DEVPK_CS          IOID_20
//#define Board_FLASH_CS_ON           0
//#define Board_FLASH_CS_OFF          1
//#define Board_DEVPK_CS_ON           1
//#define Board_DEVPK_CS_OFF          0
//
//#define Board_SPI0_MISO             IOID_18
//#define Board_SPI0_MOSI             IOID_19
//#define Board_SPI0_CLK              IOID_17
//#define Board_SPI0_CSN              PIN_UNASSIGNED
//#define Board_SPI1_MISO             PIN_UNASSIGNED
//#define Board_SPI1_MOSI             PIN_UNASSIGNED
//#define Board_SPI1_CLK              PIN_UNASSIGNED
//#define Board_SPI1_CSN              PIN_UNASSIGNED
//
///* UART when connected to SRF06EB */
//#define Board_EB_UART_TX            IOID_16
//#define Board_EB_UART_RX            IOID_17



//////////////////////////////////// JCL
/* Mapping of pins to board signals using general board aliases
* <board signal alias> <pin mapping>
*/
/* Discrete outputs */
//#define Board_RLED PIN_UNASSIGNED   //IOID_4    //JCL
//#define Board_GLED IOID_13          //IOID_2    //JCL

//cual es el que se usa??
#define Board_STK_LED1              PIN_UNASSIGNED  //IOID_10   //JCL
#define Board_STK_LED2              IOID_13//PIN_UNASSIGNED  //IOID_13         //IOID_15   //JCL
#define Board_BUZZER                PIN_UNASSIGNED //IOID_21   //JCL

#define Board_LED_ON                1
#define Board_LED_OFF               0
#define Board_BUZZER_ON             1
#define Board_BUZZER_OFF            0

//#define Board_PWR_CTRL  IOID_9


/* Discrete inputs */

// Discrete Inputs for proyect ZERO
//#define Board_BTN1  PIN_UNASSIGNED
//#define Board_BTN2  IOID_12     //PIN_UNASSIGNED       //JCL

//Discrete Inputs for Sensortag
#define Board_KEY_LEFT              IOID_12//PIN_UNASSIGNED  //IOID_0 //PIN_UNASSIGNED       //JCL ESTE es el que SE USA!
#define Board_KEY_RIGHT             PIN_UNASSIGNED  //IOID_12         //PIN_UNASSIGNED  //IOID_4    //JCL
#define Board_RELAY                 PIN_UNASSIGNED  //IOID_3    //JCL

// PowerControl manage: Vcc max30102; Vcc MPU9250  ; Vcc_Movil; CS SPI(flash)
#define Board_DP3   IOID_9
#define Sensor_PWR_ON   1
#define Sensor_PWR_OFF  0

///* Sensor outputs */
#define Board_MPU_INT               IOID_0          //JCL
#define Board_TMP_RDY               PIN_UNASSIGNED  //IOID_1    //JCL
//Max30102 Interrupt
#define Board_DP0                   IOID_3  //IOID_25   //JCL

/* I2C */
// Max30102 I2C bus
#define Board_I2C0_SDA0             IOID_5  //IOID_5    //JCL
#define Board_I2C0_SCL0             IOID_4  //IOID_6    //JCL

// MPU9250 I2C bus
#define Board_I2C0_SDA1             IOID_1  //IOID_8    //JCL
#define Board_I2C0_SCL1             IOID_2  //IOID_9    //JCL


/* SPI */  //Should change to SPI for uSD module
#define Board_SPI_FLASH_CS          PIN_UNASSIGNED //IOID_14
#define Board_SPI_DEVPK_CS          PIN_UNASSIGNED //IOID_20
#define Board_FLASH_CS_ON           0
#define Board_FLASH_CS_OFF          1
#define Board_DEVPK_CS_ON           1
#define Board_DEVPK_CS_OFF          0

#define Board_SPI0_MISO IOID_7  //IOID_12 /* RF1.20 */  //JCL
#define Board_SPI0_MOSI IOID_6  //IOID_11 /* RF1.18 */  //JCL
#define Board_SPI0_CLK  IOID_8  //IOID_10 /* RF1.16 */  //JCL

                                /* IOID_9 is also used to enable sensors (Board_MPU_POWER) */
#define Board_SPI0_CSN  IOID_9  // Be careful in order to not interfere with other sensors
                                /* Keep in mind that a logic inverter is used in the board */

#define Board_SPI1_MISO PIN_UNASSIGNED
#define Board_SPI1_MOSI PIN_UNASSIGNED
#define Board_SPI1_CLK  PIN_UNASSIGNED
#define Board_SPI1_CSN  PIN_UNASSIGNED

/////////////////////////////////////////////////////
/* UART Board */ // which is best ?

/* UART 1  */
#define Board_EB_UART_TX        PIN_UNASSIGNED   //IOID_13 //JCL //IOID_16
#define Board_EB_UART_RX        PIN_UNASSIGNED   //IOID_12 //JCL  //IOID_17

/* UART pins used by driver-2 */
#define Board_UART_TX            PIN_UNASSIGNED //IOID_12 // JCL//Board_DP5_UARTTX
#define Board_UART_RX            PIN_UNASSIGNED //IOID_13 // JCL//Board_DP4_UARTRX



//#define Board_UART_RX PIN_UNASSIGNED    //IOID_12 /* RXD */ //JCL
//#define Board_UART_TX PIN_UNASSIGNED    //IOID_13 /* TXD */ //JCL
//#define Board_UART_CTS PIN_UNASSIGNED /* CTS */
//#define Board_UART_RTS PIN_UNASSIGNED /* RTS */

/* Power control */
#define Board_MPU_POWER             IOID_9 //IOID_12 // Same PWR COntrol GRAL//JCL
#define Board_MPU_POWER_ON          1
#define Board_MPU_POWER_OFF         0

/* Audio */
#define Board_MIC_POWER             PIN_UNASSIGNED//IOID_13 //JCL
#define Board_MIC_POWER_ON          1
#define Board_MIC_POWER_OFF         0
#define Board_AUDIO_DI              PIN_UNASSIGNED  //IOID_2    //JCL
#define Board_AUDIO_CLK             PIN_UNASSIGNED  //IOID_11   //JCL



/* DevPack common */
#define Board_AUDIOFS_TDO           PIN_UNASSIGNED //JCL//IOID_16
#define Board_AUDIODO               PIN_UNASSIGNED //JCL//IOID_22
//#define Board_DP0                   PIN_UNASSIGNED //JCL//IOID_25
#define Board_DP2                   PIN_UNASSIGNED //JCL//IOID_23
#define Board_DP1                   PIN_UNASSIGNED //JCL//IOID_24
//#define Board_DP3                   PIN_UNASSIGNED //JCL//IOID_27
#define Board_DP4_UARTRX            PIN_UNASSIGNED //JCL//IOID_28
#define Board_DP5_UARTTX            PIN_UNASSIGNED //JCL//IOID_29
#define Board_DEVPK_ID              PIN_UNASSIGNED //JCL//IOID_30
#define Board_SPI_DEVPK_CS          PIN_UNASSIGNED //JCL//IOID_20

/* LCD DevPack */
#define Board_LCD_EXTCOMIN          PIN_UNASSIGNED //JCL//IOID_22
#define Board_LCD_EXTMODE           PIN_UNASSIGNED //JCL//IOID_28
#define Board_LCD_ENABLE            PIN_UNASSIGNED //JCL//IOID_29
#define Board_LCD_POWER             PIN_UNASSIGNED
#define Board_LCD_CS                Board_SPI_DEVPK_CS
#define Board_LCD_CS_ON             1
#define Board_LCD_CS_OFF            0

/* LED-Audio DevPack */
#define Board_DEVPK_LIGHT_BLUE      PIN_UNASSIGNED //JCL//IOID_23
#define Board_DEVPK_LIGHT_GREEN     PIN_UNASSIGNED //JCL//IOID_24
#define Board_DEVPK_LIGHT_WHITE     PIN_UNASSIGNED //JCL//IOID_25
#define Board_DEVPK_LIGHT_RED       PIN_UNASSIGNED //JCL//IOID_27

/* PWM outputs */
#define Board_PWMPIN0                       Board_STK_LED1
#define Board_PWMPIN1                       Board_STK_LED2
#define Board_PWMPIN2                       PIN_UNASSIGNED
#define Board_PWMPIN3                       PIN_UNASSIGNED
#define Board_PWMPIN4                       PIN_UNASSIGNED
#define Board_PWMPIN5                       PIN_UNASSIGNED
#define Board_PWMPIN6                       PIN_UNASSIGNED
#define Board_PWMPIN7                       PIN_UNASSIGNED


#else  //// Baby-MONITOR PCB

/* Same RF Configuration as 7x7 EM */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

/* Discrete outputs */
#define Board_STK_LED1              IOID_15
#define Board_STK_LED2              IOID_10 // Green Led
#define Board_BUZZER                IOID_25 // Free pad //IOID_21
#define Board_LED_ON                1
#define Board_LED_OFF               0
#define Board_BUZZER_ON             1
#define Board_BUZZER_OFF            0

/* Discrete inputs */
#define Board_KEY_LEFT              IOID_24//IOID_0
#define Board_KEY_RIGHT             IOID_4
#define Board_RELAY                 PIN_UNASSIGNED//IOID_3

/* Sensor outputs */
#define Board_MPU_INT               IOID_7
#define Board_TMP_RDY               IOID_1

/* I2C */
#define Board_I2C0_SDA0             IOID_5
#define Board_I2C0_SCL0             IOID_6
#define Board_I2C0_SDA1             IOID_8
#define Board_I2C0_SCL1             IOID_9

/* SPI */
#define Board_SPI_FLASH_CS          IOID_14
#define Board_SPI_DEVPK_CS          IOID_20
#define Board_FLASH_CS_ON           0
#define Board_FLASH_CS_OFF          1
#define Board_DEVPK_CS_ON           1
#define Board_DEVPK_CS_OFF          0

#define Board_SPI0_MISO             IOID_18
#define Board_SPI0_MOSI             IOID_19
#define Board_SPI0_CLK              IOID_17
#define Board_SPI0_CSN              PIN_UNASSIGNED
#define Board_SPI1_MISO             PIN_UNASSIGNED
#define Board_SPI1_MOSI             PIN_UNASSIGNED
#define Board_SPI1_CLK              PIN_UNASSIGNED
#define Board_SPI1_CSN              PIN_UNASSIGNED

/* UART when connected to SRF06EB */
#define Board_EB_UART_TX            IOID_16
#define Board_EB_UART_RX            IOID_17

/* Power control */
#define Board_MPU_POWER             IOID_12
#define Board_MPU_POWER_ON          1
#define Board_MPU_POWER_OFF         0

/* Audio Digital*/
#define Board_MIC_POWER             IOID_13
#define Board_MIC_POWER_ON          1
#define Board_MIC_POWER_OFF         0
#define Board_AUDIO_DI              IOID_2
#define Board_AUDIO_CLK             IOID_11

/* UART pins used by driver */
#define Board_UART_TX               Board_DP5_UARTTX
#define Board_UART_RX               Board_DP4_UARTRX

/* DevPack common */
#define Board_AUDIOFS_TDO           IOID_16
#define Board_AUDIODO               IOID_22  // NC
#define Board_DP2                   IOID_0   // NC
#define Board_DP1                   IOID_23  //IOID_24 // Analog Mic Input
#define Board_DP0                   IOID_21  //IOID_25 //Max30101 INT
#define Board_DP3                   IOID_3  //IOID_27 //Enable V5 for Max30101
#define Board_DP4_UARTRX            IOID_28
#define Board_DP5_UARTTX            IOID_29
#define Board_DEVPK_ID              IOID_30
#define Board_SPI_DEVPK_CS          IOID_20

/* LCD DevPack */
#define Board_LCD_EXTCOMIN          IOID_22
#define Board_LCD_EXTMODE           IOID_28
#define Board_LCD_ENABLE            IOID_29
#define Board_LCD_POWER             PIN_UNASSIGNED
#define Board_LCD_CS                Board_SPI_DEVPK_CS
#define Board_LCD_CS_ON             1
#define Board_LCD_CS_OFF            0

/* LED-Audio DevPack */
//#define Board_DEVPK_LIGHT_BLUE      IOID_23
#define Board_DEVPK_LIGHT_GREEN     IOID_24
#define Board_DEVPK_LIGHT_WHITE     IOID_25
#define Board_DEVPK_LIGHT_RED       IOID_27

/* PWM outputs */
#define Board_PWMPIN0                       Board_STK_LED1
#define Board_PWMPIN1                       Board_STK_LED2
#define Board_PWMPIN2                       PIN_UNASSIGNED
#define Board_PWMPIN3                       PIN_UNASSIGNED
#define Board_PWMPIN4                       PIN_UNASSIGNED
#define Board_PWMPIN5                       PIN_UNASSIGNED
#define Board_PWMPIN6                       PIN_UNASSIGNED
#define Board_PWMPIN7                       PIN_UNASSIGNED

#endif



/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC2650STK_I2C0
/* Generic PDM instance identifiers */
#define Board_PDM                   CC2650STK_PDM0
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC2650STK_SPI0
#define Board_SPI1                  CC2650STK_SPI1
/* Generic UART instance identifiers */
#define Board_UART                  CC2650STK_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC2650STK_CRYPTO0
/* Generic TRNG instance identiifer */
#define Board_TRNG                  CC2650STK_TRNG0
/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             CC2650STK_GPTIMER0A
#define Board_GPTIMER0B             CC2650STK_GPTIMER0B
#define Board_GPTIMER1A             CC2650STK_GPTIMER1A
#define Board_GPTIMER1B             CC2650STK_GPTIMER1B
#define Board_GPTIMER2A             CC2650STK_GPTIMER2A
#define Board_GPTIMER2B             CC2650STK_GPTIMER2B
#define Board_GPTIMER3A             CC2650STK_GPTIMER3A
#define Board_GPTIMER3B             CC2650STK_GPTIMER3B
/* Generic PWM instance identifiers */
#define Board_PWM0                  CC2650STK_PWM0
#define Board_PWM1                  CC2650STK_PWM1
#define Board_PWM2                  CC2650STK_PWM2
#define Board_PWM3                  CC2650STK_PWM3
#define Board_PWM4                  CC2650STK_PWM4
#define Board_PWM5                  CC2650STK_PWM5
#define Board_PWM6                  CC2650STK_PWM6
#define Board_PWM7                  CC2650STK_PWM7

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650STK_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum CC2650STK_I2CName {
    CC2650STK_I2C0 = 0,

    CC2650STK_I2CCOUNT
} CC2650STK_I2CName;

/*!
 *  @def    CC2650STK_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650STK_CryptoName {
    CC2650STK_CRYPTO0 = 0,

    CC2650STK_CRYPTOCOUNT
} CC2650STK_CryptoName;

/*!
 *  @def    CC2650STK_PdmName
 *  @brief  Enum of PDM names on the CC2650 dev board
 */
typedef enum CC2650STK_PDMName {
    CC2650STK_PDM0 = 0,
    CC2650STK_PDMCOUNT
} CC2650STK_PDMName;

/*!
 *  @def    CC2650STK_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650STK_SPIName {
    CC2650STK_SPI0 = 0,
    CC2650STK_SPI1,

    CC2650STK_SPICOUNT
} CC2650STK_SPIName;

/*!
 *  @def    CC2650STK_SDSPIName
 *  @brief  Enum of SDSPI names on the CC2650 dev board
 */
typedef enum CC2650STK_SDSPIName {
    CC2650STK_SDSPI0 = 0,

    CC2650STK_SDSPICOUNT
} CC2650STK_SDSPIName;

/*!
 *  @def    CC2650STK_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2650STK_TRNGName {
    CC2650STK_TRNG0 = 0,
    CC2650STK_TRNGCOUNT
} CC2650STK_TRNGName;

/*!
 *  @def    CC2650STK_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650STK_UARTName {
    CC2650STK_UART0 = 0,

    CC2650STK_UARTCOUNT
} CC2650STK_UARTName;

/*!
 *  @def    CC2650STK_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650STK_UdmaName {
    CC2650STK_UDMA0 = 0,

    CC2650STK_UDMACOUNT
} CC2650STK_UdmaName;
/*!
 *  @def    CC2650STK_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2650STK_GPTimerName
{
    CC2650STK_GPTIMER0A = 0,
    CC2650STK_GPTIMER0B,
    CC2650STK_GPTIMER1A,
    CC2650STK_GPTIMER1B,
    CC2650STK_GPTIMER2A,
    CC2650STK_GPTIMER2B,
    CC2650STK_GPTIMER3A,
    CC2650STK_GPTIMER3B,
    CC2650STK_GPTIMERPARTSCOUNT
} CC2650STK_GPTimerName;

/*!
 *  @def    CC2650STK_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2650STK_GPTimers
{
    CC2650STK_GPTIMER0 = 0,
    CC2650STK_GPTIMER1,
    CC2650STK_GPTIMER2,
    CC2650STK_GPTIMER3,
    CC2650STK_GPTIMERCOUNT
} CC2650STK_GPTimers;

/*!
 *  @def    CC2650STK_PWM
 *  @brief  Enum of PWM outputs on the board
 */
typedef enum CC2650STK_PWM
{
    CC2650STK_PWM0 = 0,
    CC2650STK_PWM1,
    CC2650STK_PWM2,
    CC2650STK_PWM3,
    CC2650STK_PWM4,
    CC2650STK_PWM5,
    CC2650STK_PWM6,
    CC2650STK_PWM7,
    CC2650STK_PWMCOUNT
} CC2650STK_PWM;


#ifdef __cplusplus
}
#endif

#endif /* __CC2650STK_SENSORTAG_BOARD_H__ */
