/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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
 *  @file       SDSPICC26XX.h
 *
 *  @brief      SDSPI driver implementation for a CC26XX SPI peripheral used
 *              with the SDSPI driver.
 *
 *  The SDSPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SDSPI.h>
 *  #include <ti/drivers/sdspi/SDSPICC26XX.h>
 *  @endcode
 *
 *  Refer to @ref SDSPI.h for a complete description of APIs & example of use.
 *
 *  This SDSPI driver implementation is designed to operate on a CC26XX SPI
 *  controller using a polling method.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_sdspi_SDSPICC26XX__include
#define ti_drivers_sdspi_SDSPICC26XX__include

#ifdef __cplusplus
extern "C" {
#endif

#define UNICODE

#include <stdint.h>
#include <ti/drivers/SDSPI.h>
#include <ti/mw/fatfs/ff.h>
#include <ti/mw/fatfs/diskio.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/**
 *  @addtogroup SDSPI_STATUS
 *  SDSPICC26XX_STATUS_* macros are command codes only defined in the
 *  SDSPICC26XX.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/sdspi/SDSPICC26XX.h>
 *  @endcode
 *  @{
 */

/* Add SDSPICC26XX_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup SDSPI_CMD
 *  SDSPICC26XX_CMD_* macros are command codes only defined in the
 *  SDSPICC26XX.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/sdspi/SDSPICC26XX.h>
 *  @endcode
 *  @{
 */

/* Add SDSPICC26XX_CMD_* macros here */

/** @}*/

#if defined(CC26XXWARE)
/* c99 types needed by CC26XXWare */
typedef uint32_t            SDSPIBaseAddrType;
typedef uint32_t            SDSPIDataType;
#else /* MWARE */
typedef unsigned long       SDSPIBaseAddrType;
typedef unsigned long       SDSPIDataType;
#endif

/* SDSPI function table */
extern const SDSPI_FxnTable SDSPICC26XX_fxnTable;

/*!
 *  @brief  SD Card type inserted
 */
typedef enum SDSPICC26XX_CardType {
    NOCARD = 0, /*!< Unrecognized Card */
    MMC = 1,    /*!< Multi-media Memory Card (MMC) */
    SDSC = 2,   /*!< Standard SDCard (SDSC) */
    SDHC = 3    /*!< High Capacity SDCard (SDHC) */
} SDSPICC26XX_CardType;

/*!
 *  @brief  SDSPICC26XX Hardware attributes
 *
 *  The SDSPICC26XX configuration structure describes to the SDSPICC26XX
 *  driver implementation hardware specifies on which SPI peripheral, GPIO Pins
 *  and Ports are to be used.
 *
 *  The SDSPICC26XX driver uses this information to:
 *  - configure and reconfigure specific ports/pins to initialize the SD Card
 *    for SPI mode
 *  - identify which SPI peripheral is used for data communications
 *  - identify which GPIO port and pin is used for the SPI chip select
 *    mechanism
 *  - identify which GPIO port and pin is concurrently located on the SPI's MOSI
 *    (TX) pin.
 *
 *  @remark
 *  To initialize the SD Card into SPI mode, the SDSPI driver changes the SPI's
 *  MOSI pin into a GPIO pin so it can kept driven HIGH while the SPI SCK pin
 *  can toggle. After the initialization, the TX pin is reverted back to the SPI
 *  MOSI mode.
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For CC26XXWare these definitions are found in:
 *      - inc/hw_memmap.h
 *
 *  @struct SDSPICC26XX_HWAttrs
 *  An example configuration structure could look as the following:
 *  @code
 *  const SDSPICC26XX_HWAttrs sdspiCC26XXHWattrs = {
 *      {
 *          .baseAddr = SSI3_BASE,
 *
 *          .portSCK = GPIO_PORTQ_BASE,
 *          .pinSCK = GPIO_PIN_0,
 *          .portMISO = GPIO_PORTF_BASE,
 *          .pinMISO = GPIO_PIN_0,
 *          .portMOSI = GPIO_PORTQ_BASE,
 *          .pinMOSI = GPIO_PIN_2,
 *          .portCS = GPIO_PORTH_BASE,
 *          .pinCS = GPIO_PIN_4,
 *       },
 *  };
 *  @endcode
 */
typedef struct SDSPICC26XX_HWAttrs {
    /*!< SSI Peripheral's base address */
    SDSPIBaseAddrType baseAddr;

    /*! SPI Peripheral's power manager ID */
    PowerCC26XX_Resource   powerMngrId;
    /*!< SSI SCK pin */
    uint32_t pinSCK;

    /*!< SSI MISO pin */
    uint32_t pinMISO;

    /*!< SSI MOSI pin */
    uint32_t pinMOSI;

    /*!< GPIO Pin used for the chip select */
    uint32_t pinCS;
} SDSPICC26XX_HWAttrs;

/*!
 *  @brief  SDSPICC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SDSPICC26XX_Object {
    uint32_t                driveNumber;   /*!< Drive number used by FatFs */
    DSTATUS                 diskState;     /*!< Disk status */
    SDSPICC26XX_CardType    cardType;      /*!< SDCard Card Command Class (CCC) */
    uint32_t                bitRate;       /*!< SPI bus bit rate (Hz) */
    FATFS                   filesystem;    /*!< FATFS data object */
    TCHAR*                  path;          /*!< Path string used by FatFs */
} SDSPICC26XX_Object, *SDSPICC26XX_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_sdspi_SDSPICC26XX__include */
