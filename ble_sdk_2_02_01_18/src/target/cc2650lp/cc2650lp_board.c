/******************************************************************************

 @file  cc2650lp_board.c

 @brief This file is a simple gateway to include the appropriate Board.c
        file which is located in the following directories relative to this file:
        - CC2650_LAUNCHXL
        - CC1350_LAUNCHXL
  
        The project should set the include path to Board.h to point to
        the Board.h in one of these 4 directories.
        This Board.h file will then define a symbol which is used in this
        file to include the appropriate Board.c file which is found in
        the same directory as Board.h
        This way the project can look the same (and only include this Board.c)
        file, when changing EM user only needs to update include path in
        the project options.
        Alternatively, the device specific board  files can just be included
        directly in the project.

 Group: WCS, LPC, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2015-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/*
 *   The location of this Board.h file depends on your project include path.
 *   Set it correctly to point to your CCXX50_LAUNCHXL
 */
#if defined(CC2650_LAUNCHXL)
    #include <../../boards/CC2650_LAUNCHXL/Board.h>
    #include <../../boards/CC2650_LAUNCHXL/CC2650_LAUNCHXL.c>
#elif defined(CC1350_LAUNCHXL)
    #include <../../boards/CC1350_LAUNCHXL/Board.c>
    #include <../../boards/CC1350_LAUNCHXL/CC1350_LAUNCHXL.c>
#else
    #error "Must define either 'CC2650_LAUNCHXL' or 'CC1350_LAUNCHXL'. Please \
            set include path to point to appropriate device."
#endif



