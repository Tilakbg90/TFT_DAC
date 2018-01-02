/*****************************************************************************
 * Microchip Graphics Library
 * Graphics Display Designer (GDD) Template
 *****************************************************************************
 * FileName:        Main.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30/C32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright © 2010 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Date         Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *****************************************************************************/
#ifndef _MAIN_H_
    #define _MAIN_H_

////////////////////////////// INCLUDES //////////////////////////////
////////////////////////////// INCLUDES //////////////////////////////
    #include "Compiler.h"
    #include "GenericTypeDefs.h"
    #include "Graphics/Graphics.h"
    #include "drv_spi.h"
    #include "MCHP25LC256.h"
    #include "SST25VF016.h"
    #include "TouchScreen.h"
    #include "Potentiometer.h"
    #include "Beep.h"
    #include "HardwareProfile.h"
    #include "Graphics/mchpGfxDrv.h"
    #include <cpld.h>
#include "DRV_SPI_SMCPU.h"
void Start_Home_Screen_Time(void);
void Wake_screen(void);

typedef enum
{
    HOME = 0,
    C_3D2S,
    C_2DP,
    C_3DPAS,
    C_LCWS,
    SMCPU,
    RESET,
    C_WAIT_FOR_RESET,
    C_DE,
    C_4D1S
}Screens_info;

typedef enum
{
    IDLE = 0,
    DRAWING,
    DONE,
}GLCD_state;

typedef enum
{
    SPI_IDLE ,
    SPI_IN_PROGRESS,
    SPI_DONE,
            SPI_GOOD,
}SPI_info;

typedef enum
{
    UT_DE = 0,
    UT_SF,
    UT_EF,
    UT_CF,
    UT_D3A,
    UT_D3B,
    UT_D3C,
    UT_3D_SF,
    UT_3D_EF,        
    UT_LCWS = 9,
    UT_LCWS_DL = 10,
            UT_D4A,
            UT_D4B,
            UT_D4C,
            UT_D4D,
}Unit_Type_info;

typedef enum
{
    D_CPU1=0,
            D_CPU2,
            D_SMCPU,
}CPU_info;
typedef enum{
        ZERO_MODE = 0,
        WAITING_FOR_RESET_AT_BOTH_UNITS,
        RESET_APPLIED_AT_LOCAL_UNIT,
        RESET_APPLIED_AT_REMOTE_UNIT,
        RESET_APPLIED_AT_BOTH_UNITS,
        SECTION_WAIT_FOR_PILOT_TRAIN,
        SECTION_CLEAR_AT_BOTH_UNITS,
        SECTION_OCCUPIED_AT_BOTH_UNITS,
        SECTION_ERROR_AT_BOTH_UNITS,
        ERROR_RESET_APPLIED_AT_LOCAL_UNIT,
        ERROR_RESET_APPLIED_AT_REMOTE_UNIT,
        ERROR_LOCAL_UNIT_WAITING_FOR_RESET,
        ERROR_REMOTE_UNIT_WAITING_FOR_RESET
}Reset_info;

typedef struct
{
    WORD    L_FC;
    WORD    R_FC;
    WORD    L_RC;
    WORD    R_RC;
}Conf_wheel_info;

typedef struct
{
    WORD    SF_FC;
    WORD    SF_RC;
    WORD    S_CF_FC;
    WORD    S_CF_RC;
    WORD    E_CF_FC;
    WORD    E_CF_RC;
    WORD    EF_FC;
    WORD    EF_RC;
}Conf_wheel_info_3D2S;

typedef struct
{
    WORD    SF_FC;
    WORD    SF_RC;
    WORD    S_CF_FC;
    WORD    S_CF_RC;
    WORD    S_EF_FC;
    WORD    S_EF_RC;
    WORD    EF_FC;
    WORD    EF_RC;
}Conf_wheel_info_3D_SF;

typedef struct
{
    WORD    SF_FC;
    WORD    SF_RC;
    WORD    E_SF_FC;
    WORD    E_SF_RC;
    WORD    E_CF_FC;
    WORD    E_CF_RC;
    WORD    EF_FC;
    WORD    EF_RC;
}Conf_wheel_info_3D_EF;

typedef struct
{
    WORD    A_ENTRY;
    WORD    A_EXIT;
    WORD    B_ENTRY;
    WORD    B_EXIT;
    WORD    C_ENTRY;
    WORD    C_EXIT;
}Conf_wheel_info_3D1S;

typedef struct
{
    WORD    A_ENTRY;
    WORD    A_EXIT;
    WORD    B_ENTRY;
    WORD    B_EXIT;
    WORD    C_ENTRY;
    WORD    C_EXIT;
    WORD    D_ENTRY;
    WORD    D_EXIT;
}Conf_wheel_info_4D1S;

typedef struct
{
    WORD    A_FC;
    WORD    A_RC;
    WORD    B_FC;
    WORD    B_RC;
    WORD    C_FC;
    WORD    C_RC;
    WORD    D_FC;
    WORD    D_RC;
}Conf_wheel_info_LCWS;

typedef struct {
    GLCD_state  Disp_state;
    Screens_info Screen;
    Screens_info Prv_Screen;
    BYTE        Update;
    SPI_info    SPI_state;
    Unit_Type_info  Unit_Type;
    CPU_info        CPU_Type;
    unsigned char            CPU1_address;
    unsigned char            CPU2_address;
    Reset_info      Reset_mode;
    BYTE            MessageID;
    Reset_info            DS_mode;
    Reset_info            US_mode;
    WORD            Train_Speed;
}disp_info_t;

#define WAITING_FOR_RESET   0xAA
#define RESET_APPLIED       0x55
#define NO_DATA             0x3C

#endif
