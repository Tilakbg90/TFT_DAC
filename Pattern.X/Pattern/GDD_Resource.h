/*****************************************************************************
 * FileName:        GDD_Resource.h
 * Processor:       PIC24F, PIC24H, dsPIC
 * Compiler:        MPLAB C30/XC16 (see release notes for tested revision)
 * Linker:          MPLAB LINK30/XC16
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright(c) 2012 Microchip Technology Inc.  All rights reserved.
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
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * AUTO-GENERATED CODE:  Graphics Resource Converter version: 3.29.03
 *****************************************************************************/

#ifndef GDD_RESOURCE_H_FILE
#define GDD_RESOURCE_H_FILE
/*****************************************************************************
 * SECTION:  Includes
 *****************************************************************************/
#include <Graphics/Graphics.h>
#include "HardwareProfile.h"

/*****************************************************************************
 * SECTION:  Graphics Library Firmware Check
 *****************************************************************************/
#if(GRAPHICS_LIBRARY_VERSION != 0x0306)
#warning "It is suggested to use Graphics Library version 3.06 with this version of GRC."
#endif



/*****************************************************************************
 * This is an error check for the color depth
 *****************************************************************************/
#if (COLOR_DEPTH > 16)
#error "Color Depth needs to be 16 to correctly use these resources"
#endif



/*****************************************************************************
 * SECTION:  BITMAPS
 *****************************************************************************/

/*********************************
 * Bitmap Structure
 * Label: logo_small2_db2
 * Description:  63x44 pixels, 16-bits per pixel
 ***********************************/
extern const IMAGE_FLASH logo_small2_db2;
#define logo_small2_db2_WIDTH     (63)
#define logo_small2_db2_HEIGHT    (44)
#define logo_small2_db2_SIZE      (5550)
/*********************************
 * Bitmap Structure
 * Label: No_Train
 * Description:  63x44 pixels, 16-bits per pixel
 ***********************************/
extern const IMAGE_FLASH No_Train;
#define No_Train_WIDTH     (63)
#define No_Train_HEIGHT    (44)
#define No_Train_SIZE      (5550)
/*********************************
 * Bitmap Structure
 * Label: IR_Logo_small_4_DB2
 * Description:  52x52 pixels, 16-bits per pixel
 ***********************************/
extern const IMAGE_FLASH IR_Logo_small_4_DB2;
#define IR_Logo_small_4_DB2_WIDTH     (52)
#define IR_Logo_small_4_DB2_HEIGHT    (52)
#define IR_Logo_small_4_DB2_SIZE      (5414)
/*********************************
 * Bitmap Structure
 * Label: log_bmp_blue_t_dark2
 * Description:  103x94 pixels, 16-bits per pixel
 ***********************************/
extern const IMAGE_FLASH log_bmp_blue_t_dark2;
#define log_bmp_blue_t_dark2_WIDTH     (103)
#define log_bmp_blue_t_dark2_HEIGHT    (94)
#define log_bmp_blue_t_dark2_SIZE      (19370)
/*********************************
 * Bitmap Structure
 * Label: logo_small2_db2_inv
 * Description:  63x44 pixels, 16-bits per pixel
 ***********************************/
extern const IMAGE_FLASH logo_small2_db2_inv;
#define logo_small2_db2_inv_WIDTH     (63)
#define logo_small2_db2_inv_HEIGHT    (44)
#define logo_small2_db2_inv_SIZE      (5550)
/*****************************************************************************
 * SECTION:  Fonts
 *****************************************************************************/

/*********************************
 * Installed Font Structure
 * Label: Times_New_Roman_12
 * Description:  Height: 14 pixels, 1 bit per pixel, Range: ' ' to '~'
 ***********************************/
extern const FONT_FLASH Times_New_Roman_12;
#define Times_New_Roman_12_SIZE    (1956)
/*********************************
 * Installed Font Structure
 * Label: Handel_Gothic_22
 * Description:  Height: 29 pixels, 1 bit per pixel, Range: ' ' to '~'
 ***********************************/
extern const FONT_FLASH Handel_Gothic_22;
#define Handel_Gothic_22_SIZE    (5405)
/*********************************
 * Installed Font Structure
 * Label: Book_Antiqua_18
 * Description:  Height: 23 pixels, 1 bit per pixel, Range: ' ' to '~'
 ***********************************/
extern const FONT_FLASH Book_Antiqua_18;
#define Book_Antiqua_18_SIZE    (4160)
/*********************************
 * Installed Font Structure
 * Label: Monospaced_bold_Bold_14_1
 * Description:  Height: 20 pixels, 1 bit per pixel, Range: ' ' to ''
 ***********************************/
extern const FONT_FLASH Monospaced_bold_Bold_14_1;
#define Monospaced_bold_Bold_14_1_SIZE    (2432)
/*********************************
 * Installed Font Structure
 * Label: Calibri_Light_18
 * Description:  Height: 24 pixels, 1 bit per pixel, Range: ' ' to '~'
 ***********************************/
extern const FONT_FLASH Calibri_Light_18;
#define Calibri_Light_18_SIZE    (4276)
/*********************************
 * Installed Font Structure
 * Label: Calibri_Bold_14
 * Description:  Height: 17 pixels, 1 bit per pixel, Range: ' ' to '~'
 ***********************************/
extern const FONT_FLASH Calibri_Bold_14;
#define Calibri_Bold_14_SIZE    (2284)
/*********************************
 * TTF Font File Structure
 * Label: Gentium_16
 * Description:  Height: 19 pixels, 1 bit per pixel, Range: ' ' to '~'
 ***********************************/
extern const FONT_FLASH Gentium_16;
#define Gentium_16_SIZE    (2668)
#endif

