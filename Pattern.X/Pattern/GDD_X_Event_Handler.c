/*****************************************************************************
 *
 * Software License Agreement
 *
 * Copyright © 2012 Microchip Technology Inc.  All rights reserved.
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
* AUTO-GENERATED CODE:  Graphics Display Desinger X
 *****************************************************************************/

/*****************************************************************************
 * SECTION:  Includes
 *****************************************************************************/
#include <Graphics/Graphics.h>
#include "GDD_Screens.h"
#include "Main.h"
#include "Common_DAC_info.h"

extern SMCPU_info_t  SMCPU_sysinfo;
extern BYTE update, Next;
extern unsigned char Screen_done;
extern disp_info_t Disp_Info;
char var_buffer1[10][30];
/******************************************************************************************************
* Function        : GDDDemoGOLMsgCallback
* Description    : Event handling code is written as separate if statements
* param1         : 
* param2         : 
* param3         : 
 *****************************************************************************************************/
BOOL Screen_Off;
BYTE Next_Pressed = 0;
void GDDDemoGOLMsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg)
{
	/**
	 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 AUTO GENERATED CODE. PLEASE DO NOT MODIFY
	 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	*/
	
	// <END_ID_PICT_MSG_SELECTED_(PCB_1)>


	// <START_ID_PICT_MSG_SELECTED_(PCB_253)>
	if(objMsg == PICT_MSG_SELECTED && pObj->ID == (PCB_253) )
	{
//		GDDDemoGoToScreen(C_2DP);
//                Disp_Info.Prv_Screen = Disp_Info.Screen;
//                Disp_Info.Screen = C_2DP;
//                Disp_Info.Disp_state = DRAWING;
	}
	// <END_ID_PICT_MSG_SELECTED_(PCB_253)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_151)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_151) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_166))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_166))), ST_DRAW);
		// Add code
                Disp_Info.CPU_Type = D_CPU2;
                sprintf(&var_buffer1[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                StSetText(((STATICTEXT*)(GOLFindObject(STE_158))), (XCHAR*)&var_buffer1[0][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_158))), ST_DRAW);
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_151)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_153)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_153) )
	{
            Screen_Off = 1;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_153)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_152)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_152) )
	{
		GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_152)>

        // <START_ID_BTN_MSG_RELEASED_(BTN_277)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_277) )
	{
		GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_277)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_161)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_161) )
	{
		GDDDemoGoToScreen(Disp_Info.Prv_Screen);
                Disp_Info.Screen = Disp_Info.Prv_Screen;
                Disp_Info.Prv_Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;	
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_161)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_149)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_149) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_166))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_166))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_158))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_158))), ST_DRAW);
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_149)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_165)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_165) )
	{
            Screen_Off = 1;
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_165)>

        // <START_ID_BTN_MSG_RELEASED_(BTN_487)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_487) )
	{
            Screen_Off = 1;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_487)>

        // <START_ID_BTN_MSG_RELEASED_(BTN_276)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_276) )
	{
            Screen_Off = 1;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_276)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_149)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_485) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_494))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_494))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU2;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_489))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_489))), ST_DRAW);
//            GDDDemoGoToScreen(C_WAIT_FOR_RESET);
//            Disp_Info.Screen = C_WAIT_FOR_RESET;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_149)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_484)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_484) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_494))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_494))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_489))), (XCHAR*)&var_buffer1[0][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_489))), ST_DRAW);
//            GDDDemoGoToScreen(C_WAIT_FOR_RESET);
//            Disp_Info.Screen = C_WAIT_FOR_RESET;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_149)>

        // <START_ID_BTN_MSG_RELEASED_(BTN_486)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_486) )
	{
		GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_486)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_419)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_419) )
	{
            Screen_Off = 1;
            // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_419)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_279)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_279) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_95))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_95))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_90))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_90))), ST_DRAW);
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_279)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_278)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_278) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_95))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_95))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU2;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_90))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_90))), ST_DRAW);
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_278)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_422)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_422) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_228))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_228))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_223))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_223))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_422)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_421)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_421) )
	{
            	StSetText(((STATICTEXT*)(GOLFindObject(STE_228))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_228))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU2;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_223))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_223))), ST_DRAW);
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_421)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_420)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_420) )
	{
                GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_420)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_423)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_423) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_500))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_500))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_485))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_485))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_423)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_424)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_424) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_500))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_500))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU2;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_485))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_485))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_424)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_425)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_425) )
	{
                GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_425)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_426)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_426) )
	{
            Screen_Off = 1;
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_426)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_382)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_382) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_209))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_209))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_204))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_204))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_382)>

        // <START_ID_BTN_MSG_RELEASED_(BTN_381)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_381) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_209))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_209))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU2;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_204))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_204))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_381)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_380)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_380) )
	{
                GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_380)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_379)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_379) )
	{
            Screen_Off = 1;
		// Add code 
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_379)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_558)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_558) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_537))), (XCHAR*)"CPU1");
		SetState(((STATICTEXT*)(GOLFindObject(STE_537))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU1;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_532))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_532))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_558)>


    // <START_ID_BTN_MSG_RELEASED_(BTN_557)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_557) )
	{
		StSetText(((STATICTEXT*)(GOLFindObject(STE_537))), (XCHAR*)"CPU2");
		SetState(((STATICTEXT*)(GOLFindObject(STE_537))), ST_DRAW);
                sprintf(&var_buffer1[1][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                Disp_Info.CPU_Type = D_CPU2;
                StSetText(((STATICTEXT*)(GOLFindObject(STE_532))), (XCHAR*)&var_buffer1[1][0]);
                SetState(((STATICTEXT*)(GOLFindObject(STE_532))), ST_DRAW);
                // Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_557)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_556)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_556) )
	{
                GDDDemoGoToScreen(SMCPU);
                Disp_Info.Prv_Screen = Disp_Info.Screen;
                Disp_Info.Screen = SMCPU;
                Disp_Info.Disp_state = DRAWING;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_556)>

	// <START_ID_BTN_MSG_RELEASED_(BTN_608)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_608) )
	{
            Next_Pressed = 1;
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_608)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_609)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_609) )
	{
            Next_Pressed = 1;
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_609)>


	// <START_ID_BTN_MSG_RELEASED_(BTN_610)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_610) )
	{
            Next_Pressed = 1;
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_610)>



	// <START_ID_BTN_MSG_RELEASED_(BTN_555)>
	if(objMsg == BTN_MSG_RELEASED && pObj->ID == (BTN_555) )
	{
            Screen_Off = 1;
		// Add code
	}
	// <END_ID_BTN_MSG_RELEASED_(BTN_555)>

	// EVENT_CODE_END End of GDD X Code Generation
	// DO NOT MODIFY/CHANGE THE ABOVE COMMENT

}
