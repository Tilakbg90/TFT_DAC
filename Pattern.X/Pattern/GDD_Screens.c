
/*****************************************************************************
* Microchip Graphics Library
* Graphics Display Designer (GDD) Template
*****************************************************************************
 
* Dependencies:    See INCLUDES section below
* Processor:       PIC24F, PIC24H, dsPIC, PIC32
* Compiler:        MPLAB C30 V3.22, MPLAB C32 V1.05b
* Linker:          MPLAB LINK30, MPLAB LINK32
* Company:         Microchip Technology Incorporated
*
* Software License Agreement
*
* Copyright (c) 2010 Microchip Technology Inc.  All rights reserved.
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller, which is integrated into your product or third party
* product (pursuant to the sublicense terms in the accompanying license
* agreement).  
*
* You should refer to the license agreement accompanying this Software
* for additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY
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
* Author               Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
*****************************************************************************/


/***************************************************
*INCLUDE FILES
***************************************************/
#include "Graphics/Graphics.h"
#include "GDD_Screens.h"

/***************************************************
* String literals used in the project
***************************************************/
const XCHAR Home_STE_611text[ ] = " Waiting for Communication";

const XCHAR Home_OTE_2text[ ] = "Single Section Digital Axle Counter (SSDAC)";
const XCHAR Home_OTE_5text[ ] = "Insys Digital Systems Pvt Ltd.";

const XCHAR CPU_3D2S_STE_89text[ ] = " ";
const XCHAR CPU_3D2S_STE_90text[ ] = " ";
const XCHAR CPU_3D2S_STE_91text[ ] = "CONF-3DP2S";
const XCHAR CPU_3D2S_STE_92text[ ] = "CF";
const XCHAR CPU_3D2S_STE_93text[ ] = " ";
const XCHAR CPU_3D2S_STE_94text[ ] = " ";
const XCHAR CPU_3D2S_STE_95text[ ] = "CPU1";
const XCHAR CPU_3D2S_BTN_276text[ ] = "OFF";
const XCHAR CPU_3D2S_BTN_277text[ ] = "DATA";
const XCHAR CPU_3D2S_BTN_278text[ ] = "CPU2";
const XCHAR CPU_3D2S_BTN_279text[ ] = "CPU1";
const XCHAR CPU_3D2S_STE_281text[ ] = " ";
const XCHAR CPU_3D2S_STE_283text[ ] = "SF FC";
const XCHAR CPU_3D2S_STE_284text[ ] = "SF RC";
const XCHAR CPU_3D2S_STE_285text[ ] = "EF FC";
const XCHAR CPU_3D2S_STE_286text[ ] = "EF RC";
const XCHAR CPU_3D2S_STE_287text[ ] = " ";
const XCHAR CPU_3D2S_STE_289text[ ] = " ";
const XCHAR CPU_3D2S_STE_290text[ ] = " ";
const XCHAR CPU_3D2S_STE_291text[ ] = " ";
const XCHAR CPU_3D2S_STE_292text[ ] = " ";
const XCHAR CPU_3D2S_STE_293text[ ] = " ";
const XCHAR CPU_3D2S_STE_294text[ ] = " ";
const XCHAR CPU_3D2S_STE_312text[ ] = " ";
const XCHAR CPU_3D2S_STE_313text[ ] = " ";
const XCHAR CPU_3D2S_STE_314text[ ] = " ";
const XCHAR CPU_3D2S_STE_315text[ ] = " ";
const XCHAR CPU_3D2S_STE_316text[ ] = "S-CF FC";
const XCHAR CPU_3D2S_STE_317text[ ] = "S-CF RC";
const XCHAR CPU_3D2S_STE_325text[ ] = "SF Status:";
const XCHAR CPU_3D2S_STE_326text[ ] = "E-CF Status:";
const XCHAR CPU_3D2S_STE_327text[ ] = "EF Status:";
const XCHAR CPU_3D2S_STE_329text[ ] = " ";
const XCHAR CPU_3D2S_STE_330text[ ] = " ";
const XCHAR CPU_3D2S_STE_331text[ ] = " ";
const XCHAR CPU_3D2S_STE_334text[ ] = "Speed: Km/hr";
const XCHAR CPU_3D2S_STE_335text[ ] = " ";
const XCHAR CPU_3D2S_RDB_366text[ ] = "SF";
const XCHAR CPU_3D2S_RDB_367text[ ] = "CF";
const XCHAR CPU_3D2S_RDB_368text[ ] = "EF";
const XCHAR CPU_3D2S_STE_503text[ ] = "E-CF FC";
const XCHAR CPU_3D2S_STE_504text[ ] = "E-CF RC";
const XCHAR CPU_3D2S_STE_505text[ ] = " ";
const XCHAR CPU_3D2S_STE_506text[ ] = " ";
const XCHAR CPU_3D2S_STE_507text[ ] = " ";
const XCHAR CPU_3D2S_STE_508text[ ] = " ";
const XCHAR CPU_3D2S_STE_509text[ ] = "S-CF Status:";
const XCHAR CPU_3D2S_STE_510text[ ] = " ";
const XCHAR CPU_3D2S_BTN_608text[ ] = "Next";

const XCHAR CPU_2DP_BTN_149text[ ] = "CPU1";
const XCHAR CPU_2DP_BTN_151text[ ] = "CPU2";
const XCHAR CPU_2DP_BTN_152text[ ] = "DATA";
const XCHAR CPU_2DP_BTN_153text[ ] = "OFF";
const XCHAR CPU_2DP_STE_157text[ ] = "CRC: ";
const XCHAR CPU_2DP_STE_158text[ ] = " Address:";
const XCHAR CPU_2DP_STE_159text[ ] = "  CONF-";
const XCHAR CPU_2DP_STE_163text[ ] = " ";
const XCHAR CPU_2DP_STE_164text[ ] = " ";
const XCHAR CPU_2DP_STE_165text[ ] = " ";
const XCHAR CPU_2DP_STE_166text[ ] = "CPU1";
const XCHAR CPU_2DP_STE_175text[ ] = " ";
const XCHAR CPU_2DP_STE_176text[ ] = "Local Status:";
const XCHAR CPU_2DP_STE_177text[ ] = "Remote Status:";
const XCHAR CPU_2DP_STE_179text[ ] = "Speed: Km/hr";
const XCHAR CPU_2DP_STE_196text[ ] = " ";
const XCHAR CPU_2DP_STE_302text[ ] = " ";
const XCHAR CPU_2DP_STE_303text[ ] = " ";
const XCHAR CPU_2DP_STE_304text[ ] = " ";
const XCHAR CPU_2DP_STE_305text[ ] = " ";
const XCHAR CPU_2DP_STE_306text[ ] = " ";
const XCHAR CPU_2DP_STE_307text[ ] = " ";
const XCHAR CPU_2DP_STE_308text[ ] = " ";
const XCHAR CPU_2DP_STE_309text[ ] = " ";
const XCHAR CPU_2DP_STE_266text[ ] = " ";
const XCHAR CPU_2DP_STE_267text[ ] = "CPU1";
const XCHAR CPU_2DP_STE_269text[ ] = "CPU2";
const XCHAR CPU_2DP_STE_270text[ ] = "LFC";
const XCHAR CPU_2DP_STE_272text[ ] = "LRC";
const XCHAR CPU_2DP_STE_273text[ ] = "RFC";
const XCHAR CPU_2DP_STE_274text[ ] = "RRC";
const XCHAR CPU_2DP_RDB_439text[ ] = "EF";
const XCHAR CPU_2DP_RDB_484text[ ] = "SF";

const XCHAR CPU_3DP_AS_STE_203text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_204text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_205text[ ] = "  CONF-";
const XCHAR CPU_3DP_AS_STE_206text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_207text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_208text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_209text[ ] = " CPU1";
const XCHAR CPU_3DP_AS_STE_338text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_339text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_340text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_341text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_350text[ ] = "Unit A";
const XCHAR CPU_3DP_AS_STE_351text[ ] = "Unit B";
const XCHAR CPU_3DP_AS_STE_352text[ ] = "Unit C";
const XCHAR CPU_3DP_AS_STE_356text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_357text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_372text[ ] = "Unit A Status:";
const XCHAR CPU_3DP_AS_STE_373text[ ] = "Unit B Status:";
const XCHAR CPU_3DP_AS_STE_374text[ ] = "Unit C Status:";
const XCHAR CPU_3DP_AS_STE_376text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_377text[ ] = "Speed: Km/hr";
const XCHAR CPU_3DP_AS_STE_378text[ ] = " ";
const XCHAR CPU_3DP_AS_BTN_379text[ ] = "OFF";
const XCHAR CPU_3DP_AS_BTN_380text[ ] = "DATA";
const XCHAR CPU_3DP_AS_BTN_381text[ ] = "CPU2";
const XCHAR CPU_3DP_AS_BTN_382text[ ] = "CPU1";
const XCHAR CPU_3DP_AS_STE_467text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_468text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_469text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_470text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_471text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_472text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_473text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_474text[ ] = " ";
const XCHAR CPU_3DP_AS_STE_369text[ ] = "Entry CPU1";
const XCHAR CPU_3DP_AS_STE_370text[ ] = "Entry CPU2";
const XCHAR CPU_3DP_AS_STE_371text[ ] = "Exit CPU1";
const XCHAR CPU_3DP_AS_STE_375text[ ] = "Exit CPU2";
const XCHAR CPU_3DP_AS_RDB_380text[ ] = "A";
const XCHAR CPU_3DP_AS_RDB_381text[ ] = "B";
const XCHAR CPU_3DP_AS_RDB_382text[ ] = "C";
const XCHAR CPU_3DP_AS_BTN_610text[ ] = "Next";

const XCHAR CPU_LCWS_STE_222text[ ] = " ";
const XCHAR CPU_LCWS_STE_223text[ ] = " ";
const XCHAR CPU_LCWS_STE_224text[ ] = "CONF-TWS";
const XCHAR CPU_LCWS_STE_225text[ ] = " ";
const XCHAR CPU_LCWS_STE_226text[ ] = " ";
const XCHAR CPU_LCWS_STE_227text[ ] = " ";
const XCHAR CPU_LCWS_STE_228text[ ] = "CPU1";
const XCHAR CPU_LCWS_STE_384text[ ] = "SA-FC";
const XCHAR CPU_LCWS_STE_385text[ ] = "SA-RC";
const XCHAR CPU_LCWS_STE_386text[ ] = "SB-FC";
const XCHAR CPU_LCWS_STE_387text[ ] = "SB-RC";
const XCHAR CPU_LCWS_STE_388text[ ] = "SC-FC";
const XCHAR CPU_LCWS_STE_390text[ ] = "SC-RC";
const XCHAR CPU_LCWS_STE_391text[ ] = "SD-FC";
const XCHAR CPU_LCWS_STE_392text[ ] = "SD-RC";
const XCHAR CPU_LCWS_STE_393text[ ] = " ";
const XCHAR CPU_LCWS_STE_394text[ ] = " ";
const XCHAR CPU_LCWS_STE_395text[ ] = " ";
const XCHAR CPU_LCWS_STE_396text[ ] = " ";
const XCHAR CPU_LCWS_STE_397text[ ] = " ";
const XCHAR CPU_LCWS_STE_398text[ ] = " ";
const XCHAR CPU_LCWS_STE_399text[ ] = " ";
const XCHAR CPU_LCWS_STE_400text[ ] = " ";
const XCHAR CPU_LCWS_STE_401text[ ] = " ";
const XCHAR CPU_LCWS_STE_402text[ ] = " ";
const XCHAR CPU_LCWS_STE_403text[ ] = " ";
const XCHAR CPU_LCWS_STE_404text[ ] = " ";
const XCHAR CPU_LCWS_STE_405text[ ] = " ";
const XCHAR CPU_LCWS_STE_406text[ ] = " ";
const XCHAR CPU_LCWS_STE_407text[ ] = " ";
const XCHAR CPU_LCWS_STE_408text[ ] = " ";
const XCHAR CPU_LCWS_STE_410text[ ] = "TWS Status:";
const XCHAR CPU_LCWS_STE_416text[ ] = " ";
const XCHAR CPU_LCWS_BTN_419text[ ] = "OFF";
const XCHAR CPU_LCWS_BTN_420text[ ] = "DATA";
const XCHAR CPU_LCWS_BTN_421text[ ] = "CPU2";
const XCHAR CPU_LCWS_BTN_422text[ ] = "CPU1";
const XCHAR CPU_LCWS_RDB_389text[ ] = "A";
const XCHAR CPU_LCWS_RDB_390text[ ] = "B";
const XCHAR CPU_LCWS_RDB_391text[ ] = "C";
const XCHAR CPU_LCWS_RDB_392text[ ] = "D";

const XCHAR SMCPU_BTN_161text[ ] = "CPU";
const XCHAR SMCPU_BTN_165text[ ] = "OFF";
const XCHAR SMCPU_STE_167text[ ] = " ";
const XCHAR SMCPU_STE_168text[ ] = "  Address:";
const XCHAR SMCPU_STE_170text[ ] = " ";
const XCHAR SMCPU_STE_171text[ ] = " ";
const XCHAR SMCPU_STE_172text[ ] = " ";
const XCHAR SMCPU_STE_173text[ ] = "SM CPU";
const XCHAR SMCPU_GRB_193text[ ] = "Event Data Log";
const XCHAR SMCPU_STE_243text[ ] = "From:";
const XCHAR SMCPU_STE_244text[ ] = "To:";
const XCHAR SMCPU_STE_245text[ ] = "No of Events:";
const XCHAR SMCPU_STE_246text[ ] = " ";
const XCHAR SMCPU_STE_247text[ ] = " ";
const XCHAR SMCPU_STE_248text[ ] = " ";
const XCHAR SMCPU_STE_169text[ ] = "CONF- ";
const XCHAR SMCPU_STE_609text[ ] = "Comm US Pkt Error count:";
const XCHAR SMCPU_STE_610text[ ] = "Comm DS Pkt Error count:";
const XCHAR SMCPU_STE_612text[ ] = " ";
const XCHAR SMCPU_STE_613text[ ] = " ";

const XCHAR RESET_BTN_201text[ ] = "OFF";
const XCHAR RESET_STE_211text[ ] = "CRC: 0x12345678";
const XCHAR RESET_STE_249text[ ] = "  Address: 001";
const XCHAR RESET_STE_250text[ ] = "Debug";
const XCHAR RESET_STE_252text[ ] = "Good";
const XCHAR RESET_STE_253text[ ] = "DD:MM:YY";
const XCHAR RESET_STE_254text[ ] = "HH:MM";
const XCHAR RESET_STE_256text[ ] = "SMCPU";
const XCHAR RESET_STE_419text[ ] = "3234";
const XCHAR RESET_STE_420text[ ] = "3234";
const XCHAR RESET_STE_421text[ ] = "3234";
const XCHAR RESET_STE_422text[ ] = "3234";
const XCHAR RESET_STE_423text[ ] = "3234";
const XCHAR RESET_STE_424text[ ] = "3234";
const XCHAR RESET_STE_425text[ ] = "3234";
const XCHAR RESET_STE_426text[ ] = "3234";
const XCHAR RESET_STE_427text[ ] = "3234";
const XCHAR RESET_STE_428text[ ] = "3234";
const XCHAR RESET_STE_429text[ ] = "3234";
const XCHAR RESET_STE_430text[ ] = "3234";
const XCHAR RESET_STE_431text[ ] = "3234";
const XCHAR RESET_STE_432text[ ] = "3234";
const XCHAR RESET_STE_433text[ ] = "3234";
const XCHAR RESET_STE_434text[ ] = "3234";
const XCHAR RESET_STE_437text[ ] = "3234";
const XCHAR RESET_STE_438text[ ] = "3234";
const XCHAR RESET_STE_439text[ ] = "3234";
const XCHAR RESET_STE_440text[ ] = "3234";
const XCHAR RESET_STE_441text[ ] = "3234";
const XCHAR RESET_STE_442text[ ] = "3234";
const XCHAR RESET_STE_443text[ ] = "3234";
const XCHAR RESET_STE_444text[ ] = "3234";
const XCHAR RESET_STE_445text[ ] = "3234";
const XCHAR RESET_STE_446text[ ] = "3234";
const XCHAR RESET_STE_447text[ ] = "3234";
const XCHAR RESET_STE_448text[ ] = "3234";
const XCHAR RESET_STE_449text[ ] = "3234";
const XCHAR RESET_STE_450text[ ] = "3234";
const XCHAR RESET_STE_451text[ ] = "3234";
const XCHAR RESET_STE_452text[ ] = "3234";
const XCHAR RESET_STE_453text[ ] = "3234";
const XCHAR RESET_STE_454text[ ] = "3234";
const XCHAR RESET_STE_455text[ ] = "3234";
const XCHAR RESET_STE_456text[ ] = "3234";
const XCHAR RESET_STE_457text[ ] = "3234";
const XCHAR RESET_STE_458text[ ] = "3234";
const XCHAR RESET_STE_459text[ ] = "3234";
const XCHAR RESET_STE_460text[ ] = "3234";
const XCHAR RESET_STE_461text[ ] = "3234";
const XCHAR RESET_STE_462text[ ] = "3234";
const XCHAR RESET_STE_463text[ ] = "3234";
const XCHAR RESET_STE_464text[ ] = "3234";
const XCHAR RESET_STE_465text[ ] = "3234";
const XCHAR RESET_STE_466text[ ] = "3234";
const XCHAR RESET_STE_475text[ ] = "3234";
const XCHAR RESET_STE_476text[ ] = "3234";
const XCHAR RESET_STE_477text[ ] = "3234";
const XCHAR RESET_STE_478text[ ] = "3234";
const XCHAR RESET_STE_479text[ ] = "3234";
const XCHAR RESET_STE_480text[ ] = "3234";
const XCHAR RESET_STE_481text[ ] = "3234";
const XCHAR RESET_STE_482text[ ] = "3234";
const XCHAR RESET_STE_483text[ ] = "3234";

const XCHAR WAIT_FOR_RESET_BTN_484text[ ] = "CPU1";
const XCHAR WAIT_FOR_RESET_BTN_485text[ ] = "CPU2";
const XCHAR WAIT_FOR_RESET_BTN_486text[ ] = "DATA";
const XCHAR WAIT_FOR_RESET_BTN_487text[ ] = "OFF";
const XCHAR WAIT_FOR_RESET_STE_488text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_489text[ ] = " Address: 001";
const XCHAR WAIT_FOR_RESET_STE_490text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_491text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_492text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_493text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_494text[ ] = "CPU1";
const XCHAR WAIT_FOR_RESET_STE_495text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_496text[ ] = "Local Status:";
const XCHAR WAIT_FOR_RESET_STE_511text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_497text[ ] = "Remote Status:";
const XCHAR WAIT_FOR_RESET_STE_501text[ ] = "SF Status:";
const XCHAR WAIT_FOR_RESET_STE_502text[ ] = " ";
const XCHAR WAIT_FOR_RESET_STE_417text[ ] = "Remote Status:";
const XCHAR WAIT_FOR_RESET_STE_418text[ ] = " ";

const XCHAR CPU_DE_BTN_423text[ ] = "CPU1";
const XCHAR CPU_DE_BTN_424text[ ] = "CPU2";
const XCHAR CPU_DE_BTN_425text[ ] = "DATA";
const XCHAR CPU_DE_BTN_426text[ ] = "OFF";
const XCHAR CPU_DE_STE_484text[ ] = "CRC: ";
const XCHAR CPU_DE_STE_485text[ ] = " Address:";
const XCHAR CPU_DE_STE_486text[ ] = "  CONF-";
const XCHAR CPU_DE_STE_487text[ ] = " ";
const XCHAR CPU_DE_STE_498text[ ] = " ";
const XCHAR CPU_DE_STE_499text[ ] = " ";
const XCHAR CPU_DE_STE_500text[ ] = "CPU1";
const XCHAR CPU_DE_STE_512text[ ] = " ";
const XCHAR CPU_DE_STE_513text[ ] = "DE Status:";
const XCHAR CPU_DE_STE_515text[ ] = "Speed: Km/hr";
const XCHAR CPU_DE_STE_519text[ ] = " ";
const XCHAR CPU_DE_STE_520text[ ] = " ";
const XCHAR CPU_DE_STE_521text[ ] = " ";
const XCHAR CPU_DE_STE_529text[ ] = "CPU1";
const XCHAR CPU_DE_STE_530text[ ] = "CPU2";
const XCHAR CPU_DE_STE_531text[ ] = "LC";
const XCHAR CPU_DE_RDB_535text[ ] = "DE";

const XCHAR CPU_4D1S_STE_528text[ ] = " ";
const XCHAR CPU_4D1S_STE_532text[ ] = " ";
const XCHAR CPU_4D1S_STE_533text[ ] = "  CONF-4DP1S";
const XCHAR CPU_4D1S_STE_534text[ ] = " ";
const XCHAR CPU_4D1S_STE_535text[ ] = " ";
const XCHAR CPU_4D1S_STE_536text[ ] = " ";
const XCHAR CPU_4D1S_STE_537text[ ] = " CPU1";
const XCHAR CPU_4D1S_STE_544text[ ] = "Entry A";
const XCHAR CPU_4D1S_STE_545text[ ] = "Entry B";
const XCHAR CPU_4D1S_STE_546text[ ] = "Entry C";
const XCHAR CPU_4D1S_STE_549text[ ] = "Remote Status:";
const XCHAR CPU_4D1S_BTN_555text[ ] = "OFF";
const XCHAR CPU_4D1S_BTN_556text[ ] = "DATA";
const XCHAR CPU_4D1S_BTN_557text[ ] = "CPU2";
const XCHAR CPU_4D1S_BTN_558text[ ] = "CPU1";
const XCHAR CPU_4D1S_STE_564text[ ] = " ";
const XCHAR CPU_4D1S_STE_567text[ ] = "CPU1";
const XCHAR CPU_4D1S_STE_568text[ ] = "CPU2";
const XCHAR CPU_4D1S_STE_583text[ ] = "Entry D";
const XCHAR CPU_4D1S_STE_584text[ ] = " ";
const XCHAR CPU_4D1S_STE_585text[ ] = " ";
const XCHAR CPU_4D1S_STE_586text[ ] = " ";
const XCHAR CPU_4D1S_STE_587text[ ] = " ";
const XCHAR CPU_4D1S_STE_588text[ ] = " ";
const XCHAR CPU_4D1S_STE_589text[ ] = " ";
const XCHAR CPU_4D1S_STE_590text[ ] = " ";
const XCHAR CPU_4D1S_STE_591text[ ] = " ";
const XCHAR CPU_4D1S_STE_592text[ ] = "Exit D";
const XCHAR CPU_4D1S_STE_593text[ ] = "Exit C";
const XCHAR CPU_4D1S_STE_594text[ ] = "Exit B";
const XCHAR CPU_4D1S_STE_595text[ ] = "Exit A";
const XCHAR CPU_4D1S_STE_596text[ ] = " ";
const XCHAR CPU_4D1S_STE_597text[ ] = " ";
const XCHAR CPU_4D1S_STE_598text[ ] = " ";
const XCHAR CPU_4D1S_STE_599text[ ] = " ";
const XCHAR CPU_4D1S_STE_600text[ ] = " ";
const XCHAR CPU_4D1S_STE_601text[ ] = " ";
const XCHAR CPU_4D1S_STE_602text[ ] = " ";
const XCHAR CPU_4D1S_STE_603text[ ] = " ";
const XCHAR CPU_4D1S_STE_604text[ ] = "Local  Status:";
const XCHAR CPU_4D1S_STE_605text[ ] = " ";
const XCHAR CPU_4D1S_RDB_608text[ ] = "A";
const XCHAR CPU_4D1S_RDB_610text[ ] = "B";
const XCHAR CPU_4D1S_RDB_612text[ ] = "C";
const XCHAR CPU_4D1S_RDB_614text[ ] = "D";
const XCHAR CPU_4D1S_STE_615text[ ] = "Speed: Km/hr";
const XCHAR CPU_4D1S_STE_616text[ ] = " ";
const XCHAR CPU_4D1S_BTN_609text[ ] = "Next";



/***************************************************
* Scheme Declarations
***************************************************/
GOL_SCHEME* defscheme;
GOL_SCHEME* Times_new_roman;
GOL_SCHEME* Heading;
GOL_SCHEME* Push_Button;
GOL_SCHEME* List;
GOL_SCHEME* Static_test;
GOL_SCHEME* Table1;
GOL_SCHEME* In_table;
GOL_SCHEME* in_table2;
GOL_SCHEME* in_table3;
GOL_SCHEME* Image_disable;
GOL_SCHEME* enter_new_scheme;


/***************************************************
* Function and global Declarations
***************************************************/
void (*CreateFunctionArray[NUM_GDD_SCREENS])();
void (*CreatePrimitivesFunctionArray[NUM_GDD_SCREENS])();
WORD currentGDDDemoScreenIndex;
BYTE update = 0;
static BYTE updateGPL = 0;

/***************************************************
* Function       : GDDDemoCreateFirstScreen
* Parameters     : none
* Return         : none
* Description    : Creates the first screen
***************************************************/
void GDDDemoCreateFirstScreen(void)
{
    currentGDDDemoScreenIndex = 0;
    update = 1;
    (*CreateFunctionArray[currentGDDDemoScreenIndex])();
}

/***************************************************
* Function      : GDDDemoNextScreen
* Parameters    : none
* Return        : none
* Description   : Updates counter to show next screen
***************************************************/
void GDDDemoNextScreen(void)
{
    currentGDDDemoScreenIndex++;
    if(currentGDDDemoScreenIndex >= NUM_GDD_SCREENS)
    {
        currentGDDDemoScreenIndex = 0;
    }
    update = 1;
}

/***************************************************
* Function      : GDDDemoGoToScreen
* Parameters    : int screenIndex
* Return        : none
* Description   : Show the screen referred by the index
***************************************************/
void GDDDemoGoToScreen(int screenIndex)
{
    currentGDDDemoScreenIndex = screenIndex;
    if(currentGDDDemoScreenIndex >= NUM_GDD_SCREENS)
    {
        currentGDDDemoScreenIndex = 0;
    }
    update = 1;
}

/***************************************************
* Function       : GDDDemoGOLDrawCallback
* Parameters     : none
* Return         : none
* Description    : Callback to do the actual drawing of widgets
***************************************************/
void GDDDemoGOLDrawCallback(void)
{
    if(updateGPL)
    {
        (*CreatePrimitivesFunctionArray[currentGDDDemoScreenIndex])();
        updateGPL = 0;
    }

    if(update)
    {
        (*CreateFunctionArray[currentGDDDemoScreenIndex])();
        if(CreatePrimitivesFunctionArray[currentGDDDemoScreenIndex] != NULL)
        {
            updateGPL = 1;
        }
        update = 0;
    }
}

/***************************************************
* Function       : CreateError
* Parameters     : none
* Return         : none
* Description    : Creates a Error screen 
***************************************************/
void CreateError(char* string)
{
    // Blue Screen Error
    SetColor(119);
    ClearDevice();
    SetColor(-1);

    // Flash Error Message
    if(string == NULL)
        {OutTextXY(0, 0, "Runtime Error.");}
    else
        {OutTextXY(0,0, string);}
}

/***************************************************
* Function 	      :    CreateHome
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - Home
***************************************************/
void CreateHome(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;


    PICTURE *pPCB_253;
    pPCB_253 = PictCreate(  PCB_253, //name
                       179, //left
                       30, //top
                       279, //right
                       120, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&log_bmp_blue_t_dark2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_253==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_611;
    pSTE_611 = StCreate(  STE_611, //name
                       98, //left
                       240, //top
                       385, //right
                       265, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)Home_STE_611text, //text
                      Static_test //scheme
                    );

    if(pSTE_611==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateCPU_3D2S
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - CPU_3D2S
***************************************************/
void CreateCPU_3D2S(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;


    STATICTEXT *pSTE_89;
    pSTE_89 = StCreate(  STE_89, //name
                       75, //left
                       25, //top
                       222, //right
                       50, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_89text, //text
                      Static_test //scheme
                    );

    if(pSTE_89==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_90;
    pSTE_90 = StCreate(  STE_90, //name
                       223, //left
                       25, //top
                       325, //right
                       50, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_90text, //text
                      Static_test //scheme
                    );

    if(pSTE_90==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_91;
    pSTE_91 = StCreate(  STE_91, //name
                       75, //left
                       2, //top
                       163, //right
                       25, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_91text, //text
                      Static_test //scheme
                    );

    if(pSTE_91==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_92;
    pSTE_92 = StCreate(  STE_92, //name
                       160, //left
                       2, //top
                       223, //right
                       25, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_92text, //text
                      Static_test //scheme
                    );

    if(pSTE_92==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_93;
    pSTE_93 = StCreate(  STE_93, //name
                       325, //left
                       2, //top
                       406, //right
                       25, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_93text, //text
                      Static_test //scheme
                    );

    if(pSTE_93==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_94;
    pSTE_94 = StCreate(  STE_94, //name
                       325, //left
                       25, //top
                       394, //right
                       50, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_94text, //text
                      Static_test //scheme
                    );

    if(pSTE_94==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_95;
    pSTE_95 = StCreate(  STE_95, //name
                       225, //left
                       2, //top
                       288, //right
                       25, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_95text, //text
                      Static_test //scheme
                    );

    if(pSTE_95==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_214;
    pPCB_214 = PictCreate(  PCB_214, //name
                       415, //left
                       1, //top
                       467, //right
                       51, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_214==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_82;
    pPCB_82 = PictCreate(  PCB_82, //name
                       10, //left
                       7, //top
                       72, //right
                       50, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_82==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_276;
    pBTN_276 = BtnCreate(  BTN_276, //name
                       430, //left
                       246, //top
                       479, //right
                       271, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3D2S_BTN_276text, //text
                      Push_Button //scheme
                    );

    if(pBTN_276==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_277;
    pBTN_277 = BtnCreate(  BTN_277, //name
                       430, //left
                       221, //top
                       479, //right
                       246, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3D2S_BTN_277text, //text
                      Push_Button //scheme
                    );

    if(pBTN_277==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_278;
    pBTN_278 = BtnCreate(  BTN_278, //name
                       430, //left
                       196, //top
                       479, //right
                       221, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3D2S_BTN_278text, //text
                      Push_Button //scheme
                    );

    if(pBTN_278==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_279;
    pBTN_279 = BtnCreate(  BTN_279, //name
                       430, //left
                       171, //top
                       479, //right
                       196, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3D2S_BTN_279text, //text
                      Push_Button //scheme
                    );

    if(pBTN_279==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_281;
    pSTE_281 = StCreate(  STE_281, //name
                       10, //left
                       75, //top
                       65, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_281text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_281==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_283;
    pSTE_283 = StCreate(  STE_283, //name
                       10, //left
                       55, //top
                       65, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_283text, //text
                      Static_test //scheme
                    );

    if(pSTE_283==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_284;
    pSTE_284 = StCreate(  STE_284, //name
                       65, //left
                       55, //top
                       120, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_284text, //text
                      Static_test //scheme
                    );

    if(pSTE_284==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_285;
    pSTE_285 = StCreate(  STE_285, //name
                       357, //left
                       55, //top
                       412, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_285text, //text
                      Static_test //scheme
                    );

    if(pSTE_285==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_286;
    pSTE_286 = StCreate(  STE_286, //name
                       412, //left
                       55, //top
                       467, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_286text, //text
                      Static_test //scheme
                    );

    if(pSTE_286==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_287;
    pSTE_287 = StCreate(  STE_287, //name
                       10, //left
                       100, //top
                       65, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_287text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_287==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_289;
    pSTE_289 = StCreate(  STE_289, //name
                       65, //left
                       75, //top
                       120, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_289text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_289==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_290;
    pSTE_290 = StCreate(  STE_290, //name
                       65, //left
                       100, //top
                       120, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_290text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_290==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_291;
    pSTE_291 = StCreate(  STE_291, //name
                       130, //left
                       75, //top
                       185, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_291text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_291==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_292;
    pSTE_292 = StCreate(  STE_292, //name
                       130, //left
                       100, //top
                       185, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_292text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_292==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_293;
    pSTE_293 = StCreate(  STE_293, //name
                       185, //left
                       75, //top
                       240, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_293text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_293==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_294;
    pSTE_294 = StCreate(  STE_294, //name
                       185, //left
                       100, //top
                       240, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_294text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_294==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_312;
    pSTE_312 = StCreate(  STE_312, //name
                       357, //left
                       75, //top
                       412, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_312text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_312==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_313;
    pSTE_313 = StCreate(  STE_313, //name
                       357, //left
                       100, //top
                       412, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_313text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_313==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_314;
    pSTE_314 = StCreate(  STE_314, //name
                       412, //left
                       75, //top
                       467, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_314text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_314==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_315;
    pSTE_315 = StCreate(  STE_315, //name
                       412, //left
                       100, //top
                       467, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_315text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_315==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_316;
    pSTE_316 = StCreate(  STE_316, //name
                       130, //left
                       55, //top
                       185, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_316text, //text
                      Static_test //scheme
                    );

    if(pSTE_316==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_317;
    pSTE_317 = StCreate(  STE_317, //name
                       185, //left
                       55, //top
                       240, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_317text, //text
                      Static_test //scheme
                    );

    if(pSTE_317==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_325;
    pSTE_325 = StCreate(  STE_325, //name
                       10, //left
                       125, //top
                       89, //right
                       150, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_325text, //text
                      Static_test //scheme
                    );

    if(pSTE_325==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_326;
    pSTE_326 = StCreate(  STE_326, //name
                       10, //left
                       180, //top
                       90, //right
                       205, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_326text, //text
                      Static_test //scheme
                    );

    if(pSTE_326==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_327;
    pSTE_327 = StCreate(  STE_327, //name
                       10, //left
                       205, //top
                       90, //right
                       230, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_327text, //text
                      Static_test //scheme
                    );

    if(pSTE_327==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_329;
    pSTE_329 = StCreate(  STE_329, //name
                       89, //left
                       125, //top
                       377, //right
                       150, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_329text, //text
                      Static_test //scheme
                    );

    if(pSTE_329==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_330;
    pSTE_330 = StCreate(  STE_330, //name
                       90, //left
                       180, //top
                       377, //right
                       205, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_330text, //text
                      Static_test //scheme
                    );

    if(pSTE_330==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_331;
    pSTE_331 = StCreate(  STE_331, //name
                       90, //left
                       205, //top
                       377, //right
                       230, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_331text, //text
                      Static_test //scheme
                    );

    if(pSTE_331==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_334;
    pSTE_334 = StCreate(  STE_334, //name
                       385, //left
                       125, //top
                       479, //right
                       148, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_334text, //text
                      Static_test //scheme
                    );

    if(pSTE_334==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_335;
    pSTE_335 = StCreate(  STE_335, //name
                       412, //left
                       148, //top
                       465, //right
                       170, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_335text, //text
                      Static_test //scheme
                    );

    if(pSTE_335==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_336;
    pPCB_336 = PictCreate(  PCB_336, //name
                       134, //left
                       232, //top
                       196, //right
                       271, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_336==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_337;
    pPCB_337 = PictCreate(  PCB_337, //name
                       257, //left
                       230, //top
                       319, //right
                       271, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_337==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_366;
    pRDB_366 = RbCreate(  RDB_366, //name
                       94, //left
                       254, //top
                       130, //right
                       271, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_3D2S_RDB_366text, //text
                      Static_test //scheme
                    );

    if(pRDB_366==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_367;
    pRDB_367 = RbCreate(  RDB_367, //name
                       209, //left
                       254, //top
                       245, //right
                       271, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_3D2S_RDB_367text, //text
                      Static_test //scheme
                    );

    if(pRDB_367==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_368;
    pRDB_368 = RbCreate(  RDB_368, //name
                       319, //left
                       254, //top
                       355, //right
                       271, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_3D2S_RDB_368text, //text
                      Static_test //scheme
                    );

    if(pRDB_368==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_503;
    pSTE_503 = StCreate(  STE_503, //name
                       245, //left
                       55, //top
                       300, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_503text, //text
                      Static_test //scheme
                    );

    if(pSTE_503==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_504;
    pSTE_504 = StCreate(  STE_504, //name
                       298, //left
                       55, //top
                       353, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_504text, //text
                      Static_test //scheme
                    );

    if(pSTE_504==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_505;
    pSTE_505 = StCreate(  STE_505, //name
                       245, //left
                       75, //top
                       300, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_505text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_505==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_506;
    pSTE_506 = StCreate(  STE_506, //name
                       298, //left
                       75, //top
                       353, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_506text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_506==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_507;
    pSTE_507 = StCreate(  STE_507, //name
                       245, //left
                       100, //top
                       300, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_507text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_507==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_508;
    pSTE_508 = StCreate(  STE_508, //name
                       298, //left
                       100, //top
                       353, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_508text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_508==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_509;
    pSTE_509 = StCreate(  STE_509, //name
                       10, //left
                       150, //top
                       89, //right
                       175, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_509text, //text
                      Static_test //scheme
                    );

    if(pSTE_509==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_510;
    pSTE_510 = StCreate(  STE_510, //name
                       90, //left
                       150, //top
                       378, //right
                       175, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3D2S_STE_510text, //text
                      Static_test //scheme
                    );

    if(pSTE_510==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_608;
    pBTN_608 = BtnCreate(  BTN_608, //name
                       10, //left
                       232, //top
                       59, //right
                       257, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3D2S_BTN_608text, //text
                      Push_Button //scheme
                    );

    if(pBTN_608==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateCPU_2DP
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - CPU_2DP
***************************************************/
void CreateCPU_2DP(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;


    BUTTON *pBTN_149;
    pBTN_149 = BtnCreate(  BTN_149, //name
                       417, //left
                       125, //top
                       473, //right
                       155, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_2DP_BTN_149text, //text
                      Push_Button //scheme
                    );

    if(pBTN_149==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_151;
    pBTN_151 = BtnCreate(  BTN_151, //name
                       417, //left
                       160, //top
                       473, //right
                       190, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_2DP_BTN_151text, //text
                      Push_Button //scheme
                    );

    if(pBTN_151==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_152;
    pBTN_152 = BtnCreate(  BTN_152, //name
                       417, //left
                       195, //top
                       473, //right
                       225, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_2DP_BTN_152text, //text
                      Push_Button //scheme
                    );

    if(pBTN_152==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_153;
    pBTN_153 = BtnCreate(  BTN_153, //name
                       418, //left
                       230, //top
                       474, //right
                       260, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_2DP_BTN_153text, //text
                      Push_Button //scheme
                    );

    if(pBTN_153==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_157;
    pSTE_157 = StCreate(  STE_157, //name
                       76, //left
                       38, //top
                       227, //right
                       61, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_157text, //text
                      Static_test //scheme
                    );

    if(pSTE_157==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_158;
    pSTE_158 = StCreate(  STE_158, //name
                       229, //left
                       38, //top
                       329, //right
                       61, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_158text, //text
                      Static_test //scheme
                    );

    if(pSTE_158==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_159;
    pSTE_159 = StCreate(  STE_159, //name
                       76, //left
                       12, //top
                       164, //right
                       35, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_159text, //text
                      Static_test //scheme
                    );

    if(pSTE_159==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_163;
    pSTE_163 = StCreate(  STE_163, //name
                       166, //left
                       12, //top
                       233, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_163text, //text
                      Static_test //scheme
                    );

    if(pSTE_163==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_164;
    pSTE_164 = StCreate(  STE_164, //name
                       329, //left
                       12, //top
                       410, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_164text, //text
                      Static_test //scheme
                    );

    if(pSTE_164==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_165;
    pSTE_165 = StCreate(  STE_165, //name
                       329, //left
                       39, //top
                       398, //right
                       62, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_165text, //text
                      Static_test //scheme
                    );

    if(pSTE_165==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_166;
    pSTE_166 = StCreate(  STE_166, //name
                       234, //left
                       12, //top
                       297, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_166text, //text
                      Static_test //scheme
                    );

    if(pSTE_166==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_175;
    pSTE_175 = StCreate(  STE_175, //name
                       115, //left
                       160, //top
                       388, //right
                       185, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_175text, //text
                      Static_test //scheme
                    );

    if(pSTE_175==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_176;
    pSTE_176 = StCreate(  STE_176, //name
                       10, //left
                       160, //top
                       110, //right
                       185, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_176text, //text
                      Static_test //scheme
                    );

    if(pSTE_176==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_177;
    pSTE_177 = StCreate(  STE_177, //name
                       10, //left
                       190, //top
                       110, //right
                       215, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_177text, //text
                      Static_test //scheme
                    );

    if(pSTE_177==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_179;
    pSTE_179 = StCreate(  STE_179, //name
                       10, //left
                       215, //top
                       110, //right
                       240, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_179text, //text
                      Static_test //scheme
                    );

    if(pSTE_179==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_204;
    pPCB_204 = PictCreate(  PCB_204, //name
                       10, //left
                       10, //top
                       72, //right
                       54, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_204==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_205;
    pPCB_205 = PictCreate(  PCB_205, //name
                       233, //left
                       225, //top
                       295, //right
                       267, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_205==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_215;
    pPCB_215 = PictCreate(  PCB_215, //name
                       422, //left
                       9, //top
                       474, //right
                       61, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_215==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_196;
    pSTE_196 = StCreate(  STE_196, //name
                       115, //left
                       215, //top
                       156, //right
                       239, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_196text, //text
                      Static_test //scheme
                    );

    if(pSTE_196==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_302;
    pSTE_302 = StCreate(  STE_302, //name
                       122, //left
                       95, //top
                       177, //right
                       120, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_302text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_302==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_303;
    pSTE_303 = StCreate(  STE_303, //name
                       123, //left
                       125, //top
                       178, //right
                       150, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_303text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_303==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_304;
    pSTE_304 = StCreate(  STE_304, //name
                       179, //left
                       95, //top
                       234, //right
                       120, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_304text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_304==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_305;
    pSTE_305 = StCreate(  STE_305, //name
                       179, //left
                       125, //top
                       234, //right
                       150, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_305text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_305==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_306;
    pSTE_306 = StCreate(  STE_306, //name
                       247, //left
                       95, //top
                       300, //right
                       120, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_306text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_306==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_307;
    pSTE_307 = StCreate(  STE_307, //name
                       247, //left
                       125, //top
                       300, //right
                       150, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_307text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_307==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_308;
    pSTE_308 = StCreate(  STE_308, //name
                       300, //left
                       95, //top
                       355, //right
                       120, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_308text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_308==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_309;
    pSTE_309 = StCreate(  STE_309, //name
                       300, //left
                       125, //top
                       355, //right
                       150, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_309text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_309==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_266;
    pSTE_266 = StCreate(  STE_266, //name
                       115, //left
                       190, //top
                       388, //right
                       215, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_266text, //text
                      Static_test //scheme
                    );

    if(pSTE_266==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_267;
    pSTE_267 = StCreate(  STE_267, //name
                       10, //left
                       95, //top
                       110, //right
                       120, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_267text, //text
                      Static_test //scheme
                    );

    if(pSTE_267==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_269;
    pSTE_269 = StCreate(  STE_269, //name
                       10, //left
                       125, //top
                       110, //right
                       150, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_269text, //text
                      Static_test //scheme
                    );

    if(pSTE_269==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_270;
    pSTE_270 = StCreate(  STE_270, //name
                       123, //left
                       70, //top
                       175, //right
                       95, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_270text, //text
                      Static_test //scheme
                    );

    if(pSTE_270==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_272;
    pSTE_272 = StCreate(  STE_272, //name
                       181, //left
                       70, //top
                       233, //right
                       95, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_272text, //text
                      Static_test //scheme
                    );

    if(pSTE_272==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_273;
    pSTE_273 = StCreate(  STE_273, //name
                       248, //left
                       70, //top
                       300, //right
                       95, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_273text, //text
                      Static_test //scheme
                    );

    if(pSTE_273==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_274;
    pSTE_274 = StCreate(  STE_274, //name
                       300, //left
                       70, //top
                       352, //right
                       95, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_274text, //text
                      Static_test //scheme
                    );

    if(pSTE_274==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_439;
    pRDB_439 = RbCreate(  RDB_439, //name
                       330, //left
                       249, //top
                       367, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_2DP_RDB_439text, //text
                      Static_test //scheme
                    );

    if(pRDB_439==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_484;
    pRDB_484 = RbCreate(  RDB_484, //name
                       138, //left
                       249, //top
                       175, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_2DP_RDB_484text, //text
                      Static_test //scheme
                    );

    if(pRDB_484==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateCPU_3DP_AS
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - CPU_3DP_AS
***************************************************/
void CreateCPU_3DP_AS(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;

    if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;


    STATICTEXT *pSTE_203;
    pSTE_203 = StCreate(  STE_203, //name
                       76, //left
                       39, //top
                       235, //right
                       63, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_203text, //text
                      Static_test //scheme
                    );

    if(pSTE_203==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_204;
    pSTE_204 = StCreate(  STE_204, //name
                       240, //left
                       40, //top
                       338, //right
                       63, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_204text, //text
                      Static_test //scheme
                    );

    if(pSTE_204==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_205;
    pSTE_205 = StCreate(  STE_205, //name
                       75, //left
                       10, //top
                       163, //right
                       33, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_205text, //text
                      Static_test //scheme
                    );

    if(pSTE_205==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_206;
    pSTE_206 = StCreate(  STE_206, //name
                       170, //left
                       10, //top
                       234, //right
                       33, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_206text, //text
                      Static_test //scheme
                    );

    if(pSTE_206==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_207;
    pSTE_207 = StCreate(  STE_207, //name
                       329, //left
                       11, //top
                       410, //right
                       34, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_207text, //text
                      Static_test //scheme
                    );

    if(pSTE_207==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_208;
    pSTE_208 = StCreate(  STE_208, //name
                       341, //left
                       40, //top
                       410, //right
                       63, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_208text, //text
                      Static_test //scheme
                    );

    if(pSTE_208==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_209;
    pSTE_209 = StCreate(  STE_209, //name
                       240, //left
                       10, //top
                       303, //right
                       33, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_209text, //text
                      Static_test //scheme
                    );

    if(pSTE_209==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_206;
    pPCB_206 = PictCreate(  PCB_206, //name
                       7, //left
                       11, //top
                       69, //right
                       55, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_206==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_216;
    pPCB_216 = PictCreate(  PCB_216, //name
                       420, //left
                       5, //top
                       472, //right
                       57, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_216==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_338;
    pSTE_338 = StCreate(  STE_338, //name
                       105, //left
                       90, //top
                       160, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_338text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_338==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_339;
    pSTE_339 = StCreate(  STE_339, //name
                       105, //left
                       114, //top
                       160, //right
                       138, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_339text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_339==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_340;
    pSTE_340 = StCreate(  STE_340, //name
                       160, //left
                       90, //top
                       215, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_340text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_340==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_341;
    pSTE_341 = StCreate(  STE_341, //name
                       160, //left
                       113, //top
                       215, //right
                       138, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_341text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_341==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_350;
    pSTE_350 = StCreate(  STE_350, //name
                       108, //left
                       65, //top
                       160, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_350text, //text
                      Static_test //scheme
                    );

    if(pSTE_350==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_351;
    pSTE_351 = StCreate(  STE_351, //name
                       160, //left
                       65, //top
                       212, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_351text, //text
                      Static_test //scheme
                    );

    if(pSTE_351==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_352;
    pSTE_352 = StCreate(  STE_352, //name
                       212, //left
                       65, //top
                       264, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_352text, //text
                      Static_test //scheme
                    );

    if(pSTE_352==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_356;
    pSTE_356 = StCreate(  STE_356, //name
                       212, //left
                       90, //top
                       267, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_356text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_356==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_357;
    pSTE_357 = StCreate(  STE_357, //name
                       212, //left
                       112, //top
                       267, //right
                       137, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_357text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_357==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_372;
    pSTE_372 = StCreate(  STE_372, //name
                       5, //left
                       188, //top
                       96, //right
                       213, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_372text, //text
                      Static_test //scheme
                    );

    if(pSTE_372==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_373;
    pSTE_373 = StCreate(  STE_373, //name
                       4, //left
                       213, //top
                       96, //right
                       238, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_373text, //text
                      Static_test //scheme
                    );

    if(pSTE_373==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_374;
    pSTE_374 = StCreate(  STE_374, //name
                       4, //left
                       238, //top
                       97, //right
                       263, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_374text, //text
                      Static_test //scheme
                    );

    if(pSTE_374==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_376;
    pSTE_376 = StCreate(  STE_376, //name
                       97, //left
                       238, //top
                       377, //right
                       263, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_376text, //text
                      Static_test //scheme
                    );

    if(pSTE_376==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_377;
    pSTE_377 = StCreate(  STE_377, //name
                       326, //left
                       162, //top
                       420, //right
                       187, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_377text, //text
                      Static_test //scheme
                    );

    if(pSTE_377==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_378;
    pSTE_378 = StCreate(  STE_378, //name
                       420, //left
                       163, //top
                       473, //right
                       187, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_378text, //text
                      Static_test //scheme
                    );

    if(pSTE_378==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_379;
    pBTN_379 = BtnCreate(  BTN_379, //name
                       430, //left
                       245, //top
                       479, //right
                       270, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3DP_AS_BTN_379text, //text
                      Push_Button //scheme
                    );

    if(pBTN_379==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_380;
    pBTN_380 = BtnCreate(  BTN_380, //name
                       381, //left
                       245, //top
                       430, //right
                       270, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3DP_AS_BTN_380text, //text
                      Push_Button //scheme
                    );

    if(pBTN_380==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_381;
    pBTN_381 = BtnCreate(  BTN_381, //name
                       430, //left
                       219, //top
                       479, //right
                       245, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3DP_AS_BTN_381text, //text
                      Push_Button //scheme
                    );

    if(pBTN_381==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_382;
    pBTN_382 = BtnCreate(  BTN_382, //name
                       430, //left
                       194, //top
                       479, //right
                       219, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3DP_AS_BTN_382text, //text
                      Push_Button //scheme
                    );

    if(pBTN_382==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_467;
    pSTE_467 = StCreate(  STE_467, //name
                       105, //left
                       136, //top
                       160, //right
                       161, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_467text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_467==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_468;
    pSTE_468 = StCreate(  STE_468, //name
                       105, //left
                       159, //top
                       160, //right
                       184, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_468text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_468==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_469;
    pSTE_469 = StCreate(  STE_469, //name
                       160, //left
                       136, //top
                       215, //right
                       161, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_469text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_469==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_470;
    pSTE_470 = StCreate(  STE_470, //name
                       160, //left
                       159, //top
                       215, //right
                       184, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_470text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_470==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_471;
    pSTE_471 = StCreate(  STE_471, //name
                       97, //left
                       214, //top
                       377, //right
                       238, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_471text, //text
                      Static_test //scheme
                    );

    if(pSTE_471==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_472;
    pSTE_472 = StCreate(  STE_472, //name
                       97, //left
                       188, //top
                       377, //right
                       212, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_472text, //text
                      Static_test //scheme
                    );

    if(pSTE_472==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_473;
    pSTE_473 = StCreate(  STE_473, //name
                       212, //left
                       136, //top
                       267, //right
                       161, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_473text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_473==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_474;
    pSTE_474 = StCreate(  STE_474, //name
                       212, //left
                       159, //top
                       267, //right
                       184, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_474text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_474==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_369;
    pSTE_369 = StCreate(  STE_369, //name
                       12, //left
                       90, //top
                       105, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_369text, //text
                      Static_test //scheme
                    );

    if(pSTE_369==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_370;
    pSTE_370 = StCreate(  STE_370, //name
                       12, //left
                       113, //top
                       105, //right
                       138, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_370text, //text
                      Static_test //scheme
                    );

    if(pSTE_370==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_371;
    pSTE_371 = StCreate(  STE_371, //name
                       12, //left
                       136, //top
                       106, //right
                       161, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_371text, //text
                      Static_test //scheme
                    );

    if(pSTE_371==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_375;
    pSTE_375 = StCreate(  STE_375, //name
                       12, //left
                       159, //top
                       106, //right
                       184, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_3DP_AS_STE_375text, //text
                      Static_test //scheme
                    );

    if(pSTE_375==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_377;
    pPCB_377 = PictCreate(  PCB_377, //name
                       298, //left
                       90, //top
                       360, //right
                       129, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_377==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_378;
    pPCB_378 = PictCreate(  PCB_378, //name
                       382, //left
                       65, //top
                       443, //right
                       104, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_378==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_379;
    pPCB_379 = PictCreate(  PCB_379, //name
                       381, //left
                       114, //top
                       443, //right
                       153, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_379==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_380;
    pRDB_380 = RbCreate(  RDB_380, //name
                       267, //left
                       120, //top
                       303, //right
                       137, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_3DP_AS_RDB_380text, //text
                      Static_test //scheme
                    );

    if(pRDB_380==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_381;
    pRDB_381 = RbCreate(  RDB_381, //name
                       443, //left
                       87, //top
                       479, //right
                       104, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_3DP_AS_RDB_381text, //text
                      Static_test //scheme
                    );

    if(pRDB_381==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_382;
    pRDB_382 = RbCreate(  RDB_382, //name
                       443, //left
                       136, //top
                       479, //right
                       153, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_3DP_AS_RDB_382text, //text
                      Static_test //scheme
                    );

    if(pRDB_382==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_610;
    pBTN_610 = BtnCreate(  BTN_610, //name
                       378, //left
                       207, //top
                       429, //right
                       234, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_3DP_AS_BTN_610text, //text
                      Push_Button //scheme
                    );

    if(pBTN_610==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateCPU_LCWS
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - CPU_LCWS
***************************************************/
void CreateCPU_LCWS(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;

    if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;


    STATICTEXT *pSTE_222;
    pSTE_222 = StCreate(  STE_222, //name
                       75, //left
                       25, //top
                       222, //right
                       50, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_222text, //text
                      Static_test //scheme
                    );

    if(pSTE_222==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_223;
    pSTE_223 = StCreate(  STE_223, //name
                       223, //left
                       25, //top
                       329, //right
                       50, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_223text, //text
                      Static_test //scheme
                    );

    if(pSTE_223==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_224;
    pSTE_224 = StCreate(  STE_224, //name
                       75, //left
                       2, //top
                       163, //right
                       25, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_224text, //text
                      Static_test //scheme
                    );

    if(pSTE_224==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_225;
    pSTE_225 = StCreate(  STE_225, //name
                       157, //left
                       2, //top
                       215, //right
                       25, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_225text, //text
                      Static_test //scheme
                    );

    if(pSTE_225==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_226;
    pSTE_226 = StCreate(  STE_226, //name
                       325, //left
                       2, //top
                       406, //right
                       25, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_226text, //text
                      Static_test //scheme
                    );

    if(pSTE_226==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_227;
    pSTE_227 = StCreate(  STE_227, //name
                       329, //left
                       25, //top
                       398, //right
                       50, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_227text, //text
                      Static_test //scheme
                    );

    if(pSTE_227==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_228;
    pSTE_228 = StCreate(  STE_228, //name
                       215, //left
                       2, //top
                       278, //right
                       25, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_228text, //text
                      Static_test //scheme
                    );

    if(pSTE_228==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_209;
    pPCB_209 = PictCreate(  PCB_209, //name
                       5, //left
                       0, //top
                       67, //right
                       44, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_209==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_217;
    pPCB_217 = PictCreate(  PCB_217, //name
                       415, //left
                       0, //top
                       467, //right
                       51, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_217==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_384;
    pSTE_384 = StCreate(  STE_384, //name
                       13, //left
                       55, //top
                       65, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_384text, //text
                      Static_test //scheme
                    );

    if(pSTE_384==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_385;
    pSTE_385 = StCreate(  STE_385, //name
                       65, //left
                       55, //top
                       117, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_385text, //text
                      Static_test //scheme
                    );

    if(pSTE_385==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_386;
    pSTE_386 = StCreate(  STE_386, //name
                       130, //left
                       55, //top
                       182, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_386text, //text
                      Static_test //scheme
                    );

    if(pSTE_386==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_387;
    pSTE_387 = StCreate(  STE_387, //name
                       185, //left
                       55, //top
                       237, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_387text, //text
                      Static_test //scheme
                    );

    if(pSTE_387==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_388;
    pSTE_388 = StCreate(  STE_388, //name
                       240, //left
                       55, //top
                       292, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_388text, //text
                      Static_test //scheme
                    );

    if(pSTE_388==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_390;
    pSTE_390 = StCreate(  STE_390, //name
                       292, //left
                       55, //top
                       344, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_390text, //text
                      Static_test //scheme
                    );

    if(pSTE_390==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_391;
    pSTE_391 = StCreate(  STE_391, //name
                       355, //left
                       55, //top
                       407, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_391text, //text
                      Static_test //scheme
                    );

    if(pSTE_391==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_392;
    pSTE_392 = StCreate(  STE_392, //name
                       410, //left
                       55, //top
                       462, //right
                       75, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_392text, //text
                      Static_test //scheme
                    );

    if(pSTE_392==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_393;
    pSTE_393 = StCreate(  STE_393, //name
                       10, //left
                       75, //top
                       65, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_393text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_393==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_394;
    pSTE_394 = StCreate(  STE_394, //name
                       10, //left
                       100, //top
                       65, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_394text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_394==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_395;
    pSTE_395 = StCreate(  STE_395, //name
                       65, //left
                       75, //top
                       120, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_395text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_395==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_396;
    pSTE_396 = StCreate(  STE_396, //name
                       65, //left
                       100, //top
                       120, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_396text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_396==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_397;
    pSTE_397 = StCreate(  STE_397, //name
                       130, //left
                       75, //top
                       185, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_397text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_397==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_398;
    pSTE_398 = StCreate(  STE_398, //name
                       131, //left
                       100, //top
                       186, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_398text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_398==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_399;
    pSTE_399 = StCreate(  STE_399, //name
                       185, //left
                       75, //top
                       240, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_399text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_399==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_400;
    pSTE_400 = StCreate(  STE_400, //name
                       185, //left
                       100, //top
                       240, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_400text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_400==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_401;
    pSTE_401 = StCreate(  STE_401, //name
                       240, //left
                       75, //top
                       295, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_401text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_401==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_402;
    pSTE_402 = StCreate(  STE_402, //name
                       240, //left
                       100, //top
                       295, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_402text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_402==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_403;
    pSTE_403 = StCreate(  STE_403, //name
                       295, //left
                       75, //top
                       350, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_403text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_403==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_404;
    pSTE_404 = StCreate(  STE_404, //name
                       295, //left
                       100, //top
                       350, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_404text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_404==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_405;
    pSTE_405 = StCreate(  STE_405, //name
                       355, //left
                       75, //top
                       410, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_405text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_405==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_406;
    pSTE_406 = StCreate(  STE_406, //name
                       355, //left
                       100, //top
                       410, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_406text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_406==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_407;
    pSTE_407 = StCreate(  STE_407, //name
                       410, //left
                       75, //top
                       465, //right
                       100, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_407text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_407==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_408;
    pSTE_408 = StCreate(  STE_408, //name
                       410, //left
                       100, //top
                       464, //right
                       125, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_408text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_408==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_410;
    pSTE_410 = StCreate(  STE_410, //name
                       1, //left
                       150, //top
                       91, //right
                       175, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_410text, //text
                      Static_test //scheme
                    );

    if(pSTE_410==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_416;
    pSTE_416 = StCreate(  STE_416, //name
                       92, //left
                       150, //top
                       398, //right
                       175, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_LCWS_STE_416text, //text
                      Static_test //scheme
                    );

    if(pSTE_416==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_419;
    pBTN_419 = BtnCreate(  BTN_419, //name
                       430, //left
                       246, //top
                       479, //right
                       271, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_LCWS_BTN_419text, //text
                      Push_Button //scheme
                    );

    if(pBTN_419==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_420;
    pBTN_420 = BtnCreate(  BTN_420, //name
                       430, //left
                       203, //top
                       479, //right
                       228, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_LCWS_BTN_420text, //text
                      Push_Button //scheme
                    );

    if(pBTN_420==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_421;
    pBTN_421 = BtnCreate(  BTN_421, //name
                       430, //left
                       171, //top
                       479, //right
                       196, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_LCWS_BTN_421text, //text
                      Push_Button //scheme
                    );

    if(pBTN_421==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_422;
    pBTN_422 = BtnCreate(  BTN_422, //name
                       430, //left
                       140, //top
                       479, //right
                       165, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_LCWS_BTN_422text, //text
                      Push_Button //scheme
                    );

    if(pBTN_422==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_424;
    pPCB_424 = PictCreate(  PCB_424, //name
                       95, //left
                       228, //top
                       157, //right
                       271, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_424==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_434;
    pPCB_434 = PictCreate(  PCB_434, //name
                       303, //left
                       228, //top
                       365, //right
                       271, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_434==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_389;
    pRDB_389 = RbCreate(  RDB_389, //name
                       56, //left
                       250, //top
                       92, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_LCWS_RDB_389text, //text
                      Static_test //scheme
                    );

    if(pRDB_389==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_390;
    pRDB_390 = RbCreate(  RDB_390, //name
                       163, //left
                       250, //top
                       199, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_LCWS_RDB_390text, //text
                      Static_test //scheme
                    );

    if(pRDB_390==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_391;
    pRDB_391 = RbCreate(  RDB_391, //name
                       264, //left
                       250, //top
                       300, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_LCWS_RDB_391text, //text
                      Static_test //scheme
                    );

    if(pRDB_391==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_392;
    pRDB_392 = RbCreate(  RDB_392, //name
                       371, //left
                       250, //top
                       407, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_LCWS_RDB_392text, //text
                      Static_test //scheme
                    );

    if(pRDB_392==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateSMCPU
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - SMCPU
***************************************************/
void CreateSMCPU(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(Table1 != NULL) free(Table1);
        Table1 = GOLCreateScheme();

    Table1->Color0 = RGBConvert(248, 252, 248);
    Table1->Color1 = RGBConvert(16, 132, 168);
    Table1->TextColor0 = RGBConvert(248, 252, 248);
    Table1->TextColor1 = RGBConvert(248, 252, 248);
    Table1->EmbossDkColor = RGBConvert(248, 252, 248);
    Table1->EmbossLtColor = RGBConvert(24, 116, 184);
    Table1->TextColorDisabled = RGBConvert(128, 128, 128);
    Table1->ColorDisabled = RGBConvert(208, 224, 240);
    Table1->CommonBkColor = RGBConvert(0, 48, 200);
    Table1->pFont = (void*)&Gentium_16;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;


    BUTTON *pBTN_161;
    pBTN_161 = BtnCreate(  BTN_161, //name
                       418, //left
                       199, //top
                       472, //right
                       228, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)SMCPU_BTN_161text, //text
                      Push_Button //scheme
                    );

    if(pBTN_161==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_165;
    pBTN_165 = BtnCreate(  BTN_165, //name
                       418, //left
                       229, //top
                       472, //right
                       258, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)SMCPU_BTN_165text, //text
                      Push_Button //scheme
                    );

    if(pBTN_165==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_167;
    pSTE_167 = StCreate(  STE_167, //name
                       76, //left
                       41, //top
                       240, //right
                       64, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_167text, //text
                      Static_test //scheme
                    );

    if(pSTE_167==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_168;
    pSTE_168 = StCreate(  STE_168, //name
                       241, //left
                       41, //top
                       341, //right
                       64, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_168text, //text
                      Static_test //scheme
                    );

    if(pSTE_168==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_170;
    pSTE_170 = StCreate(  STE_170, //name
                       175, //left
                       10, //top
                       235, //right
                       33, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_170text, //text
                      Static_test //scheme
                    );

    if(pSTE_170==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_171;
    pSTE_171 = StCreate(  STE_171, //name
                       329, //left
                       12, //top
                       410, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_171text, //text
                      Static_test //scheme
                    );

    if(pSTE_171==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_172;
    pSTE_172 = StCreate(  STE_172, //name
                       341, //left
                       41, //top
                       410, //right
                       64, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_172text, //text
                      Static_test //scheme
                    );

    if(pSTE_172==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_173;
    pSTE_173 = StCreate(  STE_173, //name
                       245, //left
                       10, //top
                       308, //right
                       33, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_173text, //text
                      Static_test //scheme
                    );

    if(pSTE_173==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    GROUPBOX *pGRB_193;
    pGRB_193 = GbCreate(  GRB_193, //name
                       140, //left
                       77, //top
                       331, //right
                       167, //bottom
                       GB_DRAW | GB_CENTER_ALIGN, //state
                       (XCHAR*)SMCPU_GRB_193text, //text
                      Table1 //scheme
                    );

    if(pGRB_193==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_243;
    pSTE_243 = StCreate(  STE_243, //name
                       144, //left
                       97, //top
                       246, //right
                       120, //bottom
                       ST_DRAW | ST_RIGHT_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_243text, //text
                      Static_test //scheme
                    );

    if(pSTE_243==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_244;
    pSTE_244 = StCreate(  STE_244, //name
                       144, //left
                       119, //top
                       246, //right
                       142, //bottom
                       ST_DRAW | ST_RIGHT_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_244text, //text
                      Static_test //scheme
                    );

    if(pSTE_244==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_245;
    pSTE_245 = StCreate(  STE_245, //name
                       144, //left
                       140, //top
                       246, //right
                       163, //bottom
                       ST_DRAW | ST_RIGHT_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_245text, //text
                      Static_test //scheme
                    );

    if(pSTE_245==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_246;
    pSTE_246 = StCreate(  STE_246, //name
                       245, //left
                       98, //top
                       325, //right
                       121, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_246text, //text
                      Static_test //scheme
                    );

    if(pSTE_246==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_247;
    pSTE_247 = StCreate(  STE_247, //name
                       245, //left
                       117, //top
                       325, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_247text, //text
                      Static_test //scheme
                    );

    if(pSTE_247==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_248;
    pSTE_248 = StCreate(  STE_248, //name
                       245, //left
                       139, //top
                       324, //right
                       164, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_248text, //text
                      Static_test //scheme
                    );

    if(pSTE_248==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_213;
    pPCB_213 = PictCreate(  PCB_213, //name
                       10, //left
                       16, //top
                       73, //right
                       60, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_213==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_218;
    pPCB_218 = PictCreate(  PCB_218, //name
                       420, //left
                       12, //top
                       472, //right
                       64, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_218==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_169;
    pSTE_169 = StCreate(  STE_169, //name
                       75, //left
                       10, //top
                       163, //right
                       33, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_169text, //text
                      Static_test //scheme
                    );

    if(pSTE_169==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_609;
    pSTE_609 = StCreate(  STE_609, //name
                       113, //left
                       176, //top
                       284, //right
                       199, //bottom
                       ST_DRAW | ST_RIGHT_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_609text, //text
                      Static_test //scheme
                    );

    if(pSTE_609==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_610;
    pSTE_610 = StCreate(  STE_610, //name
                       113, //left
                       206, //top
                       284, //right
                       229, //bottom
                       ST_DRAW | ST_RIGHT_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_610text, //text
                      Static_test //scheme
                    );

    if(pSTE_610==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_612;
    pSTE_612 = StCreate(  STE_612, //name
                       284, //left
                       176, //top
                       387, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_612text, //text
                      Static_test //scheme
                    );

    if(pSTE_612==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_613;
    pSTE_613 = StCreate(  STE_613, //name
                       284, //left
                       205, //top
                       387, //right
                       228, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)SMCPU_STE_613text, //text
                      Static_test //scheme
                    );

    if(pSTE_613==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateRESET
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - RESET
***************************************************/
void CreateRESET(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;


    BUTTON *pBTN_201;
    pBTN_201 = BtnCreate(  BTN_201, //name
                       420, //left
                       240, //top
                       474, //right
                       269, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)RESET_BTN_201text, //text
                      Push_Button //scheme
                    );

    if(pBTN_201==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_211;
    pSTE_211 = StCreate(  STE_211, //name
                       76, //left
                       40, //top
                       227, //right
                       63, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_211text, //text
                      Static_test //scheme
                    );

    if(pSTE_211==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_249;
    pSTE_249 = StCreate(  STE_249, //name
                       229, //left
                       41, //top
                       329, //right
                       64, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)RESET_STE_249text, //text
                      Static_test //scheme
                    );

    if(pSTE_249==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_250;
    pSTE_250 = StCreate(  STE_250, //name
                       76, //left
                       12, //top
                       164, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_250text, //text
                      Static_test //scheme
                    );

    if(pSTE_250==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_252;
    pSTE_252 = StCreate(  STE_252, //name
                       175, //left
                       12, //top
                       235, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_252text, //text
                      Static_test //scheme
                    );

    if(pSTE_252==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_253;
    pSTE_253 = StCreate(  STE_253, //name
                       329, //left
                       12, //top
                       410, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_253text, //text
                      Static_test //scheme
                    );

    if(pSTE_253==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_254;
    pSTE_254 = StCreate(  STE_254, //name
                       341, //left
                       41, //top
                       410, //right
                       64, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_254text, //text
                      Static_test //scheme
                    );

    if(pSTE_254==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_256;
    pSTE_256 = StCreate(  STE_256, //name
                       248, //left
                       12, //top
                       311, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_256text, //text
                      Static_test //scheme
                    );

    if(pSTE_256==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_268;
    pPCB_268 = PictCreate(  PCB_268, //name
                       10, //left
                       16, //top
                       73, //right
                       60, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2_inv, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_268==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_269;
    pPCB_269 = PictCreate(  PCB_269, //name
                       420, //left
                       12, //top
                       472, //right
                       64, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_269==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_419;
    pSTE_419 = StCreate(  STE_419, //name
                       10, //left
                       64, //top
                       67, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_419text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_419==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_420;
    pSTE_420 = StCreate(  STE_420, //name
                       10, //left
                       91, //top
                       67, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_420text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_420==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_421;
    pSTE_421 = StCreate(  STE_421, //name
                       10, //left
                       118, //top
                       67, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_421text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_421==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_422;
    pSTE_422 = StCreate(  STE_422, //name
                       10, //left
                       145, //top
                       67, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_422text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_422==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_423;
    pSTE_423 = StCreate(  STE_423, //name
                       10, //left
                       172, //top
                       67, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_423text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_423==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_424;
    pSTE_424 = StCreate(  STE_424, //name
                       10, //left
                       199, //top
                       67, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_424text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_424==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_425;
    pSTE_425 = StCreate(  STE_425, //name
                       10, //left
                       226, //top
                       67, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_425text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_425==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_426;
    pSTE_426 = StCreate(  STE_426, //name
                       67, //left
                       64, //top
                       124, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_426text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_426==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_427;
    pSTE_427 = StCreate(  STE_427, //name
                       67, //left
                       91, //top
                       124, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_427text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_427==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_428;
    pSTE_428 = StCreate(  STE_428, //name
                       67, //left
                       118, //top
                       124, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_428text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_428==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_429;
    pSTE_429 = StCreate(  STE_429, //name
                       67, //left
                       145, //top
                       124, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_429text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_429==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_430;
    pSTE_430 = StCreate(  STE_430, //name
                       67, //left
                       172, //top
                       124, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_430text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_430==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_431;
    pSTE_431 = StCreate(  STE_431, //name
                       67, //left
                       199, //top
                       124, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_431text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_431==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_432;
    pSTE_432 = StCreate(  STE_432, //name
                       67, //left
                       226, //top
                       124, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_432text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_432==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_433;
    pSTE_433 = StCreate(  STE_433, //name
                       124, //left
                       64, //top
                       181, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_433text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_433==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_434;
    pSTE_434 = StCreate(  STE_434, //name
                       124, //left
                       91, //top
                       181, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_434text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_434==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_437;
    pSTE_437 = StCreate(  STE_437, //name
                       124, //left
                       118, //top
                       181, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_437text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_437==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_438;
    pSTE_438 = StCreate(  STE_438, //name
                       124, //left
                       145, //top
                       181, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_438text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_438==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_439;
    pSTE_439 = StCreate(  STE_439, //name
                       124, //left
                       172, //top
                       181, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_439text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_439==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_440;
    pSTE_440 = StCreate(  STE_440, //name
                       124, //left
                       199, //top
                       181, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_440text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_440==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_441;
    pSTE_441 = StCreate(  STE_441, //name
                       124, //left
                       226, //top
                       181, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_441text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_441==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_442;
    pSTE_442 = StCreate(  STE_442, //name
                       181, //left
                       64, //top
                       238, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_442text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_442==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_443;
    pSTE_443 = StCreate(  STE_443, //name
                       181, //left
                       91, //top
                       238, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_443text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_443==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_444;
    pSTE_444 = StCreate(  STE_444, //name
                       181, //left
                       118, //top
                       238, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_444text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_444==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_445;
    pSTE_445 = StCreate(  STE_445, //name
                       181, //left
                       145, //top
                       238, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_445text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_445==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_446;
    pSTE_446 = StCreate(  STE_446, //name
                       181, //left
                       172, //top
                       238, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_446text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_446==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_447;
    pSTE_447 = StCreate(  STE_447, //name
                       181, //left
                       199, //top
                       238, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_447text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_447==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_448;
    pSTE_448 = StCreate(  STE_448, //name
                       181, //left
                       226, //top
                       238, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_448text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_448==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_449;
    pSTE_449 = StCreate(  STE_449, //name
                       238, //left
                       64, //top
                       295, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_449text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_449==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_450;
    pSTE_450 = StCreate(  STE_450, //name
                       238, //left
                       91, //top
                       295, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_450text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_450==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_451;
    pSTE_451 = StCreate(  STE_451, //name
                       238, //left
                       118, //top
                       295, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_451text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_451==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_452;
    pSTE_452 = StCreate(  STE_452, //name
                       238, //left
                       145, //top
                       295, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_452text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_452==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_453;
    pSTE_453 = StCreate(  STE_453, //name
                       238, //left
                       172, //top
                       295, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_453text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_453==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_454;
    pSTE_454 = StCreate(  STE_454, //name
                       238, //left
                       199, //top
                       295, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_454text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_454==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_455;
    pSTE_455 = StCreate(  STE_455, //name
                       238, //left
                       226, //top
                       295, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_455text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_455==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_456;
    pSTE_456 = StCreate(  STE_456, //name
                       295, //left
                       64, //top
                       352, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_456text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_456==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_457;
    pSTE_457 = StCreate(  STE_457, //name
                       296, //left
                       91, //top
                       353, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_457text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_457==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_458;
    pSTE_458 = StCreate(  STE_458, //name
                       296, //left
                       118, //top
                       353, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_458text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_458==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_459;
    pSTE_459 = StCreate(  STE_459, //name
                       296, //left
                       145, //top
                       353, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_459text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_459==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_460;
    pSTE_460 = StCreate(  STE_460, //name
                       296, //left
                       172, //top
                       353, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_460text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_460==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_461;
    pSTE_461 = StCreate(  STE_461, //name
                       295, //left
                       199, //top
                       352, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_461text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_461==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_462;
    pSTE_462 = StCreate(  STE_462, //name
                       296, //left
                       226, //top
                       353, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_462text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_462==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_463;
    pSTE_463 = StCreate(  STE_463, //name
                       352, //left
                       64, //top
                       409, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_463text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_463==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_464;
    pSTE_464 = StCreate(  STE_464, //name
                       352, //left
                       91, //top
                       409, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_464text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_464==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_465;
    pSTE_465 = StCreate(  STE_465, //name
                       353, //left
                       118, //top
                       410, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_465text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_465==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_466;
    pSTE_466 = StCreate(  STE_466, //name
                       353, //left
                       145, //top
                       410, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_466text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_466==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_475;
    pSTE_475 = StCreate(  STE_475, //name
                       352, //left
                       172, //top
                       409, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_475text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_475==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_476;
    pSTE_476 = StCreate(  STE_476, //name
                       352, //left
                       199, //top
                       409, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_476text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_476==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_477;
    pSTE_477 = StCreate(  STE_477, //name
                       353, //left
                       226, //top
                       410, //right
                       253, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_477text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_477==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_478;
    pSTE_478 = StCreate(  STE_478, //name
                       409, //left
                       64, //top
                       466, //right
                       91, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_478text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_478==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_479;
    pSTE_479 = StCreate(  STE_479, //name
                       409, //left
                       91, //top
                       466, //right
                       118, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_479text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_479==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_480;
    pSTE_480 = StCreate(  STE_480, //name
                       409, //left
                       118, //top
                       466, //right
                       145, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_480text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_480==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_481;
    pSTE_481 = StCreate(  STE_481, //name
                       409, //left
                       145, //top
                       466, //right
                       172, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_481text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_481==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_482;
    pSTE_482 = StCreate(  STE_482, //name
                       409, //left
                       172, //top
                       466, //right
                       199, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_482text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_482==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_483;
    pSTE_483 = StCreate(  STE_483, //name
                       409, //left
                       199, //top
                       466, //right
                       226, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)RESET_STE_483text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_483==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateWAIT_FOR_RESET
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - WAIT_FOR_RESET
***************************************************/
void CreateWAIT_FOR_RESET(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;


    BUTTON *pBTN_484;
    pBTN_484 = BtnCreate(  BTN_484, //name
                       417, //left
                       125, //top
                       473, //right
                       155, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)WAIT_FOR_RESET_BTN_484text, //text
                      Push_Button //scheme
                    );

    if(pBTN_484==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_485;
    pBTN_485 = BtnCreate(  BTN_485, //name
                       417, //left
                       160, //top
                       473, //right
                       190, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)WAIT_FOR_RESET_BTN_485text, //text
                      Push_Button //scheme
                    );

    if(pBTN_485==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_486;
    pBTN_486 = BtnCreate(  BTN_486, //name
                       417, //left
                       196, //top
                       473, //right
                       226, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)WAIT_FOR_RESET_BTN_486text, //text
                      Push_Button //scheme
                    );

    if(pBTN_486==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_487;
    pBTN_487 = BtnCreate(  BTN_487, //name
                       418, //left
                       230, //top
                       474, //right
                       260, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)WAIT_FOR_RESET_BTN_487text, //text
                      Push_Button //scheme
                    );

    if(pBTN_487==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_488;
    pSTE_488 = StCreate(  STE_488, //name
                       76, //left
                       38, //top
                       227, //right
                       61, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_488text, //text
                      Static_test //scheme
                    );

    if(pSTE_488==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_489;
    pSTE_489 = StCreate(  STE_489, //name
                       229, //left
                       39, //top
                       329, //right
                       62, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_489text, //text
                      Static_test //scheme
                    );

    if(pSTE_489==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_490;
    pSTE_490 = StCreate(  STE_490, //name
                       76, //left
                       12, //top
                       164, //right
                       35, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_490text, //text
                      Static_test //scheme
                    );

    if(pSTE_490==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_491;
    pSTE_491 = StCreate(  STE_491, //name
                       166, //left
                       12, //top
                       233, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_491text, //text
                      Static_test //scheme
                    );

    if(pSTE_491==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_492;
    pSTE_492 = StCreate(  STE_492, //name
                       329, //left
                       12, //top
                       410, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_492text, //text
                      Static_test //scheme
                    );

    if(pSTE_492==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_493;
    pSTE_493 = StCreate(  STE_493, //name
                       329, //left
                       39, //top
                       398, //right
                       62, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_493text, //text
                      Static_test //scheme
                    );

    if(pSTE_493==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_494;
    pSTE_494 = StCreate(  STE_494, //name
                       234, //left
                       12, //top
                       297, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_494text, //text
                      Static_test //scheme
                    );

    if(pSTE_494==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_495;
    pSTE_495 = StCreate(  STE_495, //name
                       125, //left
                       146, //top
                       398, //right
                       171, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_495text, //text
                      Static_test //scheme
                    );

    if(pSTE_495==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_496;
    pSTE_496 = StCreate(  STE_496, //name
                       25, //left
                       146, //top
                       125, //right
                       171, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_496text, //text
                      Static_test //scheme
                    );

    if(pSTE_496==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_499;
    pPCB_499 = PictCreate(  PCB_499, //name
                       10, //left
                       10, //top
                       72, //right
                       54, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_499==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_501;
    pPCB_501 = PictCreate(  PCB_501, //name
                       422, //left
                       9, //top
                       474, //right
                       61, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_501==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_511;
    pSTE_511 = StCreate(  STE_511, //name
                       125, //left
                       171, //top
                       398, //right
                       196, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_511text, //text
                      Static_test //scheme
                    );

    if(pSTE_511==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_497;
    pSTE_497 = StCreate(  STE_497, //name
                       25, //left
                       171, //top
                       125, //right
                       196, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_497text, //text
                      Static_test //scheme
                    );

    if(pSTE_497==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_501;
    pSTE_501 = StCreate(  STE_501, //name
                       25, //left
                       121, //top
                       125, //right
                       146, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_501text, //text
                      Static_test //scheme
                    );

    if(pSTE_501==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_502;
    pSTE_502 = StCreate(  STE_502, //name
                       125, //left
                       121, //top
                       398, //right
                       146, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_502text, //text
                      Static_test //scheme
                    );

    if(pSTE_502==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_417;
    pSTE_417 = StCreate(  STE_417, //name
                       25, //left
                       197, //top
                       125, //right
                       222, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_417text, //text
                      Static_test //scheme
                    );

    if(pSTE_417==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_418;
    pSTE_418 = StCreate(  STE_418, //name
                       125, //left
                       197, //top
                       398, //right
                       222, //bottom
                       ST_HIDE | ST_FRAME , //state
                       (XCHAR*)WAIT_FOR_RESET_STE_418text, //text
                      Static_test //scheme
                    );

    if(pSTE_418==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateCPU_DE
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - CPU_DE
***************************************************/
void CreateCPU_DE(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;


    BUTTON *pBTN_423;
    pBTN_423 = BtnCreate(  BTN_423, //name
                       417, //left
                       125, //top
                       473, //right
                       155, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_DE_BTN_423text, //text
                      Push_Button //scheme
                    );

    if(pBTN_423==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_424;
    pBTN_424 = BtnCreate(  BTN_424, //name
                       417, //left
                       160, //top
                       473, //right
                       190, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_DE_BTN_424text, //text
                      Push_Button //scheme
                    );

    if(pBTN_424==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_425;
    pBTN_425 = BtnCreate(  BTN_425, //name
                       417, //left
                       195, //top
                       473, //right
                       225, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_DE_BTN_425text, //text
                      Push_Button //scheme
                    );

    if(pBTN_425==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_426;
    pBTN_426 = BtnCreate(  BTN_426, //name
                       418, //left
                       230, //top
                       474, //right
                       260, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_DE_BTN_426text, //text
                      Push_Button //scheme
                    );

    if(pBTN_426==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_484;
    pSTE_484 = StCreate(  STE_484, //name
                       76, //left
                       39, //top
                       229, //right
                       62, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_484text, //text
                      Static_test //scheme
                    );

    if(pSTE_484==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_485;
    pSTE_485 = StCreate(  STE_485, //name
                       229, //left
                       39, //top
                       329, //right
                       62, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_485text, //text
                      Static_test //scheme
                    );

    if(pSTE_485==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_486;
    pSTE_486 = StCreate(  STE_486, //name
                       76, //left
                       12, //top
                       164, //right
                       35, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_486text, //text
                      Static_test //scheme
                    );

    if(pSTE_486==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_487;
    pSTE_487 = StCreate(  STE_487, //name
                       166, //left
                       12, //top
                       233, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_487text, //text
                      Static_test //scheme
                    );

    if(pSTE_487==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_498;
    pSTE_498 = StCreate(  STE_498, //name
                       329, //left
                       12, //top
                       410, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_498text, //text
                      Static_test //scheme
                    );

    if(pSTE_498==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_499;
    pSTE_499 = StCreate(  STE_499, //name
                       329, //left
                       39, //top
                       398, //right
                       62, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_499text, //text
                      Static_test //scheme
                    );

    if(pSTE_499==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_500;
    pSTE_500 = StCreate(  STE_500, //name
                       234, //left
                       12, //top
                       297, //right
                       35, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_500text, //text
                      Static_test //scheme
                    );

    if(pSTE_500==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_512;
    pSTE_512 = StCreate(  STE_512, //name
                       115, //left
                       160, //top
                       388, //right
                       185, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_512text, //text
                      Static_test //scheme
                    );

    if(pSTE_512==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_513;
    pSTE_513 = StCreate(  STE_513, //name
                       10, //left
                       160, //top
                       110, //right
                       185, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_513text, //text
                      Static_test //scheme
                    );

    if(pSTE_513==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_515;
    pSTE_515 = StCreate(  STE_515, //name
                       10, //left
                       215, //top
                       110, //right
                       240, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_515text, //text
                      Static_test //scheme
                    );

    if(pSTE_515==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_516;
    pPCB_516 = PictCreate(  PCB_516, //name
                       10, //left
                       10, //top
                       72, //right
                       54, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_516==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_517;
    pPCB_517 = PictCreate(  PCB_517, //name
                       233, //left
                       225, //top
                       295, //right
                       267, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_517==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_518;
    pPCB_518 = PictCreate(  PCB_518, //name
                       422, //left
                       9, //top
                       474, //right
                       61, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_518==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_519;
    pSTE_519 = StCreate(  STE_519, //name
                       115, //left
                       215, //top
                       156, //right
                       239, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_519text, //text
                      Static_test //scheme
                    );

    if(pSTE_519==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_520;
    pSTE_520 = StCreate(  STE_520, //name
                       122, //left
                       95, //top
                       177, //right
                       120, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_520text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_520==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_521;
    pSTE_521 = StCreate(  STE_521, //name
                       123, //left
                       125, //top
                       178, //right
                       150, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_521text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_521==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_529;
    pSTE_529 = StCreate(  STE_529, //name
                       10, //left
                       95, //top
                       110, //right
                       120, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_529text, //text
                      Static_test //scheme
                    );

    if(pSTE_529==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_530;
    pSTE_530 = StCreate(  STE_530, //name
                       10, //left
                       125, //top
                       110, //right
                       150, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_530text, //text
                      Static_test //scheme
                    );

    if(pSTE_530==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_531;
    pSTE_531 = StCreate(  STE_531, //name
                       123, //left
                       70, //top
                       175, //right
                       95, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_DE_STE_531text, //text
                      Static_test //scheme
                    );

    if(pSTE_531==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_535;
    pRDB_535 = RbCreate(  RDB_535, //name
                       330, //left
                       249, //top
                       367, //right
                       267, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_DE_RDB_535text, //text
                      Static_test //scheme
                    );

    if(pRDB_535==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateCPU_4D1S
* Parameters      :    none
* Return          :    none
* Description     :    Creates GOL widgets used in screen - CPU_4D1S
***************************************************/
void CreateCPU_4D1S(void)
{
    GOLFree();
    SetColor(RGBConvert(0, 48, 200));
    ClearDevice();


     if(Static_test != NULL) free(Static_test);
        Static_test = GOLCreateScheme();

    Static_test->Color0 = RGBConvert(48, 100, 0);
    Static_test->Color1 = RGBConvert(0, 0, 248);
    Static_test->TextColor0 = RGBConvert(248, 252, 248);
    Static_test->TextColor1 = RGBConvert(8, 224, 8);
    Static_test->EmbossDkColor = RGBConvert(248, 204, 0);
    Static_test->EmbossLtColor = RGBConvert(0, 204, 0);
    Static_test->TextColorDisabled = RGBConvert(32, 212, 32);
    Static_test->ColorDisabled = RGBConvert(208, 224, 240);
    Static_test->CommonBkColor = RGBConvert(0, 48, 200);
    Static_test->pFont = (void*)&Calibri_Bold_14;

    if(defscheme != NULL) free(defscheme);
        defscheme = GOLCreateScheme();

    defscheme->Color0 = RGBConvert(32, 168, 224);
    defscheme->Color1 = RGBConvert(16, 132, 168);
    defscheme->TextColor0 = RGBConvert(24, 24, 24);
    defscheme->TextColor1 = RGBConvert(248, 252, 248);
    defscheme->EmbossDkColor = RGBConvert(248, 204, 0);
    defscheme->EmbossLtColor = RGBConvert(24, 116, 184);
    defscheme->TextColorDisabled = RGBConvert(128, 128, 128);
    defscheme->ColorDisabled = RGBConvert(208, 224, 240);
    defscheme->CommonBkColor = RGBConvert(208, 236, 240);
    defscheme->pFont = (void*)&Gentium_16;

    if(Push_Button != NULL) free(Push_Button);
        Push_Button = GOLCreateScheme();

    Push_Button->Color0 = RGBConvert(200, 204, 0);
    Push_Button->Color1 = RGBConvert(0, 0, 248);
    Push_Button->TextColor0 = RGBConvert(0, 0, 0);
    Push_Button->TextColor1 = RGBConvert(248, 252, 248);
    Push_Button->EmbossDkColor = RGBConvert(0, 48, 200);
    Push_Button->EmbossLtColor = RGBConvert(0, 48, 200);
    Push_Button->TextColorDisabled = RGBConvert(248, 0, 0);
    Push_Button->ColorDisabled = RGBConvert(208, 224, 240);
    Push_Button->CommonBkColor = RGBConvert(0, 48, 200);
    Push_Button->pFont = (void*)&Monospaced_bold_Bold_14_1;

    if(enter_new_scheme != NULL) free(enter_new_scheme);
        enter_new_scheme = GOLCreateScheme();

    enter_new_scheme->Color0 = RGBConvert(0, 0, 200);
    enter_new_scheme->Color1 = RGBConvert(16, 132, 168);
    enter_new_scheme->TextColor0 = RGBConvert(248, 252, 0);
    enter_new_scheme->TextColor1 = RGBConvert(248, 252, 248);
    enter_new_scheme->EmbossDkColor = RGBConvert(0, 0, 0);
    enter_new_scheme->EmbossLtColor = RGBConvert(24, 116, 184);
    enter_new_scheme->TextColorDisabled = RGBConvert(0, 0, 200);
    enter_new_scheme->ColorDisabled = RGBConvert(208, 224, 240);
    enter_new_scheme->CommonBkColor = RGBConvert(0, 0, 152);
    enter_new_scheme->pFont = (void*)&Calibri_Light_18;


    STATICTEXT *pSTE_528;
    pSTE_528 = StCreate(  STE_528, //name
                       76, //left
                       40, //top
                       242, //right
                       64, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_528text, //text
                      Static_test //scheme
                    );

    if(pSTE_528==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_532;
    pSTE_532 = StCreate(  STE_532, //name
                       243, //left
                       40, //top
                       341, //right
                       63, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_532text, //text
                      Static_test //scheme
                    );

    if(pSTE_532==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_533;
    pSTE_533 = StCreate(  STE_533, //name
                       75, //left
                       10, //top
                       163, //right
                       33, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_533text, //text
                      Static_test //scheme
                    );

    if(pSTE_533==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_534;
    pSTE_534 = StCreate(  STE_534, //name
                       170, //left
                       10, //top
                       234, //right
                       33, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_534text, //text
                      Static_test //scheme
                    );

    if(pSTE_534==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_535;
    pSTE_535 = StCreate(  STE_535, //name
                       329, //left
                       11, //top
                       410, //right
                       34, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_535text, //text
                      Static_test //scheme
                    );

    if(pSTE_535==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_536;
    pSTE_536 = StCreate(  STE_536, //name
                       341, //left
                       40, //top
                       410, //right
                       63, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_536text, //text
                      Static_test //scheme
                    );

    if(pSTE_536==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_537;
    pSTE_537 = StCreate(  STE_537, //name
                       240, //left
                       10, //top
                       303, //right
                       33, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_537text, //text
                      Static_test //scheme
                    );

    if(pSTE_537==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_538;
    pPCB_538 = PictCreate(  PCB_538, //name
                       7, //left
                       11, //top
                       69, //right
                       55, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&logo_small2_db2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_538==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_539;
    pPCB_539 = PictCreate(  PCB_539, //name
                       420, //left
                       5, //top
                       472, //right
                       57, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&IR_Logo_small_4_DB2, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_539==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_544;
    pSTE_544 = StCreate(  STE_544, //name
                       51, //left
                       65, //top
                       103, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_544text, //text
                      Static_test //scheme
                    );

    if(pSTE_544==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_545;
    pSTE_545 = StCreate(  STE_545, //name
                       103, //left
                       65, //top
                       155, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_545text, //text
                      Static_test //scheme
                    );

    if(pSTE_545==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_546;
    pSTE_546 = StCreate(  STE_546, //name
                       155, //left
                       65, //top
                       207, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_546text, //text
                      Static_test //scheme
                    );

    if(pSTE_546==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_549;
    pSTE_549 = StCreate(  STE_549, //name
                       4, //left
                       175, //top
                       102, //right
                       200, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_549text, //text
                      Static_test //scheme
                    );

    if(pSTE_549==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_555;
    pBTN_555 = BtnCreate(  BTN_555, //name
                       430, //left
                       245, //top
                       479, //right
                       270, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_4D1S_BTN_555text, //text
                      Push_Button //scheme
                    );

    if(pBTN_555==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_556;
    pBTN_556 = BtnCreate(  BTN_556, //name
                       430, //left
                       214, //top
                       479, //right
                       239, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_4D1S_BTN_556text, //text
                      Push_Button //scheme
                    );

    if(pBTN_556==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_557;
    pBTN_557 = BtnCreate(  BTN_557, //name
                       430, //left
                       188, //top
                       479, //right
                       214, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_4D1S_BTN_557text, //text
                      Push_Button //scheme
                    );

    if(pBTN_557==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_558;
    pBTN_558 = BtnCreate(  BTN_558, //name
                       430, //left
                       163, //top
                       479, //right
                       188, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_4D1S_BTN_558text, //text
                      Push_Button //scheme
                    );

    if(pBTN_558==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_564;
    pSTE_564 = StCreate(  STE_564, //name
                       103, //left
                       175, //top
                       383, //right
                       200, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_564text, //text
                      Static_test //scheme
                    );

    if(pSTE_564==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_567;
    pSTE_567 = StCreate(  STE_567, //name
                       12, //left
                       90, //top
                       51, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_567text, //text
                      Static_test //scheme
                    );

    if(pSTE_567==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_568;
    pSTE_568 = StCreate(  STE_568, //name
                       12, //left
                       113, //top
                       51, //right
                       138, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_568text, //text
                      Static_test //scheme
                    );

    if(pSTE_568==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_583;
    pSTE_583 = StCreate(  STE_583, //name
                       207, //left
                       65, //top
                       259, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_583text, //text
                      Static_test //scheme
                    );

    if(pSTE_583==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_584;
    pSTE_584 = StCreate(  STE_584, //name
                       51, //left
                       90, //top
                       103, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_584text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_584==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_585;
    pSTE_585 = StCreate(  STE_585, //name
                       51, //left
                       115, //top
                       103, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_585text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_585==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_586;
    pSTE_586 = StCreate(  STE_586, //name
                       103, //left
                       90, //top
                       155, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_586text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_586==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_587;
    pSTE_587 = StCreate(  STE_587, //name
                       103, //left
                       115, //top
                       155, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_587text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_587==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_588;
    pSTE_588 = StCreate(  STE_588, //name
                       155, //left
                       90, //top
                       207, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_588text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_588==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_589;
    pSTE_589 = StCreate(  STE_589, //name
                       155, //left
                       115, //top
                       207, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_589text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_589==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_590;
    pSTE_590 = StCreate(  STE_590, //name
                       207, //left
                       90, //top
                       259, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_590text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_590==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_591;
    pSTE_591 = StCreate(  STE_591, //name
                       207, //left
                       115, //top
                       259, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_591text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_591==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_592;
    pSTE_592 = StCreate(  STE_592, //name
                       421, //left
                       64, //top
                       473, //right
                       89, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_592text, //text
                      Static_test //scheme
                    );

    if(pSTE_592==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_593;
    pSTE_593 = StCreate(  STE_593, //name
                       369, //left
                       65, //top
                       421, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_593text, //text
                      Static_test //scheme
                    );

    if(pSTE_593==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_594;
    pSTE_594 = StCreate(  STE_594, //name
                       317, //left
                       65, //top
                       369, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_594text, //text
                      Static_test //scheme
                    );

    if(pSTE_594==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_595;
    pSTE_595 = StCreate(  STE_595, //name
                       265, //left
                       65, //top
                       317, //right
                       90, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_595text, //text
                      Static_test //scheme
                    );

    if(pSTE_595==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_596;
    pSTE_596 = StCreate(  STE_596, //name
                       265, //left
                       90, //top
                       317, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_596text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_596==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_597;
    pSTE_597 = StCreate(  STE_597, //name
                       265, //left
                       115, //top
                       317, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_597text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_597==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_598;
    pSTE_598 = StCreate(  STE_598, //name
                       317, //left
                       90, //top
                       369, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_598text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_598==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_599;
    pSTE_599 = StCreate(  STE_599, //name
                       317, //left
                       115, //top
                       369, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_599text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_599==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_600;
    pSTE_600 = StCreate(  STE_600, //name
                       369, //left
                       90, //top
                       421, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_600text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_600==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_601;
    pSTE_601 = StCreate(  STE_601, //name
                       369, //left
                       115, //top
                       421, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_601text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_601==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_602;
    pSTE_602 = StCreate(  STE_602, //name
                       421, //left
                       90, //top
                       473, //right
                       115, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_602text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_602==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_603;
    pSTE_603 = StCreate(  STE_603, //name
                       421, //left
                       115, //top
                       473, //right
                       140, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_603text, //text
                      enter_new_scheme //scheme
                    );

    if(pSTE_603==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_604;
    pSTE_604 = StCreate(  STE_604, //name
                       4, //left
                       150, //top
                       103, //right
                       175, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_604text, //text
                      Static_test //scheme
                    );

    if(pSTE_604==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_605;
    pSTE_605 = StCreate(  STE_605, //name
                       103, //left
                       150, //top
                       383, //right
                       175, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_605text, //text
                      Static_test //scheme
                    );

    if(pSTE_605==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_608;
    pRDB_608 = RbCreate(  RDB_608, //name
                       229, //left
                       236, //top
                       265, //right
                       253, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_4D1S_RDB_608text, //text
                      Static_test //scheme
                    );

    if(pRDB_608==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_610;
    pRDB_610 = RbCreate(  RDB_610, //name
                       363, //left
                       219, //top
                       399, //right
                       236, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_4D1S_RDB_610text, //text
                      Static_test //scheme
                    );

    if(pRDB_610==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_611;
    pPCB_611 = PictCreate(  PCB_611, //name
                       163, //left
                       225, //top
                       224, //right
                       264, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_611==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_612;
    pRDB_612 = RbCreate(  RDB_612, //name
                       363, //left
                       236, //top
                       399, //right
                       253, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_4D1S_RDB_612text, //text
                      Static_test //scheme
                    );

    if(pRDB_612==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    PICTURE *pPCB_613;
    pPCB_613 = PictCreate(  PCB_613, //name
                       289, //left
                       225, //top
                       351, //right
                       264, //bottom
                       PICT_DRAW, //state
                       1, //scale
                       (void*)&No_Train, //bitmap
                      defscheme //scheme
                    );

    if(pPCB_613==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    RADIOBUTTON *pRDB_614;
    pRDB_614 = RbCreate(  RDB_614, //name
                       363, //left
                       253, //top
                       399, //right
                       270, //bottom
                       RB_DRAW | RB_GROUP, //state
                       (XCHAR*)CPU_4D1S_RDB_614text, //text
                      Static_test //scheme
                    );

    if(pRDB_614==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_615;
    pSTE_615 = StCreate(  STE_615, //name
                       4, //left
                       200, //top
                       102, //right
                       225, //bottom
                       ST_DRAW | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_615text, //text
                      Static_test //scheme
                    );

    if(pSTE_615==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    STATICTEXT *pSTE_616;
    pSTE_616 = StCreate(  STE_616, //name
                       104, //left
                       201, //top
                       157, //right
                       225, //bottom
                       ST_DRAW | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_4D1S_STE_616text, //text
                      Static_test //scheme
                    );

    if(pSTE_616==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }

    BUTTON *pBTN_609;
    pBTN_609 = BtnCreate(  BTN_609, //name
                       4, //left
                       239, //top
                       55, //right
                       266, //bottom
                       0, //radius
                       BTN_DRAW, //state
                       NULL, //bitmap
                       (XCHAR*)CPU_4D1S_BTN_609text, //text
                      Push_Button //scheme
                    );

    if(pBTN_609==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }


}
/***************************************************
* Function 	      :    CreateHome
* Parameters      :    none
* Return          :    none
* Description     :    Creates GPL widgets used in screen - Home
***************************************************/
void CreatePrimitivesForHome(void)
{
    SetLineType(0);
    SetLineThickness(0);
    SetColor(RGBConvert(248, 252, 248));
    SetFont((void*)&Book_Antiqua_18);
    while(!OutTextXY(  49, //x
                       159, //x
                      (XCHAR*)Home_OTE_2text //text
                    )
         );

    SetLineType(0);
    SetLineThickness(0);
    SetColor(RGBConvert(208, 12, 8));
    SetFont((void*)&Handel_Gothic_22);
    while(!OutTextXY(  65, //x
                       120, //x
                      (XCHAR*)Home_OTE_5text //text
                    )
         );


}



/***************************************************
* Function 	      :    CreateCPU_2DP
* Parameters      :    none
* Return          :    none
* Description     :    Creates GPL widgets used in screen - CPU_2DP
***************************************************/
void CreatePrimitivesForCPU_2DP(void)
{
    SetLineType(0);
    SetLineThickness(1);
    SetColor(RGBConvert(208, 12, 8));
    while(!Line(  170, //x1
                    268, //y1
                    309, //x2
                   268 //y2
                 )
         );


}



/***************************************************
* Function 	      :    CreateCPU_3DP_AS
* Parameters      :    none
* Return          :    none
* Description     :    Creates GPL widgets used in screen - CPU_3DP_AS
***************************************************/
void CreatePrimitivesForCPU_3DP_AS(void)
{
    SetLineType(0);
    SetLineThickness(1);
    SetColor(RGBConvert(208, 12, 8));
    while(!Line(  377, //x1
                    106, //y1
                    432, //x2
                   106 //y2
                 )
         );

    SetLineType(0);
    SetLineThickness(1);
    SetColor(RGBConvert(208, 12, 8));
    while(!Line(  377, //x1
                    156, //y1
                    432, //x2
                   156 //y2
                 )
         );

    SetLineType(0);
    SetLineThickness(1);
    SetColor(RGBConvert(208, 12, 8));
    while(!Line(  292, //x1
                    137, //y1
                    347, //x2
                   137 //y2
                 )
         );


}



/***************************************************
* Function 	      :    CreateCPU_DE
* Parameters      :    none
* Return          :    none
* Description     :    Creates GPL widgets used in screen - CPU_DE
***************************************************/
void CreatePrimitivesForCPU_DE(void)
{
    SetLineType(0);
    SetLineThickness(1);
    SetColor(RGBConvert(208, 12, 8));
    while(!Line(  170, //x1
                    268, //y1
                    308, //x2
                   268 //y2
                 )
         );


}




/***************************************************
* Function       : CreateFunctionArray
* Parameters     : none
* Return         : none
* Description    : Creates a array of GOL function pointers
***************************************************/
void (*CreateFunctionArray[NUM_GDD_SCREENS])(void)=
    
{
    &CreateHome,
    &CreateCPU_3D2S,
    &CreateCPU_2DP,
    &CreateCPU_3DP_AS,
    &CreateCPU_LCWS,
    &CreateSMCPU,
    &CreateRESET,
    &CreateWAIT_FOR_RESET,
    &CreateCPU_DE,
    &CreateCPU_4D1S,
};



/***************************************************
* Function       : CreatePrimitivesFunctionArray
* Parameters     : none
* Return         : none
* Description    : Creates a array of GPL function pointers
***************************************************/
void (*CreatePrimitivesFunctionArray[NUM_GDD_SCREENS])(void)=
    
{
    &CreatePrimitivesForHome,
    NULL,
    &CreatePrimitivesForCPU_2DP,
    &CreatePrimitivesForCPU_3DP_AS,
    NULL,
    NULL,
    NULL,
    NULL,
    &CreatePrimitivesForCPU_DE,
    NULL,
};


