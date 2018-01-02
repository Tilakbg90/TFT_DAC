/*****************************************************************************
 * Microchip Graphics Library
 * Graphics Display Designer (GDD) Template
 *****************************************************************************
 * FileName:        Main.c
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
#include "Main.h"
#include "GDD_Screens.h"
#define DEBUG_SCREEN 0
// Configuration bits
#if defined(__dsPIC33F__) || defined(__PIC24H__)
_FOSCSEL(FNOSC_PRI);
_FOSC(FCKSM_CSECMD &OSCIOFNC_OFF &POSCMD_XT);
_FWDT(FWDTEN_OFF);
#elif defined(__dsPIC33E__) || defined(__PIC24E__)
_FOSCSEL(FNOSC_FRC);			
_FOSC(FCKSM_CSECMD & POSCMD_XT & OSCIOFNC_OFF & IOL1WAY_OFF);
_FWDT(FWDTEN_OFF);
_FPOR(FPWRT_PWR128 & BOREN_ON & ALTI2C1_ON & ALTI2C2_ON);
_FICD(ICS_PGD1 & RSTPRI_PF & JTAGEN_OFF);
#elif defined(__PIC32MX__)
    #pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FWDTEN = OFF, FCKSM = CSECME, FPBDIV = DIV_1
    #pragma config OSCIOFNC = ON, POSCMOD = XT, FSOSCEN = ON, FNOSC = PRIPLL
    #pragma config CP = OFF, BWP = OFF, PWP = OFF
#else
    #if defined(__PIC24FJ256GB110__)
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2(0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_XT & FNOSC_PRIPLL & PLLDIV_DIV2 & IOL1WAY_OFF)
    #endif
    #if defined(__PIC24FJ256GA110__)
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_XT & FNOSC_PRIPLL & IOL1WAY_OFF)
    #endif
    #if defined(__PIC24FJ128GA010__)
_CONFIG2(FNOSC_PRIPLL & POSCMOD_XT) // Primary XT OSC with PLL
_CONFIG1(JTAGEN_OFF & FWDTEN_OFF)   // JTAG off, watchdog timer off
    #endif
	#if defined (__PIC24FJ256GB210__)
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & ALTVREF_ALTVREDIS & WINDIS_OFF & FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF) 
_CONFIG2( POSCMOD_XT & IOL1WAY_OFF & OSCIOFNC_OFF & OSCIOFNC_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
_CONFIG3( WPFP_WPFP255 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM) 
	#endif
	#if defined (__PIC24FJ256DA210__)
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & ALTVREF_ALTVREDIS & WINDIS_OFF & FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF) 
_CONFIG2( POSCMOD_HS & IOL1WAY_OFF & OSCIOFNC_OFF & OSCIOFNC_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
_CONFIG3( WPFP_WPFP255 & SOSCSEL_SOSC & WUTSEL_LEG & ALTPMP_ALTPMPEN & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
	#endif	
#endif
/////////////////////////////////////////////////////////////////////////////
// SPI Device Initialization Function 
/////////////////////////////////////////////////////////////////////////////
#if defined (USE_SST25VF016)
    // initialize GFX3 SST25 flash SPI
    #define FlashInit(pInitData) SST25Init((DRV_SPI_INIT_DATA*)pInitData)                    
#elif defined (USE_MCHP25LC256)
    // initialize EEPROM on Explorer 16
    #define FlashInit(pInitData) MCHP25LC256Init((DRV_SPI_INIT_DATA*)pInitData)  
#elif defined (USE_M25P80)       
    #define FlashInit(pInitData) SST25Init((DRV_SPI_INIT_DATA*)pInitData)
#endif
  


/////////////////////////////////////////////////////////////////////////////
// SPI Channel settings
/////////////////////////////////////////////////////////////////////////////
#if defined (SPI_CHANNEL_1_ENABLE) || defined (SPI_CHANNEL_2_ENABLE) || defined (SPI_CHANNEL_3_ENABLE) || defined (SPI_CHANNEL_4_ENABLE)
    #if defined (USE_SST25VF016)
        #ifdef __PIC32MX
            const DRV_SPI_INIT_DATA SPI_Init_Data = {SST25_SPI_CHANNEL, 1, 0, 0, 1, 1, 0};
            #ifdef USE_TOUCHSCREEN_AR1020
                const DRV_SPI_INIT_DATA ar1020SpiInit = {AR1020_SPI_CHANNEL,    44, 0, 0, 0, 0, 0};
            #endif
        #else    
            const DRV_SPI_INIT_DATA SPI_Init_Data = {SST25_SPI_CHANNEL, 3, 6, 0, 1, 1, 0};
            #ifdef USE_TOUCHSCREEN_AR1020
                const DRV_SPI_INIT_DATA ar1020SpiInit = {AR1020_SPI_CHANNEL,    2,  3, 0, 0, 0, 0};        
            #endif
        #endif
    #elif defined (USE_MCHP25LC256)       
        const DRV_SPI_INIT_DATA SPI_Init_Data = {MCHP25LC256_SPI_CHANNEL, 6, 3, 0, 1, 1, 0};    
    #elif defined (USE_M25P80)
            const DRV_SPI_INIT_DATA SPI_Init_Data = {SST25_SPI_CHANNEL, 3, 6, 0, 1, 1, 0};
    #endif    
#endif

/////////////////////////////////////////////////////////////////////////////
// TouchScreen Init Values
/////////////////////////////////////////////////////////////////////////////
#ifdef USE_TOUCHSCREEN_RESISTIVE
#define TOUCH_INIT_VALUES   (NULL)
#endif
#ifdef USE_TOUCHSCREEN_AR1020
#define TOUCH_INIT_VALUES   ((void *)&ar1020SpiInit)
#endif

#define GCPU_ACTIVE_DIR TRISGbits.TRISG2
#define GCPU_ACTIVE LATGbits.LATG2

disp_info_t Disp_Info;
disp_info_t Disp_Info1,Disp_Info2;

Conf_wheel_info       Wheel, Wheel1, Wheel2;
Conf_wheel_info       Track_Wheel1, Track_Wheel2;

Conf_wheel_info_3D2S    Wheel1_3D2S,Wheel2_3D2S;
Conf_wheel_info_3D2S    Track_Wheel1_3D2S,Track_Wheel2_3D2S;

Conf_wheel_info_3D1S    Wheel_3D1S, Wheel_3D1S_SC;
Conf_wheel_info_4D1S    Wheel_4D1S, Wheel_4D1S_SC;
Conf_wheel_info_LCWS    Wheel_LCWS;
extern unsigned char Active_data;
unsigned char CPU_Version = 1;
/////////////////////////////////////////////////////////////////////////////
//                            LOCAL PROTOTYPES
/////////////////////////////////////////////////////////////////////////////
void            TickInit(void);                 // starts tick counter
void 		InitializeBoard(void);
volatile unsigned char Home_Screen_Time_Up = 0;
void Count_message(void);
void Com_message(void);
void Update_Time_Date(void);
void Update_Debug_blocks(void);
void Update_C_2DP_Screen(void);
void Update_C_3DP2S_Screen(void);
void Update_C_3DP_SF_Screen(void);
void Update_C_3DP_EF_Screen(void);

void Update_C_LCWS_Screen(void);
void Update_SMCPU_Message(void);
void Update_UT_DE_Screen(void);
void Update_3D1S_Screen(void);
void Update_4D1S_Screen(void);
/* */
#include "Common_DAC_info.h"
extern dac_sysinfo_t DAC_sysinfo;
extern unsigned char Packet_src;
char Error_message[70][35] =
{
"Prelimnary Relay 2 Failure",
"Prelimnary Relay 1 Failure",
"Critical Relay 2 Failure",
"Critical Relay 1 Failure",
"Transient Power Failure at DS1",
"Transient Power Failure at DS2",
"Transient Power Failure at US1",
"Transient Power Failure at US2",
"Inoperative Network address",
"Incorrect CRC of Code",
"Inoperative Configuration",
"Inoperative Counts",
"Ram test Failed",
"Direct Out Count",
"AD SUP Missing",
"",
"AD1 SUP Low",
"AD2 SUP Low",
"AD1 Pulsing",
"AD2 Pulsing",
"AD State Missing",
"AD SUP Pulsating",
"AD State Fail",
"AD not Pulsating",
"AD1 Board Missing",
"AD2 Board Missing",
"Event Logger Board Missing",
"Modem Board Missing",
"CRPR2 Board Missing",
"CRPR1 Board Missing",
"CRPR2 Board Missing",
"Associate CPU not Active",
"No Carrier in Modem",
"Interprocess Failure",
"Failure at DS",
"Failure at US",
"AD Pulse Mismatch",
"Bootup AD Fail",
"Axle Decptive",
"Double Coil Influence",
"Com Fail - LU1 to US1",
"Com Fail - LU1 to US2",
"Com Fail - LU1 to DS1",
"Com Fail - LU1 to DS2",
"Com Fail - US1 to LU1",
"Com Fail - US2 to LU1",
"Com Fail - DS1 to LU1",
"Com Fail - DS2 to LU1",
"Com Fail - LU2 to US1",
"Com Fail - LU2 to US2",
"Com Fail - LU2 to DS1",
"Com Fail - LU2 to DS2",
"Com Fail - US1 to LU2",
"Com Fail - US2 to LU2",
"Com Fail - DS1 to LU2",
"Com Fail - DS2 to LU2"
};
/////////////////////////////////////////////////////////////////////////////
// Function: WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER* pObj, GOL_MSG* pMsg)
// Input: objMsg - translated message for the object,
//        pObj - pointer to the object,
//        pMsg - pointer to the non-translated, raw GOL message
// Output: if the function returns non-zero the message will be processed by default
// Overview: it's a user defined function. GOLMsg() function calls it each

//           time the valid message for the object received
/////////////////////////////////////////////////////////////////////////////
WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg)
{
    WORD    objectID;

    objectID = GetObjID(pObj);

    GDDDemoGOLMsgCallback(objMsg, pObj, pMsg);
    
    // Add additional code here...

    return (1);
}

/////////////////////////////////////////////////////////////////////////////
// Function: WORD GOLDrawCallback()
// Output: if the function returns non-zero the draw control will be passed to GOL
// Overview: it's a user defined function. GOLDraw() function calls it each
//           time when GOL objects drawing is completed. User drawing should be done here.
//           GOL will not change color, line type and clipping region settings while

//           this function returns zero.
/////////////////////////////////////////////////////////////////////////////
WORD GOLDrawCallback(void)
{
    GDDDemoGOLDrawCallback();

    // Add additional code here...

    return (1);
}


/////////////////////////////////////////////////////////////////////////////
// Function: Timer3 ISR
// Input: none
// Output: none
// Overview: increments tick counter. Tick is approx. 1 ms.
/////////////////////////////////////////////////////////////////////////////
#ifdef __PIC32MX__
    #define __T3_ISR    __ISR(_TIMER_3_VECTOR, ipl4)
#else
    #define __T3_ISR    __attribute__((interrupt, shadow, auto_psv))
#endif

/* */
void __T3_ISR _T3Interrupt(void)
{
    TMR3 = 0;
    // Clear flag
    #ifdef __PIC32MX__
    mT3ClearIntFlag();
    #else
    IFS0bits.T3IF = 0;
    #endif

    TouchDetectPosition();
}

/////////////////////////////////////////////////////////////////////////////
// Function: void TickInit(void)
// Input: none
// Output: none
// Overview: Initilizes the tick timer.
/////////////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Section: Tick Delay
 *********************************************************************/
#define SAMPLE_PERIOD       500 // us
#define TICK_PERIOD			(GetPeripheralClock() * SAMPLE_PERIOD) / 4000000

/* */
void TickInit(void)
{

    // Initialize Timer4
    #ifdef __PIC32MX__
    OpenTimer3(T3_ON | T3_PS_1_8, TICK_PERIOD);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_4);
    #else
    TMR3 = 0;
    PR3 = TICK_PERIOD;
    IFS0bits.T3IF = 0;  //Clear flag
    IEC0bits.T3IE = 1;  //Enable interrupt
    T3CONbits.TON = 1;  //Run timer
    #endif
    
}

volatile UINT32 T2_count;

void __attribute__ ((interrupt,no_auto_psv)) _T2Interrupt (void)
{
    T2_count++;
    if(1000000 == T2_count)
    {
        Home_Screen_Time_Up++;
        IEC0bits.T2IE = 0;      /* Disable the Timer2 interrupt */
        T2CONbits.TON = 0;      /* Disable timer2 */
        IFS0bits.T2IF = 0;      /* Clear Timer interrupt flag */
    }
}

void Start_Home_Screen_Time(void)
{
    Home_Screen_Time_Up++;
    T2_count = 0;

    TMR2 = 0;
    PR2 = 0x9C40;
    T2CONbits.TCKPS = 3;
    T2CONbits.TON = 1;

    IFS0bits.T2IF = 0;
    IPC1bits.T2IP = 0b001;
    IEC0bits.T2IE = 1;

}
/////////////////////////////////////////////////////////////////////////////
// Function: InitializeBoard()
// Input: none
// Output: none
// Overview: Initializes the hardware components including the PIC device
//           used.
/////////////////////////////////////////////////////////////////////////////

void InitializeBoard(void)
{

    #if defined (PIC24FJ256DA210_DEV_BOARD) && defined(USE_KEYBOARD)

    CLKDIVbits.CPDIV = 0;   // 8MHz input, 32MHz System Clock

     ANSA = 0x0000;
     ANSB = 0x0020;		// RB5 as potentiometer input
     ANSC = 0x0010;		// RC4 as touch screen X+, RC14 as external source of secondary oscillator
     ANSD = 0x0000;
     ANSE = 0x0000;		// RE9 used as S2
     ANSF = 0x0000;
     ANSG = 0x0080;		// RG8 used as S1, RG7 as touch screen Y+

     U1CNFG2bits.UTRDIS = 1;

    #else
    
        /////////////////////////////////////////////////////////////////////////////
        // ADC Explorer 16 Development Board Errata (work around 2)
        // RB15 should be output
        /////////////////////////////////////////////////////////////////////////////
        #ifndef MEB_BOARD
            LATBbits.LATB15 = 0;
            TRISBbits.TRISB15 = 0;
        #endif
    #endif


        #ifdef MEB_BOARD
            CPLDInitialize();
            CPLDSetGraphicsConfiguration(GRAPHICS_HW_CONFIG);
            CPLDSetSPIFlashConfiguration(SPI_FLASH_CHANNEL);
        #endif // #ifdef MEB_BOARD

    #if defined(__dsPIC33F__) || defined(__PIC24H__) || defined(__dsPIC33E__) || defined(__PIC24E__)

        // Configure Oscillator to operate the device at 40Mhz
        // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
        #if defined(__dsPIC33E__) || defined(__PIC24E__)
			//Fosc = 8M * 60/(2*2) = 120MHz for 8M input clock
			PLLFBD = 58;    			// M=60         
		#else
        	// Fosc= 8M*40(2*2)=80Mhz for 8M input clock
        	PLLFBD = 38;                    // M=40
        #endif
        CLKDIVbits.PLLPOST = 0;         // N1=2
        CLKDIVbits.PLLPRE = 0;          // N2=2
        OSCTUN = 0;                     // Tune FRC oscillator, if FRC is used
    
        // Disable Watch Dog Timer
        RCONbits.SWDTEN = 0;
    
        // Clock switching to incorporate PLL
        __builtin_write_OSCCONH(0x03);  // Initiate Clock Switch to Primary
    
        // Oscillator with PLL (NOSC=0b011)
        __builtin_write_OSCCONL(0x01);  // Start clock switching
        while(OSCCONbits.COSC != 0b011);
    
        // Wait for Clock switch to occur	
        // Wait for PLL to lock
        while(OSCCONbits.LOCK != 1)
        { };
 
       #if defined(__dsPIC33F__) || defined(__PIC24H__)
        // Set PMD0 pin functionality to digital
        AD1PCFGL = AD1PCFGL | 0x1000;

        #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)
            AD1PCFGLbits.PCFG6 = 1;
            AD1PCFGLbits.PCFG7 = 1;
            AD1PCFGLbits.PCFG8 = 1;
        #endif
        
        #elif defined(__dsPIC33E__) || defined(__PIC24E__)
            ANSELE = 0x00;
            ANSELDbits.ANSD6 = 0;

		    // Set all touch screen related pins to Analog mode.
	        ANSELBbits.ANSB11 = 1; 
        #endif

    #elif defined(__PIC32MX__)
        INTEnableSystemMultiVectoredInt();
        SYSTEMConfigPerformance(GetSystemClock());
    #endif // #if defined(__dsPIC33F__) || defined(__PIC24H__)
    

    #if defined (EXPLORER_16)
/************************************************************************
* For Explorer 16 RD12 is connected to EEPROM chip select.
* To prevent a conflict between this EEPROM and SST25 flash
* the chip select of the EEPROM SPI should be pulled up.
************************************************************************/
        // Set IOs directions for EEPROM SPI
        MCHP25LC256_CS_LAT = 1;			    // set initial CS value to 1 (not asserted)
    	MCHP25LC256_CS_TRIS = 0;			// set CS pin to output
	#endif // #if defined (EXPLORER_16)

    // Initialize graphics library and create default style scheme for GOL
    GOLInit();  
    
// Set the other chip selects to a known state
#ifdef MIKRO_BOARD
    // SD Card chip select
    LATGbits.LATG9 = 1;
    TRISGbits.TRISG9 = 0;
    
    // MP3 Codac
    // reset
    LATAbits.LATA5 = 0;
    TRISAbits.TRISA5 = 0;
    // chip select
    LATAbits.LATA2 = 1;
    TRISAbits.TRISA2 = 0;
    // chip select
    LATAbits.LATA3 = 1;
    TRISAbits.TRISA3 = 0;

    AD1PCFGbits.PCFG11 = 1;
    AD1PCFGbits.PCFG10 = 1;
#endif

    //The following are PIC device specific settings for the SPI channel
    //used. 
    
    //Set IOs directions for SST25 SPI
    #if defined (GFX_PICTAIL_V3) || defined (MEB_BOARD) || defined(GFX_PICTAIL_LCC) || defined(MIKRO_BOARD) || defined(GFX_PICTAIL_V3E)
	    
        SST25_CS_LAT = 1;
        SST25_CS_TRIS = 0;
  
        #ifndef __PIC32MX__
            SST25_SCK_TRIS = 0;
            SST25_SDO_TRIS = 0;
            SST25_SDI_TRIS = 1;
            #if defined(__PIC24FJ256GB210__) || defined(__dsPIC33E__) || defined(__PIC24E__)
            	SST25_SDI_ANS = 0;
    	    #endif
        #endif
    #elif defined (PIC24FJ256DA210_DEV_BOARD)
        SST25_CS_LAT = 1;
        SST25_CS_TRIS = 0;
    
        // Set the pins to be digital 
    	SST25_SDI_ANS = 0;
        SST25_SDO_ANS = 0;

        SST25_SCK_TRIS = 0;
        SST25_SDO_TRIS = 0;
        SST25_SDI_TRIS = 1;
        
	#endif

    // set the peripheral pin select for the PSI channel used
    #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)
        AD1PCFGL = 0xFFFF;
        RPOR9bits.RP18R = 11;                   // assign RP18 for SCK2
        RPOR8bits.RP16R = 10;                   // assign RP16 for SDO2
        RPINR22bits.SDI2R = 17;                 // assign RP17 for SDI2	
    #elif defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GA110__) || defined (__PIC24FJ256GB210__)
        __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
        RPOR10bits.RP21R = 11;                  // assign RP21 for SCK2
        RPOR9bits.RP19R = 10;                   // assign RP19 for SDO2
        RPINR22bits.SDI2R = 26;                 // assign RP26 for SDI2
        __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    #elif defined(__PIC24FJ256DA210__)

        __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    	#if (SST25_SPI_CHANNEL == 1)
    	    RPOR1bits.RP2R = 8;                 // assign RP2 for SCK1
    	    RPOR0bits.RP1R = 7;                 // assign RP1 for SDO1
    	    RPINR20bits.SDI1R = 0;              // assign RP0 for SDI1
        #elif (SST25_SPI_CHANNEL == 2)
            RPOR1bits.RP2R = 11;                // assign RP2 for SCK2
    	    RPOR0bits.RP1R = 10;                // assign RP1 for SDO2
    	    RPINR22bits.SDI2R = 0;              // assign RP0 for SDI2
    	#endif

        __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS

    #endif

	// initialize the Flash Memory driver
    FlashInit(&SPI_Init_Data);
   
    // initialize the timer that manages the tick counter
    TickInit(); 
                      
    // initialize the components for Resistive Touch Screen
    TouchInit(NVMWrite, NVMRead, NVMSectorErase, TOUCH_INIT_VALUES);   
                
    HardwareButtonInit();           	// Initialize the hardware buttons
    U1CNFG2bits.UTRDIS = 1;
    SPI_Initialize();

    GCPU_ACTIVE_DIR = 0;
    GCPU_ACTIVE = 1;    
}

#define BKLT_CNTRL_DIR  TRISDbits.TRISD0
#define BKLT_CNTRL      LATDbits.LATD0

#define SW_WAKE_DIR     TRISCbits.TRISC13
#define SW_WAKE         PORTCbits.RC13

void Wake_screen(void)
{
    Home_Screen_Time_Up = 0;
    GDDDemoCreateFirstScreen();
}
UINT spi_count = 0;
extern OBJ_HEADER  *_pGolObjects;
extern BOOL Screen_Off;
UINT delay_count;
extern WORD currentGDDDemoScreenIndex;
unsigned char Screen_done;
#include "DRV_SPI_SMCPU.h"
#include "Common_DAC_info.h"
#include "GDD_Resource.h"
extern BOOL Update_GLCD;
extern glcd_info_t GLCD_Info;
extern SMCPU_info_t SMCPU_sysinfo;
UINT var_count,Error_EF,Error_SF;
char var_buffer[60][40];
char var_buffer2[60][40];
char var_buffer_d[10][40];
void Update_Reset_Message();
BYTE Next,Hide_count =0;
extern UINT16 checksum_G;
void Update_GLCD_Data()
{
    if(IsObjUpdated(_pGolObjects))
    {
        return;
    }
    switch(Disp_Info.Screen)
    {
        case HOME:
            if(Active_data == 0)
            {
                break;
            }
            if(Home_Screen_Time_Up == 2)
            {
                //To do, go to screen based on the configuration
                 //GDDDemoGoToScreen(C_2DP);
                 //Disp_Info.Screen = C_2DP;
                 Home_Screen_Time_Up++;
                 GLCD_Info.State = GLCD_SS_WAIT;
                 Disp_Info.SPI_state = SPI_IDLE;
                 Disp_Info.CPU_Type = D_CPU1;
            }
            else if(Home_Screen_Time_Up==0)
            {
                //Start_Home_Screen_Time();
                Home_Screen_Time_Up++;
            }
//            if(0)
            if(Disp_Info.SPI_state == SPI_DONE)
            {
#if DEBUG_SCREEN == 1
                if(1)
                {
                    GDDDemoGoToScreen(RESET);
                    Disp_Info.Screen = RESET;
                    Disp_Info.Prv_Screen = Disp_Info.Screen;
#else
                if(Disp_Info.MessageID == WAITING_FOR_RESET)
                {
        
                    Disp_Info.Prv_Screen = Disp_Info.Screen;
                    GDDDemoGoToScreen(C_WAIT_FOR_RESET);
                    Disp_Info.Screen = C_WAIT_FOR_RESET;
#endif
                }
                else if(Disp_Info.MessageID == NO_DATA)
                {

                }
                else if(Disp_Info.MessageID == RESET_APPLIED || Disp_Info.MessageID == WAITING_FOR_RESET){
                    Disp_Info.Prv_Screen = Disp_Info.Screen;
                    if(Disp_Info.Unit_Type == UT_SF || Disp_Info.Unit_Type == UT_EF)
                    {
                         GDDDemoGoToScreen(C_2DP);
                         Disp_Info.Screen = C_2DP;
                    }
                    if(Disp_Info.Unit_Type == UT_DE)
                    {
                         GDDDemoGoToScreen(C_DE);
                         Disp_Info.Screen = C_DE;
                    }
                    if(Disp_Info.Unit_Type == UT_CF || Disp_Info.Unit_Type == UT_3D_SF || Disp_Info.Unit_Type == UT_3D_EF)
                    {
                         GDDDemoGoToScreen(C_3D2S);
                         Disp_Info.Screen = C_3D2S;
                    }
                    if(Disp_Info.Unit_Type == UT_LCWS || Disp_Info.Unit_Type == UT_LCWS_DL)
                    {
                         GDDDemoGoToScreen(C_LCWS);
                         Disp_Info.Screen = C_LCWS;
                    }
                    if(Disp_Info.Unit_Type == UT_D3A || Disp_Info.Unit_Type == UT_D3B || Disp_Info.Unit_Type == UT_D3C)
                    {
                         GDDDemoGoToScreen(C_3DPAS);
                         Disp_Info.Screen = C_3DPAS;
                    }
                }
                Disp_Info.SPI_state = SPI_GOOD;
            }
            break;
        case C_3D2S:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Update_Time_Date();
                    if(Disp_Info.Unit_Type == UT_CF)
                    {
                        Update_C_3DP2S_Screen();
                    }
                    if(Disp_Info.Unit_Type == UT_3D_SF)
                    {
                        Update_C_3DP_SF_Screen();
                    }
                    if(Disp_Info.Unit_Type == UT_3D_EF)
                    {
                        Update_C_3DP_EF_Screen();
                    }
                }
                Update_GLCD = 0;
            }
            break;
        case C_DE:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Update_Time_Date();
                    Update_UT_DE_Screen();
                }
                Update_GLCD = 0;
            }
            break;
        case C_2DP:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Update_Time_Date();
                    Update_C_2DP_Screen();
                }
                Update_GLCD = 0;
            }
            break;
        case C_3DPAS:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Update_Time_Date();
                    Update_3D1S_Screen();
                }
                Update_GLCD = 0;
            }
            break;
        case C_4D1S:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Update_Time_Date();
                    Update_4D1S_Screen();
                }
                Update_GLCD = 0;
            }
            break;
        case C_LCWS:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                    StSetText(((STATICTEXT*) (GOLFindObject(STE_224))), (XCHAR*)"Conf-TWS");
                    SetState(((STATICTEXT*) (GOLFindObject(STE_224))), ST_DRAW);
                    if(Disp_Info.Unit_Type == UT_LCWS)
                    {
                        StSetText(((STATICTEXT*) (GOLFindObject(STE_225))), (XCHAR*)"TWS-SL");
                        SetState(((STATICTEXT*) (GOLFindObject(STE_225))), ST_DRAW);
                    }
                    else
                    {
                        StSetText(((STATICTEXT*) (GOLFindObject(STE_225))), (XCHAR*)"TWS-DL");
                        SetState(((STATICTEXT*) (GOLFindObject(STE_225))), ST_DRAW);

                        RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_390)), (XCHAR*)"C");
                        SetState(((RADIOBUTTON*) (GOLFindObject(RDB_390))), ST_DRAW);

                        RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_391)), (XCHAR*)"B");
                        SetState(((RADIOBUTTON*) (GOLFindObject(RDB_391))), ST_DRAW);
                    }
                    StSetText(((STATICTEXT*) (GOLFindObject(STE_410))), (XCHAR*)"TWS Status:");
                    SetState(((STATICTEXT*) (GOLFindObject(STE_410))), ST_DRAW);
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
//                    StSetText(((STATICTEXT*) (GOLFindObject(STE_416))), (XCHAR*)"SPI ERROR");
//                    SetState(((STATICTEXT*) (GOLFindObject(STE_416))), ST_DRAW);
                }
                else
                {
                    Update_Time_Date();
                    Update_C_LCWS_Screen();
                }
                Update_GLCD = 0;
            }
            break;
        case SMCPU:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Update_Time_Date();
                    if(Disp_Info.Unit_Type == UT_DE)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"DE");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-DE");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_SF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"SF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-2DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_EF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"EF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-2DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_CF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"CF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-3DP");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_3D_SF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"3D-SF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-3DP");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);                        
                    }   
                    else if(Disp_Info.Unit_Type == UT_3D_EF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"EF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-3DP");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);                        
                    }
                    else if(Disp_Info.Unit_Type == UT_LCWS)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"TWS-SL");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-TWS");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_LCWS_DL)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"TWS-DL");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-TWS");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D3A)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit A");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-3DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D3B)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit B");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-3DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D3C)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit C");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-3DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4A)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit A");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-4DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4B)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit B");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-4DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4C)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit C");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-4DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4D)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_170))), (XCHAR*)"Unit D");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_170))), ST_DRAW);
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_169))), (XCHAR*)"CONF-4DP1S");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_169))), ST_DRAW);
                    }
//                        StSetText(((STATICTEXT*)(GOLFindObject(STE_178))), (XCHAR*)"Status:");
//                        SetState(((STATICTEXT*)(GOLFindObject(STE_178))), ST_DRAW);
                    Update_SMCPU_Message();
                }
                Update_GLCD = 0;
            }
            break;
        case RESET:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                sprintf(&var_buffer[36][0],"%04x\n",GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]);
                if(0)//GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=(MAX_G_PACKET_LEN - 3))
                {
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_252))), (XCHAR*)"SPI ERROR");
                    SetState(((STATICTEXT*)(GOLFindObject(STE_252))), ST_DRAW);
                }
                else
                {
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_252))), (XCHAR*)&var_buffer[36][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_252))), ST_DRAW);
                }
                Update_Time_Date();
                Update_Debug_blocks();
                Update_GLCD = 0;
            }
            break;
        case C_WAIT_FOR_RESET:
            if(Disp_Info.Disp_state == DRAWING)
            {
                         Disp_Info.Disp_state = DONE;
            }
            if(Update_GLCD)
            {
                if(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN - 3]!=0xAA)
                {
                }
                else
                {
                    Hide_count =0;
                    Update_Time_Date();
                    if(Disp_Info.Unit_Type == UT_DE)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"DE");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_SF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"SF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_EF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"EF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_CF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"CF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_3D_SF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"3D-SF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_3D_EF)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"3D-EF");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_LCWS)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"TWS-SL");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_LCWS_DL)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"TWS-DL");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D3A)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit A");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D3B)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit B");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D3C)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit C");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4A)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit A");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4B)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit B");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4C)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit C");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    else if(Disp_Info.Unit_Type == UT_D4D)
                    {
                        StSetText(((STATICTEXT*)(GOLFindObject(STE_491))), (XCHAR*)"Unit D");
                        SetState(((STATICTEXT*)(GOLFindObject(STE_491))), ST_DRAW);
                    }
                    Update_Reset_Message();
                }
                Update_GLCD = 0;
            }
            break;
        default:
            break;
    }
}
char var_buffer_r[7][40];
char var_count_temp;
unsigned char SF_error, EF_error;

void Update_3D1S_Screen()
{
    StSetText(((STATICTEXT*) (GOLFindObject(STE_205))), (XCHAR*) "CONF-3DP1S");
    SetState(((STATICTEXT*) (GOLFindObject(STE_205))), ST_DRAW);

    if(Packet_src != 1)
    {

        SetState(((BUTTON*) (GOLFindObject(BTN_610))),BTN_NOPANEL | BTN_HIDE);
    }
    //Entry Counts
    var_count = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
    sprintf(&var_buffer[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_338))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_338))), ST_DRAW);
    Wheel_3D1S.A_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
    sprintf(&var_buffer2[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_339))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_339))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
    sprintf(&var_buffer[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_340))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_340))), ST_DRAW);
    Wheel_3D1S.B_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);
    sprintf(&var_buffer2[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_341))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_341))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][26] + ((UINT)(GLCD_Info.CPU_Data[0][27])<<8);
    sprintf(&var_buffer[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_356))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_356))), ST_DRAW);
    Wheel_3D1S.C_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][26] + ((UINT)(GLCD_Info.CPU_Data[1][27])<<8);
    sprintf(&var_buffer2[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_357))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_357))), ST_DRAW);


    //Exit count
    var_count = GLCD_Info.CPU_Data[0][28] + ((UINT)(GLCD_Info.CPU_Data[0][29])<<8);
    sprintf(&var_buffer[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_467))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_467))), ST_DRAW);
    Wheel_3D1S.A_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][28] + ((UINT)(GLCD_Info.CPU_Data[1][29])<<8);
    sprintf(&var_buffer2[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_468))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_468))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
    sprintf(&var_buffer[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_469))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_469))), ST_DRAW);
    Wheel_3D1S.B_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);
    sprintf(&var_buffer2[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_470))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_470))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
    sprintf(&var_buffer[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_473))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_473))), ST_DRAW);
    Wheel_3D1S.C_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
    sprintf(&var_buffer2[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_474))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_474))), ST_DRAW);

    if(Disp_Info.Unit_Type == UT_D3A)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_206))), (XCHAR*) "Unit A");
    }
    else if(Disp_Info.Unit_Type == UT_D3B)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_206))), (XCHAR*) "Unit B");
    }
    else if(Disp_Info.Unit_Type == UT_D3C)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_206))), (XCHAR*) "Unit C");
    }
    SetState(((STATICTEXT*) (GOLFindObject(STE_206))), ST_DRAW);

    if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        // DP A
        if(Wheel_3D1S.A_ENTRY > Wheel_3D1S_SC.A_ENTRY)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_377))), (void*)&logo_small2_db2_inv);
            SetState(((PICTURE*)(GOLFindObject(PCB_377))), PICT_DRAW);            
        }
        else if(Wheel_3D1S.A_EXIT > Wheel_3D1S_SC.A_EXIT)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_377))), (void*)&logo_small2_db2);
            SetState(((PICTURE*)(GOLFindObject(PCB_377))), PICT_DRAW);            
        }
        else
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_377))), (void*)&No_Train);
            SetState(((PICTURE*)(GOLFindObject(PCB_377))), PICT_DRAW);
        }
        
        // DP B
        if(Wheel_3D1S.B_ENTRY > Wheel_3D1S_SC.B_ENTRY)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_378))), (void*)&logo_small2_db2);
            SetState(((PICTURE*)(GOLFindObject(PCB_378))), PICT_DRAW);            
        }
        else if(Wheel_3D1S.B_EXIT > Wheel_3D1S_SC.B_EXIT)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_378))), (void*)&logo_small2_db2_inv);
            SetState(((PICTURE*)(GOLFindObject(PCB_378))), PICT_DRAW);            
        }
        else
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_378))), (void*)&No_Train);
            SetState(((PICTURE*)(GOLFindObject(PCB_378))), PICT_DRAW);
        }
        
        // DP C
        if(Wheel_3D1S.C_ENTRY > Wheel_3D1S_SC.C_ENTRY)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_379))), (void*)&logo_small2_db2);
            SetState(((PICTURE*)(GOLFindObject(PCB_379))), PICT_DRAW);            
        }
        else if(Wheel_3D1S.C_EXIT > Wheel_3D1S_SC.C_EXIT)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_379))), (void*)&logo_small2_db2_inv);
            SetState(((PICTURE*)(GOLFindObject(PCB_379))), PICT_DRAW);            
        }
        else
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_379))), (void*)&No_Train);
            SetState(((PICTURE*)(GOLFindObject(PCB_379))), PICT_DRAW);
        }
    }
    else
    {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_377))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_377))), PICT_DRAW);
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_378))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_378))), PICT_DRAW);
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_379))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_379))), PICT_DRAW);
                    
                    Wheel_3D1S_SC.A_ENTRY = Wheel_3D1S.A_ENTRY;
                    Wheel_3D1S_SC.B_ENTRY = Wheel_3D1S.B_ENTRY;   
                    Wheel_3D1S_SC.C_ENTRY = Wheel_3D1S.C_ENTRY;
                    Wheel_3D1S_SC.A_EXIT = Wheel_3D1S.A_EXIT;
                    Wheel_3D1S_SC.B_EXIT = Wheel_3D1S.B_EXIT;   
                    Wheel_3D1S_SC.C_EXIT = Wheel_3D1S.C_EXIT;
    }

        var_count = GLCD_Info.CPU_Data[0][54];
        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(Wheel_3D1S.A_ENTRY==0 && Wheel_3D1S.B_ENTRY==0 && Wheel_3D1S.C_ENTRY==0 && Wheel_3D1S.A_EXIT==0 && Wheel_3D1S.B_EXIT==0 && Wheel_3D1S.C_EXIT==0)
                {
                    sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                    sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                }
                else
                {
                    sprintf(&var_buffer_r[0][0], "Pilot train in progress\n");
                    sprintf(&var_buffer_r[1][0], "Pilot train in progress\n");                    
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[1][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
//                sprintf(&var_buffer_r_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
//                sprintf(&var_buffer_r_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
//                sprintf(&var_buffer_r_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
//                sprintf(&var_buffer_r_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
//                sprintf(&var_buffer_r_r[3][0], "Waiting for pilot train\n");
                if(Wheel_3D1S.A_ENTRY==0 && Wheel_3D1S.B_ENTRY==0 && Wheel_3D1S.C_ENTRY==0 && Wheel_3D1S.A_EXIT==0 && Wheel_3D1S.B_EXIT==0 && Wheel_3D1S.C_EXIT==0)
                {
                    sprintf(&var_buffer_r[2][0], "Waiting for pilot train\n");
                }
                else
                {
                    sprintf(&var_buffer_r[2][0], "Pilot train in progress\n");                    
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
//                sprintf(&var_buffer_r_r[3][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[2][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
//                sprintf(&var_buffer_r_r[3][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[2][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
/*                if(var_count!=255)
                {
                    sprintf(&var_buffer_r_r[3][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r_r[3][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r_r[3][0],"System Failure\n");
                }
*/
                sprintf(&var_buffer_r[2][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
//                sprintf(&var_buffer_r_r[3][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
//                sprintf(&var_buffer_r_r[3][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
//                sprintf(&var_buffer_r_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
//                sprintf(&var_buffer_r_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            default:
//                sprintf(&var_buffer_r_r[3][0], "\n");
                sprintf(&var_buffer_r[2][0], "\n");
                break;
        }

        if(Disp_Info.Unit_Type == UT_D3A)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_472))), (XCHAR*) & var_buffer_r[1][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_472))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_471))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_471))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_376))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_376))), ST_DRAW);
        }
        else if(Disp_Info.Unit_Type == UT_D3B)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_472))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_472))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_471))), (XCHAR*) & var_buffer_r[1][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_471))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_376))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_376))), ST_DRAW);
        }
        else if(Disp_Info.Unit_Type == UT_D3C)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_472))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_472))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_471))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_471))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_376))), (XCHAR*) & var_buffer_r[1][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_376))), ST_DRAW);
        }


        sprintf(&var_buffer[6][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
            StSetText(((STATICTEXT*) (GOLFindObject(STE_203))), (XCHAR*) & var_buffer[6][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_203))), ST_DRAW);

                    sprintf(&var_buffer[7][0],"%d\n",Disp_Info.Train_Speed);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_378))), (XCHAR*)&var_buffer[7][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_378))), ST_DRAW);
}

void Update_4D1S_Screen()
{
    StSetText(((STATICTEXT*) (GOLFindObject(STE_533))), (XCHAR*) "CONF-4DP1S");
    SetState(((STATICTEXT*) (GOLFindObject(STE_533))), ST_DRAW);

    if(Packet_src != 1)
    {
        SetState(((BUTTON*) (GOLFindObject(BTN_609))),BTN_NOPANEL | BTN_HIDE);
    }//Entry Counts
    var_count = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
    sprintf(&var_buffer[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_584))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_584))), ST_DRAW);
    Wheel_4D1S.A_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
    sprintf(&var_buffer2[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_585))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_585))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
    sprintf(&var_buffer[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_586))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_586))), ST_DRAW);
    Wheel_4D1S.B_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);
    sprintf(&var_buffer2[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_587))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_587))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][26] + ((UINT)(GLCD_Info.CPU_Data[0][27])<<8);
    sprintf(&var_buffer[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_588))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_588))), ST_DRAW);
    Wheel_4D1S.C_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][26] + ((UINT)(GLCD_Info.CPU_Data[1][27])<<8);
    sprintf(&var_buffer2[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_589))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_589))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][28] + ((UINT)(GLCD_Info.CPU_Data[0][29])<<8);
    sprintf(&var_buffer[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_590))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_590))), ST_DRAW);
    Wheel_4D1S.D_ENTRY =  var_count;
    var_count = GLCD_Info.CPU_Data[1][28] + ((UINT)(GLCD_Info.CPU_Data[1][29])<<8);
    sprintf(&var_buffer2[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_591))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_591))), ST_DRAW);

    //Exit count

    var_count = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
    sprintf(&var_buffer[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_596))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_596))), ST_DRAW);
    Wheel_4D1S.A_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);
    sprintf(&var_buffer2[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_597))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_597))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
    sprintf(&var_buffer[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_598))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_598))), ST_DRAW);
    Wheel_4D1S.B_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
    sprintf(&var_buffer2[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_599))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_599))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][34] + ((UINT)(GLCD_Info.CPU_Data[0][35])<<8);
    sprintf(&var_buffer[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_600))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_600))), ST_DRAW);
    Wheel_4D1S.C_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][34] + ((UINT)(GLCD_Info.CPU_Data[1][35])<<8);
    sprintf(&var_buffer2[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_601))), (XCHAR*)&var_buffer2[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_601))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][36] + ((UINT)(GLCD_Info.CPU_Data[0][37])<<8);
    sprintf(&var_buffer[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_602))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_602))), ST_DRAW);
    Wheel_4D1S.D_EXIT =  var_count;
    var_count = GLCD_Info.CPU_Data[1][36] + ((UINT)(GLCD_Info.CPU_Data[1][37])<<8);
    sprintf(&var_buffer2[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_603))), (XCHAR*)&var_buffer2[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_603))), ST_DRAW);

    if(Disp_Info.Unit_Type == UT_D4A)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_534))), (XCHAR*) "Unit A");
    }
    else if(Disp_Info.Unit_Type == UT_D4B)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_534))), (XCHAR*) "Unit B");
    }
    else if(Disp_Info.Unit_Type == UT_D4C)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_534))), (XCHAR*) "Unit C");
    }
	else if(Disp_Info.Unit_Type == UT_D4D)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_534))), (XCHAR*) "Unit D");
    }
    SetState(((STATICTEXT*) (GOLFindObject(STE_534))), ST_DRAW);
    unsigned char A_Dir = 0,B_Dir = 0,C_Dir = 0,D_Dir = 0;//0-no train, 1-entry, 2-exit
    if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        // DP A
        if(Wheel_4D1S.A_ENTRY > Wheel_4D1S_SC.A_ENTRY)
        {
            A_Dir = 1;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_607))), (void*)&logo_small2_db2_inv);
//            SetState(((PICTURE*)(GOLFindObject(PCB_607))), PICT_DRAW);
        }
        else if(Wheel_4D1S.A_EXIT > Wheel_4D1S_SC.A_EXIT)
        {
            A_Dir = 2;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_607))), (void*)&logo_small2_db2);
//            SetState(((PICTURE*)(GOLFindObject(PCB_607))), PICT_DRAW);
        }
        else
        {
            A_Dir = 0;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_607))), (void*)&No_Train);
//            SetState(((PICTURE*)(GOLFindObject(PCB_607))), PICT_DRAW);
        }

        // DP B
        if(Wheel_4D1S.B_ENTRY > Wheel_4D1S_SC.B_ENTRY)
        {
            B_Dir = 1;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_609))), (void*)&logo_small2_db2);
//            SetState(((PICTURE*)(GOLFindObject(PCB_609))), PICT_DRAW);
        }
        else if(Wheel_4D1S.B_EXIT > Wheel_4D1S_SC.B_EXIT)
        {
            B_Dir = 2;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_609))), (void*)&logo_small2_db2_inv);
//            SetState(((PICTURE*)(GOLFindObject(PCB_609))), PICT_DRAW);
        }
        else
        {
            B_Dir = 0;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_609))), (void*)&No_Train);
//            SetState(((PICTURE*)(GOLFindObject(PCB_609))), PICT_DRAW);
        }

        // DP C
        if(Wheel_4D1S.C_ENTRY > Wheel_4D1S_SC.C_ENTRY)
        {
            C_Dir = 1;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_611))), (void*)&logo_small2_db2);
//            SetState(((PICTURE*)(GOLFindObject(PCB_611))), PICT_DRAW);
        }
        else if(Wheel_4D1S.C_EXIT > Wheel_4D1S_SC.C_EXIT)
        {
            C_Dir = 2;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_611))), (void*)&logo_small2_db2_inv);
//            SetState(((PICTURE*)(GOLFindObject(PCB_611))), PICT_DRAW);
        }
        else
        {
            C_Dir = 0;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_611))), (void*)&No_Train);
//            SetState(((PICTURE*)(GOLFindObject(PCB_611))), PICT_DRAW);
        }

		// DP D
        if(Wheel_4D1S.D_ENTRY > Wheel_4D1S_SC.D_ENTRY)
        {
            D_Dir = 1;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_613))), (void*)&logo_small2_db2);
//            SetState(((PICTURE*)(GOLFindObject(PCB_613))), PICT_DRAW);
        }
        else if(Wheel_4D1S.D_EXIT > Wheel_4D1S_SC.D_EXIT)
        {
            D_Dir = 2;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_613))), (void*)&logo_small2_db2_inv);
//            SetState(((PICTURE*)(GOLFindObject(PCB_613))), PICT_DRAW);
        }
        else
        {
            D_Dir = 0;
//            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_613))), (void*)&No_Train);
//            SetState(((PICTURE*)(GOLFindObject(PCB_613))), PICT_DRAW);
        }
    }
    else
    {
//                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_607))), (void*)&No_Train);
//                    SetState(((PICTURE*)(GOLFindObject(PCB_607))), PICT_DRAW);
//                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_609))), (void*)&No_Train);
//                    SetState(((PICTURE*)(GOLFindObject(PCB_609))), PICT_DRAW);
//                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_611))), (void*)&No_Train);
//                    SetState(((PICTURE*)(GOLFindObject(PCB_611))), PICT_DRAW);
//                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_613))), (void*)&No_Train);
//                    SetState(((PICTURE*)(GOLFindObject(PCB_613))), PICT_DRAW);

                    Wheel_4D1S_SC.A_ENTRY = Wheel_4D1S.A_ENTRY;
                    Wheel_4D1S_SC.B_ENTRY = Wheel_4D1S.B_ENTRY;
                    Wheel_4D1S_SC.C_ENTRY = Wheel_4D1S.C_ENTRY;
                    Wheel_4D1S_SC.D_ENTRY = Wheel_4D1S.D_ENTRY;
                    Wheel_4D1S_SC.A_EXIT = Wheel_4D1S.A_EXIT;
                    Wheel_4D1S_SC.B_EXIT = Wheel_4D1S.B_EXIT;
                    Wheel_4D1S_SC.C_EXIT = Wheel_4D1S.C_EXIT;
                    Wheel_4D1S_SC.D_EXIT = Wheel_4D1S.D_EXIT;
    }

//    SetState(((RADIOBUTTON*) (GOLFindObject(RDB_608))), ST_HIDE);
//    SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_HIDE);
//    SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_HIDE);
//    SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_HIDE);

    if(A_Dir == 0 && B_Dir == 0 && C_Dir == 0 && D_Dir == 0)
    {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_611))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_611))), PICT_DRAW);
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_613))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_613))), PICT_DRAW);
    }
    else
    {
        if(A_Dir == 1 || B_Dir == 1 || C_Dir == 1 || D_Dir == 1)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_611))), (void*)&logo_small2_db2_inv);
            SetState(((PICTURE*)(GOLFindObject(PCB_611))), PICT_DRAW);
        }
        if(A_Dir == 2 || B_Dir == 2 || C_Dir == 2 || D_Dir == 2)
        {
            PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_613))), (void*)&logo_small2_db2_inv);
            SetState(((PICTURE*)(GOLFindObject(PCB_613))), PICT_DRAW);
        }

        if(A_Dir == 1)
        {
            RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_608)), (XCHAR*)"A");
            SetState(((RADIOBUTTON*) (GOLFindObject(RDB_608))), ST_DRAW);

            if(B_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_610)), (XCHAR*)"B");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_HIDE);
            }
            if(C_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_612)), (XCHAR*)"C");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_HIDE);
            }
            if(D_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_614)), (XCHAR*)"D");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_HIDE);
            }
        }
        if(B_Dir == 1)
        {
            RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_608)), (XCHAR*)"B");
            SetState(((RADIOBUTTON*) (GOLFindObject(RDB_608))), ST_DRAW);

            if(A_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_610)), (XCHAR*)"A");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_HIDE);
            }

            if(C_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_612)), (XCHAR*)"C");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_HIDE);
            }
            if(D_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_614)), (XCHAR*)"D");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_HIDE);
            }
        }
        if(C_Dir == 1)
        {
            RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_608)), (XCHAR*)"C");
            SetState(((RADIOBUTTON*) (GOLFindObject(RDB_608))), ST_DRAW);

            if(A_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_610)), (XCHAR*)"A");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_HIDE);
            }
            if(B_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_612)), (XCHAR*)"B");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_DRAW);
            }else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_HIDE);
            }
            if(D_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_614)), (XCHAR*)"D");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_HIDE);
            }
        }
        if(D_Dir == 1)
        {
            RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_608)), (XCHAR*)"D");
            SetState(((RADIOBUTTON*) (GOLFindObject(RDB_608))), ST_DRAW);

            if(A_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_610)), (XCHAR*)"A");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_610))), ST_HIDE);
            }
            if(B_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_612)), (XCHAR*)"B");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_612))), ST_HIDE);
            }
            if(C_Dir == 2)
            {
                RbSetText((RADIOBUTTON*) (GOLFindObject(RDB_614)), (XCHAR*)"C");
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_DRAW);
            }
            else
            {
                SetState(((RADIOBUTTON*) (GOLFindObject(RDB_614))), ST_HIDE);
            }
        }

    }
        var_count = GLCD_Info.CPU_Data[0][54];
        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(Wheel_4D1S.A_ENTRY==0 && Wheel_4D1S.B_ENTRY==0 && Wheel_4D1S.C_ENTRY==0 && Wheel_4D1S.D_ENTRY==0 && Wheel_4D1S.A_EXIT==0 && Wheel_4D1S.B_EXIT==0 && Wheel_4D1S.C_EXIT==0 && Wheel_4D1S.D_EXIT==0)
                {
                    sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                    sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                }
                else
                {
                    sprintf(&var_buffer_r[0][0], "Pilot train in progress\n");
                    sprintf(&var_buffer_r[1][0], "Pilot train in progress\n");                                        
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[1][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }

            StSetText(((STATICTEXT*) (GOLFindObject(STE_605))), (XCHAR*) & var_buffer_r[1][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_605))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_564))), (XCHAR*) & var_buffer_r[0][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_564))), ST_DRAW);



            sprintf(&var_buffer[8][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
            StSetText(((STATICTEXT*) (GOLFindObject(STE_528))), (XCHAR*) & var_buffer[8][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_528))), ST_DRAW);

            sprintf(&var_buffer[9][0],"%d\n",Disp_Info.Train_Speed);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_616))), (XCHAR*)&var_buffer[9][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_616))), ST_DRAW);
}
extern wordtype_t COM_DS_pkt_error_cnt, COM_US_pkt_error_cnt;
void Update_SMCPU_Message(void)
{
        StSetText(((STATICTEXT*)(GOLFindObject(STE_246))), SMCPU_sysinfo.Start_Date);
        SetState(((STATICTEXT*)(GOLFindObject(STE_246))), ST_DRAW);

        StSetText(((STATICTEXT*)(GOLFindObject(STE_247))), SMCPU_sysinfo.End_Date);
        SetState(((STATICTEXT*)(GOLFindObject(STE_247))), ST_DRAW);

        StSetText(((STATICTEXT*)(GOLFindObject(STE_248))), SMCPU_sysinfo.Number_of_Event_String);
        SetState(((STATICTEXT*)(GOLFindObject(STE_248))), ST_DRAW);

        sprintf(&var_buffer[4][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
        StSetText(((STATICTEXT*) (GOLFindObject(STE_167))), (XCHAR*) & var_buffer[4][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_167))), ST_DRAW);

        sprintf(&var_buffer[5][0], "%u\n",COM_US_pkt_error_cnt.Word);
        StSetText(((STATICTEXT*)(GOLFindObject(STE_612))), &var_buffer[5][0]);
        SetState(((STATICTEXT*)(GOLFindObject(STE_612))), ST_DRAW);

        sprintf(&var_buffer[6][0], "%u\n",COM_DS_pkt_error_cnt.Word);
        StSetText(((STATICTEXT*)(GOLFindObject(STE_613))), &var_buffer[6][0]);
        SetState(((STATICTEXT*)(GOLFindObject(STE_613))), ST_DRAW);
        //error number
/*                if(Disp_Info.Unit_Type == UT_SF)
                {
		            var_count = GLCD_Info.CPU_Data[0][54];
					Error_SF = GLCD_Info.CPU_Data[0][11];
					Error_EF = GLCD_Info.CPU_Data[0][12];
                    switch(Disp_Info.DS_mode)
					{
						case WAITING_FOR_RESET_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[0][0], "Reset Applied\n");
							break;
						case RESET_APPLIED_AT_REMOTE_UNIT:
							sprintf(&var_buffer[0][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "Reset Applied\n");
							break;
						case SECTION_WAIT_FOR_PILOT_TRAIN:
							sprintf(&var_buffer[0][0], "Waiting for pilot train\n");
							break;
						case SECTION_CLEAR_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "SECTION CLEAR\n");
							break;
						case SECTION_OCCUPIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "SECTION OCCUPIED\n");
							break;
						case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
							sprintf(&var_buffer[0][0], "Waiting for Reset\n");
							break;
						case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
							if(var_count!=255)
							{
								sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[0][0], "Reset Applied\n");
							}
							break;
						case SECTION_ERROR_AT_BOTH_UNITS:
                                                        if(var_count!= 255)
                                                        {
                                                            sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                                                        }
                                                        else
                                                        {
                                                            sprintf(&var_buffer[0][0]," \n");
                                                        }
							break;
						case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[0][0],"Reset Applied\n");
							break;
						case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
							if(Error_SF!=0)
							{
								sprintf(&var_buffer[0][0],"%d %s\n",Error_SF - 1,&Error_message[Error_SF - 1][0]);
							}
							else
							{
								sprintf(&var_buffer[0][0],"Waiting for Reset\n");
							}
							break;
						default:
							sprintf(&var_buffer[0][0], "\n");
							break;
					}
                }
                if(Disp_Info.Unit_Type == UT_EF)
                {
		            var_count = GLCD_Info.CPU_Data[0][54];
					Error_SF = GLCD_Info.CPU_Data[0][11];
					Error_EF = GLCD_Info.CPU_Data[0][12];
					switch(Disp_Info.US_mode)
					{
						case WAITING_FOR_RESET_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[0][0], "Reset Applied\n");
							break;
						case RESET_APPLIED_AT_REMOTE_UNIT:
							sprintf(&var_buffer[0][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "Reset Applied\n");
							break;
						case SECTION_WAIT_FOR_PILOT_TRAIN:
							sprintf(&var_buffer[0][0], "Waiting for pilot train\n");
							break;
						case SECTION_CLEAR_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "SECTION CLEAR\n");
							break;
						case SECTION_OCCUPIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[0][0], "SECTION OCCUPIED\n");
							break;
						case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
							sprintf(&var_buffer[0][0], "Waiting for Reset\n");
							break;
						case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
							if(var_count!=255)
							{
								sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[0][0], "Reset Applied\n");
							}
							break;
						case SECTION_ERROR_AT_BOTH_UNITS:
                                                        if(var_count!=255)
                                                        {
                                                            sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                                                        }
                                                        else
                                                        {
                                                            sprintf(&var_buffer[0][0]," \n");
                                                        }
							break;
						case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[0][0],"Reset Applied\n");
							break;
						case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
							if(Error_EF!=0)
							{
								sprintf(&var_buffer[0][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF-1][0]);
							}
							else
							{
								sprintf(&var_buffer[0][0],"Waiting for Reset\n");
							}
							break;
						default:
							sprintf(&var_buffer[0][0], "\n");
							break;
					}
				}

				    if(Disp_Info.Unit_Type == UT_CF)
	{
		var_count = GLCD_Info.CPU_Data[0][54];
		SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer[0][0],"System Failure\n");
                }
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer[0][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
				if(var_count!=255)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer[0][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer[0][0], "\n");
                break;
        }
	}

            if(Disp_Info.Unit_Type == UT_DE)
	{
			var_count = GLCD_Info.CPU_Data[0][54];
            Error_EF = GLCD_Info.CPU_Data[0][11];
		switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                if(Error_EF!=0)
                    sprintf(&var_buffer[0][0],"%d %s\n",Error_EF-1,&Error_message[Error_EF-1][0]);
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                if(var_count != 255)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(Error_EF!=0)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF - 1][0]);
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer[0][0], "\n");
                break;
        }
	}

            if(Disp_Info.Unit_Type == UT_LCWS)
	{
		var_count = GLCD_Info.CPU_Data[0][54];
        Error_EF = GLCD_Info.CPU_Data[0][11];
		switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                if(Error_EF!=0)
                    sprintf(&var_buffer[0][0],"%d %s\n",Error_EF-1,&Error_message[Error_EF-1][0]);
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "255 SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "255 SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                if(var_count != 255)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(Error_EF!=0)
                {
                    sprintf(&var_buffer[0][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF - 1][0]);
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer[0][0], "\n");
                break;
        }
	}

        StSetText(((STATICTEXT*)(GOLFindObject(STE_184))), (XCHAR*)&var_buffer[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_184))), ST_DRAW);*/

}

char reset_message[15][40]=
{
    "None",
    "Waiting For Reset",
    "Reset Applied at Local Unit",
    "Reset Applied at Remote Unit",
    "Reset Applied at both units",
    "Waiting for pilot train",
    "SECTION CLEAR",
    "SECTION OCCUPIED",
    "Error at both units",
    "Error Reset Applied Local Unit",
    "Error Reset Applied Remote Unit",
    "Error Reset Wait at Local Unit",
    "Error Reset Wait at Remote Unit"
};


void Update_Reset_Message() {
    char *ds_buf_ptr, *us_buf_ptr, mode_val;
    unsigned int var_count;
    var_count = GLCD_Info.CPU_Data[0][54];
    if(Disp_Info.Unit_Type == UT_SF)
    {
        // DS is local for SF
        // US is Remote for SF
        ds_buf_ptr = &var_buffer_r[0][0];
        us_buf_ptr = &var_buffer_r[1][0];
        mode_val = Disp_Info.DS_mode;
    }
    else
    {
        // DS is Remote for EF
        // US is Local for EF
        ds_buf_ptr = &var_buffer_r[1][0];
        us_buf_ptr = &var_buffer_r[0][0];
        mode_val = Disp_Info.US_mode;
    }

    if(Disp_Info.Unit_Type == UT_SF || Disp_Info.Unit_Type == UT_EF)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_490))), (XCHAR*) "CONF-2DP1S");
        SetState(((STATICTEXT*) (GOLFindObject(STE_490))), ST_DRAW);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);
        SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
        SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
        SetState(((STATICTEXT*)(GOLFindObject(STE_511))), ST_DRAW);

        switch(mode_val)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[1][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_495))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_511))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);
    }
    if(Disp_Info.Unit_Type == UT_CF || Disp_Info.Unit_Type == UT_3D_SF || Disp_Info.Unit_Type == UT_3D_EF)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_490))), (XCHAR*) "CONF-3DP");
        SetState(((STATICTEXT*) (GOLFindObject(STE_490))), ST_DRAW);

        if(Disp_Info.Unit_Type == UT_CF)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_501))), (XCHAR*) "SF Status:");
            SetState(((STATICTEXT*)(GOLFindObject(STE_501))), ST_DRAW);
            SetState(((STATICTEXT*)(GOLFindObject(STE_502))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "S-CF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "E-CF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_417))), (XCHAR*) "EF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_417))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
        }
        if(Disp_Info.Unit_Type == UT_3D_SF)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_501))), (XCHAR*) "EF Status:");
            SetState(((STATICTEXT*)(GOLFindObject(STE_501))), ST_DRAW);
            SetState(((STATICTEXT*)(GOLFindObject(STE_502))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "E-SF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "C-SF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_417))), (XCHAR*) "CF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_417))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
        }
        if(Disp_Info.Unit_Type == UT_3D_EF)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_501))), (XCHAR*) "CF Status:");
            SetState(((STATICTEXT*)(GOLFindObject(STE_501))), ST_DRAW);
            SetState(((STATICTEXT*)(GOLFindObject(STE_502))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "C-EF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "S-EF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_417))), (XCHAR*) "SF Status:");
            SetState(((STATICTEXT*) (GOLFindObject(STE_417))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
        }
        
        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[1][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[3][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[2][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[2][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[2][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[3][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[2][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[3][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[3][0], "\n");
                sprintf(&var_buffer_r[2][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_502))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_502))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_495))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_511))), (XCHAR*) & var_buffer_r[2][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_418))), (XCHAR*) & var_buffer_r[3][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
    }
    if(Disp_Info.Unit_Type == UT_D3A || Disp_Info.Unit_Type == UT_D3B || Disp_Info.Unit_Type == UT_D3C)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_490))), (XCHAR*) "CONF-3DP1S");
        SetState(((STATICTEXT*) (GOLFindObject(STE_490))), ST_DRAW);
        if(Disp_Info.Unit_Type == UT_D3A)
        {
            
            StSetText(((STATICTEXT*) (GOLFindObject(STE_501))), (XCHAR*) "A--B  Status:");//0 Local
            SetState(((STATICTEXT*)(GOLFindObject(STE_501))), ST_DRAW);
            SetState(((STATICTEXT*)(GOLFindObject(STE_502))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "B  Status:");//1
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "A--C  Status:");//2
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_417))), (XCHAR*) "C  Status:");//3
            SetState(((STATICTEXT*) (GOLFindObject(STE_417))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
        }
        else if(Disp_Info.Unit_Type == UT_D3B)
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_501))), (XCHAR*) "B--C  Status:");//0 Local
            SetState(((STATICTEXT*)(GOLFindObject(STE_501))), ST_DRAW);
            SetState(((STATICTEXT*)(GOLFindObject(STE_502))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "C  Status:");//1
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "B--A  Status:");//2
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_417))), (XCHAR*) "A  Status:");//3
            SetState(((STATICTEXT*) (GOLFindObject(STE_417))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
            
        }
        else
        {
            StSetText(((STATICTEXT*) (GOLFindObject(STE_501))), (XCHAR*) "C--A  Status:");//0 Local
            SetState(((STATICTEXT*)(GOLFindObject(STE_501))), ST_DRAW);
            SetState(((STATICTEXT*)(GOLFindObject(STE_502))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "A  Status:");//1
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "C--B  Status:");//2
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_417))), (XCHAR*) "B  Status:");//3
            SetState(((STATICTEXT*) (GOLFindObject(STE_417))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);
            
        }
        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[1][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[1][0], "\n");
                sprintf(&var_buffer_r[0][0], "\n");
                break;
        }
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[3][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[2][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[2][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[2][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[3][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[2][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[3][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[3][0], "\n");
                sprintf(&var_buffer_r[2][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_502))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_502))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_495))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_511))), (XCHAR*) & var_buffer_r[2][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_418))), (XCHAR*) & var_buffer_r[3][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_418))), ST_DRAW);

        
        
    }
        if(Disp_Info.Unit_Type == UT_D4A || Disp_Info.Unit_Type == UT_D4B || Disp_Info.Unit_Type == UT_D4C || Disp_Info.Unit_Type == UT_D4D)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_490))), (XCHAR*) "CONF-4DP1S");
        SetState(((STATICTEXT*) (GOLFindObject(STE_490))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "Local Status:");//1
            SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

            StSetText(((STATICTEXT*) (GOLFindObject(STE_497))), (XCHAR*) "Remote Status:");//2
            SetState(((STATICTEXT*) (GOLFindObject(STE_497))), ST_DRAW);
            SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[1][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[1][0],"System Failure\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[0][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[1][0], "\n");
                sprintf(&var_buffer_r[0][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_495))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_511))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_511))), ST_DRAW);

    }

    if(Disp_Info.Unit_Type == UT_LCWS || Disp_Info.Unit_Type == UT_LCWS_DL)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_490))), (XCHAR*) "CONF-TWS");
        SetState(((STATICTEXT*) (GOLFindObject(STE_490))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "TWS Status:");
        SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);
                    //error number
            var_count = GLCD_Info.CPU_Data[0][54];
            Error_EF = GLCD_Info.CPU_Data[0][11];
    switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                if(Error_EF!=0)
                    sprintf(&var_buffer_r[0][0],"%d %s\n",Error_EF-1,&Error_message[Error_EF-1][0]);
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                if(var_count != 255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(Error_EF!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF - 1][0]);
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_495))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);
    }
    if(Disp_Info.Unit_Type == UT_DE)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_490))), (XCHAR*) "CONF-DE");
        SetState(((STATICTEXT*) (GOLFindObject(STE_490))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_496))), (XCHAR*) "DE Status:");
        SetState(((STATICTEXT*) (GOLFindObject(STE_496))), ST_DRAW);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);
                    //error number
            var_count = GLCD_Info.CPU_Data[0][54];
            Error_EF = GLCD_Info.CPU_Data[0][11];
    switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                if(Error_EF!=0)
                    sprintf(&var_buffer_r[0][0],"%d %s\n",Error_EF-1,&Error_message[Error_EF-1][0]);
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                if(var_count != 255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(Error_EF!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF - 1][0]);
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_495))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_495))), ST_DRAW);
    }
        sprintf(&var_buffer_r[4][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
        StSetText(((STATICTEXT*) (GOLFindObject(STE_488))), (XCHAR*) & var_buffer_r[4][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_488))), ST_DRAW);
}

void Count_message(void)
{
//                    StSetText(((STATICTEXT*)(GOLFindObject(STE_165))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
//                    SetState(((STATICTEXT*)(GOLFindObject(STE_165))), ST_DRAW);
//
//                    StSetText(((STATICTEXT*)(GOLFindObject(STE_164))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
//                    SetState(((STATICTEXT*)(GOLFindObject(STE_164))), ST_DRAW);

//                    var_count = GLCD_Info.CPU_Data[0][8] + ((UINT)(GLCD_Info.CPU_Data[0][9])<<8);
                  //  sprintf(&var_buffer[0][0],"%d\n",var_count);
//

}


void Com_message(void)
{

}

void Update_C_LCWS_Screen()
{
    if(Disp_Info.DS_mode == SECTION_CLEAR_AT_BOTH_UNITS)
    {
    //AFC
    var_count = 0;
    sprintf(&var_buffer[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_393))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_393))), ST_DRAW);
    Wheel_LCWS.A_FC =  var_count;

    sprintf(&var_buffer2[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_394))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_394))), ST_DRAW);

    //BFC
    sprintf(&var_buffer[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_397))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_397))), ST_DRAW);
    Wheel_LCWS.B_FC =  var_count;

    sprintf(&var_buffer2[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_398))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_398))), ST_DRAW);

    //CFC
    sprintf(&var_buffer[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_401))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_401))), ST_DRAW);
    Wheel_LCWS.C_FC =  var_count;

    sprintf(&var_buffer2[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_402))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_402))), ST_DRAW);

    //DFC
    sprintf(&var_buffer[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_405))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_405))), ST_DRAW);
    Wheel_LCWS.D_FC =  var_count;

    sprintf(&var_buffer2[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_406))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_406))), ST_DRAW);

    /*************************************************************************************/

    //ARC
    sprintf(&var_buffer[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_395))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_395))), ST_DRAW);
    Wheel_LCWS.A_RC =  var_count;

    sprintf(&var_buffer2[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_396))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_396))), ST_DRAW);

    //BRC
    sprintf(&var_buffer[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_399))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_399))), ST_DRAW);
    Wheel_LCWS.B_RC =  var_count;

    sprintf(&var_buffer2[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_400))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_400))), ST_DRAW);

    //C_RC
    sprintf(&var_buffer[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_403))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_403))), ST_DRAW);
    Wheel_LCWS.C_RC =  var_count;

    sprintf(&var_buffer2[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_404))), (XCHAR*)&var_buffer2[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_404))), ST_DRAW);

    //D_RC
    sprintf(&var_buffer[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_407))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_407))), ST_DRAW);
    Wheel_LCWS.D_RC =  var_count;

    sprintf(&var_buffer2[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_408))), (XCHAR*)&var_buffer2[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_408))), ST_DRAW);
    }
    else
    {
    //AFC
    var_count = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);
    sprintf(&var_buffer[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_393))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_393))), ST_DRAW);
    Wheel_LCWS.A_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][38] + ((UINT)(GLCD_Info.CPU_Data[1][39])<<8);
    sprintf(&var_buffer2[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_394))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_394))), ST_DRAW);

    //BFC
    var_count = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
    sprintf(&var_buffer[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_397))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_397))), ST_DRAW);
    Wheel_LCWS.B_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
    sprintf(&var_buffer2[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_398))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_398))), ST_DRAW);

    //CFC
    var_count = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
    sprintf(&var_buffer[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_401))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_401))), ST_DRAW);
    Wheel_LCWS.C_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);
    sprintf(&var_buffer2[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_402))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_402))), ST_DRAW);

    //DFC
    var_count = GLCD_Info.CPU_Data[0][40] + ((UINT)(GLCD_Info.CPU_Data[0][41])<<8);
    sprintf(&var_buffer[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_405))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_405))), ST_DRAW);
    Wheel_LCWS.D_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][40] + ((UINT)(GLCD_Info.CPU_Data[1][41])<<8);
    sprintf(&var_buffer2[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_406))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_406))), ST_DRAW);

    /*************************************************************************************/

    //ARC
    var_count = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);
    sprintf(&var_buffer[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_395))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_395))), ST_DRAW);
    Wheel_LCWS.A_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][46] + ((UINT)(GLCD_Info.CPU_Data[1][47])<<8);
    sprintf(&var_buffer2[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_396))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_396))), ST_DRAW);

    //BRC
    var_count = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
    sprintf(&var_buffer[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_399))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_399))), ST_DRAW);
    Wheel_LCWS.B_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);
    sprintf(&var_buffer2[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_400))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_400))), ST_DRAW);

    //C_RC
    var_count = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
    sprintf(&var_buffer[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_403))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_403))), ST_DRAW);
    Wheel_LCWS.C_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
    sprintf(&var_buffer2[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_404))), (XCHAR*)&var_buffer2[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_404))), ST_DRAW);

    //D_RC
    var_count = GLCD_Info.CPU_Data[0][48] + ((UINT)(GLCD_Info.CPU_Data[0][49])<<8);
    sprintf(&var_buffer[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_407))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_407))), ST_DRAW);
    Wheel_LCWS.D_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][48] + ((UINT)(GLCD_Info.CPU_Data[1][49])<<8);
    sprintf(&var_buffer2[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_408))), (XCHAR*)&var_buffer2[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_408))), ST_DRAW);
    }

    if(Disp_Info.Unit_Type == UT_LCWS)
    {
        if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
        {
            if(Wheel_LCWS.A_FC > Wheel_LCWS.C_FC)
            {
                //FC 1st slot
                if(Wheel_LCWS.A_FC > Wheel_LCWS.B_FC)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                }
                else if((Wheel_LCWS.A_FC == Wheel_LCWS.B_FC) && (Wheel_LCWS.C_FC <= Wheel_LCWS.D_FC))
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                }
                else
                {
                    PictSetBitmap(((PICTURE*) (GOLFindObject(PCB_424))), (void*) &No_Train);
                    SetState(((PICTURE*) (GOLFindObject(PCB_424))), PICT_DRAW);
                }
                // FC 2nd Slot
                if(Wheel_LCWS.C_FC > Wheel_LCWS.D_FC)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
                }
                else
                {
                    PictSetBitmap(((PICTURE*) (GOLFindObject(PCB_434))), (void*) &No_Train);
                    SetState(((PICTURE*) (GOLFindObject(PCB_434))), PICT_DRAW);
                }
            }
            else if(Wheel_LCWS.D_RC > Wheel_LCWS.B_RC)
            {
                //RC 2nd slot
                if(Wheel_LCWS.D_RC > Wheel_LCWS.C_RC)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
                }
                else if((Wheel_LCWS.D_RC == Wheel_LCWS.C_RC) && (Wheel_LCWS.B_RC <= Wheel_LCWS.A_RC))
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
                }
                else
                {
                    PictSetBitmap(((PICTURE*) (GOLFindObject(PCB_434))), (void*) &No_Train);
                    SetState(((PICTURE*) (GOLFindObject(PCB_434))), PICT_DRAW);
                }
                // RC 1st Slot
                if(Wheel_LCWS.B_RC > Wheel_LCWS.A_RC)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                }
                else
                {
                    PictSetBitmap(((PICTURE*) (GOLFindObject(PCB_424))), (void*) &No_Train);
                    SetState(((PICTURE*) (GOLFindObject(PCB_424))), PICT_DRAW);
                }
            }
        }
        else
        {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&No_Train);
                        SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&No_Train);
                        SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
        }
    }
    else if(Disp_Info.Unit_Type == UT_LCWS_DL)
    {
        if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
        {
            if(Wheel_LCWS.A_FC > Wheel_LCWS.C_FC)
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&logo_small2_db2_inv);
                SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);                
                //Fwd count
            }
            else if(Wheel_LCWS.C_RC > Wheel_LCWS.A_RC)
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&logo_small2_db2);
                SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                //Rev count
            }
            else
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&No_Train);
                SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                // No Train
            }

            if(Wheel_LCWS.B_FC > Wheel_LCWS.D_FC)
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&logo_small2_db2_inv);
                SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
                //Fwd count
            }
            else if(Wheel_LCWS.D_RC > Wheel_LCWS.B_RC)
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&logo_small2_db2);
                SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
                //Rev count
            }
            else
            {
                // No Train
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&No_Train);
                SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
            }
        }
        else
        {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_424))), (void*)&No_Train);
                        SetState(((PICTURE*)(GOLFindObject(PCB_424))), PICT_DRAW);
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_434))), (void*)&No_Train);
                        SetState(((PICTURE*)(GOLFindObject(PCB_434))), PICT_DRAW);
        }
    }
            //error number
            var_count = GLCD_Info.CPU_Data[0][54];
            Error_EF = GLCD_Info.CPU_Data[0][11];
    switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                if(Error_EF!=0)
                    sprintf(&var_buffer_r[0][0],"%d %s\n",Error_EF-1,&Error_message[Error_EF-1][0]);
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(Wheel_LCWS.A_FC==0 && Wheel_LCWS.B_FC==0 && Wheel_LCWS.C_FC==0 && Wheel_LCWS.D_FC==0 && Wheel_LCWS.A_RC==0 && Wheel_LCWS.B_RC==0 && Wheel_LCWS.C_RC==0 && Wheel_LCWS.D_RC==0)
                {
                    sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                }
                else
                {
                    sprintf(&var_buffer_r[0][0], "Pilot train in progress\n");                    
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                if(var_count != 255)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(Error_EF!=0)
                {
                    sprintf(&var_buffer_r[0][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF - 1][0]);
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_416))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_416))), ST_DRAW);

        sprintf(&var_buffer_r[1][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
        StSetText(((STATICTEXT*) (GOLFindObject(STE_222))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_222))), ST_DRAW);
}

void Update_C_3DP2S_Screen()
{
    if(Packet_src == 1)
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_91))), (XCHAR*) "CONFR-3DP");
    }
    else
    {
        StSetText(((STATICTEXT*) (GOLFindObject(STE_91))), (XCHAR*) "CONF-3DP");
    }
	SetState(((STATICTEXT*)(GOLFindObject(STE_91))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_92))), (XCHAR*) "CF");
	SetState(((STATICTEXT*)(GOLFindObject(STE_92))), ST_DRAW);

	Wheel1_3D2S.SF_FC   = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
	Wheel1_3D2S.S_CF_FC = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
	Wheel1_3D2S.S_CF_RC = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
	Wheel1_3D2S.SF_RC   = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
	Wheel1_3D2S.E_CF_FC = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);
	Wheel1_3D2S.EF_FC   = GLCD_Info.CPU_Data[0][40] + ((UINT)(GLCD_Info.CPU_Data[0][41])<<8);
	Wheel1_3D2S.EF_RC   = GLCD_Info.CPU_Data[0][48] + ((UINT)(GLCD_Info.CPU_Data[0][49])<<8);
	Wheel1_3D2S.E_CF_RC = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);	
    
	Wheel2_3D2S.SF_FC   = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);
	Wheel2_3D2S.S_CF_FC = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
	Wheel2_3D2S.S_CF_RC = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);
	Wheel2_3D2S.SF_RC   = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
	Wheel2_3D2S.E_CF_FC = GLCD_Info.CPU_Data[1][38] + ((UINT)(GLCD_Info.CPU_Data[1][39])<<8);
	Wheel2_3D2S.EF_FC   = GLCD_Info.CPU_Data[1][40] + ((UINT)(GLCD_Info.CPU_Data[1][41])<<8);
	Wheel2_3D2S.EF_RC   = GLCD_Info.CPU_Data[1][48] + ((UINT)(GLCD_Info.CPU_Data[1][49])<<8);
	Wheel2_3D2S.E_CF_RC = GLCD_Info.CPU_Data[1][46] + ((UINT)(GLCD_Info.CPU_Data[1][47])<<8);	
	
	if(Packet_src != 1)
    {
        SetState(((BUTTON*) (GOLFindObject(BTN_608))),BTN_NOPANEL | BTN_HIDE);
    }
	else
    {
        Disp_Info1.Reset_mode = GLCD_Info.CPU_Data[0][10];
        Disp_Info1.DS_mode = Disp_Info1.Reset_mode & 0x0F;
        Disp_Info1.US_mode = (Disp_Info1.Reset_mode & 0xF0)>>4;
        //US
        if(Disp_Info1.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
        {
            Wheel1_3D2S.SF_FC    = Wheel1_3D2S.SF_FC    - Track_Wheel1_3D2S.SF_FC   ;
            Wheel1_3D2S.S_CF_FC  = Wheel1_3D2S.S_CF_FC  - Track_Wheel1_3D2S.S_CF_FC ;
            Wheel1_3D2S.S_CF_RC  = Wheel1_3D2S.S_CF_RC  - Track_Wheel1_3D2S.S_CF_RC ;
            Wheel1_3D2S.SF_RC    = Wheel1_3D2S.SF_RC    - Track_Wheel1_3D2S.SF_RC   ;
        }
        else if(Disp_Info1.US_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
        {
            if(Disp_Info1.US_mode == SECTION_CLEAR_AT_BOTH_UNITS)
            {
                Track_Wheel1_3D2S.SF_FC    = Wheel1_3D2S.SF_FC   ;
                Track_Wheel1_3D2S.S_CF_FC  = Wheel1_3D2S.S_CF_FC ;
                Track_Wheel1_3D2S.S_CF_RC  = Wheel1_3D2S.S_CF_RC ;
                Track_Wheel1_3D2S.SF_RC    = Wheel1_3D2S.SF_RC   ;
            }
            else
            {
                Track_Wheel1_3D2S.SF_FC    = 0;
                Track_Wheel1_3D2S.S_CF_FC  = 0;
                Track_Wheel1_3D2S.S_CF_RC  = 0;
                Track_Wheel1_3D2S.SF_RC    = 0;
            }
            Wheel1_3D2S.SF_FC    = 0;
            Wheel1_3D2S.S_CF_FC  = 0;
            Wheel1_3D2S.S_CF_RC  = 0;
            Wheel1_3D2S.SF_RC    = 0;
        }
        //DS
        if(Disp_Info1.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
        {
            Wheel1_3D2S.E_CF_FC  = Wheel1_3D2S.E_CF_FC  - Track_Wheel1_3D2S.E_CF_FC ;
            Wheel1_3D2S.EF_FC    = Wheel1_3D2S.EF_FC    - Track_Wheel1_3D2S.EF_FC   ;
            Wheel1_3D2S.EF_RC    = Wheel1_3D2S.EF_RC    - Track_Wheel1_3D2S.EF_RC   ;
            Wheel1_3D2S.E_CF_RC  = Wheel1_3D2S.E_CF_RC  - Track_Wheel1_3D2S.E_CF_RC ;
        }
        else if(Disp_Info1.DS_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
        {
            if(Disp_Info1.DS_mode == SECTION_CLEAR_AT_BOTH_UNITS)
            {
                Track_Wheel1_3D2S.E_CF_FC  = Wheel1_3D2S.E_CF_FC ;
                Track_Wheel1_3D2S.EF_FC    = Wheel1_3D2S.EF_FC   ;
                Track_Wheel1_3D2S.EF_RC    = Wheel1_3D2S.EF_RC   ;
                Track_Wheel1_3D2S.E_CF_RC  = Wheel1_3D2S.E_CF_RC ;
            }
            else
            {
                Track_Wheel1_3D2S.E_CF_FC  = 0;
                Track_Wheel1_3D2S.EF_FC    = 0;
                Track_Wheel1_3D2S.EF_RC    = 0;
                Track_Wheel1_3D2S.E_CF_RC  = 0;
            }
            Wheel1_3D2S.E_CF_FC  = 0;
            Wheel1_3D2S.EF_FC    = 0;
            Wheel1_3D2S.EF_RC    = 0;
            Wheel1_3D2S.E_CF_RC  = 0;
        }
        Disp_Info2.Reset_mode = GLCD_Info.CPU_Data[1][10];
        Disp_Info2.DS_mode = Disp_Info2.Reset_mode & 0x0F;
        Disp_Info2.US_mode = (Disp_Info2.Reset_mode & 0xF0)>>4;
        //US
        if(Disp_Info2.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
        {
            Wheel2_3D2S.SF_FC    = Wheel2_3D2S.SF_FC    - Track_Wheel2_3D2S.SF_FC   ;
            Wheel2_3D2S.S_CF_FC  = Wheel2_3D2S.S_CF_FC  - Track_Wheel2_3D2S.S_CF_FC ;
            Wheel2_3D2S.S_CF_RC  = Wheel2_3D2S.S_CF_RC  - Track_Wheel2_3D2S.S_CF_RC ;
            Wheel2_3D2S.SF_RC    = Wheel2_3D2S.SF_RC    - Track_Wheel2_3D2S.SF_RC   ;
        }
        else if(Disp_Info2.US_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
        {
            if(Disp_Info2.US_mode == SECTION_CLEAR_AT_BOTH_UNITS)
            {
                Track_Wheel2_3D2S.SF_FC    = Wheel2_3D2S.SF_FC   ;
                Track_Wheel2_3D2S.S_CF_FC  = Wheel2_3D2S.S_CF_FC ;
                Track_Wheel2_3D2S.S_CF_RC  = Wheel2_3D2S.S_CF_RC ;
                Track_Wheel2_3D2S.SF_RC    = Wheel2_3D2S.SF_RC   ;
            }
            else
            {
                Track_Wheel2_3D2S.SF_FC    = 0;
                Track_Wheel2_3D2S.S_CF_FC  = 0;
                Track_Wheel2_3D2S.S_CF_RC  = 0;
                Track_Wheel2_3D2S.SF_RC    = 0;
            }
            Wheel2_3D2S.SF_FC    = 0;
            Wheel2_3D2S.S_CF_FC  = 0;
            Wheel2_3D2S.S_CF_RC  = 0;
            Wheel2_3D2S.SF_RC    = 0;
        }
        //DS
        if(Disp_Info2.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
        {
            Wheel2_3D2S.E_CF_FC  = Wheel2_3D2S.E_CF_FC  - Track_Wheel2_3D2S.E_CF_FC ;
            Wheel2_3D2S.EF_FC    = Wheel2_3D2S.EF_FC    - Track_Wheel2_3D2S.EF_FC   ;
            Wheel2_3D2S.EF_RC    = Wheel2_3D2S.EF_RC    - Track_Wheel2_3D2S.EF_RC   ;
            Wheel2_3D2S.E_CF_RC  = Wheel2_3D2S.E_CF_RC  - Track_Wheel2_3D2S.E_CF_RC ;
        }
        else if(Disp_Info2.DS_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
        {
            if(Disp_Info2.DS_mode == SECTION_CLEAR_AT_BOTH_UNITS)
            {
                Track_Wheel2_3D2S.E_CF_FC  = Wheel2_3D2S.E_CF_FC ;
                Track_Wheel2_3D2S.EF_FC    = Wheel2_3D2S.EF_FC   ;
                Track_Wheel2_3D2S.EF_RC    = Wheel2_3D2S.EF_RC   ;
                Track_Wheel2_3D2S.E_CF_RC  = Wheel2_3D2S.E_CF_RC ;
            }
            else
            {
                Track_Wheel2_3D2S.E_CF_FC  = 0;
                Track_Wheel2_3D2S.EF_FC    = 0;
                Track_Wheel2_3D2S.EF_RC    = 0;
                Track_Wheel2_3D2S.E_CF_RC  = 0;
            }
            Wheel2_3D2S.E_CF_FC  = 0;
            Wheel2_3D2S.EF_FC    = 0;
            Wheel2_3D2S.EF_RC    = 0;
            Wheel2_3D2S.E_CF_RC  = 0;
        }
    }
    //US Remote FC - SF FC
    sprintf(&var_buffer[0][0],"%d\n",Wheel1_3D2S.SF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_281))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_281))), ST_DRAW);
    
    sprintf(&var_buffer2[0][0],"%d\n",Wheel2_3D2S.SF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_287))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_287))), ST_DRAW);

    //US Local FC - CF FC
    sprintf(&var_buffer[1][0],"%d\n",Wheel1_3D2S.S_CF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_291))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_291))), ST_DRAW);

    sprintf(&var_buffer2[1][0],"%d\n",Wheel2_3D2S.S_CF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_292))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_292))), ST_DRAW);

    //US Local RC - CF RC

    sprintf(&var_buffer[2][0],"%d\n",Wheel1_3D2S.S_CF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_293))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_293))), ST_DRAW);
    sprintf(&var_buffer2[2][0],"%d\n",Wheel2_3D2S.S_CF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_294))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_294))), ST_DRAW);

    //US Remote RC - SF RC

    sprintf(&var_buffer[3][0],"%d\n",Wheel1_3D2S.SF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_289))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_289))), ST_DRAW);

    sprintf(&var_buffer2[3][0],"%d\n",Wheel2_3D2S.SF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_290))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_290))), ST_DRAW);

    /*************************************************************************************/
    
    //DS Local FC - CF FC

    sprintf(&var_buffer[4][0],"%d\n",Wheel1_3D2S.E_CF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_505))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_505))), ST_DRAW);

    sprintf(&var_buffer2[4][0],"%d\n",Wheel2_3D2S.E_CF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_507))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_507))), ST_DRAW);

    //DS Remote FC - EF FC

    sprintf(&var_buffer[5][0],"%d\n",Wheel1_3D2S.EF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_312))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_312))), ST_DRAW);

    sprintf(&var_buffer2[5][0],"%d\n",Wheel2_3D2S.EF_FC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_313))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_313))), ST_DRAW);
	
    //DS Remote RC - EF RC
    sprintf(&var_buffer[6][0],"%d\n",Wheel1_3D2S.EF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_314))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_314))), ST_DRAW);
	
    sprintf(&var_buffer2[6][0],"%d\n",Wheel2_3D2S.EF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_315))), (XCHAR*)&var_buffer2[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_315))), ST_DRAW);

    //DS Local RC - CF RC
    sprintf(&var_buffer[7][0],"%d\n",Wheel1_3D2S.E_CF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_506))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_506))), ST_DRAW);

    sprintf(&var_buffer2[7][0],"%d\n",Wheel2_3D2S.E_CF_RC);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_508))), (XCHAR*)&var_buffer2[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_508))), ST_DRAW);

    if(Disp_Info.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS || Disp_Info.US_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
    {
        if(Wheel1_3D2S.SF_RC == Wheel1_3D2S.S_CF_RC)
        {
            if(Wheel1_3D2S.SF_FC == Wheel1_3D2S.S_CF_FC)
            {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
            }
            else if(Wheel1_3D2S.SF_FC > Wheel1_3D2S.S_CF_FC)
             {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2_inv);
                        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
             }
             else
             {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2);
                        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
             }
        }
        else
        {
            if(Wheel1_3D2S.SF_RC == Wheel1_3D2S.S_CF_RC)
            {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
            }
            else if(Wheel1_3D2S.SF_RC < Wheel1_3D2S.S_CF_RC)
             {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2);
                        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
             }
             else
             {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2_inv);
                        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
             }
        }
    }
    else
    {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
    }

    if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS || Disp_Info.DS_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
    {
        if(Wheel1_3D2S.EF_RC == Wheel1_3D2S.E_CF_RC)
        {
            if(Wheel1_3D2S.EF_FC == Wheel1_3D2S.E_CF_FC)
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&No_Train);
                SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
            }
            else if(Wheel1_3D2S.EF_FC < Wheel1_3D2S.E_CF_FC)
             {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
             }
             else
            {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
            }
        }
        else
        {
            if(Wheel1_3D2S.EF_RC == Wheel1_3D2S.E_CF_RC)
            {
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&No_Train);
                SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
            }
            else if(Wheel1_3D2S.EF_RC > Wheel1_3D2S.E_CF_RC)
             {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
             }
             else
            {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
            }
        }
    }
    else
    {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);
    }
            //error number
        var_count = GLCD_Info.CPU_Data[0][54];
		SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(Wheel1_3D2S.SF_RC==0  && Wheel1_3D2S.S_CF_FC==0 && Wheel1_3D2S.S_CF_RC==0 && Wheel1_3D2S.E_CF_FC==0 && Wheel1_3D2S.E_CF_RC==0 && Wheel1_3D2S.EF_FC==0 && Wheel1_3D2S.EF_RC==0) 
                {
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                }
                else
                {
                sprintf(&var_buffer_r[0][0], "Pilot train in progress\n");
                sprintf(&var_buffer_r[1][0], "Pilot train in progress\n");
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255 && var_count!=2)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(Wheel1_3D2S.SF_RC==0  && Wheel1_3D2S.S_CF_FC==0 && Wheel1_3D2S.S_CF_RC==0 && Wheel1_3D2S.E_CF_FC==0 && Wheel1_3D2S.E_CF_RC==0 && Wheel1_3D2S.EF_FC==0 && Wheel1_3D2S.EF_RC==0) 
                {
                sprintf(&var_buffer_r[3][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[2][0], "Waiting for pilot train\n");
                }
                else
                {
                sprintf(&var_buffer_r[3][0], "Pilot train in progress\n");
                sprintf(&var_buffer_r[2][0], "Pilot train in progress\n");
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[2][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[2][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[3][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[2][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[3][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0],"System Failure\n");
                if(var_count!=255 && var_count!=2)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[3][0], "\n");
                sprintf(&var_buffer_r[2][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_329))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_329))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_510))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_510))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_330))), (XCHAR*) & var_buffer_r[2][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_330))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_331))), (XCHAR*) & var_buffer_r[3][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_331))), ST_DRAW);

        sprintf(&var_buffer_r[4][0],"%d\n",Disp_Info.Train_Speed);
        StSetText(((STATICTEXT*)(GOLFindObject(STE_335))), (XCHAR*)&var_buffer_r[4][0]);
        SetState(((STATICTEXT*)(GOLFindObject(STE_335))), ST_DRAW);

        sprintf(&var_buffer_r[5][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
        StSetText(((STATICTEXT*) (GOLFindObject(STE_89))), (XCHAR*) & var_buffer_r[5][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_89))), ST_DRAW);
}

    signed int img1 = 0;
    signed int img2 = 0;

void Update_C_3DP_SF_Screen()
{
    signed int train_direction = 0;
    WORD temp_cnt;

    Conf_wheel_info_3D_SF    Wheel_3D_SF;

        SetState(((BUTTON*) (GOLFindObject(BTN_608))), BTN_NOPANEL |BTN_HIDE);
	StSetText(((STATICTEXT*) (GOLFindObject(STE_91))), (XCHAR*) "CONF-3DP");
	SetState(((STATICTEXT*)(GOLFindObject(STE_91))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_92))), (XCHAR*) "SF");
	SetState(((STATICTEXT*)(GOLFindObject(STE_92))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_325))), (XCHAR*) "EF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_325))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_509))), (XCHAR*) "E-SF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_509))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_326))), (XCHAR*) "C-SF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_326))), ST_DRAW);

 	StSetText(((STATICTEXT*) (GOLFindObject(STE_327))), (XCHAR*) "CF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_327))), ST_DRAW);

//Columns


        StSetText(((STATICTEXT*) (GOLFindObject(STE_283))), (XCHAR*) "SF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_283))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_284))), (XCHAR*) "SF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_284))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_316))), (XCHAR*) "S-CF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_316))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_317))), (XCHAR*) "S-CF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_317))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_503))), (XCHAR*) "S-EF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_503))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_504))), (XCHAR*) "S-EF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_504))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_285))), (XCHAR*) "EF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_285))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_286))), (XCHAR*) "EF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_286))), ST_DRAW);

        

    //SF_FC
    var_count = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
    sprintf(&var_buffer[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_281))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_281))), ST_DRAW);
    Wheel_3D_SF.SF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
    sprintf(&var_buffer2[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_287))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_287))), ST_DRAW);

    //SF_EF_FC
    var_count = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);
    sprintf(&var_buffer[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_505))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_505))), ST_DRAW);
    Wheel_3D_SF.S_EF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][38] + ((UINT)(GLCD_Info.CPU_Data[1][39])<<8);
    sprintf(&var_buffer2[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_507))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_507))), ST_DRAW);

    //SF_CF_FC
    var_count = GLCD_Info.CPU_Data[0][40] + ((UINT)(GLCD_Info.CPU_Data[0][41])<<8);
    sprintf(&var_buffer[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_291))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_291))), ST_DRAW);
    Wheel_3D_SF.S_CF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][40] + ((UINT)(GLCD_Info.CPU_Data[1][41])<<8);
    sprintf(&var_buffer2[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_292))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_292))), ST_DRAW);

    //SF_EF
    var_count = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
    sprintf(&var_buffer[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_312))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_312))), ST_DRAW);
    Wheel_3D_SF.EF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
    sprintf(&var_buffer2[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_313))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_313))), ST_DRAW);

///////RC
        //SF_RC
    var_count = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
    sprintf(&var_buffer[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_289))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_289))), ST_DRAW);
    Wheel_3D_SF.SF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);
    sprintf(&var_buffer2[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_290))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_290))), ST_DRAW);

    //SF_EF_RC
    var_count = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);
    sprintf(&var_buffer[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_506))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_506))), ST_DRAW);
    Wheel_3D_SF.S_EF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][46] + ((UINT)(GLCD_Info.CPU_Data[1][47])<<8);
    sprintf(&var_buffer2[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_508))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_508))), ST_DRAW);

    //SF_CF_RC
    var_count = GLCD_Info.CPU_Data[0][48] + ((UINT)(GLCD_Info.CPU_Data[0][49])<<8);
    sprintf(&var_buffer[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_293))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_293))), ST_DRAW);
    Wheel_3D_SF.S_CF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][48] + ((UINT)(GLCD_Info.CPU_Data[1][49])<<8);
    sprintf(&var_buffer2[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_294))), (XCHAR*)&var_buffer2[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_294))), ST_DRAW);

    //EF_RC
    var_count = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
    sprintf(&var_buffer[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_314))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_314))), ST_DRAW);
    Wheel_3D_SF.EF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);
    sprintf(&var_buffer2[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_315))), (XCHAR*)&var_buffer2[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_315))), ST_DRAW);

    if(Disp_Info.US_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
    {
        temp_cnt = Wheel_3D_SF.SF_RC + Wheel_3D_SF.S_CF_RC + Wheel_3D_SF.EF_RC + Wheel_3D_SF.SF_FC + Wheel_3D_SF.S_CF_FC + Wheel_3D_SF.EF_FC;
        if(temp_cnt!=0)
        {
            if((Wheel_3D_SF.SF_RC == Wheel_3D_SF.S_CF_RC) && (Wheel_3D_SF.S_CF_RC == Wheel_3D_SF.EF_RC) && (Wheel_3D_SF.EF_RC == Wheel_3D_SF.SF_RC))
            {
                //forward
                train_direction = 1;
            }
            else
            {
                //reverse
                train_direction = -1;
            }
        }        
    }
    if(Disp_Info.DS_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
    {
        temp_cnt = Wheel_3D_SF.SF_RC + Wheel_3D_SF.S_CF_RC + Wheel_3D_SF.EF_RC + Wheel_3D_SF.SF_FC + Wheel_3D_SF.S_CF_FC + Wheel_3D_SF.EF_FC;
        if(temp_cnt!=0)
        {
            if((Wheel_3D_SF.SF_RC == Wheel_3D_SF.S_CF_RC) && (Wheel_3D_SF.S_CF_RC == Wheel_3D_SF.EF_RC) && (Wheel_3D_SF.EF_RC == Wheel_3D_SF.SF_RC))
            {
                //forward
                train_direction = 1;
            }
            else
            {
                //reverse
                train_direction = -1;
            }
        }        
    }
    if(Disp_Info.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        if((Wheel_3D_SF.SF_RC == Wheel_3D_SF.S_CF_RC) && (Wheel_3D_SF.S_CF_RC == Wheel_3D_SF.EF_RC) && (Wheel_3D_SF.EF_RC == Wheel_3D_SF.SF_RC))
		{
            //forward
            train_direction = 1;
        }
        else
        {
            //reverse
            train_direction = -1;
        }
    }
    if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        if((Wheel_3D_SF.SF_RC == Wheel_3D_SF.S_CF_RC) && (Wheel_3D_SF.S_CF_RC == Wheel_3D_SF.EF_RC) && (Wheel_3D_SF.EF_RC == Wheel_3D_SF.SF_RC))
		{
            //forward
            train_direction = 1;
        }
        else
        {
            //reverse
            train_direction = -1;
        }    
    }
    if(train_direction == 0)
    {
        img1 = 0;
        img2 = 0;
    }
    else
    {
        if(train_direction == 1)
        {
            //forward - image1
            if((Wheel_3D_SF.SF_FC >= Wheel_3D_SF.S_CF_FC) || (Wheel_3D_SF.SF_FC > Wheel_3D_SF.EF_FC))
            {
                img1 = 1;
            }
            else
            {
                img1 = 0;
            }
            //image2 
            if(Wheel_3D_SF.S_CF_FC > Wheel_3D_SF.EF_FC)
            {
                img2 = 1;
            }
            else if(img2 != 1)
            {
                img2 = 0;
            }
        }
        else
        {
            //reverse
            if((Wheel_3D_SF.EF_RC >= Wheel_3D_SF.S_CF_RC) || (Wheel_3D_SF.EF_RC > Wheel_3D_SF.SF_RC))
            {
                img2 = -1;
            }
            else
            {
                img2 = 0;
            }
            //image2 
            if(Wheel_3D_SF.S_CF_RC > Wheel_3D_SF.SF_RC)
            {
                img1 = -1;
            }
            else if(img1 != -1)
            {
                img1 = 0;
            }
        }
    }
    
    
    
    //display image
    if(img1 == 1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2_inv);
        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
    }
    else if(img1 == -1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2);
        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);        
    }
    else
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&No_Train);
        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);              
    }
    if(img2 == 1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2_inv);
        SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);        
    }
    else if(img2 == -1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2);
        SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);        
    }
    else
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&No_Train);
        SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);        
    }
    
            //error number
            var_count = GLCD_Info.CPU_Data[0][54];
        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(train_direction ==0)
                {
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                }
                else
                {
                sprintf(&var_buffer_r[0][0], "Pilot train in progress\n");
                sprintf(&var_buffer_r[1][0], "Pilot train in progress\n");
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255 && var_count!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255 && var_count!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255 && var_count!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(train_direction ==0)
                {
                sprintf(&var_buffer_r[3][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[2][0], "Waiting for pilot train\n");
                }
                else
                {
                sprintf(&var_buffer_r[3][0], "Pilot train in progress\n");
                sprintf(&var_buffer_r[2][0], "Pilot train in progress\n");
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[2][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[2][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255 && var_count!=0)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[3][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[2][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[3][0], "Waiting for reset\n");
                if(var_count!=255 && var_count!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0],"System Failure\n");
                if(var_count!=255 && var_count!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[3][0], "\n");
                sprintf(&var_buffer_r[2][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_329))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_329))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_510))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_510))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_330))), (XCHAR*) & var_buffer_r[2][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_330))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_331))), (XCHAR*) & var_buffer_r[3][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_331))), ST_DRAW);

        sprintf(&var_buffer_r[4][0],"%d\n",Disp_Info.Train_Speed);
        StSetText(((STATICTEXT*)(GOLFindObject(STE_335))), (XCHAR*)&var_buffer_r[4][0]);
        SetState(((STATICTEXT*)(GOLFindObject(STE_335))), ST_DRAW);

        sprintf(&var_buffer_r[5][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
        StSetText(((STATICTEXT*) (GOLFindObject(STE_89))), (XCHAR*) & var_buffer_r[5][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_89))), ST_DRAW);
}

void Update_C_3DP_EF_Screen()
{
    signed int train_direction = 0;
    WORD temp_cnt;
    Conf_wheel_info_3D_EF    Wheel_3D_EF;


    SetState(((BUTTON*) (GOLFindObject(BTN_608))),BTN_NOPANEL | BTN_HIDE);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_91))), (XCHAR*) "CONF-3DP");
	SetState(((STATICTEXT*)(GOLFindObject(STE_91))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_92))), (XCHAR*) "EF");
	SetState(((STATICTEXT*)(GOLFindObject(STE_92))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_325))), (XCHAR*) "CF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_325))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_509))), (XCHAR*) "C-EF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_509))), ST_DRAW);

	StSetText(((STATICTEXT*) (GOLFindObject(STE_326))), (XCHAR*) "S-EF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_326))), ST_DRAW);

 	StSetText(((STATICTEXT*) (GOLFindObject(STE_327))), (XCHAR*) "SF Status");
	SetState(((STATICTEXT*)(GOLFindObject(STE_327))), ST_DRAW);

//Columns


        StSetText(((STATICTEXT*) (GOLFindObject(STE_283))), (XCHAR*) "SF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_283))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_284))), (XCHAR*) "SF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_284))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_316))), (XCHAR*) "E-SF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_316))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_317))), (XCHAR*) "E-SF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_317))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_503))), (XCHAR*) "E-CF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_503))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_504))), (XCHAR*) "E-CF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_504))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_285))), (XCHAR*) "EF FC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_285))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_286))), (XCHAR*) "EF RC");
	SetState(((STATICTEXT*)(GOLFindObject(STE_286))), ST_DRAW);


//SF_FC
    var_count = GLCD_Info.CPU_Data[0][48] + ((UINT)(GLCD_Info.CPU_Data[0][49])<<8);
    sprintf(&var_buffer[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_281))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_281))), ST_DRAW);
    Wheel_3D_EF.SF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][48] + ((UINT)(GLCD_Info.CPU_Data[1][49])<<8);
    sprintf(&var_buffer2[0][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_287))), (XCHAR*)&var_buffer2[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_287))), ST_DRAW);

    //E_CF_FC
    var_count = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
    sprintf(&var_buffer[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_505))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_505))), ST_DRAW);
    Wheel_3D_EF.E_CF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);
    sprintf(&var_buffer2[1][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_507))), (XCHAR*)&var_buffer2[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_507))), ST_DRAW);

    //E_SF_FC
    var_count = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);
    sprintf(&var_buffer[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_291))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_291))), ST_DRAW);
    Wheel_3D_EF.E_SF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][38] + ((UINT)(GLCD_Info.CPU_Data[1][39])<<8);
    sprintf(&var_buffer2[2][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_292))), (XCHAR*)&var_buffer2[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_292))), ST_DRAW);

    //EF
    var_count = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
    sprintf(&var_buffer[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_312))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_312))), ST_DRAW);
    Wheel_3D_EF.EF_FC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
    sprintf(&var_buffer2[3][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_313))), (XCHAR*)&var_buffer2[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_313))), ST_DRAW);

///////RC
        //SF_RC
    var_count = GLCD_Info.CPU_Data[0][40] + ((UINT)(GLCD_Info.CPU_Data[0][41])<<8);
    sprintf(&var_buffer[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_289))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_289))), ST_DRAW);
    Wheel_3D_EF.SF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][40] + ((UINT)(GLCD_Info.CPU_Data[1][41])<<8);
    sprintf(&var_buffer2[4][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_290))), (XCHAR*)&var_buffer2[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_290))), ST_DRAW);

    //E_CF_RC
    var_count = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
    sprintf(&var_buffer[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_506))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_506))), ST_DRAW);
    Wheel_3D_EF.E_CF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
    sprintf(&var_buffer2[5][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_508))), (XCHAR*)&var_buffer2[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_508))), ST_DRAW);

    //E_SF_RC
    var_count = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);
    sprintf(&var_buffer[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_293))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_293))), ST_DRAW);
    Wheel_3D_EF.E_SF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][46] + ((UINT)(GLCD_Info.CPU_Data[1][47])<<8);
    sprintf(&var_buffer2[6][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_294))), (XCHAR*)&var_buffer2[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_294))), ST_DRAW);

    //EF_RC
    var_count = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
    sprintf(&var_buffer[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_314))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_314))), ST_DRAW);
    Wheel_3D_EF.EF_RC =  var_count;

    var_count = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);
    sprintf(&var_buffer2[7][0],"%d\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_315))), (XCHAR*)&var_buffer2[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_315))), ST_DRAW);


    if(Disp_Info.US_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
    {
        temp_cnt = Wheel_3D_EF.EF_RC + Wheel_3D_EF.E_CF_RC + Wheel_3D_EF.SF_RC + Wheel_3D_EF.EF_FC + Wheel_3D_EF.E_CF_FC + Wheel_3D_EF.SF_FC;
        if(temp_cnt!=0)
        {
            if((Wheel_3D_EF.EF_RC == Wheel_3D_EF.E_CF_RC) && (Wheel_3D_EF.E_CF_RC == Wheel_3D_EF.SF_RC) && (Wheel_3D_EF.SF_RC == Wheel_3D_EF.EF_RC))
            {
                //forward
                train_direction = 1;
            }
            else
            {
                //reverse
                train_direction = -1;
            }
        }
    }
    if(Disp_Info.DS_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
    {
        temp_cnt = Wheel_3D_EF.EF_RC + Wheel_3D_EF.E_CF_RC + Wheel_3D_EF.SF_RC + Wheel_3D_EF.EF_FC + Wheel_3D_EF.E_CF_FC + Wheel_3D_EF.SF_FC;
        if(temp_cnt!=0)
        {
            if((Wheel_3D_EF.EF_RC == Wheel_3D_EF.E_CF_RC) && (Wheel_3D_EF.E_CF_RC == Wheel_3D_EF.SF_RC) && (Wheel_3D_EF.SF_RC == Wheel_3D_EF.EF_RC))
            {
                //forward
                train_direction = 1;
            }
            else
            {
                //reverse
                train_direction = -1;
            }
        }
    }
    
    
    if(Disp_Info.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        if((Wheel_3D_EF.EF_RC == Wheel_3D_EF.E_CF_RC) && (Wheel_3D_EF.E_CF_RC == Wheel_3D_EF.SF_RC) && (Wheel_3D_EF.SF_RC == Wheel_3D_EF.EF_RC))
		{
            //forward
            train_direction = 1;
        }
        else
        {
            //reverse
            train_direction = -1;
        }
    }
    if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        if((Wheel_3D_EF.EF_RC == Wheel_3D_EF.E_CF_RC) && (Wheel_3D_EF.E_CF_RC == Wheel_3D_EF.SF_RC) && (Wheel_3D_EF.SF_RC == Wheel_3D_EF.EF_RC))
		{
            //forward
            train_direction = 1;
        }
        else
        {
            //reverse
            train_direction = -1;
        }    
    }
    if(train_direction == 0)
    {
        img1 = 0;
        img2 = 0;
    }
    else
    {
        if(train_direction == 1)
        {
            //forward - image1
            if((Wheel_3D_EF.SF_FC >= Wheel_3D_EF.E_CF_FC) || (Wheel_3D_EF.SF_FC > Wheel_3D_EF.EF_FC))
            {
                img1 = 1;
            }
            else
            {
                img1 = 0;
            }
            //image2 
            if(Wheel_3D_EF.E_CF_FC > Wheel_3D_EF.EF_FC)
            {
                img2 = 1;
            }
            else if(img2 != 1)
            {
                img2 = 0;
            }
        }
        else
        {
            //reverse
            if((Wheel_3D_EF.EF_RC >= Wheel_3D_EF.E_CF_RC) || (Wheel_3D_EF.EF_RC > Wheel_3D_EF.SF_RC))
            {
                img2 = -1;
            }
            else
            {
                img2 = 0;
            }
            //image2 
            if(Wheel_3D_EF.E_CF_RC > Wheel_3D_EF.SF_RC)
            {
                img1 = -1;
            }
            else if(img1 != -1)
            {
                img1 = 0;
            }
        }
    }
    
    
    
    //display image
    if(img1 == 1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2_inv);
        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);
    }
    else if(img1 == -1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&logo_small2_db2);
        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);        
    }
    else
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_336))), (void*)&No_Train);
        SetState(((PICTURE*)(GOLFindObject(PCB_336))), PICT_DRAW);              
    }
    if(img2 == 1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2_inv);
        SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);        
    }
    else if(img2 == -1)
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&logo_small2_db2);
        SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);        
    }
    else
    {
        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_337))), (void*)&No_Train);
        SetState(((PICTURE*)(GOLFindObject(PCB_337))), PICT_DRAW);        
    }

            //error number
        var_count = GLCD_Info.CPU_Data[0][54];
        SF_error = GLCD_Info.CPU_Data[0][12];
        EF_error = GLCD_Info.CPU_Data[0][11];
        switch(Disp_Info.US_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(train_direction ==0)
                {
                sprintf(&var_buffer_r[0][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[1][0], "Waiting for pilot train\n");
                }
                else
                {
                sprintf(&var_buffer_r[0][0], "Pilot train in progress\n");
                sprintf(&var_buffer_r[1][0], "Pilot train in progress\n");
                }                
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[1][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[1][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[0][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[0][0],"System Failure\n");
                if(var_count!=255 && var_count!=2)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(SF_error!=0)
                {
                    sprintf(&var_buffer_r[1][0],"%d %s\n",SF_error-1,&Error_message[SF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[1][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[0][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[1][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[0][0], "Reset Applied\n");
                sprintf(&var_buffer_r[1][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[0][0], "\n");
                sprintf(&var_buffer_r[1][0], "\n");
                break;
        }
        switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(train_direction ==0)
                {
                sprintf(&var_buffer_r[3][0], "Waiting for pilot train\n");
                sprintf(&var_buffer_r[2][0], "Waiting for pilot train\n");
                }
                else
                {
                sprintf(&var_buffer_r[3][0], "Pilot train in progress\n");
                sprintf(&var_buffer_r[2][0], "Pilot train in progress\n");
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Clear\n");
                sprintf(&var_buffer_r[2][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0], "Sectiom Occupied\n");
                sprintf(&var_buffer_r[2][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[3][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[3][0],"System Failure\n");
                }
                sprintf(&var_buffer_r[2][0], "Waiting for reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer_r[3][0], "Waiting for reset\n");
                if(var_count!=255)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer_r[3][0],"System Failure\n");
                if(var_count!=255 && var_count!=2)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(EF_error!=0)
                {
                    sprintf(&var_buffer_r[2][0],"%d %s\n",EF_error-1,&Error_message[EF_error-1][0]);
                }
                else
                {
                    sprintf(&var_buffer_r[2][0],"System Failure\n");
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer_r[3][0], "Waiting For Reset\n");
                sprintf(&var_buffer_r[2][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer_r[3][0], "Reset Applied\n");
                sprintf(&var_buffer_r[2][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer_r[3][0], "\n");
                sprintf(&var_buffer_r[2][0], "\n");
                break;
        }
        StSetText(((STATICTEXT*) (GOLFindObject(STE_329))), (XCHAR*) & var_buffer_r[0][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_329))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_510))), (XCHAR*) & var_buffer_r[1][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_510))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_330))), (XCHAR*) & var_buffer_r[2][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_330))), ST_DRAW);

        StSetText(((STATICTEXT*) (GOLFindObject(STE_331))), (XCHAR*) & var_buffer_r[3][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_331))), ST_DRAW);

        sprintf(&var_buffer_r[4][0],"%d\n",Disp_Info.Train_Speed);
        StSetText(((STATICTEXT*)(GOLFindObject(STE_335))), (XCHAR*)&var_buffer_r[4][0]);
        SetState(((STATICTEXT*)(GOLFindObject(STE_335))), ST_DRAW);

        sprintf(&var_buffer_r[5][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
        StSetText(((STATICTEXT*) (GOLFindObject(STE_89))), (XCHAR*) & var_buffer_r[5][0]);
        SetState(((STATICTEXT*) (GOLFindObject(STE_89))), ST_DRAW);
}

WORD Track_count=0;
BYTE F_count = 0, R_count = 0;
extern const XCHAR CPU_2DP_STE_304text[ ];
extern const XCHAR CPU_2DP_STE_305text[ ];
extern const XCHAR CPU_2DP_STE_306text[ ];
extern const XCHAR CPU_2DP_STE_307text[ ];
extern const XCHAR CPU_2DP_STE_308text[ ];
extern const XCHAR CPU_2DP_STE_309text[ ];

extern GOL_SCHEME* Static_test;
void CreateError(char* string);

void Hide_Redraw()
{
    if(Hide_count == 1)
        return;

    STATICTEXT *pSTE_304;
    pSTE_304 = StCreate(  STE_304, //name
                       179, //left
                       95, //top
                       234, //right
                       120, //bottom
                       ST_HIDE | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_304text, //text
                      Static_test //scheme
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
                       ST_HIDE | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_305text, //text
                      Static_test //scheme
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
                       ST_HIDE | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_306text, //text
                      Static_test //scheme
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
                       ST_HIDE | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_307text, //text
                      Static_test //scheme
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
                       ST_HIDE | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_308text, //text
                      Static_test //scheme
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
                       ST_HIDE | ST_CENTER_ALIGN | ST_FRAME , //state
                       (XCHAR*)CPU_2DP_STE_309text, //text
                      Static_test //scheme
                    );

    if(pSTE_309==NULL)
    {
      CreateError(0);
      while(1); //Fatal Error, Check for memory leak or heap size
    }
    Hide_count = 1;
}

void Update_UT_DE_Screen()
{
    StSetText(((STATICTEXT*)(GOLFindObject(STE_486))), (XCHAR*)"CONF-DE");
    SetState(((STATICTEXT*)(GOLFindObject(STE_486))), ST_DRAW);

    StSetText(((STATICTEXT*)(GOLFindObject(STE_487))), (XCHAR*)"DE");
    SetState(((STATICTEXT*)(GOLFindObject(STE_487))), ST_DRAW);
    
            var_count = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);
            Wheel.L_FC = var_count;
            var_count = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);
            Wheel.L_RC = var_count;
            var_count = Wheel.L_FC - Wheel.L_RC;
            
            sprintf(&var_buffer[0][0],"%d\n",var_count);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_520))), (XCHAR*)&var_buffer[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_520))), ST_DRAW);
                        
            
            
            // Local Forward count CPU2
            var_count = GLCD_Info.CPU_Data[1][38] + ((UINT)(GLCD_Info.CPU_Data[1][39])<<8);
            Wheel.L_FC = var_count;
            var_count = GLCD_Info.CPU_Data[1][46] + ((UINT)(GLCD_Info.CPU_Data[1][47])<<8);
            Wheel.L_RC = var_count;
            var_count = Wheel.L_FC - Wheel.L_RC;
            
            sprintf(&var_buffer2[0][0],"%d\n",var_count);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_521))), (XCHAR*)&var_buffer2[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_521))), ST_DRAW);

            if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
            {
                Wheel.L_FC = Wheel.L_FC - Wheel.L_RC;
                if(Wheel.L_FC < Track_count)
                {
                    Track_count = Wheel.L_FC;
                    F_count = 0;
                    R_count = 1;
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_517))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_517))), PICT_DRAW);                    
                }
                else if(Wheel.L_FC > Track_count)
                {
                    Track_count = Wheel.L_FC;
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_517))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_517))), PICT_DRAW);
                    F_count = 1;
                    R_count = 0;                    
                }   
                else
                {
                    if(F_count == 1)
                    {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_517))), (void*)&logo_small2_db2_inv);
                        SetState(((PICTURE*)(GOLFindObject(PCB_517))), PICT_DRAW);
                    }
                    else
                    {
                        PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_517))), (void*)&logo_small2_db2);
                        SetState(((PICTURE*)(GOLFindObject(PCB_517))), PICT_DRAW);
                    }                    
                }
            }
            else
            {
                Track_count = 0;
                PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_517))), (void*)&No_Train);
                SetState(((PICTURE*)(GOLFindObject(PCB_517))), PICT_DRAW);
            }


            //error number
            var_count = GLCD_Info.CPU_Data[0][54];
            Error_EF = GLCD_Info.CPU_Data[0][11];
    switch(Disp_Info.DS_mode)
        {
            case WAITING_FOR_RESET_AT_BOTH_UNITS:
                sprintf(&var_buffer[4][0], "Waiting For Reset\n");
                if(Error_EF!=0)
                    sprintf(&var_buffer[4][0],"%d %s\n",Error_EF-1,&Error_message[Error_EF-1][0]);
                break;
            case RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[4][0], "Reset Applied\n");
                break;
            case RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[4][0], "Waiting For Reset\n");
                break;
            case RESET_APPLIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[4][0], "Reset Applied\n");
                break;
            case SECTION_WAIT_FOR_PILOT_TRAIN:
                if(Wheel.L_FC == 0)
                {
                    sprintf(&var_buffer[4][0], "Waiting for pilot train\n");
                }
                else
                {
                    sprintf(&var_buffer[4][0], "Pilot train in progress\n");
                }
                break;
            case SECTION_CLEAR_AT_BOTH_UNITS:
                sprintf(&var_buffer[4][0], "SECTION CLEAR\n");
                break;
            case SECTION_OCCUPIED_AT_BOTH_UNITS:
                sprintf(&var_buffer[4][0], "SECTION OCCUPIED\n");
                break;
            case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer[4][0], "Waiting For Reset\n");
                break;
            case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
                sprintf(&var_buffer[4][0], "Reset Applied\n");
                break;
            case SECTION_ERROR_AT_BOTH_UNITS:
                sprintf(&var_buffer[4][0], "Waiting For Reset\n");
                if(var_count != 255)
                {
                    sprintf(&var_buffer[4][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                }
                else if(Error_EF!=0)
                {
                    sprintf(&var_buffer[4][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF - 1][0]);
                }
                break;
            case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
                sprintf(&var_buffer[4][0], "Reset Applied\n");
                break;
            case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:
                sprintf(&var_buffer[4][0], "Waiting For Reset\n");
                break;
            default:
                sprintf(&var_buffer[4][0], "\n");
                break;
        }
            StSetText(((STATICTEXT*)(GOLFindObject(STE_512))), (XCHAR*)&var_buffer[4][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_512))), ST_DRAW);

            sprintf(&var_buffer[6][0],"%d\n",Disp_Info.Train_Speed);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_519))), (XCHAR*)&var_buffer[6][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_519))), ST_DRAW);

            sprintf(&var_buffer[5][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
            StSetText(((STATICTEXT*) (GOLFindObject(STE_484))), (XCHAR*) & var_buffer[5][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_484))), ST_DRAW);
}

void Update_C_2DP_Screen()
{

    if(Packet_src == 1)
    {
        StSetText(((STATICTEXT*)(GOLFindObject(STE_159))), (XCHAR*)"CONFR-2DP1S");
    }
    else
    {
        StSetText(((STATICTEXT*)(GOLFindObject(STE_159))), (XCHAR*)"CONF-2DP1S");        
    }
    SetState(((STATICTEXT*)(GOLFindObject(STE_159))), ST_DRAW);
    if(Disp_Info.Unit_Type == UT_SF)
    {
            StSetText(((STATICTEXT*)(GOLFindObject(STE_163))), (XCHAR*)"SF");
            SetState(((STATICTEXT*)(GOLFindObject(STE_163))), ST_DRAW);
            
            Wheel1.L_FC = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);         
            Wheel1.R_FC = GLCD_Info.CPU_Data[0][40] + ((UINT)(GLCD_Info.CPU_Data[0][41])<<8);
            Wheel1.L_RC = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);
            Wheel1.R_RC = GLCD_Info.CPU_Data[0][48] + ((UINT)(GLCD_Info.CPU_Data[0][49])<<8);
            
            Wheel2.L_FC = GLCD_Info.CPU_Data[1][38] + ((UINT)(GLCD_Info.CPU_Data[1][39])<<8);         
            Wheel2.R_FC = GLCD_Info.CPU_Data[1][40] + ((UINT)(GLCD_Info.CPU_Data[1][41])<<8);
            Wheel2.L_RC = GLCD_Info.CPU_Data[1][46] + ((UINT)(GLCD_Info.CPU_Data[1][47])<<8);
            Wheel2.R_RC = GLCD_Info.CPU_Data[1][48] + ((UINT)(GLCD_Info.CPU_Data[1][49])<<8);
            if(Packet_src == 1)
            {
                Disp_Info1.Reset_mode = GLCD_Info.CPU_Data[0][10];
                Disp_Info1.DS_mode = Disp_Info1.Reset_mode & 0x0F;
                Disp_Info1.US_mode = (Disp_Info1.Reset_mode & 0xF0)>>4;
                if(Disp_Info1.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
                {
                    Wheel1.L_FC = Wheel1.L_FC - Track_Wheel1.L_FC;
                    Wheel1.R_FC = Wheel1.R_FC - Track_Wheel1.R_FC;
                    Wheel1.L_RC = Wheel1.L_RC - Track_Wheel1.L_RC;
                    Wheel1.R_RC = Wheel1.R_RC - Track_Wheel1.R_RC;
                }
                else if(Disp_Info1.DS_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
                {
                    if(Disp_Info1.DS_mode == SECTION_CLEAR_AT_BOTH_UNITS)
                    {
                        Track_Wheel1.L_FC = Wheel1.L_FC;
                        Track_Wheel1.R_FC = Wheel1.R_FC;
                        Track_Wheel1.L_RC = Wheel1.L_RC;
                        Track_Wheel1.R_RC = Wheel1.R_RC;
                    }
                    else
                    {
                        Track_Wheel1.L_FC = 0;
                        Track_Wheel1.R_FC = 0;
                        Track_Wheel1.L_RC = 0;
                        Track_Wheel1.R_RC = 0;
                    }
                    Wheel1.L_FC = 0;
                    Wheel1.R_FC = 0;
                    Wheel1.L_RC = 0;
                    Wheel1.R_RC = 0;
                }
                
                Disp_Info2.Reset_mode = GLCD_Info.CPU_Data[1][10];
                Disp_Info2.DS_mode = Disp_Info2.Reset_mode & 0x0F;
                Disp_Info2.US_mode = (Disp_Info2.Reset_mode & 0xF0)>>4;
                if(Disp_Info2.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
                {
                    Wheel2.L_FC = Wheel2.L_FC - Track_Wheel2.L_FC;
                    Wheel2.R_FC = Wheel2.R_FC - Track_Wheel2.R_FC;
                    Wheel2.L_RC = Wheel2.L_RC - Track_Wheel2.L_RC;
                    Wheel2.R_RC = Wheel2.R_RC - Track_Wheel2.R_RC;
                }
                else if(Disp_Info2.DS_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
                {
                    if(Disp_Info2.DS_mode == SECTION_CLEAR_AT_BOTH_UNITS)
                    {
                        Track_Wheel2.L_FC = Wheel2.L_FC;
                        Track_Wheel2.R_FC = Wheel2.R_FC;
                        Track_Wheel2.L_RC = Wheel2.L_RC;
                        Track_Wheel2.R_RC = Wheel2.R_RC;
                    }
                    else
                    {
                        Track_Wheel2.L_FC = 0;
                        Track_Wheel2.R_FC = 0;
                        Track_Wheel2.L_RC = 0;
                        Track_Wheel2.R_RC = 0;
                    }
                    Wheel2.L_FC = 0;
                    Wheel2.R_FC = 0;
                    Wheel2.L_RC = 0;
                    Wheel2.R_RC = 0;
                }
            }
            
            // Local Forward count CPU1
            sprintf(&var_buffer[0][0],"%d\n",Wheel1.L_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_302))), (XCHAR*)&var_buffer[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_302))), ST_DRAW);

            // Local Forward count CPU2
            sprintf(&var_buffer2[0][0],"%d\n",Wheel2.L_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_303))), (XCHAR*)&var_buffer2[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_303))), ST_DRAW);

            // Remote Forward count CPU1
            sprintf(&var_buffer[1][0],"%d\n",Wheel1.R_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_306))), (XCHAR*)&var_buffer[1][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_306))), ST_DRAW);

            // Remote Forward count CPU2
            sprintf(&var_buffer2[1][0],"%d\n",Wheel2.R_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_307))), (XCHAR*)&var_buffer2[1][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_307))), ST_DRAW);

            /*Reverse count*/
            // Local Reverse count CPU1
            
            sprintf(&var_buffer[2][0],"%d\n",Wheel1.L_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_304))), (XCHAR*)&var_buffer[2][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_304))), ST_DRAW);
            
            // Local Reverse count CPU2
            sprintf(&var_buffer2[2][0],"%d\n",Wheel2.L_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_305))), (XCHAR*)&var_buffer2[2][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_305))), ST_DRAW);

            // Remote Reverse count CPU1
            sprintf(&var_buffer[3][0],"%d\n",Wheel1.R_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_308))), (XCHAR*)&var_buffer[3][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_308))), ST_DRAW);
            
            // Remote Reverse count CPU2
            sprintf(&var_buffer2[3][0],"%d\n",Wheel2.R_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_309))), (XCHAR*)&var_buffer2[3][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_309))), ST_DRAW);

            if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS || Disp_Info.DS_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
            {
                if(Wheel1.L_FC==0 && Wheel1.R_FC==0 && Wheel1.L_RC==0 && Wheel1.R_RC==0)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
                }
                else if(Wheel1.L_FC != Wheel1.R_FC)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
                }
                else
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
                }
            }
            else
            {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
            }
    }
    else if(Disp_Info.Unit_Type == UT_EF)
    {

            StSetText(((STATICTEXT*)(GOLFindObject(STE_163))), (XCHAR*)"EF");
            SetState(((STATICTEXT*)(GOLFindObject(STE_163))), ST_DRAW);

            Wheel1.L_FC = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);         
            Wheel1.R_FC = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
            Wheel1.L_RC = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
            Wheel1.R_RC = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
            
            Wheel2.L_FC = GLCD_Info.CPU_Data[1][30] + ((UINT)(GLCD_Info.CPU_Data[1][31])<<8);         
            Wheel2.R_FC = GLCD_Info.CPU_Data[1][32] + ((UINT)(GLCD_Info.CPU_Data[1][33])<<8);
            Wheel2.L_RC = GLCD_Info.CPU_Data[1][22] + ((UINT)(GLCD_Info.CPU_Data[1][23])<<8);
            Wheel2.R_RC = GLCD_Info.CPU_Data[1][24] + ((UINT)(GLCD_Info.CPU_Data[1][25])<<8);

            if(Packet_src == 1)
            {
                Disp_Info1.Reset_mode = GLCD_Info.CPU_Data[0][10];
                Disp_Info1.DS_mode = Disp_Info1.Reset_mode & 0x0F;
                Disp_Info1.US_mode = (Disp_Info1.Reset_mode & 0xF0)>>4;
                if(Disp_Info1.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
                {
                    Wheel1.L_FC = Wheel1.L_FC - Track_Wheel1.L_FC;
                    Wheel1.R_FC = Wheel1.R_FC - Track_Wheel1.R_FC;
                    Wheel1.L_RC = Wheel1.L_RC - Track_Wheel1.L_RC;
                    Wheel1.R_RC = Wheel1.R_RC - Track_Wheel1.R_RC;
                }
                else if(Disp_Info1.US_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
                {
                    if(Disp_Info1.US_mode == SECTION_CLEAR_AT_BOTH_UNITS)
                    {
                        Track_Wheel1.L_FC = Wheel1.L_FC;
                        Track_Wheel1.R_FC = Wheel1.R_FC;
                        Track_Wheel1.L_RC = Wheel1.L_RC;
                        Track_Wheel1.R_RC = Wheel1.R_RC;
                    }
                    else
                    {
                        Track_Wheel1.L_FC = 0;
                        Track_Wheel1.R_FC = 0;
                        Track_Wheel1.L_RC = 0;
                        Track_Wheel1.R_RC = 0;
                    }
                    Wheel1.L_FC = 0;
                    Wheel1.R_FC = 0;
                    Wheel1.L_RC = 0;
                    Wheel1.R_RC = 0;
                }
                
                Disp_Info2.Reset_mode = GLCD_Info.CPU_Data[1][10];
                Disp_Info2.DS_mode = Disp_Info2.Reset_mode & 0x0F;
                Disp_Info2.US_mode = (Disp_Info2.Reset_mode & 0xF0)>>4;
                if(Disp_Info2.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
                {
                    Wheel2.L_FC = Wheel2.L_FC - Track_Wheel2.L_FC;
                    Wheel2.R_FC = Wheel2.R_FC - Track_Wheel2.R_FC;
                    Wheel2.L_RC = Wheel2.L_RC - Track_Wheel2.L_RC;
                    Wheel2.R_RC = Wheel2.R_RC - Track_Wheel2.R_RC;
                }
                else if(Disp_Info2.US_mode != SECTION_WAIT_FOR_PILOT_TRAIN)
                {
                    if(Disp_Info2.US_mode == SECTION_CLEAR_AT_BOTH_UNITS)
                    {
                        Track_Wheel2.L_FC = Wheel2.L_FC;
                        Track_Wheel2.R_FC = Wheel2.R_FC;
                        Track_Wheel2.L_RC = Wheel2.L_RC;
                        Track_Wheel2.R_RC = Wheel2.R_RC;
                    }
                    else 
                    {
                        Track_Wheel2.L_FC = 0;
                        Track_Wheel2.R_FC = 0;
                        Track_Wheel2.L_RC = 0;
                        Track_Wheel2.R_RC = 0;
                    }                    
                    Wheel2.L_FC = 0;
                    Wheel2.R_FC = 0;
                    Wheel2.L_RC = 0;
                    Wheel2.R_RC = 0;
                }
            }
            
            // Local Forward count CPU1

            sprintf(&var_buffer[0][0],"%d\n",Wheel1.L_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_302))), (XCHAR*)&var_buffer[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_302))), ST_DRAW);


            // Local Forward count CPU2
            sprintf(&var_buffer2[0][0],"%d\n",Wheel2.L_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_303))), (XCHAR*)&var_buffer2[0][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_303))), ST_DRAW);

            // Remote Forward count CPU1
            sprintf(&var_buffer[1][0],"%d\n",Wheel1.R_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_306))), (XCHAR*)&var_buffer[1][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_306))), ST_DRAW);

            // Remote Forward count CPU2
            sprintf(&var_buffer2[1][0],"%d\n",Wheel2.R_RC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_307))), (XCHAR*)&var_buffer2[1][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_307))), ST_DRAW);

            /*Reverse count*/
                        // Local Reverse count CPU1
            
            sprintf(&var_buffer[2][0],"%d\n",Wheel1.L_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_304))), (XCHAR*)&var_buffer[2][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_304))), ST_DRAW);


            // Local Reverse count CPU2
            sprintf(&var_buffer2[2][0],"%d\n",Wheel2.L_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_305))), (XCHAR*)&var_buffer2[2][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_305))), ST_DRAW);

            // Remote Reverse count CPU1
            sprintf(&var_buffer[3][0],"%d\n",Wheel1.R_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_308))), (XCHAR*)&var_buffer[3][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_308))), ST_DRAW);

            // Remote Reverse count CPU2
            sprintf(&var_buffer2[3][0],"%d\n",Wheel2.R_FC);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_309))), (XCHAR*)&var_buffer2[3][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_309))), ST_DRAW);

            if(Disp_Info.US_mode == SECTION_OCCUPIED_AT_BOTH_UNITS || Disp_Info.US_mode == SECTION_WAIT_FOR_PILOT_TRAIN)
            {
                if(Wheel1.L_FC==0 && Wheel1.R_FC==0 && Wheel1.L_RC==0 && Wheel1.R_RC==0)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
                }
                else if(Wheel1.L_FC != Wheel1.R_FC)
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&logo_small2_db2);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
                }
                else
                {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&logo_small2_db2_inv);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
                }
            }
            else
            {
                    PictSetBitmap(((PICTURE*)(GOLFindObject(PCB_205))), (void*)&No_Train);
                    SetState(((PICTURE*)(GOLFindObject(PCB_205))), PICT_DRAW);
            }
    }
            //error number
            var_count = GLCD_Info.CPU_Data[0][54];
            Error_SF = GLCD_Info.CPU_Data[0][11];
            Error_EF = GLCD_Info.CPU_Data[0][12];
                if(Disp_Info.Unit_Type == UT_SF)
                {
                    switch(Disp_Info.DS_mode)
					{
						case WAITING_FOR_RESET_AT_BOTH_UNITS:
							sprintf(&var_buffer[5][0], "Waiting For Reset\n");
							sprintf(&var_buffer[4][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[5][0], "Waiting For Reset\n");
							sprintf(&var_buffer[4][0], "Reset Applied\n");
							break;
						case RESET_APPLIED_AT_REMOTE_UNIT:
							sprintf(&var_buffer[5][0], "Reset Applied\n");
							sprintf(&var_buffer[4][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[5][0], "Reset Applied\n");
							sprintf(&var_buffer[4][0], "Reset Applied\n");
							break;
						case SECTION_WAIT_FOR_PILOT_TRAIN:
                            if(Wheel1.L_FC==0 && Wheel1.R_FC==0 && Wheel1.L_RC==0 && Wheel1.R_RC==0)
                            {
							sprintf(&var_buffer[4][0], "Waiting for pilot train\n");
							sprintf(&var_buffer[5][0], "Waiting for pilot train\n");
							}
                            else
                            {
							sprintf(&var_buffer[4][0], "Pilot train in progress\n");
							sprintf(&var_buffer[5][0], "Pilot train in progress\n");
                            }break;
						case SECTION_CLEAR_AT_BOTH_UNITS:
							sprintf(&var_buffer[4][0], "SECTION CLEAR\n");
							sprintf(&var_buffer[5][0], "SECTION CLEAR\n");
							break;
						case SECTION_OCCUPIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[4][0], "SECTION OCCUPIED\n");
							sprintf(&var_buffer[5][0], "SECTION OCCUPIED\n");
							break;
						case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
							sprintf(&var_buffer[4][0], "Waiting for Reset\n");
							if(var_count!=255)
							{
								sprintf(&var_buffer[5][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[5][0], "Reset Applied\n");
							}
							break;
						case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
							sprintf(&var_buffer[5][0], "Waiting for Reset\n");
							if(var_count!=255)
							{
								sprintf(&var_buffer[4][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[4][0], "Reset Applied\n");
							}
							break;
						case SECTION_ERROR_AT_BOTH_UNITS:
							sprintf(&var_buffer[5][0],"System Failure\n");
                                                        if(var_count!= 255)
                                                        {
                                                            sprintf(&var_buffer[4][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                                                        }
                                                        else
                                                        {
                                                            sprintf(&var_buffer[4][0]," \n");
                                                        }
							break;
						case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[4][0],"Reset Applied\n");
							if(var_count!=255)
							{
								sprintf(&var_buffer[5][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[5][0],"Waiting for Reset\n");
							}
							break;
						case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:

							sprintf(&var_buffer[5][0],"Reset Applied\n");
							if(Error_SF!=0)
							{
								sprintf(&var_buffer[4][0],"%d %s\n",Error_SF - 1,&Error_message[Error_SF - 1][0]);
							}
							else
							{
								sprintf(&var_buffer[4][0],"Waiting for Reset\n");
							}
							break;
						default:
							sprintf(&var_buffer[4][0], "\n");
							sprintf(&var_buffer[5][0], "\n");
							break;
					}
                }
                if(Disp_Info.Unit_Type == UT_EF)
                {
					switch(Disp_Info.US_mode)
					{
						case WAITING_FOR_RESET_AT_BOTH_UNITS:
							sprintf(&var_buffer[5][0], "Waiting For Reset\n");
							sprintf(&var_buffer[4][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[5][0], "Waiting For Reset\n");
							sprintf(&var_buffer[4][0], "Reset Applied\n");
							break;
						case RESET_APPLIED_AT_REMOTE_UNIT:
							sprintf(&var_buffer[5][0], "Reset Applied\n");
							sprintf(&var_buffer[4][0], "Waiting For Reset\n");
							break;
						case RESET_APPLIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[5][0], "Reset Applied\n");
							sprintf(&var_buffer[4][0], "Reset Applied\n");
							break;
						case SECTION_WAIT_FOR_PILOT_TRAIN:
							if(Wheel1.L_FC==0 && Wheel1.R_FC==0 && Wheel1.L_RC==0 && Wheel1.R_RC==0)
                            {
                                sprintf(&var_buffer[4][0], "Waiting for pilot train\n");
                                sprintf(&var_buffer[5][0], "Waiting for pilot train\n");
                            }
                            else
                            {
                                sprintf(&var_buffer[4][0], "Pilot train in progress\n");
                                sprintf(&var_buffer[5][0], "Pilot train in progress\n");                                
                            }
							break;
						case SECTION_CLEAR_AT_BOTH_UNITS:
							sprintf(&var_buffer[4][0], "SECTION CLEAR\n");
							sprintf(&var_buffer[5][0], "SECTION CLEAR\n");
							break;
						case SECTION_OCCUPIED_AT_BOTH_UNITS:
							sprintf(&var_buffer[4][0], "SECTION OCCUPIED\n");
							sprintf(&var_buffer[5][0], "SECTION OCCUPIED\n");
							break;
						case ERROR_LOCAL_UNIT_WAITING_FOR_RESET:
							sprintf(&var_buffer[4][0], "Waiting for Reset\n");
							if(var_count!=255)
							{
								sprintf(&var_buffer[5][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[5][0], "Reset Applied\n");
							}
							break;
						case ERROR_REMOTE_UNIT_WAITING_FOR_RESET:
							sprintf(&var_buffer[5][0], "Waiting for Reset\n");
							if(var_count!=255)
							{
								sprintf(&var_buffer[4][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[4][0], "Reset Applied\n");
							}
							break;
						case SECTION_ERROR_AT_BOTH_UNITS:
							sprintf(&var_buffer[5][0],"System Failure\n");
                                                        if(var_count!=255)
                                                        {
                                                            sprintf(&var_buffer[4][0],"%d %s\n",var_count,&Error_message[var_count][0]);
                                                        }
                                                        else
                                                        {
                                                            sprintf(&var_buffer[4][0]," \n");
                                                        }
							break;
						case ERROR_RESET_APPLIED_AT_LOCAL_UNIT:
							sprintf(&var_buffer[4][0],"Reset Applied\n");
							if(var_count!=255)
							{
								sprintf(&var_buffer[5][0],"%d %s\n",var_count,&Error_message[var_count][0]);
							}
							else
							{
								sprintf(&var_buffer[5][0],"Waiting for Reset\n");
							}
							break;
						case ERROR_RESET_APPLIED_AT_REMOTE_UNIT:

							sprintf(&var_buffer[5][0],"Reset Applied\n");
							if(Error_EF!=0)
							{
								sprintf(&var_buffer[4][0],"%d %s\n",Error_EF - 1,&Error_message[Error_EF-1][0]);
							}
							else
							{
								sprintf(&var_buffer[4][0],"Waiting for Reset\n");
							}
							break;
						default:
							sprintf(&var_buffer[4][0], "\n");
							sprintf(&var_buffer[5][0], "\n");
							break;
					}
				}

            StSetText(((STATICTEXT*)(GOLFindObject(STE_175))), (XCHAR*)&var_buffer[4][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_175))), ST_DRAW);

            StSetText(((STATICTEXT*)(GOLFindObject(STE_266))), (XCHAR*)&var_buffer[5][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_266))), ST_DRAW);

            sprintf(&var_buffer[6][0],"%d\n",Disp_Info.Train_Speed);
            StSetText(((STATICTEXT*)(GOLFindObject(STE_196))), (XCHAR*)&var_buffer[6][0]);
            SetState(((STATICTEXT*)(GOLFindObject(STE_196))), ST_DRAW);

            sprintf(&var_buffer[7][0], "C:0x%04X%04X|V:S%03d\n",DAC_sysinfo.Checksum.DWord.HiWord.Word,DAC_sysinfo.Checksum.DWord.LoWord.Word,CPU_Version);
            StSetText(((STATICTEXT*) (GOLFindObject(STE_157))), (XCHAR*) & var_buffer[7][0]);
            SetState(((STATICTEXT*) (GOLFindObject(STE_157))), ST_DRAW);
}

void Update_Time_Date(void)
{
    switch(Disp_Info.Screen)
    {
        case HOME:
            break;
        case C_DE:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_499))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_499))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_498))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_498))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_485))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_485))), ST_DRAW);
            break;
        case C_2DP:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_165))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_165))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_164))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_164))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_158))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_158))), ST_DRAW);
            break;
        case C_3D2S:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_94))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_94))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_93))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_93))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_90))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_90))), ST_DRAW);
            break;
        case C_3DPAS:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_208))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_208))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_207))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_207))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_204))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_204))), ST_DRAW);
            break;
        case C_LCWS:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_227))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_227))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_226))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_226))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_223))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_223))), ST_DRAW);
                    break;
        case C_4D1S:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_536))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_536))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_535))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_535))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_532))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_532))), ST_DRAW);
            break;
        case SMCPU:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_172))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_172))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_171))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_171))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_168))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_168))), ST_DRAW);
            break;
        case RESET:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_254))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_254))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_253))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_253))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_249))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_249))), ST_DRAW);

            break;
        case C_WAIT_FOR_RESET:
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_493))), (XCHAR*)SMCPU_sysinfo.Disp_Time);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_493))), ST_DRAW);

                    StSetText(((STATICTEXT*)(GOLFindObject(STE_492))), (XCHAR*)SMCPU_sysinfo.Disp_Date);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_492))), ST_DRAW);

                    if(Disp_Info.CPU_Type == D_CPU1)
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU1_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    else
                    {
                        sprintf(&var_buffer_d[0][0],"A:%03d | N:%03d\n",Disp_Info.CPU2_address, SMCPU_sysinfo.Event_Logger_ID);
                    }
                    StSetText(((STATICTEXT*)(GOLFindObject(STE_489))), (XCHAR*)&var_buffer_d[0][0]);
                    SetState(((STATICTEXT*)(GOLFindObject(STE_489))), ST_DRAW);
        default:
            break;
    }
}

void Update_Debug_blocks(void)
{
    var_count = GLCD_Info.CPU_Data[0][0];
    sprintf(&var_buffer[0][0],"A:%03d | N:%03d\n",var_count,SMCPU_sysinfo.Event_Logger_ID);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_249))), (XCHAR*)&var_buffer[0][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_249))), ST_DRAW);

    var_count = checksum_G;
    sprintf(&var_buffer[1][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_419))), (XCHAR*)&var_buffer[1][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_419))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][9];
    sprintf(&var_buffer[2][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_420))), (XCHAR*)&var_buffer[2][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_420))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][10];
    sprintf(&var_buffer[3][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_421))), (XCHAR*)&var_buffer[3][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_421))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][11];
    sprintf(&var_buffer[4][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_422))), (XCHAR*)&var_buffer[4][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_422))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][12];
    sprintf(&var_buffer[5][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_423))), (XCHAR*)&var_buffer[5][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_423))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][13] + ((UINT)(GLCD_Info.CPU_Data[0][14])<<8);
    sprintf(&var_buffer[6][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_424))), (XCHAR*)&var_buffer[6][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_424))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][15] + ((UINT)(GLCD_Info.CPU_Data[0][16])<<8);
    sprintf(&var_buffer[7][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_425))), (XCHAR*)&var_buffer[7][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_425))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][17] + ((UINT)(GLCD_Info.CPU_Data[0][18])<<8);
    sprintf(&var_buffer[8][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_426))), (XCHAR*)&var_buffer[8][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_426))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][19] + ((UINT)(GLCD_Info.CPU_Data[0][20])<<8);
    sprintf(&var_buffer[9][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_427))), (XCHAR*)&var_buffer[9][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_427))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][21];
    sprintf(&var_buffer[10][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_428))), (XCHAR*)&var_buffer[10][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_428))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][22] + ((UINT)(GLCD_Info.CPU_Data[0][23])<<8);
    sprintf(&var_buffer[11][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_429))), (XCHAR*)&var_buffer[11][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_429))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][24] + ((UINT)(GLCD_Info.CPU_Data[0][25])<<8);
    sprintf(&var_buffer[12][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_430))), (XCHAR*)&var_buffer[12][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_430))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][26] + ((UINT)(GLCD_Info.CPU_Data[0][27])<<8);
    sprintf(&var_buffer[13][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_431))), (XCHAR*)&var_buffer[13][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_431))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][28] + ((UINT)(GLCD_Info.CPU_Data[0][29])<<8);
    sprintf(&var_buffer[14][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_432))), (XCHAR*)&var_buffer[14][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_432))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][30] + ((UINT)(GLCD_Info.CPU_Data[0][31])<<8);
    sprintf(&var_buffer[15][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_433))), (XCHAR*)&var_buffer[15][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_433))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][32] + ((UINT)(GLCD_Info.CPU_Data[0][33])<<8);
    sprintf(&var_buffer[16][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_434))), (XCHAR*)&var_buffer[16][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_434))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][34] + ((UINT)(GLCD_Info.CPU_Data[0][35])<<8);
    sprintf(&var_buffer[17][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_437))), (XCHAR*)&var_buffer[17][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_437))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][36] + ((UINT)(GLCD_Info.CPU_Data[0][37])<<8);
    sprintf(&var_buffer[18][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_438))), (XCHAR*)&var_buffer[18][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_438))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][38] + ((UINT)(GLCD_Info.CPU_Data[0][39])<<8);
    sprintf(&var_buffer[19][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_439))), (XCHAR*)&var_buffer[19][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_439))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][40] + ((UINT)(GLCD_Info.CPU_Data[0][41])<<8);
    sprintf(&var_buffer[20][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_440))), (XCHAR*)&var_buffer[20][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_440))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][42] + ((UINT)(GLCD_Info.CPU_Data[0][43])<<8);
    sprintf(&var_buffer[21][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_441))), (XCHAR*)&var_buffer[21][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_441))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][44] + ((UINT)(GLCD_Info.CPU_Data[0][45])<<8);
    sprintf(&var_buffer[22][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_442))), (XCHAR*)&var_buffer[22][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_442))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][46] + ((UINT)(GLCD_Info.CPU_Data[0][47])<<8);
    sprintf(&var_buffer[23][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_443))), (XCHAR*)&var_buffer[23][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_443))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][48] + ((UINT)(GLCD_Info.CPU_Data[0][49])<<8);
    sprintf(&var_buffer[24][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_444))), (XCHAR*)&var_buffer[24][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_444))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][50] + ((UINT)(GLCD_Info.CPU_Data[0][51])<<8);
    sprintf(&var_buffer[25][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_445))), (XCHAR*)&var_buffer[25][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_445))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][52] + ((UINT)(GLCD_Info.CPU_Data[0][53])<<8);
    sprintf(&var_buffer[26][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_446))), (XCHAR*)&var_buffer[26][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_446))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][54];
    sprintf(&var_buffer[27][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_447))), (XCHAR*)&var_buffer[27][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_447))), ST_DRAW);


    var_count = DAC_sysinfo.Unit_Type;
    sprintf(&var_buffer[28][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_448))), (XCHAR*)&var_buffer[28][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_448))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][1];
    sprintf(&var_buffer[29][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_449))), (XCHAR*)&var_buffer[29][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_449))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][2];
    sprintf(&var_buffer[30][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_450))), (XCHAR*)&var_buffer[30][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_450))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][3];
    sprintf(&var_buffer[31][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_451))), (XCHAR*)&var_buffer[31][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_451))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][4];
    sprintf(&var_buffer[32][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_452))), (XCHAR*)&var_buffer[32][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_452))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][5];
    sprintf(&var_buffer[33][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_453))), (XCHAR*)&var_buffer[33][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_453))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][6];
    sprintf(&var_buffer[34][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_454))), (XCHAR*)&var_buffer[34][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_454))), ST_DRAW);

    var_count = GLCD_Info.CPU_Data[0][7];
    sprintf(&var_buffer[35][0],"%04x\n",var_count);
    StSetText(((STATICTEXT*)(GOLFindObject(STE_455))), (XCHAR*)&var_buffer[35][0]);
    SetState(((STATICTEXT*)(GOLFindObject(STE_455))), ST_DRAW);

}

volatile unsigned int sense_key,key;
volatile unsigned int temp_count,temp_count2;
int main(void)
{
    GOL_MSG msg;                    // GOL message structure to interact with GOL
    spi_count = 0;
    Home_Screen_Time_Up =0;
    Screen_Off = 0;
    Next = 1;
    InitializeBoard();
    BKLT_CNTRL_DIR = 0;
    BKLT_CNTRL = 1;
    GDDDemoCreateFirstScreen();
    GCPU_ACTIVE = 1;
    while(1)
    {
        Update_SPI_Data();
        sense_key = SW_WAKE;
        if(sense_key == 1)
        {
            for(temp_count = 0; temp_count <0x100; temp_count++);//minute delay
            sense_key = 0;
            for(temp_count = 0; temp_count <15; temp_count++)
            {
                key = SW_WAKE;
                if(key == 1)
                    sense_key++;                     
                for(temp_count2 = 0; temp_count2 <0xFF; temp_count2++);
            }
            if(sense_key == 15)
                sense_key = 1;
            else
                sense_key = 0;
            while(sense_key)
            {
                //if pressed
                while(SW_WAKE == 1);//wait till key is removed
                for(temp_count = 0; temp_count <0x100; temp_count++);//minute delay
                sense_key = 0;
                for(temp_count = 0; temp_count <15; temp_count++)
                {
                    key = SW_WAKE;
                    if(key == 1)
                        sense_key++;                     
                    for(temp_count2 = 0; temp_count2 <0xFF; temp_count2++);
                }
                if(sense_key == 15)
                {
                    sense_key = 1;
                }
                else
                {
                    sense_key = 1;
                    break;
                }
            }
        }
        
        if(Screen_Off)  //Screen is off, wake when key is pressed
        {
            ClearDevice();
            BKLT_CNTRL = 0;
            GCPU_ACTIVE = 0;
            if(sense_key== 1)
            {
 //               InitializeBoard();
                Wake_screen();
                BKLT_CNTRL = 1;
                Screen_Off = 0;
                GCPU_ACTIVE = 1;
                Disp_Info.Screen = HOME;
                Home_Screen_Time_Up =2;
            }
        }
        else
        {
            GCPU_ACTIVE = 1;
            if(sense_key == 1) //z
            {
                Screen_Off = 1;
            }
            else
            {
                if(GOLDraw())               // Draw GOL object
                {
                    TouchGetMsg(&msg);      // Get message from touch screen
                    Update_GLCD_Data();
                    GOLMsg(&msg);           // Process message
                }
            }
        }
    }//end while
}

