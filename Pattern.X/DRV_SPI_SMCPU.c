#include <xc.h>
#include <PPS.h>
#include "Main.h"
#include "DRV_SPI_SMCPU.h"
#include "Common_DAC_info.h"
#include "GDD_Screens.h"
#include "CRC16.h"

extern disp_info_t Disp_Info;
unsigned char temp;
glcd_info_t GLCD_Info;		/* structure that handles Lcd scheduler and holds lcd Information */
dac_sysinfo_t DAC_sysinfo;
SMCPU_info_t  SMCPU_sysinfo;
extern volatile unsigned char Home_Screen_Time_Up;
unsigned char Packet_src = 0;
extern BYTE Next_Pressed;
void SPI_Initialize()
{
    SPI1CON1 = 0;
    SPI1CON2 = 0;
    SPI1STAT = 0;
        U1CNFG2bits.UTRDIS = 1;
    #ifdef  __PIC24FJ256DA210__
    __builtin_write_OSCCONL(OSCCON & 0xbf); //clear the bit 6 of OSCCONL to unlock Pin Re-map
        iPPSOutput(OUT_PIN_PPS_RP18,OUT_FN_PPS_SDO1);  //RP19 - RG8 - SDO
        iPPSInput(IN_FN_PPS_SCK1IN,IN_PIN_PPS_RPI37);    //RP26 - RG7 - SDI
        iPPSInput(IN_FN_PPS_SDI1,IN_PIN_PPS_RP16);  //RPI40 - RC3 - SS
    __builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock Pin Re-map
    #endif

    TRISFbits.TRISF3 = 1;            // set  SDI1 as an input
    TRISBbits.TRISB5 = 0;            // set  SDO1 as an output
    TRISCbits.TRISC14 = 1;             // set  SCK1 as an output
    TRISGbits.TRISG3 = 1;             //  set  SS1 as an input

  SPI1CON2bits.SPIBEN = 1;
  IFS0bits.SPI1IF = 0;

  IEC0bits.SPI1IE = 0;

    SPI1STAT = 0;
    SPI1CON1 = 0;
    SPI1CON2 = 0;               //

    SPI1CON1bits.DISSCK = 0;    // Internal SPI Clock is enabled
                                // Idle state for the clock is a high level; active state is a low level
                                // Serial output data changes on transition from Idle clock state to active clock state
    SPI1CON1bits.DISSDO = 1;    // Internal SDO is Disabled for now. It will be enabled when the SS is asserted.
    SPI1CON1bits.MODE16 = 0;    // Operate in 8 bit mode
    SPI1CON1bits.SSEN = 0;      // Disable SS pin, as it is used as port
    SPI1CON1bits.CKP = 1;       // Idle state for the clock is a high
    SPI1CON1bits.MSTEN = 0;     // Operate in Slave Mode
    SPI1STATbits.SPIROV = 0;
    SPI1CON2bits.SPIBEN = 1;    // Enable 8 Byte Enhanced SPI Buffers for both Tx and Rx

    IEC0bits.SPI1IE = 0;        // Disable SPI Interrupts
    IEC0bits.SPF1IE = 0;

    SPI1STATbits.SPIEN = 0;     // Enable Module
    SPI1CON1bits.DISSDO = 0;            // Enable the SDO Pin.
    temp = SPI1BUF;
    SPI1STATbits.SPIROV = 0;
    SPI1CON2bits.SPIBEN = 1;    // Enable 8 Byte Enhanced SPI Buffers for both Tx and Rx
    SPI1STATbits.SPIEN = 1;     // Enable Module

    GLCD_Info.Packet_Max_length = MAX_G_PACKET_LEN;
    de_init_spi();
}

BOOL Update_GLCD;
    unsigned char Temp;
    UINT16 checksum_G = 0;
void Update_SPI_Data(void)
{
	switch (GLCD_Info.State)
	{
                case GLCD_SS_WAIT:
                        if(G_SS == 0)
                        {
                            re_init_spi();
                            GLCD_Info.Packet_index =0;
                            GLCD_Info.State = GLCD_RCV_DATA;
                        }
                        else
                        {
                            GLCD_Info.Packet_index = 0;
                        }
                        break;
		case GLCD_RCV_DATA:
                if(G_SS == 1)
                {
                    GLCD_Info.State = GLCD_SS_WAIT;
                    break;
                }
                //
                if (SPI1STATbits.SRXMPT == 0)
                {
                    GLCD_Info.Rx_Message_Buffer[GLCD_Info.Packet_index] = SPI1BUF;
                    GLCD_Info.Packet_index++;
                }
                if(SPI1STATbits.SPITBF == 0) 
                {
                    if(Next_Pressed)
                    {
                        SPI1BUF = 0xC3;
                    }
                    else
                    {
                        SPI1BUF = 0x3C;
                    }
                }
                if(GLCD_Info.Packet_index >= GLCD_Info.Packet_Max_length)
                {
                    GLCD_Info.State = GLCD_PROCESS_RX_PACKET;
                }
			break;

		case GLCD_PROCESS_RX_PACKET:
                    Next_Pressed = 0;
                checksum_G = Crc16((const BYTE *)GLCD_Info.Rx_Message_Buffer, MAX_G_PACKET_LEN-2);
                de_init_spi();
                //GLCD_Info.Packet_index =0;
        		if (checksum_G == ((UINT16)((UINT16)GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN-1]<<8)+(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN-2])))
                //if(checksum_G == ((UINT16)((UINT16)GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN-1]<<8)+(GLCD_Info.Rx_Message_Buffer[MAX_G_PACKET_LEN-2])))                
                //if(1)
                {
                    Process_Rx_Packet();
                    Update_GLCD = 1;
                }
                GLCD_Info.State = GLCD_SS_WAIT;
			break;
                default:
			break;
	}
 
}
#include <time.h>
   struct tm  ts;
    char       buf[80];
    wordtype_t COM_DS_pkt_error_cnt, COM_US_pkt_error_cnt;
    unsigned char Active_data = 1;
void Process_Rx_Packet(void)
{
    unsigned char u_count;
    for(u_count = 0;u_count<(CPU_PACKET_LEN);u_count++)
    {
        GLCD_Info.CPU_Data[CPU1][u_count] = GLCD_Info.Rx_Message_Buffer[u_count];
        GLCD_Info.CPU_Data[CPU2][u_count] = GLCD_Info.Rx_Message_Buffer[u_count+CPU_PACKET_LEN];
    }
    for(u_count = 0;u_count<(SMCPU_PACKET_LEN);u_count++)
    {
        GLCD_Info.SMCPU_Data[u_count] = GLCD_Info.Rx_Message_Buffer[(CPU_PACKET_LEN*2) + u_count];
    }

    DAC_sysinfo.Checksum.DWord.HiWord.Byte.Lo = GLCD_Info.Rx_Message_Buffer[((CPU_PACKET_LEN*2))];
    DAC_sysinfo.Checksum.DWord.HiWord.Byte.Hi = GLCD_Info.Rx_Message_Buffer[((CPU_PACKET_LEN*2))+1];
    DAC_sysinfo.Checksum.DWord.LoWord.Byte.Lo = GLCD_Info.Rx_Message_Buffer[((CPU_PACKET_LEN*2))+2];
    DAC_sysinfo.Checksum.DWord.LoWord.Byte.Hi = GLCD_Info.Rx_Message_Buffer[((CPU_PACKET_LEN*2))+3];
    DAC_sysinfo.Unit_Type = GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_UT];
    DAC_sysinfo.SW_Version = GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_SV];

    SMCPU_sysinfo.Event_count.DWord.LoWord.Byte.Lo  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_EC];
    SMCPU_sysinfo.Event_count.DWord.LoWord.Byte.Hi  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_EC + 1];
    SMCPU_sysinfo.Event_count.DWord.HiWord.Byte.Lo  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_EC + 2];
    SMCPU_sysinfo.Event_count.DWord.HiWord.Byte.Hi  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_EC + 3];

    SMCPU_sysinfo.Start_time.DWord.LoWord.Byte.Lo  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ST];
    SMCPU_sysinfo.Start_time.DWord.LoWord.Byte.Hi  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ST + 1];
    SMCPU_sysinfo.Start_time.DWord.HiWord.Byte.Lo  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ST + 2];
    SMCPU_sysinfo.Start_time.DWord.HiWord.Byte.Hi  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ST + 3];

    SMCPU_sysinfo.End_time.DWord.LoWord.Byte.Lo  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ET];
    SMCPU_sysinfo.End_time.DWord.LoWord.Byte.Hi  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ET + 1];
    SMCPU_sysinfo.End_time.DWord.HiWord.Byte.Lo  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ET + 2];
    SMCPU_sysinfo.End_time.DWord.HiWord.Byte.Hi  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_ET + 3];

    SMCPU_sysinfo.Present_time.DWord.LoWord.Byte.Lo  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_PT];
    SMCPU_sysinfo.Present_time.DWord.LoWord.Byte.Hi  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_PT + 1];
    SMCPU_sysinfo.Present_time.DWord.HiWord.Byte.Lo  =  GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_PT + 2];
    SMCPU_sysinfo.Present_time.DWord.HiWord.Byte.Hi  =   GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_PT + 3];

    SMCPU_sysinfo.Event_Logger_ID = GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_N_ID];
    if(GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_SRC_ID]==0x3c)
    {
        Packet_src = 1;
    }
    else{
        Packet_src = 0;
    }
    COM_DS_pkt_error_cnt.Byte.Lo = GLCD_Info.Rx_Message_Buffer[134];
    COM_DS_pkt_error_cnt.Byte.Hi = GLCD_Info.Rx_Message_Buffer[134+1];
    COM_US_pkt_error_cnt.Byte.Lo = GLCD_Info.Rx_Message_Buffer[136];
    COM_US_pkt_error_cnt.Byte.Hi = GLCD_Info.Rx_Message_Buffer[136+1];

    time_t Present_time,Start_time,End_time;
    Present_time = (time_t)SMCPU_sysinfo.Present_time.LWord;
    Start_time = (time_t)SMCPU_sysinfo.Start_time.LWord;
    End_time = (time_t)SMCPU_sysinfo.End_time.LWord;

    ts = *gmtime(&Present_time);
    strftime(SMCPU_sysinfo.Disp_Date, sizeof(SMCPU_sysinfo.Disp_Date), "%d:%m:%Y", &ts);
    strftime(SMCPU_sysinfo.Disp_Time, sizeof(SMCPU_sysinfo.Disp_Time), "%H:%M", &ts);

    ts = *gmtime(&Start_time);
    strftime(SMCPU_sysinfo.Start_Date, sizeof(SMCPU_sysinfo.Disp_Date), "%d:%m:%Y", &ts);

    ts = *gmtime(&End_time);
    strftime(SMCPU_sysinfo.End_Date, sizeof(SMCPU_sysinfo.Disp_Date), "%d:%m:%Y", &ts);

    sprintf(&SMCPU_sysinfo.Number_of_Event_String[0],"%ld\n",SMCPU_sysinfo.Event_count.LWord);

    if(Disp_Info.SPI_state == SPI_IDLE)
    {
        Disp_Info.SPI_state = SPI_DONE;
    }
    Disp_Info.Unit_Type = GLCD_Info.SMCPU_Data[SMCPU_PK_OFFSET_UT];
    Disp_Info.CPU1_address = GLCD_Info.CPU_Data[0][0];
    Disp_Info.CPU2_address = GLCD_Info.CPU_Data[1][0];
    Disp_Info.MessageID = GLCD_Info.CPU_Data[0][21];
    Disp_Info.Reset_mode = GLCD_Info.CPU_Data[0][10];

    Disp_Info.DS_mode = Disp_Info.Reset_mode & 0x0F;
    Disp_Info.US_mode = (Disp_Info.Reset_mode & 0xF0)>>4;

    Disp_Info.Train_Speed = GLCD_Info.CPU_Data[0][13] + ((UINT)(GLCD_Info.CPU_Data[0][14])<<8);
    if(Disp_Info.MessageID == NO_DATA && Active_data==1)
    {
        GDDDemoGoToScreen(HOME);
        Disp_Info.Prv_Screen = Disp_Info.Screen;
        Disp_Info.Screen = HOME;
        Active_data = 0;
        return;
    }
    else if(Disp_Info.MessageID != NO_DATA && Active_data==0)
    {
        Active_data = 1;
        Home_Screen_Time_Up = 2;
    }

    if(Disp_Info.Screen==C_WAIT_FOR_RESET && Disp_Info.MessageID == RESET_APPLIED)
    {
        if(Disp_Info.Unit_Type == UT_SF || Disp_Info.Unit_Type == UT_EF)
        {
            GDDDemoGoToScreen(C_2DP);
            Disp_Info.Prv_Screen = Disp_Info.Screen;
            Disp_Info.Screen = C_2DP;
        }
        else if(Disp_Info.Unit_Type == UT_DE)
        {
            GDDDemoGoToScreen(C_DE);
            Disp_Info.Prv_Screen = Disp_Info.Screen;
            Disp_Info.Screen = C_DE;
        }
        else if(Disp_Info.Unit_Type == UT_CF || Disp_Info.Unit_Type == UT_3D_SF || Disp_Info.Unit_Type == UT_3D_EF)
        {
            GDDDemoGoToScreen(C_3D2S);
            Disp_Info.Prv_Screen = Disp_Info.Screen;
            Disp_Info.Screen = C_3D2S;
        }
        else if(Disp_Info.Unit_Type == UT_LCWS || Disp_Info.Unit_Type == UT_LCWS_DL)
        {
            GDDDemoGoToScreen(C_LCWS);
            Disp_Info.Prv_Screen = Disp_Info.Screen;
            Disp_Info.Screen = C_LCWS;
        }
        else if(Disp_Info.Unit_Type == UT_D3A || Disp_Info.Unit_Type == UT_D3B || Disp_Info.Unit_Type == UT_D3C)
        {
            GDDDemoGoToScreen(C_3DPAS);
            Disp_Info.Prv_Screen = Disp_Info.Screen;
            Disp_Info.Screen = C_3DPAS;
        }
        else if(Disp_Info.Unit_Type == UT_D4A || Disp_Info.Unit_Type == UT_D4B || Disp_Info.Unit_Type == UT_D4C || Disp_Info.Unit_Type == UT_D4D)
        {
            GDDDemoGoToScreen(C_4D1S);
            Disp_Info.Prv_Screen = Disp_Info.Screen;
            Disp_Info.Screen = C_4D1S;
        }
    }

}


void de_init_spi(void)
{
    // Disable the Module and Free up the SDO Line for the other CPU to Send Data Out.
    SPI1CON1bits.DISSDO = 1;            // Disable the SDO Pin.
    SPI1STATbits.SPIEN = 0;     // Disable Module
    SPI_SDO_TRIS = INPUT_PORT;    // Make SDO as Inout for not to drive the SPI Signals
}

void re_init_spi(void)
{
    unsigned int Temp = 0;
    /* Prepare the Module */
    SPI1STATbits.SPIEN = 0;     // Enable Module
    SPI_SDO_TRIS = OUTPUT_PORT;    // Make SDO as Output to drive the SPI Signals
    SPI1CON1bits.DISSDO = 0;            // Enable the SDO Pin.
    Temp = SPI1BUF;
    SPI1STATbits.SPIROV = 0;
    SPI1CON2bits.SPIBEN = 1;    // Enable 8 Byte Enhanced SPI Buffers for both Tx and Rx
    SPI1STATbits.SPIEN = 1;     // Enable Module    
}

