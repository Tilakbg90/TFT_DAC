

#ifdef  __PIC24FJ256DA210__


// PIC24FJ256DA210 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>


#endif
#include <PPS.h>

#define SS1_S          PORTGbits.RG3

#define SPI_MASTER  0x0060	// select 8-bit master mode, CKE=1, CKP=0
#define SPI_ENABLE  0x8000	// enable SPI port, clear status
#define SPI_SLAVE   0x0040	// select 8-bit slave mode, CKE=1, CKP=0

struct def_sys_timer
{
    unsigned int ms_1;
    unsigned int ms_10;
    unsigned int ms_100;
    unsigned int ms_200;
    unsigned int ms_500;
    unsigned int ms_1000;
    unsigned int ms_5000;
};

struct def_sys_timer sys_timer_counter;
struct def_sys_timer sys_timer_flags;

#define COM_BUFFER_MAX_BYTES 50

struct
{
    unsigned char tx_buffer[COM_BUFFER_MAX_BYTES];
    unsigned char rx_buffer[COM_BUFFER_MAX_BYTES];
    unsigned int tx_rd_ptr;
    unsigned int tx_wr_ptr;
    unsigned int rx_rd_ptr;
    unsigned int rx_wr_ptr;
    unsigned char tx_in_progress;
    unsigned char ss1_state;
    unsigned char ss2_state;
}spi_data1, spi_data2;

unsigned char tx[20] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00};    // Array of Data to be sent and received
unsigned char test_t[20],test_r[20];

void init_1ms_timer(void)
{
    // Configure Timer 2 to run at 16MHz, interrupt flag to set once in 1mS

    T2CON = 0;
    TMR2 = 0;               // Reset Timer to 0
    PR2 = 16000;            // 1mS / 62.5nS = 16000.
    IEC0bits.T2IE = 0;      // Disable Interrupts for Timer 2
    IFS0bits.T2IF = 0;      // Clear Interrupt Flag
    T2CONbits.TON = 1;      // Start Timer
}

void manage_timer(void)
{
    if(IFS0bits.T2IF)
    {
        IFS0bits.T2IF = 0;
        sys_timer_flags.ms_1 = 1;
        sys_timer_counter.ms_10 = sys_timer_counter.ms_10 + 1;
        if(sys_timer_counter.ms_10 >= 100)
        {
            sys_timer_counter.ms_10 = 0;
            sys_timer_flags.ms_10 = 1;
        }
        sys_timer_counter.ms_100 = sys_timer_counter.ms_100 + 1;
        if(sys_timer_counter.ms_100 >= 100)
        {
            sys_timer_counter.ms_100 = 0;
            sys_timer_flags.ms_100 = 1;
        }

        sys_timer_counter.ms_200 = sys_timer_counter.ms_200 + 1;
        if(sys_timer_counter.ms_200 >= 200)
        {
            sys_timer_counter.ms_200 = 0;
            sys_timer_flags.ms_200 = 1;
        }
        sys_timer_counter.ms_500 = sys_timer_counter.ms_500 + 1;
        if(sys_timer_counter.ms_500 >= 500)
        {
            sys_timer_counter.ms_500 = 0;
            sys_timer_flags.ms_500 = 1;
        }

        sys_timer_counter.ms_1000 = sys_timer_counter.ms_1000 + 1;
        if(sys_timer_counter.ms_1000 >= 1000)
        {
            sys_timer_counter.ms_1000 = 0;
            sys_timer_flags.ms_1000 = 1;
        }

        sys_timer_counter.ms_5000 = sys_timer_counter.ms_5000 + 1;
        if(sys_timer_counter.ms_5000 >= 5000)
        {
            sys_timer_counter.ms_5000 = 0;
            sys_timer_flags.ms_5000 = 1;
        }

    }
}


void init_Channel (void)
{
    TRISA = 0;
    PORTA=0;
    LATA=0;
    ANSA = 0x0000;
    ANSD = 0x0000;
    ANSG = 0x0000;
    ANSF = 0x0000;
    ANSB = 0x0000;
    ANSC = 0x0000;
}


void init_SPI1_IO (void) //SLAVE SMCPU
{
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

}

unsigned char writeSPI2( unsigned char i )
{
    SPI2BUF = i;					// write to buffer for TX
    while(!SPI2STATbits.SRMPT);	// wait for transfer to complete
    return SPI2BUF;    				// read the received value
}//writeSPI2

extern unsigned char temp;
void writeSPI1_array()
{
    temp = SPI1BUF;
    temp = 0;
    while(temp<8)
    {
    SPI1BUF = temp;					// write to buffer for TX
    temp++;
    }

}//writeSPI2

unsigned char readSPI2( unsigned char i )
{
    unsigned char read_val  = 0;
    if(!SPI2STATbits.SRXMPT)	// wait for transfer to complete
    {
    read_val = SPI2BUF;
    }
    return read_val;    				// read the received value
}

void readSPI1_array( unsigned char *arr)
{
    while(!SPI1STATbits.SPIRBF);
    while(!SPI1STATbits.SRXMPT)
    {
    *arr = SPI1BUF;
    arr++;
    }
}

void SPI1INTInit()
{
  SPI1CON2bits.SPIBEN = 1;
  IFS0bits.SPI1IF = 0;

  IEC0bits.SPI1IE = 0;
    SPI1CON1 = SPI_SLAVE;  // select mode
    SPI1STAT = SPI_ENABLE;  // enable the peripheral
    temp = SPI1BUF;

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

    IFS0bits.SPI1IF = 0;        // Clear Interrupt Flags
    IFS0bits.SPF1IF = 0;

//    SPI1CON1bits.CKE = 0;
//    SPI1CON1bits.CKP = 0;
        SPI1STATbits.SPIEN = 0;     // Enable Module
    SPI1CON1bits.DISSDO = 0;            // Enable the SDO Pin.
    temp = SPI1BUF;
    SPI1STATbits.SPIROV = 0;
    SPI1CON2bits.SPIBEN = 1;    // Enable 8 Byte Enhanced SPI Buffers for both Tx and Rx
    SPI1STATbits.SPIEN = 1;     // Enable Module
} // SPI2INTInit

unsigned char CPU_selected = 0;

void main2 (void)
{
   unsigned char n=0,i=0;
   CLKDIVbits.CPDIV = 0;   // 8MHz input, 32MHz System Clock
    SPI1BUF=0;

    init_Channel();
    init_SPI1_IO();

    init_1ms_timer();
    SPI1INTInit();
    return;
    //writeSPI1_array();
    n=1;
    while(1)
    {

        if(1)
        {
            writeSPI1_array();
            readSPI1_array(&spi_data1.rx_buffer[0]); //What slave has recv

            //n=1;
        }
        else
        {

        }

    }
}
