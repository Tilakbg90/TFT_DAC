/* 
 * File:   DRV_SPI_SMCPU.h
 * Author: I15076
 *
 * Created on November 22, 2014, 11:59 AM
 */

#ifndef DRV_SPI_SMCPU_H
#define	DRV_SPI_SMCPU_H
#define TRIS_G_SS               TRISGbits.TRISG3
#define SPI_SDO_TRIS            TRISBbits.TRISB5
#define G_SS                    PORTGbits.RG3

#define     NUMBER_OF_CPU       2
#define     CPU_PACKET_LEN      55
#define     SMCPU_PACKET_LEN    26
#define MAX_G_PACKET_LEN            144                 /*Number of bytes from SMCPU*/

#define     CPU1    0
#define     CPU2    1

#define OFFSET_CODE_CHECKSUM    (CPU_DATA_LEN)
#define OFFSET_UNIT_TYPE        (OFFSET_CODE_CHECKSUM+4)
#define OFFSET_SW_V             (OFFSET_UNIT_TYPE + 1)
#define OFFSET_EVENT_COUNT      (OFFSET_SW_V + 1)
#define OFFSET_START_TIME       (OFFSET_EVENT_COUNT + 4)
#define OFFSET_END_TIME         (OFFSET_START_TIME + 4)
#define OFFSET_PRESENT_TIME     (OFFSET_END_TIME+ 4)
#define OFFSET_CPU1_M_ID        (OFFSET_PRESENT_TIME + 6)
#define OFFSET_CPU2_M_ID        (OFFSET_CPU1_M_ID + 1)
#define OFFSET_CRC              (OFFSET_CPU2_M_ID +1)



#define SMCPU_PK_OFFSET_CRC 0
#define SMCPU_PK_OFFSET_UT  SMCPU_PK_OFFSET_CRC+4
#define SMCPU_PK_OFFSET_SV  SMCPU_PK_OFFSET_UT+1
#define SMCPU_PK_OFFSET_EC  SMCPU_PK_OFFSET_SV+1
#define SMCPU_PK_OFFSET_ST  SMCPU_PK_OFFSET_EC+4
#define SMCPU_PK_OFFSET_ET  SMCPU_PK_OFFSET_ST+4
#define SMCPU_PK_OFFSET_PT  SMCPU_PK_OFFSET_ET+4
#define SMCPU_PK_OFFSET_N_ID  SMCPU_PK_OFFSET_PT+4
#define SMCPU_PK_OFFSET_SRC_ID  SMCPU_PK_OFFSET_N_ID+1
#define OFFSET_COM_DS_ERR_CNT   SMCPU_PK_OFFSET_SRC_ID+1
#define OFFSET_COM_US_ERR_CNT   OFFSET_COM_DS_ERR_CNT+2
#define OFFSET_UNUSED           OFFSET_COM_US_ERR_CNT+2
#define SMCPU_EVENT_ID      136
#define COMM_SRC_ID         137

typedef enum
			{
                                GLCD_SS_WAIT = 0,
                                GLCD_RCV_DATA,
                                GLCD_PROCESS_RX_PACKET,
			}GLCD_Driver_State;

typedef enum
                        {
                                GLCD_DATA_CPU1,
                                GLCD_DATA_CPU2,
                                GLCD_DATA_SMCPU
                        }GLCD_Data;

/* Lcd_info_Scheduler */
typedef struct {
			GLCD_Driver_State	State;	         	/*LCD display driver States */
                        GLCD_Data               Data_pack;
			BYTE                    GLCD_ON;
                        BYTE                    Packet_index;					/* Line Number to be updated */
			BYTE                    Packet_Max_length;					/* Coloumn Number to be updated */
			BYTE                    Rx_Message_Buffer[MAX_G_PACKET_LEN]; /* Buffer to Store character which is to be writtern into Lcd */
                        BYTE                    CPU_Data[NUMBER_OF_CPU][CPU_PACKET_LEN];
                        BYTE                    SMCPU_Data[SMCPU_PACKET_LEN];
} glcd_info_t;

void SPI_Initialize(void);
void Update_SPI_Data(void);
void Process_Rx_Packet(void);
void de_init_spi(void);
void re_init_spi(void);
#endif	/* DRV_SPI_SMCPU_H */

