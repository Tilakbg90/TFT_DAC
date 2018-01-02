/* 
 * File:   Common_DAC_info.h
 * Author: I15076
 *
 * Created on November 22, 2014, 1:14 PM
 */

#ifndef COMMON_DAC_INFO_H
#define	COMMON_DAC_INFO_H
/* Macros */
/* Macro function which convert BCD to binary */
#define BCD_TO_BIN(bcd_val)		(((bcd_val)&15)+((bcd_val)>>4)*10)

/* Macro function which convert Binary to BCD  */
#define BIN_TO_BCD(bin_val)		((((bin_val)/10)<<4)+(bin_val)%10)

#ifndef ROMSIZE
#define ROMSIZE 131072			/* 128kb-131072 instructions can be stored */
#endif

#define SET_HIGH					(1)
#define SET_LOW						(0)
#define TIMEOUT_EVENT				(0)
#define DELAY_250MS					(250)
#define DELAY_5MS					(5)
#define DELAY_2MS					(2)
#define MAXIMUM_NO_OF_CPU			(2)
#define CPU1_ID						(1)
#define CPU2_ID						(2)
#define SECONDS_PER_DAY				(86400)		/* SECONDS_PER_DAY = 24(hour)*60(min)*60(sec)  */

#define EVENTS_PER_CHIP				(4096)		/* 65536(one EEPROM Chip Size in Bytes) / 16 (no of Bytes per Event) */
#define NO_OF_SERIAL_EEPROMS		(4)			/* No of serial EEPROM available for Data Logging */
#define MAXIMUM_EVENTS				(16384)		/* EVENTS_PER_CHIP (8192) * (2) NO_OF_SERIAL_EEPROMS */
#define EVENT_RECORD_SIZE			(16)		/* Size of Event record in Bytes */
#define NO_OF_EVENT_REGISTERS		(6)			/* Number of Event Registers */
#define EVENTS_PER_REGISTER			(8)			/* Number of events per event register */
#define EEPROM_BLOCK_SIZE			(65536)		/* Serial EEPROM one block size */
#define BLOCKS_PER_CHIP				(1)			/* No of blocks per chip */
#define MAXIMUM_BLOCKS				(4)			/* BLOCKS_PER_CHIP (1) * (4) NO_OF_SERIAL_EEPROMS */

#define LATEST_EVENT_TOKEN			0xAA
#define OLD_EVENT_TOKEN				0x55

#define LARGEST_CONVERTABLE_INTEGER	 (9999)     /* This is the maximum number displayed for comm Err count*/

/*
 *	Defines the datatype
 */

typedef signed char SCHAR;



/*
 * I/O Ports
 */
#define CPU_DETECT_PORT			PORTAbits.RA7		/* CPU1 Board presence is checked by this port */



#define	RTC_SCL						RD6		/* RTC Serial Clock */
#define	RTC_SCL_DIR					TRISD6  /* RTC Serial Clock Port Direction */

#define RTC_SDA						RD5		/* RTC Serial Data */
#define	RTC_SDA_DIR					TRISD5	/* RTC Serial Data Port Direction */

#define INPUT_PORT					1
#define OUTPUT_PORT					0
#define MAXIMUM_MESSAGE_LENGTH		20
#define SPI_MESSAGE_LENGTH			40
#define GCPU_SPI_MESSAGE_LENGTH			64

#define COMM1						1
#define COMM2						2
#define VOLTAGE_OK					1
#define VOLTAGE_LOW					0
#define BOARD_MISSING				0
#define CATASTROPHIC_ERROR			0
#define NORMAL						1
#define CRC32_CHECKSUM_BAD			0
#define CRC32_CHECKSUM_OK			1
#define FALSE						0				/* logical zero indicates FALSE */
#define TRUE						!FALSE			/* logical non zero indicates TRUE */

/* Offset pointing to SPI Data */
#define US_FWD_COUNT_OFFSET			(8)			/* Index to access US FWD Count in Cpu1/Cpu2 Data Buffer */
#define US_REV_COUNT_OFFSET			(10)		/* Index to access US REV Count in Cpu1/Cpu2 Data Buffer */
#define DS_FWD_COUNT_OFFSET			(12)		/* Index to access DS FWD Count in Cpu1/Cpu2 Data Buffer */
#define DS_REV_COUNT_OFFSET			(14)		/* Index to access DS REV Count in Cpu1/Cpu2 Data Buffer */
#define VALIDATED_CHECKSUM_OFFSET	(8)
#define LOCAL_CHECKSUM_OFFSET		(14)
#define UNIT_TYPE_OFFSET			(12)
#define SOFWARE_VERSION_OFFSET		(13)
#define MODE_OFFSET					(26)
#define DS_ERR_CODE_OFFSET			(27)
#define US_ERR_CODE_OFFSET			(28)
#define MESSAGE_TYPE_OFFSET			(37)

#define READ_RESET_INFO					0xAA
#define READ_AXLE_COUNT					0x55

#define ACQUIRE_DAC_INFO			(0)
#define DAC_INFO_ACQUIRED			(1)

/*
 * Event ID - Table
 */
typedef enum {
			EVENT_SM_POWERED_ON = 0,
			EVENT_DATE_TIME_CHANGED,
			EVENT_DIRECT_OUT_COUNT,
			EVENT_US_BLOCK_CLEAR,
			EVENT_US_BLOCK_OCCUPIED,
			EVENT_DS_BLOCK_CLEAR,
			EVENT_DS_BLOCK_OCCUPIED,
			EVENT_DAC_US_RESET_APPLIED,
			EVENT_DAC_US_RESET_COMPLETED,
			EVENT_SYSTEM_NORMAL,
			EVENT_SYSTEM_DEFECTIVE,
			EVENT_BOARDS_MISSING,
			EVENT_ALL_BOARDS_FOUND,
			EVENT_PD1_BOARD_MISSING,
			EVENT_PD1_BOARD_FOUND,
			EVENT_PD2_BOARD_MISSING,
			EVENT_PD2_BOARD_FOUND,
			EVENT_MODEM_BOARD_A_MISSING,
			EVENT_MODEM_BOARD_A_FOUND,
			EVENT_MODEM_BOARD_B_MISSING,
			EVENT_MODEM_BOARD_B_FOUND,
			EVENT_RELAY_BOARD_A_MISSING,
			EVENT_RELAY_BOARD_A_FOUND,
			EVENT_RELAY_BOARD_B_MISSING,
			EVENT_RELAY_BOARD_B_FOUND,
			EVENT_PEER_CPU_MISSING,
			EVENT_PEER_CPU_FOUND,
			EVENT_LU1_US1_COMM_FAILURE,
			EVENT_LU1_US1_COMM_RESTORED,
			EVENT_LU1_US2_COMM_FAILURE,
			EVENT_LU1_US2_COMM_RESTORED,
			EVENT_LU1_DS1_COMM_FAILURE,
			EVENT_LU1_DS1_COMM_RESTORED,
			EVENT_LU1_DS2_COMM_FAILURE,
			EVENT_LU1_DS2_COMM_RESTORED,
			EVENT_LU2_US1_COMM_FAILURE,
			EVENT_LU2_US1_COMM_RESTORED,
			EVENT_LU2_US2_COMM_FAILURE,
			EVENT_LU2_US2_COMM_RESTORED,
			EVENT_LU2_DS1_COMM_FAILURE,
			EVENT_LU2_DS1_COMM_RESTORED,
			EVENT_LU2_DS2_COMM_FAILURE,
			EVENT_LU2_DS2_COMM_RESTORED,
			EVENT_US1_LU1_COMM_FAILURE,
			EVENT_US1_LU1_COMM_RESTORED,
			EVENT_US2_LU1_COMM_FAILURE,
			EVENT_US2_LU1_COMM_RESTORED,
			EVENT_DS1_LU1_COMM_FAILURE,
			EVENT_DS1_LU1_COMM_RESTORED,
			EVENT_DS2_LU1_COMM_FAILURE,
			EVENT_DS2_LU1_COMM_RESTORED,
			EVENT_US1_LU2_COMM_FAILURE,
			EVENT_US1_LU2_COMM_RESTORED,
			EVENT_US2_LU2_COMM_FAILURE,
			EVENT_US2_LU2_COMM_RESTORED,
			EVENT_DS1_LU2_COMM_FAILURE,
			EVENT_DS1_LU2_COMM_RESTORED,
			EVENT_DS2_LU2_COMM_FAILURE,
			EVENT_DS2_LU2_COMM_RESTORED,
			EVENT_PEER_CPU_COMM_FAILURE,
			EVENT_PEER_CPU_COMM_RESTORED,
			EVENT_PD1_FAILED,
			EVENT_PD1_NORMAL,
			EVENT_PD2_FAILED,
			EVENT_PD2_NORMAL,
			EVENT_PEER_CPU_FAILED,
			EVENT_PEER_CPU_NORMAL,
			EVENT_VITAL_RELAY_A_FAILED,
			EVENT_VITAL_RELAY_A_NORMAL,
			EVENT_VITAL_RELAY_B_FAILED,
			EVENT_VITAL_RELAY_B_NORMAL,
			EVENT_PREP_RELAY_A_FAILED,
			EVENT_PREP_RELAY_A_NORMAL,
			EVENT_MODEM_A_ERROR,
			EVENT_MODEM_A_NORMAL,
			EVENT_MODEM_B_ERROR,
			EVENT_MODEM_B_NORMAL,
			EVENT_US_DAC_FAILED,
			EVENT_US_DAC_NORMAL,
			EVENT_DS_DAC_FAILED,
			EVENT_DS_DAC_NORMAL,
			EVENT_US1_POWER_FAILED,
			EVENT_US1_POWER_RESTORED,
			EVENT_US2_POWER_FAILED,
			EVENT_US2_POWER_RESTORED,
			EVENT_DS1_POWER_FAILED,
			EVENT_DS1_POWER_RESTORED,
			EVENT_DS2_POWER_FAILED,
			EVENT_DS2_POWER_RESTORED,
			EVENT_VDD_VOLTAGE_HIGH,
			EVENT_VDD_VOLTAGE_LOW,
			EVENT_VDD_VOLTAGE_NORMAL,
			EVENT_DAC_DS_RESET_APPLIED,
			EVENT_DAC_DS_RESET_COMPLETED,
			EVENT_PREP_RELAY_B_FAILED,
			EVENT_PREP_RELAY_B_NORMAL,
			EVENT_NOT_OCCURRED = 255
} event_id_t;

typedef union {
	UINT16 Word;
	struct	{
		BYTE	Lo;		/* Low Byte (little-Endian) */
		BYTE	Hi;		/* High Byte (little-Endian)*/
	} Byte ;
} wordtype_t;

typedef union {
	UINT32 LWord;
	struct	{
		wordtype_t	LoWord;		/* Low Word (little-Endian) */
		wordtype_t	HiWord;		/* High Word (little-Endian) */
	} DWord ;
} longtype_t;

typedef union {
	struct	{
		unsigned b0 : 1;		/* Bit 0 */
		unsigned b1 : 1;		/* Bit 1 */
		unsigned b2 : 1;		/* Bit 2 */
		unsigned b3 : 1;		/* Bit 3 */
		unsigned b4 : 1;		/* Bit 4 */
		unsigned b5 : 1;		/* Bit 5 */
		unsigned b6 : 1;		/* Bit 6 */
		unsigned b7 : 1;		/* Bit 7 */
	} Bit ;
	BYTE Byte;
} bitadrb_t;

typedef union {
	struct	{
		unsigned b0  : 1;		/* Bit 0  */
		unsigned b1  : 1;		/* Bit 1  */
		unsigned b2  : 1;		/* Bit 2  */
		unsigned b3  : 1;		/* Bit 3  */
		unsigned b4  : 1;		/* Bit 4  */
		unsigned b5  : 1;		/* Bit 5  */
		unsigned b6  : 1;		/* Bit 6  */
		unsigned b7  : 1;		/* Bit 7  */
		unsigned b8  : 1;		/* Bit 8  */
		unsigned b9  : 1;		/* Bit 9  */
		unsigned b10 : 1;		/* Bit 10 */
		unsigned b11 : 1;		/* Bit 11 */
		unsigned b12 : 1;		/* Bit 12 */
		unsigned b13 : 1;		/* Bit 13 */
		unsigned b14 : 1;		/* Bit 14 */
		unsigned b15 : 1;		/* Bit 15 */
	} Bit ;
	UINT16 Word;
} bitadrw_t;

typedef union {
	struct {
		unsigned System_Status     :1;		/* Byte 1, Bit 0 - Logical 0 - System Defective, Logical 1 - System normal */
		unsigned Network_Addr      :1;		/* Byte 1, Bit 1 - Logical 0 - Invalid Network address, Logical 1 - valid Network address*/
		unsigned Flash_CheckSum    :1;		/* Byte 1, Bit 2 - Logical 0 - Bad Checksum, Logical 1 - Checksum OK */
		unsigned Unit_Board_Status :1;		/* Byte 1, Bit 3 - Logical 0 - Board Missing, Logical 1 - Board Present */
		unsigned Local_Reset_Done  :1;		/* Byte 1, Bit 4 - Logical 0 - Not resetted, Logical 1 - Reset done */
		unsigned US_Track_Status   :1;		/* Byte 1, Bit 5 - Logical 0 - US Track Occupied, Logical 1 - US Track Clear */
		unsigned DS_Track_Status   :1;		/* Byte 1, Bit 6 - Logical 0 - DS Track Occupied, Logical 1 - DS Track Clear */
		unsigned Local_Reset_Done2 :1;      /* Byte 1, Bit 7 - notused */
		unsigned Direct_Out_Count  :1;		/* Byte 2, Bit 0 - Logical 0 - Not Occured,	Logical 1 - Direct out count occured */
		unsigned PD1_Status :1;				/* Byte 2, Bit 1 - Logical 0 - PD1 Defective, Logical 1 - PD1 Normal */
		unsigned PD2_Status :1;				/* Byte 2, Bit 2 - Logical 0 - PD2 Defective, Logical 1 - PD2 Normal*/
		unsigned Vital_Relay_A :1;			/* Byte 2, Bit 3 - Logical 0 - Vital_Relay_A fail, Logical 1 - Vital_Relay_A ok*/
		unsigned Vital_Relay_B :1;			/* Byte 2, Bit 4 - Logical 0 - Vital_Relay_B fail, Logical 1 - Vital_Relay_B ok*/
		unsigned Preparatory_Relay :1;		/* Byte 2, Bit 5 - Logical 0 - Preparatory_Relay fail, Logical 1 - Preparatory_Relay noraml*/
		unsigned Preparatory_Reset_US :1;		/* Byte 2, Bit 6 - Logical 0 - No Preparatory_Reset, Logical 1 - Preparatory_Reset done*/
		unsigned Local_Power_Status :1;		/* Byte 2, Bit 7 - Logical 0 - Local_Power_Status OFF,	Logical 1 - Local_Power_Status ON*/
		unsigned Peer_System_Status :1;		/* Byte 3, Bit 0 - Logical 0 - Peer_System_Status defective, Logical 1 - Peer_System_Status normal*/
		unsigned Peer_CPU_Link :1;			/* Byte 3, Bit 1 - Logical 0 - Peer_CPU_Link fail, Logical 1 - Peer_CPU_Link ok*/
		unsigned Power_Fail_at_US1 :1;		/* Byte 3, Bit 2 - Logical 0 - Power_Fail_at_US1,  Logical 1 - Power_ok_at_US1*/
		unsigned Power_Fail_at_US2 :1;		/* Byte 3, Bit 3 - Logical 0 - Power_Fail_at_US2,  Logical 1 - Power_ok_at_US2*/
		unsigned Power_Fail_at_DS1 :1;		/* Byte 3, Bit 4 - Logical 0 - Power_Fail_at_DS1,  Logical 1 - Power_ok_at_DS1*/
		unsigned Power_Fail_at_DS2 :1;		/* Byte 3, Bit 5 - Logical 0 - Power_Fail_at_DS2,  Logical 1 - Power_ok_at_DS2*/
		unsigned DS_System_Status :1;		/* Byte 3, Bit 6 - Logical 0 - DS_System_Status defective, Logical 1 - DS_System_Status normal*/
		unsigned US_System_Status :1;		/* Byte 3, Bit 7 - Logical 0 - US_System_Status defective, Logical 1 - US_System_Status normal*/
		unsigned LU1_to_US1_Link  :1;		/* Byte 4, Bit 0 - Logical 0 - LU1_to_US1_Link fail, Logical 1 - LU1_to_US1_Link ok*/
		unsigned LU1_to_US2_Link  :1;		/* Byte 4, Bit 1 - Logical 0 - LU1_to_US2_Link fail, Logical 1 - LU1_to_US2_Link ok*/
		unsigned LU1_to_DS1_Link  :1;		/* Byte 4, Bit 2 - Logical 0 - LU1_to_DS1_Link fail, Logical 1 - LU1_to_DS1_Link ok*/
		unsigned LU1_to_DS2_Link  :1;		/* Byte 4, Bit 3 - Logical 0 - LU1_to_DS2_Link fail, Logical 1 - LU1_to_DS2_Link  ok*/
		unsigned US1_to_LU1_Link  :1;		/* Byte 4, Bit 4 - Logical 0 - US1_to_LU1_Link fail, Logical 1 - US1_to_LU1_Link  ok*/
		unsigned US2_to_LU1_Link  :1;		/* Byte 4, Bit 5 - Logical 0 - US2_to_LU1_Link fail, Logical 1 - US2_to_LU1_Link  ok*/
		unsigned DS1_to_LU1_Link  :1;		/* Byte 4, Bit 6 - Logical 0 - DS1_to_LU1_Link fail, Logical 1 - DS1_to_LU1_Link  ok*/
		unsigned DS2_to_LU1_Link  :1;		/* Byte 4, Bit 7 - Logical 0 - DS2_to_LU1_Link fail, Logical 1 - DS2_to_LU1_Link  ok*/
		unsigned LU2_to_US1_Link  :1;		/* Byte 5, Bit 0 - Logical 0 - LU2_to_US1_Link fail, Logical 1 - LU2_to_US1_Link  ok*/
		unsigned LU2_to_US2_Link  :1;		/* Byte 5, Bit 1 - Logical 0 - LU2_to_US2_Link fail, Logical 1 - LU2_to_US2_Link  ok*/
		unsigned LU2_to_DS1_Link  :1;		/* Byte 5, Bit 2 - Logical 0 - LU2_to_DS1_Link fail, Logical 1 - LU2_to_DS1_Link  ok*/
		unsigned LU2_to_DS2_Link  :1;		/* Byte 5, Bit 3 - Logical 0 - LU2_to_DS2_Link fail, Logical 1 - LU2_to_DS2_Link  ok*/
		unsigned US1_to_LU2_Link  :1;		/* Byte 5, Bit 4 - Logical 0 - US1_to_LU2_Link fail, Logical 1 - US1_to_LU2_Link ok*/
		unsigned US2_to_LU2_Link  :1;		/* Byte 5, Bit 5 - Logical 0 - US2_to_LU2_Link fail, Logical 1 - US2_to_LU2_Link ok*/
		unsigned DS1_to_LU2_Link  :1;		/* Byte 5, Bit 6 - Logical 0 - DS1_to_LU2_Link fail, Logical 1 - DS1_to_LU2_Link ok*/
		unsigned DS2_to_LU2_Link  :1;		/* Byte 5, Bit 7 - Logical 0 - DS2_to_LU2_Link fail, Logical 1 - DS2_to_LU2_Link ok*/
		unsigned PD1_Board_Present    :1;  	/* Byte 6, Bit 0 - Logical 0 - PD1_board missing,    Logical 1 - PD1_board present*/
	  	unsigned PD2_Board_Present    :1;  	/* Byte 6, Bit 1 - Logical 0 - PD2_board missing,    Logical 1 - PD2_board present*/
	  	unsigned SM_Cpu_Present       :1;  	/* Byte 6, Bit 2 - Logical 0 - SM_CPU missing,       Logical 1 - SM_CPU present*/
	  	unsigned Modem_Card_A_Present :1;  	/* Byte 6, Bit 3 - Logical 0 - Modem_card _A missing,Logical 1 - Modem_card _A present */
	  	unsigned Modem_Card_B_Present :1;  	/* Byte 6, Bit 4 - Logical 0 - Modem_card _B missing,Logical 1 - Modem_card _B present*/
 	  	unsigned Relay_Drive_A_Present:1;  	/* Byte 6, Bit 5 - Logical 0 - Relay drive_A missing,Logical 1 - Relay drive_A present */
	  	unsigned Relay_Drive_B_Present:1;  	/* Byte 6, Bit 6 - Logical 0 - Relay drive_B missing,Logical 1 - Relay drive_B present */
	  	unsigned Peer_Cpu_Present     :1;  	/* Byte 6, Bit 7 - Logical 0 - Peer CPU fail, Logical 1 -Peer CPU present*/
		unsigned Modem_A			  :1;  	/* Byte 7, Bit 0 - Logical 0 - Modem_A error, Logical 1 -Modem_A ok*/
		unsigned Modem_B			  :1;	/* Byte 7, Bit 1 - Logical 0 - Modem_B error, Logical 1 -Modem_B ok */
		unsigned Modem_A_Err_Toggle_Bit:1;	/* Byte 7, Bit 2 - Modem_A_Err_Toggle_Bit -for Each modem err this bit toggles*/
		unsigned Modem_B_Err_Toggle_Bit:1;	/* Byte 7, Bit 3 */
		unsigned RAM_Check			   :1;  /* Byte 7, Bit 4 */
		unsigned Preparatory_Relay1	   :1;  /* Byte 7, Bit 5 */
		unsigned Preparatory_Reset_DS  :1;	/* Byte 7, Bit 6 */
		unsigned 					   :1;
		} Flags;
	BYTE Byte[7];
} dac_status_t;

typedef union {
	struct {
		unsigned US_Block :1;			/* Byte 0, Bit 0 - Logical 0 - US_Block occupied,Logical 1 - US_Block clear*/
		unsigned DS_Block :1;			/* Byte 0, Bit 1 - Logical 0 - DS_Block occupied,Logical 1 - DS_Block clear*/
		unsigned DAC_DS_Reset :1;			/* Byte 0, Bit 2 - Logical 0 - Event not occured,Logical 1 - DAC_Reset applied*/
		unsigned System :1;				/* Byte 0, Bit 3 - Logical 0 - System defective, Logical 1 - System normal*/
		unsigned Direct_Out_Count :1;	/* Byte 0, Bit 4 - Logical 0 - Event not occured,Logical 1 -Direct outcount*/
		unsigned : 2;					/* Byte 0, Bit 5 to 7 - not used*/
		unsigned DAC_US_Reset	:1;		/* Byte 0,Bit 7 - Logical 0 - Event_Not_Occured,Logical 1 - DAC_US_Reset applied*/
		unsigned Board :1;				/* Byte 1, Bit 0 - Logical 0 - Board missing,Logical 1 -Board found*/
		unsigned PD1_Board :1;			/* Byte 1, Bit 1 - Logical 0 - PD1_Board missing,Logical 1 -PD1_Board found*/
		unsigned PD2_Board :1;			/* Byte 1, Bit 2 - Logical 0 - PD2_Board missing,Logical 1 -PD2_Board found*/
		unsigned Modem_Board_A :1;		/* Byte 1, Bit 3 - Logical 0 - Modem_Board_A missing,Logical 1 -Modem_Board_A found*/
		unsigned Modem_Board_B :1;		/* Byte 1, Bit 4 - Logical 0 - Modem_Board_B missing,Logical 1 -Modem_Board_B found*/
		unsigned Relay_Board_A :1;		/* Byte 1, Bit 5 - Logical 0 - Relay_Board_A missing,Logical 1 -Relay_Board_A found*/
		unsigned Relay_Board_B :1;		/* Byte 1, Bit 6 - Logical 0 - Relay_Board_B missing,Logical 1 -Relay_Board_B found*/
		unsigned Peer_CPU_Board :1;		/* Byte 1, Bit 7 - Logical 0 - Peer_CPU_Board missing, Logical 1-Peer_CPU_Board found*/
		unsigned LU1_US1_Comm :1;		/* Byte 2, Bit 0 - Logical 0 - LU1_US1_Comm failure,Logical 1 -LU1_US1_Comm restored*/
		unsigned LU1_US2_Comm :1;		/* Byte 2, Bit 1 - Logical 0 - LU1_US2_Comm failure,Logical 1 -LU1_US2_Comm restored*/
		unsigned LU1_DS1_Comm :1;		/* Byte 2, Bit 2 - Logical 0 - LU1_DS1_Comm failure,Logical 1 -LU1_DS1_Comm restored*/
		unsigned LU1_DS2_Comm :1;		/* Byte 2, Bit 3 - Logical 0 - LU1_DS2_Comm failure,Logical 1 -LU1_DS2_Comm restored*/
		unsigned LU2_US1_Comm :1;		/* Byte 2, Bit 4 - Logical 0 - LU2_US1_Comm failure,Logical 1 -LU2_US1_Comm restored*/
		unsigned LU2_US2_Comm :1;		/* Byte 2, Bit 5 - Logical 0 - LU2_US2_Comm failure,Logical 1 -LU2_US2_Comm restored*/
		unsigned LU2_DS1_Comm :1;		/* Byte 2, Bit 6 - Logical 0 - LU2_DS1_Comm failure,Logical 1 -LU2_DS1_Comm restored*/
		unsigned LU2_DS2_Comm :1;		/* Byte 2, Bit 7 - Logical 0 - LU2_DS2_Comm failure,Logical 1 -LU2_DS2_Comm restored*/
		unsigned US1_LU1_Comm :1;		/* Byte 3, Bit 0 - Logical 0 - US1_LU1_Comm failure,Logical 1 -US1_LU1_Comm restored*/
		unsigned US2_LU1_Comm :1;		/* Byte 3, Bit 1 - Logical 0 - US2_LU1_Comm failure,Logical 1 -US2_LU1_Comm restored*/
		unsigned DS1_LU1_Comm :1;		/* Byte 3, Bit 2 - Logical 0 - DS1_LU1_Comm failure,Logical 1 -DS1_LU1_Comm restored*/
		unsigned DS2_LU1_Comm :1;		/* Byte 3, Bit 3 - Logical 0 - DS2_LU1_Comm failure,Logical 1 -DS2_LU1_Comm restored*/
		unsigned US1_LU2_Comm :1;		/* Byte 3, Bit 4 - Logical 0 - US1_LU2_Comm failure,Logical 1 -US1_LU2_Comm restored*/
		unsigned US2_LU2_Comm :1;		/* Byte 3, Bit 5 - Logical 0 - US2_LU2_Comm failure,Logical 1 -US2_LU2_Comm restored*/
		unsigned DS1_LU2_Comm :1;		/* Byte 3, Bit 6 - Logical 0 - DS1_LU2_Comm failure,Logical 1 -DS1_LU2_Comm restored*/
		unsigned DS2_LU2_Comm :1;		/* Byte 3, Bit 7 - Logical 0 - DS2_LU2_Comm failure,Logical 1 -DS2_LU2_Comm restored*/
		unsigned PD1 :1;				/* Byte 4, Bit 0 - Logical 0 -  PD1 failed,Logical 1 - PD1 normal*/
		unsigned PD2 :1;				/* Byte 4, Bit 1 - Logical 0 -  PD2 failed,Logical 1 - PD2 normal*/
		unsigned Peer_CPU :1;			/* Byte 4, Bit 2 - Logical 0 -  Peer_CPU failed,Logical 1 - Peer_CPU normal*/
		unsigned Peer_CPU_Comm :1;		/* Byte 4, Bit 3 - Logical 0 -  Peer_CPU_Comm failure,Logical 1 - Peer_CPU_Comm restored*/
		unsigned Vital_Relay_A :1;		/* Byte 4, Bit 4 - Logical 0 -  Vital_Relay_A failed,Logical 1 - Vital_Relay_A normal*/
		unsigned Vital_Relay_B :1;		/* Byte 4, Bit 5 - Logical 0 -  Vital_Relay_B failed,Logical 1 - Vital_Relay_B normal*/
		unsigned Prep_Relay_A :1;		/* Byte 4, Bit 6 - Logical 0 -  Prep_Relay_A failed,Logical 1 - Prep_Relay_A normal*/
		unsigned Prep_Relay_B : 1;		/* Byte 4, Bit 7 - Logical 0 -  Prep_Relay_B failed,Logical 1 - Prep_Relay_B normal*/
		unsigned Modem_A :1;			/* Byte 5, Bit 0 - Logical 0 -  Modem_A error,Logical 1 - Modem_A  normal*/
		unsigned Modem_B :1;			/* Byte 5, Bit 1 - Logical 0 -  Modem_B error,Logical 1 - Modem_B normal*/
		unsigned US_DAC :1;				/* Byte 5, Bit 2 - Logical 0 -  US_DAC failed,Logical 1 - US_DAC normal*/
		unsigned DS_DAC :1;				/* Byte 5, Bit 3 - Logical 0 -  DS_DAC failed,Logical 1 - DS_DAC normal*/
		unsigned US1_Power :1;			/* Byte 5, Bit 4 - Logical 0 -  US1_Power failed,Logical 1 - US1_Power restored*/
		unsigned US2_Power :1;			/* Byte 5, Bit 5 - Logical 0 -  US2_Power failed,Logical 1 - US2_Power restored*/
		unsigned DS1_Power :1;			/* Byte 5, Bit 6 - Logical 0 -  DS1_Power failed,Logical 1 - DS1_Power restored*/
		unsigned DS2_Power :1;			/* Byte 5, Bit 7 - Logical 0 -  DS2_Power failed,Logical 1 - DS2_Power restored*/
		} Id;
	BYTE Byte[6];
} event_register_t;


typedef union {
	struct {
		unsigned SM_Network_Addr : 1;		/* Byte 0, Bit 0 - Logical 0 - Invalid network address, Logical 1 - Valid network address */
		unsigned B1_Board_Status : 1;		/* Byte 0, Bit 1 - Logical 0 - CPU1 board missing, Logical 1 - CPU1 board present */
		unsigned B2_Board_Status : 1;		/* Byte 0, Bit 2 - Logical 0 - CPU2 board missing, Logical 1 - CPU2 board present*/
		unsigned Flash_CheckSum : 1;		/* Byte 0, Bit 3 - Logical 0 - Bad checksum, Logical 1 - Check sum ok*/
		unsigned System_Status  : 1;		/* Byte 0, Bit 4 - Logical 0 - System defective, Logical 1 - System normal*/
		unsigned Query_Pending  : 1;		/* Byte 0, Bit 5 - Logical 0 - No query, Logical 1 - previous query not completed*/
		unsigned 				: 2;
		unsigned RTC_I2C_Failed : 1;		/* Byte 1, Bit 0 - Logical 0 - RTC_I2C comm failed, Logical 1 - RTC_I2C comm ok*/
		unsigned MEM_I2C_Failed : 1;		/* Byte 1, Bit 1 - Logical 0 - MEM_I2C comm failed, Logical 1 -MEM_I2C comm ok*/
		unsigned MEM_Corrupted  : 1;		/* Byte 1, Bit 2 - Logical 0 - LRC Corrupted while reading Serial EEPROM, Logical 1- Not Corrupted */
		unsigned Inhibit_Logging_P	  :1;	/* Byte 1, Bit 3  Event Logging enable bit */
		unsigned Inhibit_Logging_N	  :1;	/* Byte 1, Bit 4  Event Logging enable bit */
		unsigned Erasure_in_Progress_P :1;	/* Byte 1, Bit 5 */
		unsigned Erasure_in_Progress_N :1;	/* Byte 1, Bit 6 */
		unsigned MDM_TX_Mode   :1;
		} Flags ;
	BYTE Byte[2];
} sm_status_t;

/* Structure to holds Dac external communication CRC error count information */
typedef struct {
			UINT16  CPU1_CommA_Error_Count;
			UINT16  CPU1_CommB_Error_Count;
			UINT16  CPU2_CommA_Error_Count;
			UINT16  CPU2_CommB_Error_Count;
			struct
				{
				unsigned Old_Cpu1_CommA_Err_State;
				unsigned New_Cpu1_CommA_Err_State;
				unsigned Old_Cpu1_CommB_Err_State;
				unsigned New_Cpu1_CommB_Err_State;
				unsigned Old_Cpu2_CommA_Err_State;
				unsigned New_Cpu2_CommA_Err_State;
				unsigned Old_Cpu2_CommB_Err_State;
				unsigned New_Cpu2_CommB_Err_State;
				}Flags;
}dac_comm_error;

/* Event Record Structure */
typedef union {
	struct __attribute__((packed)) {
		BYTE	Token;				/* Identification to find OLD_EVENT or NEW_EVENT */
		long	Date_Time;			/* Date and Time stamp of Event occurance */
		BYTE	Source_Id;			/* Upper nibble - channel no, lower nible - TAWD address */
		BYTE	Event_Id;			/* Event Identification code */
		UINT16	US_Fwd_AxleCount;	/* Upstream Forward Axle Count */
		UINT16	US_Rev_AxleCount;	/* Upstream Reverse Axle Count */
		UINT16	DS_Fwd_AxleCount;	/* Downstream Forward Axle Count */
		UINT16	DS_Rev_AxleCount;	/* Downstream Reverse Axle Count */
		BYTE 	LRC;				/* Check sum */
	} Field ;
	BYTE Byte[EVENT_RECORD_SIZE];
} event_record_t ;

typedef struct{

              longtype_t CPU1_Checksum;
              longtype_t CPU2_Checksum;
}checksum;

typedef struct{
				longtype_t Checksum;
				BYTE		Unit_Type;
				BYTE		SW_Version;
} dac_sysinfo_t;

typedef struct{
				longtype_t Event_count;
				longtype_t Start_time;
				longtype_t End_time;
                                longtype_t Present_time;
                                unsigned char SRC_ID;
                                unsigned char Event_Logger_ID;
                                char Disp_Date[30];
                                char Disp_Time[30];
                                char Start_Date[30];
                                char End_Date[30];
                                char Number_of_Event_String[30];

}SMCPU_info_t;

#endif	/* COMMON_DAC_INFO_H */

