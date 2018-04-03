#ifndef ADS114S08IRHBR_H
#define ADS114S08IRHBR_H

#include <stdint.h>
#include "HwApi_Gpio.h"


#define GPIO_ADC_STRT  MAKEGPIONUM(1 , 3)
#define GPIO_ADC_RDYn  GPIONUM(1 , 2) 
#define GPIO_SPI1_CS2n  GPIONUM(3 , 24) 


#define NUM_REGISTERS 18
/*
* Address masks used for register addressing with
* either a REGRD of REGWR mask
*
*/
#define ID_ADDR			0x00
#define STATUS_ADDR		0x01
#define INPMUX_ADDR		0x02
#define PGA_ADDR		0x03
#define DATARATE_ADDR	0x04
#define REF_ADDR		0x05
#define IDACMAG_ADDR	0x06
#define IDACMUX_ADDR	0x07
#define VBIAS_ADDR		0x08
#define SYS_ADDR		0x09
#define OFCAL0_ADDR		0x0A
#define OFCAL1_ADDR		0x0B
#define OFCAL2_ADDR		0x0C
#define FSCAL0_ADDR		0x0D
#define FSCAL1_ADDR		0x0E
#define FSCAL2_ADDR		0x0F
#define GPIODAT_ADDR	0x10
#define GPIOCON_ADDR	0x11


#define REG_ID_DEFAULT			0x4
#define REG_STATUS_DEFAULT		0x80
#define REG_INPMUX				0x1
#define REG_PGA_DEFAULT			0x0
#define REG_BITRATE_DEFAULT		0x14
#define REG_REF_DEFAULT			0x10
#define REG_IDACMAG_DEFAULT		0x0
#define REG_IDACMUX_DEFAULT		0xFF
#define REG_VBIAS_DEFAULT		0
#define REG_SYS_DEFAULT			0x10
#define REG_OFCAL0_DEFAULT		0
#define REG_OFCAL1_DEFAULT		0
#define REG_FSCAL0_DEFAULT		0
#define REG_FSCAL1_DEFAULT		0x40
#define REG_GPIODAT_DEFAULT		0x0
#define REG_GPIOCON_DEFAULT		0x0



//Control Commands
//9.5.3 Commands
//Commands are used to control the ADC, access the configuration registers, and retrieve data.Many of the
//commands are stand - alone(that is, single - byte).The register write and register read commands, however, are
//multibyte, consisting of two command bytes plus the register data byte or bytes.The commands are listed in
//Table 24.

#define NOP_OPCODE_CONTROL_COMMAND					0x00
#define WAKE_OPCODE_CONTROL_COMMAND					0x02
#define SLEEP_OPCODE_CONTROL_COMMAND				0x04
#define RESET_OPCODE_CONTROL_COMMAND				0x06
#define START_OPCODE_CONTROL_COMMAND				0x08
#define STOP_OPCODE_CONTROL_COMMAND					0x0A
#define SFOCAL_OPCODE_CONTROL_COMMAND				0x19
#define SYOCAL_OPCODE_CONTROL_COMMAND				0x16
#define SYGCAL_OPCODE_CONTROL_COMMAND				0x17
#define RDATA_OPCODE_CONTROL_COMMAND				0x12
#define REGRD_OPCODE_CONTROL_COMMAND				0x20
#define REGWR_OPCODE_CONTROL_COMMAND				0x40




//00h ID xxh RESERVED DEV_ID[2:0]
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned DEV_ID : 3;
		unsigned RESERVED : 5;

	}u;
} ADS114S08_ID;


//01h STATUS 80h 

typedef union
{
	uint8_t Value;
	struct
	{
		
		unsigned RESERVED : 5;
		unsigned FL_POR :1 ;
		unsigned RDY : 1;
		unsigned FL_P_RAILP : 1;
		unsigned FL_P_RAILN : 1; 
		unsigned FL_N_RAILP : 1; 
		unsigned FL_N_RAILN : 1;
		unsigned FL_REF_L1 : 1; 
		unsigned FL_REF_L0 : 1;

	}u;
} ADS114S08_STATUS;


// 02h INPMUX
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned MUXN : 4;
		unsigned MUXP : 4;

	}u;

} ADS114S08_INPMUX;


// 03h PGA
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned GAIN : 3;
		unsigned PGA_EN : 2;
		unsigned DELAY : 3;
	}u;
} ADS114S08_PGA;




// 04h DATARATE
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned DR : 4;
		unsigned FILTER :1;
		unsigned MODE : 1;
		unsigned CLK : 1;
		unsigned G_CHOP : 1;
	}u;
} ADS114S08_DATARATE;


// 05h REF 
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned REFCON : 2;
		unsigned REFSEL : 2;
		unsigned REFN_BUF : 1;
		unsigned REFP_BUF : 1;
		unsigned FL_REF_EN : 2;
	}u;
} ADS114S08_REF;



// 06h IDACMAG
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned IMAG : 4;
		unsigned ZERO1 : 1;
		unsigned ZERO2 : 1;
		unsigned PSW : 1;
		unsigned FL_RAIL_EN : 1;
	}u;
} ADS114S08_IDACMAG;



// 07h IDACMUX
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned I1MUX : 4;
		unsigned I2MUX : 4;
	}u;
} ADS114S08_IDACMUX;



// 08h VBIAS 
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned VB_LEVEL : 1;		
		unsigned VB_AINC : 1;
		unsigned VB_AIN5 : 1;
		unsigned VB_AIN4 : 1;
		unsigned VB_AIN3 : 1; 
		unsigned VB_AIN2 : 1;
		unsigned VB_AIN1 : 1; 
		unsigned VB_AIN0 : 1;

	}u;
} ADS114S08_VBIAS;



// 09h SYS
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned SENDSTAT : 1;
		unsigned CRC : 1;
		unsigned TIMEOUT : 1;
		unsigned CAL_SAMP : 2;
		unsigned SYS_MON : 3
	}u;
} ADS114S08_SYS;



// 0Bh OFCAL0
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned OFC : 8;
		 
	}u;
} ADS114S08_OFCAL0;

// 0Ch OFCAL1
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned OFC : 8;

	}u;
} ADS114S08_OFCAL1;


// FSCAL0 0Eh
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned FSC : 8;

	}u;
} ADS114S08_FSCAL0;

// FSCAL1 0Fh
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned FSC : 8;

	}u;
} ADS114S08_FSCAL1;

// GPIODAT 10h
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned DAT : 4;
		unsigned DIR : 4;

	}u;
} ADS114S08_GPIODAT;



// GPIOCON 11h
typedef union
{
	uint8_t Value;
	struct
	{
		unsigned CON : 4;
		unsigned ReservedZero : 4;

	}u;
} ADS114S08_GPIOCON;


//Input Multiplexer(INPMUX) Register Field Descriptions
//Selects the ADC positive input channel.
typedef enum INPMUX_Fields
{

	INPMUX_AIN0 = 0,
	INPMUX_AIN1 = 1,
	INPMUX_AIN2 = 2,
	INPMUX_AIN3 = 3,
	INPMUX_AIN4 = 4,
	INPMUX_AIN5 = 5,
	INPMUX_AIN6 = 6,
	INPMUX_AIN7 = 7,
	INPMUX_AIN8 = 8,
	INPMUX_AIN9 = 9,
	INPMUX_AIN10 = 10,
	INPMUX_AIN11 = 11,
	INPMUX_AINCOM = 12

} INPMUX_Fields;



//Programmable conversion delay selection
//Sets the programmable conversion delay time for the first conversion after a
//WREG when a configuration change resets of the digital filter and triggers a
//new conversion(1)
typedef enum PGA_DELAY_Fields
{
	PGA_14 = 0,
	PGA_25 = 1,
	PGA_64 = 2,
	PGA_256 = 3,
	PGA_1024 = 4,
	PGA_2048 = 5,
	PGA_4096 = 6,
	PGA_1 = 7

} PGA_DELAY_Fields;

//Enables or bypasses the PGA.
typedef enum PGA_ENABLE_Fields
{
	PGA_POWERED_DOWN_AND_BYPASSED = 0,
	PGA_ENABLED = 1

} PGA_ENABLE_Fields;


//Configures the PGA gain
typedef enum PGA_PAGE_SELECTION
{

	PGA_GAIN_1 = 0,
	PGA_GAIN_2 = 1,
	PGA_GAIN_4 = 2,
	PGA_GAIN_8 = 3,
	PGA_GAIN_16 = 4,
	PGA_GAIN_32 = 5,
	PGA_GAIN_64 = 6,
	PGA_GAIN_128 = 7,

} PGA_PAGE_SELECTION;


// Enables the global chop function.When enabled, the device automatically
// swaps the inputs and takes the average of two consecutive readings to
// cancel the offset voltage.

typedef enum DATARATE_SELECTION_G_CHOP
{
	DR_G_CHOP_DISABLE = 0,
	DR_G_CHOP_ENABLE = 1

} DATARATE_SELECTION_G_CHOP;


// Configures the clock source to use either the internal oscillator or an
// external clock

typedef enum DATARATE_SELECTION_CLK
{
	DR_CLK_INTERNAL_4_096MHZ = 0,
	DR_CLK_EXTERNAL_CLOCK = 1

} DATARATE_SELECTION_CLK;


// Configures the ADC for either continuous conversion or single - shot
// conversion mode
typedef enum DATARATE_SELECTION_MOD
{
	DR_MOD_CONTINUOUS_CONVERSION_MODE = 0,
	DR_MOD_SINGLE_SHOT_CONVERSION_MODE = 1

} DATARATE_SELECTION_MOD;

// Configures the ADC to use either the sinc3 or the low-latency filter
typedef enum DATARATE_SELECTION_FILTER
{
	DR_FILTER_SINC3 = 0,	
	DR_FILTER_LOW_LATENCY_FILTER = 1

} DATARATE_SELECTION_FILTER;


// Data rate selection
// Configures the output data rate
typedef enum DATARATE_SELECTION_DR
{
	DR_SEL_2_5_SPS = 0,
	DR_SEL_5_SPS = 1,
	DR_SEL_10_SPS = 2,
	DR_SEL_16_6_SPS = 3,
	DR_SEL_20_SPS = 4,
	DR_SEL_50_SPS = 5,
	DR_SEL_60_SPS = 6,
	DR_SEL_100_SPS = 7,
	DR_SEL_200_SPS = 8,
	DR_SEL_400_SPS = 9,
	DR_SEL_800_SPS = 10,
	DR_SEL_1000_SPS = 11,
	DR_SEL_2000_SPS = 12,
	DR_SEL_4000_SPS = 13,
	DR_SEL_4000_SPS = 14,

} DATARATE_SELECTION_DR;



//System Control(SYS) Register Field Descriptions



//Enables a set of system monitor measurements using the ADC.
typedef enum SYS_CONTROL_SYSMON
{
	SYS_CONTROL_SYSMON_DISABLE = 0,
	SYS_CONTROL_SYSMON_NORMAL = 1,
	SYS_CONTROL_SYSMON_INTERNAL_TEMP = 2,
	SYS_CONTROL_SYSMON_AVDD_AVSS_4_MEASUREMENT_GAIN_SET_TO_1 = 3,
	SYS_CONTROL_SYSMON_DVDD_4_MEASUREMENT_GAIN_SET_TO_1 = 4,
	SYS_CONTROL_SYSMON_BURNOUT_CURRENT_SOURCES_ENABLED_0_2_MA = 5,
	SYS_CONTROL_SYSMON_BURNOUT_CURRENT_SOURCES_ENABLED_1_MA = 6,
	SYS_CONTROL_SYSMON_BURNOUT_CURRENT_SOURCES_ENABLED_10_MA = 7,

} SYS_CONTROL_SYSMON;






//Calibration sample size selection
//Configures the number of samples averaged for self and system offset and
//system gain calibration.

typedef enum SYS_CONTROL_SAMP
{
	SYS_CONTROL_CAL_SAMP_1_SAMPLE = 0,
	SYS_CONTROL_CAL_SAMP_4_SAMPLE = 1,
	SYS_CONTROL_CAL_SAMP_8_SAMPLE = 2,
	SYS_CONTROL_CAL_SAMP_16_SAMPLE = 3

} SYS_CONTROL_SAMP;


// SPI timeout enable
// Enables the SPI timeout function.
typedef enum SYS_CONTROL_TIMEOUT
{
	SYS_CONTROL_SPI_TIMEOUT_DISABLE = 0, // default
	SYS_CONTROL_SPI_TIMEOUT_ENABLE = 1

} SYS_CONTROL_TIMEOUT;


// CRC enable
// Enables the CRC byte appended to the conversion result.When enabled,
// CRC is calculate
typedef enum SYS_CONTROL_CRC
{
	SYS_CONTROL_CRC_DISABLE = 0, // default
	SYS_CONTROL_CRC_ENABLE = 1

} SYS_CONTROL_CRC;

//Enables the STATUS byte prepended to the conversion result.
typedef enum SYS_CONTROL_STATUS
{
	SYS_CONTROL_STATUS_DISABLE = 0, // default
	SYS_CONTROL_STATUS_ENABLE = 1

} SYS_CONTROL_STATUS;


 
 





int ADC114S08_Init();
int ADC114S08_PutToSleep();
int ADC114S08_StartByCommand();
int ADC114S08_StartByCommand();
int ADC114S08_WakeUpTheDevice();
int ADC114S08_ResetDeviceToDefaultSettings();
int ADC114S08_SelfOffsetCalibration();
void AADC114S08_PrintAllRegs();
int ADC114S08_SystemGainCalibration();
int ADC114S08_ReadChannelData(uint32_t *dStatus, uint32_t *dCRC, int *iData);
int ADC114S08_ReadLink1BackPowerFromAntenna(float *power);
int ADC114S08_ReadLink1TransmittingPower(float *power);
int ADC114S08_ReadANLG1_IN1(float *power);
int ADC114S08_ReadANLG1_IN2(float *power);
int ADC114S08_ReadLink2BackPowerFromAntenna(float *power);
int ADC114S08_ReadLink2TransmittingPower(float *power);
int ADC114S08_Read12VInputVoltage(float *power);
int ADC114S08_Read5VOutputOf5VDC_DC(float *power);
int ADC114S08_Read3_7VDC_DC(float *power);
int ADC114S08_ReadGBESwitch1_1VCoreVoltage(float *power);
int ADC114S08_ReadADCInternalTemperature(float *temperature);
int ADC114S08_ReadADCInternalPowerSupply(float *power);







extern ADS114S08_ID			m_regID;
extern ADS114S08_STATUS		m_regStatus;
extern ADS114S08_INPMUX     m_regINPMUX;
extern ADS114S08_PGA		m_regPGA;
extern ADS114S08_DATARATE	m_regDataRate;
extern ADS114S08_REF		m_regREF;
extern ADS114S08_IDACMAG	m_regIDACMAG;
extern ADS114S08_IDACMUX	m_regIDACMUX;
extern ADS114S08_VBIAS		m_regVBIAS;
extern ADS114S08_SYS		m_regSYS;
extern ADS114S08_OFCAL0		m_regOFCAL0;
extern ADS114S08_OFCAL1		m_regOFCAL1;
extern ADS114S08_FSCAL0		m_regFSCAL0;
extern ADS114S08_FSCAL1		m_regFSCAL1;
extern ADS114S08_GPIODAT    m_regGPIODAT;
extern ADS114S08_GPIOCON    m_regGPIOCON;





#endif 