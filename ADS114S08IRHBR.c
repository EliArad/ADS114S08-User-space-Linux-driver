#include "ADS114S08IRHBR.h"
#include "spi.h"


ADS114S08_ID		m_regID;
ADS114S08_STATUS	m_regStatus;
ADS114S08_INPMUX    m_regINPMUX;
ADS114S08_PGA		m_regPGA;
ADS114S08_DATARATE	m_regDataRate;
ADS114S08_REF		m_regREF;
ADS114S08_IDACMAG	m_regIDACMAG;
ADS114S08_IDACMUX	m_regIDACMUX;
ADS114S08_VBIAS		m_regVBIAS;
ADS114S08_SYS		m_regSYS;
ADS114S08_OFCAL0	m_regOFCAL0;
ADS114S08_OFCAL1	m_regOFCAL1;
ADS114S08_FSCAL0	m_regFSCAL0;
ADS114S08_FSCAL1	m_regFSCAL1;
ADS114S08_GPIODAT   m_regGPIODAT;
ADS114S08_GPIOCON   m_regGPIOCON;


static int ADC114S08_DataRead(uint32_t *dStatus, uint32_t *dCRC, int *iData);
static int ADC114S08_RegWrite(unsigned int regnum, uint8_t data);
static int ADC114S08_RegRead(unsigned int regnum, uint8_t *data);


int adcSpiFD = -1;

int ADC114S08_Init()
{
	adcSpiFD = SPI_Init(1, "/dev/spidev1.1" ,8, 500000);

	ADC114S08_RegRead(ID_ADDR, &m_regID.Value);
	if (m_regID.u.DEV_ID != 0x4)
	{
		printf("Cannot find ADC Device ADC114S08  %d\n", m_regID.Value);
		return 0;
	} 

	ADC114S08_RegRead(ID_ADDR, &m_regStatus.Value);
	if (m_regStatus.Value != REG_STATUS_DEFAULT)
	{
		printf("Default status of ADC114S08 incorrect %d\n", m_regID.Value);
		return 0;
	}
	ADC114S08_RegRead(ID_ADDR, &m_regIPMux.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regPGA.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regDataRate.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regREF.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regIDACMAG.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regIDACMUX.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regVBIAS.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regSYS.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regOFCAL0.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regOFCAL1.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regFSCAL0.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regFSCAL1.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regGPIODAT.Value);
	ADC114S08_RegRead(ID_ADDR, &m_regGPIOCON.Value);

	GPIO_Export(GPIO_ADC_STRT);
	GPIO_Export(GPIO_ADC_RDYn);
	//SPI1_CS2n	SPI #1 A / D access Select.Act.Low
	GPIO_Export(GPIO_SPI1_CS2n);

	GPIO_Direction(GPIO_SPI1_CS2n, GPIO_OUT_DIR);
	GPIO_Direction(GPIO_ADC_STRT, GPIO_OUT_DIR);
	GPIO_Direction(GPIO_ADC_RDYn, GPIO_IN_DIR);
	GPIO_Open(GPIO_ADC_STRT);
	GPIO_Open(GPIO_ADC_RDYn);
	GPIO_Open(GPIO_SPI1_CS2n);



	// Set DATARATE register (ADR = 04h).The recommended sample rate is 200SPS and single sample.
	// This yield the following register value: 38h
	m_regDataRate.u.DR = DR_SEL_200_SPS;
	ADC114S08_RegWrite(DATARATE_ADDR, m_regDataRate.Value);

	// Select channel to sampled by setting negative input mux  to AINCOM and channel will be the positive mux ( see ADS114SMonitor.c)
	m_regINPMUX.u.MUXN = INPMUX_AINCOM;
	ADC114S08_RegWrite(INPMUX_ADDR, m_regINPMUX.Value);

	//Set appropriate PGA gain and if in bypass mode by write to register PGA(ADR = 03h).
	// Generally, the PGA gain is one, So the PGA register should be set to : 08h

	m_regPGA.u.GAIN = 0;
	m_regPGA.u.PGA_EN = 1;
	ADC114S08_RegWrite(PGA_ADDR, m_regPGA.Value);

	m_regSYS.u.CRC = SYS_CONTROL_CRC_ENABLE;
	m_regSYS.u.SENDSTAT = SYS_CONTROL_STATUS_ENABLE;
	m_regSYS.u.SYS_MON = SYS_CONTROL_SYSMON_NORMAL;
	// leave the spi timeout disable 
	ADC114S08_RegWrite(SYS_ADDR, m_regSYS.Value);

	//Set reference control register (REF ADR = 05h  to internal 2.5v reference always on by write value 3Ah
	m_regREF.Value = 0x3A;
	ADC114S08_RegWrite(REF_ADDR, m_regREF.Value);

	ADC114S08_WakeUpTheDevice();
	

}

int ADC114S08_StartByCommand()
{
	return SPI_WriteByte(adcSpiFD, START_OPCODE_CONTROL_COMMAND);
}

int ADC114S08_StartByCommand()
{
	return SPI_WriteByte(adcSpiFD, STOP_OPCODE_CONTROL_COMMAND);
}

int ADC114S08_PutToSleep()
{
	return SPI_WriteByte(adcSpiFD, SLEEP_OPCODE_CONTROL_COMMAND);
}

int ADC114S08_SystemGainCalibration()
{
	return SPI_WriteByte(adcSpiFD, SYGCAL_OPCODE_CONTROL_COMMAND);
}
 


static int ADC114S08_RegWrite(unsigned int regnum, uint8_t data)
{
	unsigned long ulDataTx[3];
	ulDataTx[0] = REGWR_OPCODE_CONTROL_COMMAND + (regnum & 0x1f);
	ulDataTx[1] = 0x00;
	ulDataTx[2] = data;
	
	return SPI_Write(adcSpiFD, ulDataTx, 3);
	//clearChipSelect();
	//xferWord(ulDataTx[0]);
	//xferWord(ulDataTx[1]);
	//xferWord(ulDataTx[2]);
	//setChipSelect();
}

int ADC114S08_WakeUpTheDevice()
{
	uint8_t res = SPI_WriteByte(adcSpiFD, WAKE_OPCODE_CONTROL_COMMAND);
	return res;
}

int ADC114S08_ResetDeviceToDefaultSettings()
{
	return SPI_WriteByte(adcSpiFD, RESET_OPCODE_CONTROL_COMMAND);
}

int ADC114S08_SelfOffsetCalibration()
{
	return SPI_WriteByte(adcSpiFD, SFOCAL_OPCODE_CONTROL_COMMAND);
}

void AADC114S08_PrintAllRegs()
{
	uint8_t data;
	for (int i = 0; i < NUM_REGISTERS; i++)
	{
		ADC114S08_RegRead(i, &data);
		printf("Register[%d] = %x\n", i, data);
	}
}

static int ADC114S08_RegRead(unsigned int regnum, uint8_t *data)
{
	int i;
	uint32_t ulDataTx[2];
	uint32_t ulDataRx[3];
	uint32_t junk;
	ulDataTx[0] = REGRD_OPCODE_CONTROL_COMMAND + (regnum & 0x1f);
	ulDataTx[1] = 0x00;

	if (SPI_Write(adcSpiFD, ulDataTx, 2) == 0)
		return 0;
	return SPI_ReadByte(adcSpiFD, data);	 
}

int ADC114S08_ReadChannelData(uint32_t *dStatus, uint32_t *dCRC, int *dData)
{

	*dStatus = 0;
	*dData = 0;
	*dCRC = 0;
	 
	GPIO_Write(GPIO_ADC_STRT);
	usleep(2);

	/* wait for nDRDY_REG to deassert as a known valid data */
	while (GPIO_Read(GPIO_ADC_RDYn) == 1)
	{
		usleep(2000);
	}

	/* Now read out the results as conversion is completed */
	return ADC114S08_DataRead(dStatus, dCRC, dData);

}
static int ADC114S08_DataRead(uint32_t *dStatus, uint32_t *dCRC, int *iData)
{
	 
	int iData0;
	int iData1;
	
	if (m_regSYS.u.SENDSTAT == SYS_CONTROL_STATUS_ENABLE)
	{		 
		if (SPI_ReadByte(adcSpiFD, dStatus) == 0)
			return 0;
	}

	// get the conversion data
	if (SPI_ReadByte(adcSpiFD, &iData0) == 0)
		return 0;
	if (SPI_ReadByte(adcSpiFD, &iData1) == 0)
		return 0;
	*iData = (iData1 << 8) + iData0;

	if (m_regSYS.u.CRC == SYS_CONTROL_CRC_ENABLE)
	{
		if (SPI_ReadByte(adcSpiFD, dCRC) == 0)
			return 0;
	}

	return 1;
}

