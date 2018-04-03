#include "ADS114S08IRHBR.h"
#include "spi.h"


static float GetVoltageFromDigital(int Vdig)
{
	float Vanlg = 2.5* Vdig / ((1 << m_regPGA.u.GAIN)*(215 - 1));
	return Vanlg;
}

static int ADC114S08_ReadChannel(uint8_t channel, float *power)
{

	uint32_t dStatus = 0;
	uint32_t dCRC = 0;
	int Vdig = 0;

	m_regINPMUX.u.MUXP = channel;
	ADC114S08_RegWrite(INPMUX_ADDR, m_regINPMUX.Value);

	if (ADC114S08_ReadChannelData(&dStatus, &dCRC, &Vdig) == 1)
	{
		*power = GetVoltageFromDigital(Vdig);
		printf("Power = %f", *power);
		return 1;
	}
	return 0;
}

//ANT1_BK_PWR	Link #1 back power from Antenna	TBD	Antenna Alarm
int ADC114S08_ReadLink1BackPowerFromAntenna(float *power)
{
	return ADC114S08_ReadChannel(INPMUX_AIN0, power);
}

int ADC114S08_ReadLink1TransmittingPower(float *power)
{
 
	return ADC114S08_ReadChannel(INPMUX_AIN1, power);

}

int ADC114S08_ReadANLG1_IN1(float *power)
{

	return ADC114S08_ReadChannel(INPMUX_AIN2, power);

}

int ADC114S08_ReadANLG1_IN2(float *power)
{

	return ADC114S08_ReadChannel(INPMUX_AIN3, power);

}

//	ANT2_BK_PWR	Link #2 back power from Antenna	TBD	Antenna Alarm
int ADC114S08_ReadLink2BackPowerFromAntenna(float *power)
{
	return ADC114S08_ReadChannel(INPMUX_AIN4, power);
}

// TX2_PWR	Link #2 transmitting power	According user setup	Higher power – reduce RF gain, Low power – increase RF gain, very low power – set antenna alarm
int ADC114S08_ReadLink2TransmittingPower(float *power)
{
	return ADC114S08_ReadChannel(INPMUX_AIN5, power);
}

//	V12_ADC	12v input voltage.Due resistor divider the actual input voltage is : V12_ADC*9.7 / 1.5	11.0v – 13v	Power alarm
int ADC114S08_Read12VInputVoltage(float *power)
{
	int res = ADC114S08_ReadChannel(INPMUX_AIN8, power);
	*power = *power * 9.7 / 1.5;
	return res;
}

//	V5p0_ADC	5v output of 5v DC / DC  - V5p0_ADC*7.64/3.32
int ADC114S08_Read5VOutputOf5VDC_DC(float *power)
{
	int res = ADC114S08_ReadChannel(INPMUX_AIN9, power);
	*power = *power *7.64 / 3.32;
	return res;
}

//	V3p7_ADC	3.7v  DC / DC output.
int ADC114S08_Read3_7VDC_DC(float *power)
{
	int res = ADC114S08_ReadChannel(INPMUX_AIN10, power);
	*power = *power * 2;
	return res;
}

// V1p1	GBE Switch 1.1v core voltage	1v – 1.2v	Power alarm
int ADC114S08_ReadGBESwitch1_1VCoreVoltage(float *power)
{
	int res = ADC114S08_ReadChannel(INPMUX_AIN11, power);
	return res;
}

int ADC114S08_ReadADCInternalTemperature(float *temperature)
{

	uint32_t dStatus = 0;
	uint32_t dCRC = 0;
	int Vdig = 0;
	int res;
	m_regSYS.u.SYS_MON = SYS_CONTROL_SYSMON_INTERNAL_TEMP;
	ADC114S08_RegWrite(SYS_ADDR, m_regSYS.Value);


	if ((res = ADC114S08_ReadChannelData(&dStatus, &dCRC, &Vdig)) == 1)
	{
		float Tk = (Vdig - 8.9e-3) / 403e-6;
		*temperature = Tk - 273.15;
		printf("Internal temperature = %f", *temperature);
	}

	// return to normal operation
	m_regSYS.u.SYS_MON = SYS_CONTROL_SYSMON_NORMAL;
	ADC114S08_RegWrite(SYS_ADDR, m_regSYS.Value);
	
	return res;
}

int ADC114S08_ReadADCInternalPowerSupply(float *power)
{

	return 1;
}


 