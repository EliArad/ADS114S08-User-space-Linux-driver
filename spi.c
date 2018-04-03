#include <stdint.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "spi.h"


static const char *device = "/dev/spidev1.1";
//static uint8_t bits = 8;
//static uint32_t speed = 500000;
static uint16_t delay;

//ADC.Only SPI mode 1 (CPOL = 0, CPHA = 1) i

int SPI_Init(int spiMode, char *SpiDev, uint8_t bits, uint32_t speed)
{
	int ret = 0;
	int fd;
	int mode;
	int readSpeed;

	fd = open(SpiDev, O_RDWR);
	if (fd < 0)
	{
		return 0;
	}

	/*
	* spi mode
	*/
	ret = ioctl(fd, SPI_IOC_WR_MODE, &spiMode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
		return 0;
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);	
	if (ret == -1)
	{
		pabort("can't get spi mode");
		return 0;
	}

	/*
	* bits per word
	*/
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't set bits per word");
		return 0;
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't get bits per word");
		return 0;
	}

	/*
	* max speed hz
	*/
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't set max speed hz");
		return 0;
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &readSpeed);
	if (ret == -1)
	{
		pabort("can't get max speed hz");
		return 0;
	}

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", readSpeed, readSpeed / 1000);

	return 1;
}

int SPI_Write(int fd, uint8_t *wr_buf, int size)
{
	if (write(fd, wr_buf, size) != size)
	{
		perror("Write Error");
		return 0;
	}
	return 1;
}

int SPI_Read(int fd, uint8_t *rd_buf, int size)
{
	if (read(fd, rd_buf, size) != size)
	{
		perror("Read Error");
		return 0;
	}
	return 1;
}

int SPI_WriteByte(int fd, uint8_t data)
{
	if (write(fd, data, 1) != 1)
	{
		perror("Write Error");
		return 0;
	}
	return 1;
}

int SPI_ReadByte(int fd, uint8_t *data)
{
	if (read(fd, data, 1) != 1)
	{
		perror("Write Error");
		return 0;
	}
	return 1;
}



