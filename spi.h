#ifndef SPIDEV_US_H
#define SPIDEV_US_H

#include <stdint.h>

int SPI_Write(int fd, char *wr_buf, int size);
int SPI_Read(int fd, char *rd_buf, int size);
int SPI_Init(int spiMode, char *SpiDev, uint8_t bits, uint32_t speed);
int SPI_WriteByte(int fd, uint8_t data);
int SPI_ReadByte(int fd, uint8_t *data);



#endif 