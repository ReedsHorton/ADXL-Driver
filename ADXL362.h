#ifndef __ADXL362_H
#define __ADXL362_H

#include <stdio.h>


/**************** export primitives *********************************/

bool SPI_begin(bool spiPort);
bool ADXL_begin();
bool SPI_Write_Single(uint8_t dest_register, uint8_t byte2write);
uint8_t SPI_Read_Single(uint8_t register2read);
uint8_t Read_X_Short();
uint8_t Read_Y_Short();
uint8_t Read_Z_Short();
void Read_XYZ_Short(uint8_t *XYZ);
int16_t Read_X_Long();
int16_t Read_Y_Long();
int16_t Read_Z_Long();
void Read_XYZ_Long(int16_t *XYZ);
bool StandbyMode();
bool MeasurementMode();

#endif // __ADXL362_H
