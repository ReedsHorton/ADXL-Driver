/*
 * ADXL362.c
 *
 * Methods for initializing communication with Analog Devices ADXL362
 * Accelerometer on TI cc2640R2 (should be abstracted for any
 * board, untested). Consists of SPI communication protocols.
 *
 *
 * Reed Horton
 * Auracle
 * July 2018
 *
 */ 

#include <ti/drivers/SPI.h>
#include <stdio.h>
#include "ADXL362.h"
#include "Board.h"

//#define ADXL362_DEBUG
#define bitrate 8000000

//SPI SW CS Pin Declaration
static PIN_Config spipins[] = {
         Board_DIO26_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_PULLUP,
         PIN_TERMINATE

};
static PIN_Handle 	spiPinHandle;
static PIN_State 	spiPinState;
static SPI_Handle 	spi;


/***************** SPI_begin() *****************/
/* Initializes SPI Communication for board. Takes a bool which tells 
 * the function whether to open SPI0 or SPI1 on the cc2640. Returns
 * true on successful opening of SPI communication, false on failure.
 * Assumes transfers of 1 byte, at speed defined as Macro bitrate.
 * Makes use of software Chip Select (as ti-RTOS default incompatible)
 * through PIN outputs.
 */
bool SPI_begin(bool spiPort) {
	//SPI initialization
	SPI_Params  spiParams;
	SPI_init();
	SPI_Params_init(&spiParams);
	spiParams.dataSize = 8;
	spiParams.bitRate = bitrate;
	spiParams.mode = SPI_MASTER;

	//Opening SPI handle
	if (spiPort){
		spi = SPI_open(Board_SPI1, &spiParams);
	}
	else {
		spi = SPI_open(Board_SPI0, &spiParams);
	}

	if (spi == NULL){
        printf("SPI_open failed!\n");
        return (false);
    }
    //Setting up CS output Pin and setting to High
    spiPinHandle = PIN_open(&spiPinState, spipins);
    PIN_setOutputValue(spiPinHandle, Board_DIO26_ANALOG, 1);

	#ifdef ADXL362_DEBUG
		printf("SPI Opened successfully\n");
	#endif

    return (true);
 }

 /***************** ADXL_begin() *****************/
/* Soft resets ADXL settings and turns on measurement mode. Returns
 * true if both operations are successful, otherwise returns false.
 * ADXL will not output correct XYZ data until approximately 1 ms 
 * after measurment mode turned on.
 */
bool ADXL_begin() {
	bool reset, measurment;
	int i;
	i = 0;

 	reset = SPI_Write_Single(0x1F, 0x52); // soft reset board
 	while(SPI_Read_Single(0x2D) == 0 && i<5){//Confirm Turn on measurement
 	   measurment = MeasurementMode();
 	   i++;
 	}

 	if (SPI_Read_Single(0x2D) == 0) measurment = 0;

 	if(!reset | !measurment) {
 		#ifdef ADXL362_DEBUG
			printf("Reset: %d, Measurement, %d\n", reset, measurment);
		#endif
 		return(false);
 	}
 	else {
 		#ifdef ADXL362_DEBUG
			printf("ADXL Reset and in measurement mode\n");
		#endif
 		return (true);
 	}
}

/***************** StandbyMode() *****************/
/* Puts the sensor into standby mode where power is 10nA
 */
bool StandbyMode() {
	return SPI_Write_Single(0x2D, 0x00);

}

/***************** MeasurementMode() *****************/
/* Puts the sensor into measurement mode
 */
bool MeasurementMode() {
	return SPI_Write_Single(0x2D, 0x02);

}

/***************** Read_X_Short() *****************/
/* Reads the register XDATA and returns the 8 bit value
 */
uint8_t Read_X_Short() {
	return SPI_Read_Single(0x08);
}

/***************** Read_Y_Short() *****************/
/* Reads the register YDATA and returns the 8 bit value
 */
uint8_t Read_Y_Short() {
	return SPI_Read_Single(0x09); 
}
/***************** Read_Z_Short() *****************/
/* Reads the register ZDATA and returns the 8 bit value
 */
uint8_t Read_Z_Short() {
	return SPI_Read_Single(0x0A); 
}


/***************** Read_XYZ_Short() *****************/
/* Reads XYZ short register values and inputs them into an array 
 * passed into the function. Returns void. 
 */

void Read_XYZ_Short(uint8_t *XYZ){
	XYZ[0] = Read_X_Short();
	XYZ[1] = Read_Y_Short();
	XYZ[2] = Read_Z_Short();
}

/***************** Read_X_Long() *****************/
/* Reads the registers XDATA_L and XDATA_H and returns the sign
 * extended 12 bit value. 
 */
int16_t Read_X_Long() {
	uint8_t L, H;
	int16_t Full;

	L = SPI_Read_Single(0x0E); 
	H = SPI_Read_Single(0x0F);

	Full = (H << 8) | L;  //combine the two

	return(Full);
}
/***************** Read_Y_Long() *****************/
/* Reads the registers YDATA_L and YDATA_H and returns the sign
 * extended 12 bit value. 
 */
int16_t Read_Y_Long() {
	uint8_t L, H;
	int16_t Full;

	L = SPI_Read_Single(0x10); 
	H = SPI_Read_Single(0x11);

	Full = (H << 8) | L; //combine the two

	return(Full);
}

/***************** Read_Z_Long() *****************/
/* Reads the registers ZDATA_L and ZDATA_H and returns the sign
 * extended 12 bit value. 
 */
int16_t Read_Z_Long() {
	uint8_t L, H;
	int16_t Full;

	L = SPI_Read_Single(0x12); 
	H = SPI_Read_Single(0x13);

	Full = (H << 8) | L;

	return(Full);
}

/***************** Read_XYZ_Long() *****************/
/* Reads XYZ Long register values and inputs them into an array 
 * passed into the function. Returns void. 
 */

void Read_XYZ_Long(int16_t *XYZ){
	XYZ[0] = Read_X_Long();
	XYZ[1] = Read_Y_Long();
	XYZ[2] = Read_Z_Long();
}

/***************** SPI_Write_Single() *****************/
/* Write one byte to a register. Take the address of the desired 
 * register and the byte to be written. Returns true on successful
 * write and false on failure. 
 */
bool SPI_Write_Single(uint8_t dest_register, uint8_t byte2write) {

	uint8_t transmitBuffer[3];
	SPI_Transaction spiTransaction;
	bool success;

	transmitBuffer[0] = 0x0A;			//write command
	transmitBuffer[1] = dest_register; 	//register to write to
	transmitBuffer[2] = byte2write;		//byte to be written


	spiTransaction.count = 3;
	spiTransaction.txBuf = transmitBuffer;	
	spiTransaction.rxBuf = NULL;

	PIN_setOutputValue(spiPinHandle, Board_DIO26_ANALOG, 0); //CS low
	success = SPI_transfer(spi, &spiTransaction); //send 3 bytes
	PIN_setOutputValue(spiPinHandle, Board_DIO26_ANALOG, 1); // CS high

	#ifdef ADXL362_DEBUG
		if(success) {
			printf("%X written to %X\n", byte2write, dest_register);
		}
		else {
			printf("%X not written to %X\n",byte2write,dest_register);
		}
	#endif

	return(success);
}

/***************** SPI_Read_Single() *****************/
/* Reads one byte from a register. Takes the address of the desired 
 * register. Returns the byte at the register read. 
 */
uint8_t SPI_Read_Single(uint8_t register2read) {
	
	uint8_t transmitBuffer[3];
	uint8_t recieveBuffer[1];
	SPI_Transaction spiTransaction;
	bool success;

	transmitBuffer[0] = 0x0B;		//read command
	transmitBuffer[1] = register2read; //register to read from

	spiTransaction.count = 2;
	spiTransaction.txBuf = transmitBuffer;
	spiTransaction.rxBuf = NULL;

	PIN_setOutputValue(spiPinHandle, Board_DIO26_ANALOG, 0); // CS low
	success = SPI_transfer(spi, &spiTransaction); //command to read

	#ifdef ADXL362_DEBUG
		if(success)	{
			printf("Reading register: %X\n", register2read);
		}
		else {
			printf("SPI Read Command failed\n");
		}
	#endif
	
	spiTransaction.count = 1;
   	spiTransaction.txBuf = NULL;
   	spiTransaction.rxBuf = recieveBuffer;

   	success = SPI_transfer(spi, &spiTransaction); //reading
   	PIN_setOutputValue(spiPinHandle, Board_DIO26_ANALOG, 1); //CS high

   	#ifdef ADXL362_DEBUG
		if(success)	{
			printf("Value is %X\n", recieveBuffer[0]);
		}
		else {
			printf("SPI Read byte failed\n");
		}
	#endif

	return(recieveBuffer[0]);
}
