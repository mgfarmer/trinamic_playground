#include <stdio.h>
#include <wiringPi.h>
#include <bcm2835.h>
#include "SPI_TMC.h"

// Include the IC(s) you want to use
#include "TMC-API/tmc/ic/TMC5160/TMC5160.h"

#define MOTOR0			0 // Motor 0
#define MAX_VEL 600000
#define MAX_ACC 5000

void resetMotorDrivers(uint8 motor);

void waitFor(int position, int error) {
	int pos = tmc5160_readInt(MOTOR0, TMC5160_XACTUAL);
	while (pos != position)
	{
		//delay(100);
		pos = tmc5160_readInt(MOTOR0, TMC5160_XACTUAL);
	}
	printf("TMC5160 Position: %d\n", tmc5160_readInt(MOTOR0, TMC5160_XACTUAL));
}


int main(int argc, char **argv) {
	if (!bcm2835_init())
      return 1;

	wiringPiSetupGpio();
  	
	// Initiate SPI 
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); // MSB first
  	bcm2835_spi_setDataMode(BCM2835_SPI_MODE3); // SPI Mode 3
  	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); // 1 MHz clock
  	bcm2835_spi_chipSelect(BCM2835_SPI_CS0); // define CS pin
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // set CS polarity to low
	

	/***** TMC5160-BOB Example *****/
	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	digitalWrite(2, HIGH); // Apply VCC_IO voltage to Motor 0
	digitalWrite(3, LOW);  // Use internal clock
	digitalWrite(4, LOW);  // Enable driver stage

	resetMotorDrivers(MOTOR0);

	// MULTISTEP_FILT=1, EN_PWM_MODE=1 enables stealthChop™
	tmc5160_writeInt(MOTOR0, TMC5160_GCONF, 0x0000000C);

	// TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle™)
	tmc5160_writeInt(MOTOR0, TMC5160_CHOPCONF, 0x000100C3);

	// IHOLD=8, IRUN=15 (max. current), IHOLDDELAY=6
	tmc5160_writeInt(MOTOR0, TMC5160_IHOLD_IRUN, 0x00080F0A);

	// TPOWERDOWN=10: Delay before power down in stand still
	tmc5160_writeInt(MOTOR0, TMC5160_TPOWERDOWN, 0x0000000A);

	// TPWMTHRS=500
	tmc5160_writeInt(MOTOR0, TMC5160_TPWMTHRS, 0x000001F4); 

	// Values for speed and acceleration
	tmc5160_writeInt(MOTOR0, TMC5160_VSTART, 1);
	tmc5160_writeInt(MOTOR0, TMC5160_A1, MAX_ACC);
	tmc5160_writeInt(MOTOR0, TMC5160_V1, 26843);
	tmc5160_writeInt(MOTOR0, TMC5160_AMAX, MAX_ACC);
	tmc5160_writeInt(MOTOR0, TMC5160_VMAX, MAX_VEL);
	tmc5160_writeInt(MOTOR0, TMC5160_DMAX, MAX_ACC);
	tmc5160_writeInt(MOTOR0, TMC5160_D1, MAX_ACC);
	tmc5160_writeInt(MOTOR0, TMC5160_VSTOP, 10);
	tmc5160_writeInt(MOTOR0, TMC5160_RAMPMODE, TMC5160_MODE_POSITION);

	waitFor(0, 10);

	int targetPos = 5 * 51200;
	// put your main code here, to run repeatedly:
	printf("TMC5160 Position: %d\n", tmc5160_readInt(MOTOR0, TMC5160_XACTUAL));

	tmc5160_writeInt(MOTOR0, TMC5160_XTARGET, targetPos);
	waitFor(targetPos, 10);

	delay(500);

	tmc5160_writeInt(MOTOR0, TMC5160_XTARGET, 0); //XTARGET=0
	waitFor(0, 10);

	digitalWrite(4, HIGH); // Disable driver stage
	digitalWrite(2, LOW); // Remove VCC_IO voltage from Motor 0

	// End SPI communication
  	bcm2835_spi_end();
   	bcm2835_close();

	return 0;
}

void resetMotorDrivers(uint8 motor)
{
	if(!tmc5160_readInt(MOTOR0, TMC5160_VACTUAL)) 
	{
		digitalWrite(2, LOW); // Apply VCC_IO voltage to Motor 0
		delay(10);
		digitalWrite(2, HIGH); // Apply VCC_IO voltage to Motor 0
		delay(10);
	}
}