#include <stddef.h>
#include <stdio.h>
#include <util/delay.h>
#include "twi.h"
#include "lidar-lite.h"
#include "serial.h"


uint8_t SLAVE=0x62;

uint8_t ACQ_COMMANDS = 0x00;
uint8_t FULL_DELAY_LOW = 0x10;
uint8_t FULL_DELAY_HIGH = 0x11;
uint8_t STATUS = 0x01;
uint8_t BOARD_TEMPERATURE = 0xE0;
uint8_t FACTORY_RESET = 0xE4;
uint8_t DETECTION_SENSIVITY = 0x1C;
uint8_t ACQUISITION_COUNT = 0x05;
uint8_t QUICK_TERMINATION = 0xE5;
uint8_t SOC_TEMPERATURE = 0xEC;
uint8_t HIGH_ACCURACY_MODE = 0xEB;

/*
enum
  {
    ACQ_COMMANDS = 0x00,
    STATUS = 0x01,
    ACQUISITION_COUNT = 0x05,
    FULL_DELAY_LOW = 0x10,
    FULL_DELAY_HIGH = 0x11,
    UNIT_ID_0 = 0x16,
    UNIT_ID_1 = 0x17,
    UNIT_ID_2 = 0x18,
    UNIT_ID_3 = 0x19,
    I2C_SEC_ADDR = 0x1A,
    I2C_CONFIG = 0x1B,
    DETECTION_SENSITIVITY = 0x1C,
    LIB_VERSION = 0x30,
    CORR_DATA = 0x52,
    CP_VER_LO = 0x72,
    CP_VER_HI = 0x73, 
    BOARD_TEMPERATURE = 0xE0,
    HARDWARE_VERSION = 0xE1,
    POWER_MODE = 0xE2,
    MEASUREMENT_INTERVAL = 0xE3,
    FACTORY_RESET = 0xE4,
    QUICK_TERMINATION = 0xE5,
    START_BOOTLOADER = 0xE6,
    ENABLE_FLASH_STORAGE = 0xEA,
    HIGH_ACCURACY_MODE = 0xEB,
    SOC_TEMPERATURE = 0xEC,
    ENABLE_ANT_RADIO = 0xF0,
};
*/

uint8_t lidar_status;

void leer_status(uint8_t address, uint8_t *data)
{
    lidar_status = *data;    
}


uint8_t *distance;

void leer_medicion(uint8_t address, uint8_t *data)
{
	distance = data;
//	serial_put_char(0x30+(*data));
//	serial_put_char(0x30+(*(data+1)));
}


int lidar_v4_init()
{
	uint8_t reset = 0x01;
	uint8_t sensivity = 0x20;
	uint8_t count = 0x6f;


/* 0x00: Enable measurement quick termination. The device
         terminates the distance measurement early if it anticipates the
	 signal peak in the correlation record will reach the maximum value.
   0x08: Disable measurement quick termination.
*/
	uint8_t termination = 0x00;

    _delay_ms(22);
/*
    twi_write(SLAVE, &FACTORY_RESET, 1, NULL);
    twi_write(SLAVE, &reset, 1, NULL);
    _delay_ms(25);
    twi_write(SLAVE, &SLAVE, 1, NULL);
    twi_read(SLAVE, 1, leer_status);
	twi_wait();
    _delay_ms(22);
    twi_write(SLAVE, &DETECTION_SENSIVITY, 1, NULL);
    twi_write(SLAVE, &sensivity, 1, NULL);
    _delay_ms(22);
*/
	uint8_t buf[2];

	buf[0] = DETECTION_SENSIVITY;
	buf[1] = sensivity;
    twi_write(SLAVE, &buf[0], 2, NULL);

	buf[0] = ACQUISITION_COUNT;
	buf[1] = count;

    //twi_write(SLAVE, &ACQUISITION_COUNT, 1, NULL);
    //twi_write(SLAVE, &count, 1, NULL);
    twi_write(SLAVE, &buf[0], 2, NULL);

	buf[0] = QUICK_TERMINATION;
	buf[1] = termination;
    //twi_write(SLAVE, &QUICK_TERMINATION, 1, NULL);
    //twi_write(SLAVE, &termination, 1, NULL);
    twi_write(SLAVE, &buf[0], 2, NULL);

	buf[0] = HIGH_ACCURACY_MODE;
	buf[1] = 0x14;
    //twi_write(SLAVE, &QUICK_TERMINATION, 1, NULL);
    //twi_write(SLAVE, &termination, 1, NULL);
    twi_write(SLAVE, &buf[0], 2, NULL);
}

uint16_t lidar_v4_get_distance()
{
	uint16_t distancia_cm = 0;
	uint8_t buf[2];

	/* take range */
	buf[0] = ACQ_COMMANDS;
	buf[1] = 0x04;	/* solicita una medicion */
	twi_write(SLAVE, &buf[0], 2, NULL);

	/* wait while busy */
	lidar_status = 1;
	while ((lidar_status&0x01) != 0) {
		twi_write(SLAVE, &STATUS, 1, NULL);
		twi_read(SLAVE, 1, leer_status);
		twi_wait();
	}

	/* get distance */
	twi_write(SLAVE, &FULL_DELAY_LOW, 1, NULL);
	twi_read(SLAVE, 2, leer_medicion);
	twi_wait();

	distancia_cm = (uint16_t) (*distance | *(distance+1) << 8);

	return distancia_cm;
}

void temp()
{
	char a[10];

	uint16_t distancia_cm = 0;
	uint8_t d = 0x04;
	uint8_t dire = 0x10;

	uint8_t buf[2] = {0x00, 0x04};


    //twi_write(SLAVE, &ACQ_COMMANDS, 1, NULL);
    //twi_write(SLAVE, &d, 1, NULL);
    twi_write(SLAVE, &buf[0], 2, NULL);
	lidar_status=25;
	while ((lidar_status&0x01)!=0) {
		twi_write(SLAVE, &STATUS, 1, NULL);
		twi_read(SLAVE, 1, leer_status);
		twi_wait();
	}

    twi_write(SLAVE, &dire, 1, NULL);
    twi_read(SLAVE, 2, leer_medicion);
	twi_wait();

	distancia_cm = (uint16_t) (*distance | *(distance+1) << 8);
	//sprintf(a, "valor: %i", (*distance));
	sprintf(a, "valor: %i", distancia_cm);
	serial_put_str(a);


}

/*------------------------------------------------------------------------------
  Take Range

  Initiate a distance measurement by writing to register 0x00.
------------------------------------------------------------------------------*/
void lidar_v4_take_range()
{
    uint8_t data_range = 0x04;

    // write(ACQ_COMMANDS, &data, 1);
    twi_write(SLAVE, &ACQ_COMMANDS, 1, NULL);
    twi_write(SLAVE, &data_range, 1, NULL);
	twi_wait();
} 


void lidar_v4_distancia()
{
	char a[10];

    uint8_t data_range = 0x04;

    // write(ACQ_COMMANDS, &data, 1);
    twi_write(SLAVE, &ACQ_COMMANDS, 1, NULL);
    twi_write(SLAVE, &data_range, 1, NULL);

	lidar_status=0;
    twi_write(SLAVE, &FULL_DELAY_LOW, 1, NULL);
    twi_read(SLAVE, 1, leer_status);
	twi_wait();

	sprintf(a, "%i", lidar_status);
	serial_put_str(a);
} 


/*------------------------------------------------------------------------------
  Read Distance

  Read and return the result of the most recent distance measurement.
------------------------------------------------------------------------------*/
uint16_t lidar_v4_read_distance()
{
    uint16_t dist;
//    uint8_t *data = (uint8_t *)&distance;

    // Read two bytes from registers 0x10 and 0x11
    //read(FULL_DELAY_LOW, data, 2);
	twi_write(SLAVE, &FULL_DELAY_LOW , 1,NULL);
	twi_read(SLAVE, 2, leer_medicion);
	twi_wait();
	//_delay_ms(5);
	

	dist = (uint16_t)(*distance | (*(distance+1) << 8));

    return dist; //This is the distance in centimeters
} 


/*------------------------------------------------------------------------------
  Wait for Busy Flag

  Blocking function to wait until the Lidar Lite's internal busy flag goes low
------------------------------------------------------------------------------*/
void lidar_v4_wait()
{
    uint8_t busy_lidar;

    do
    {
        busy_lidar = lidar_v4_get_busy_flag();
    } while (busy_lidar);

}


/*------------------------------------------------------------------------------
  Get Busy Flag

  Read BUSY flag from device registers. Function will return 0x00 if not busy.
------------------------------------------------------------------------------*/
uint8_t lidar_v4_get_busy_flag()
{
//    uint8_t status = 0;
    uint8_t busy_flag; // busyFlag monitors when the device is done with a measurement

	lidar_status = 0;
    // Read status register to check busy flag
    // read(STATUS, &status, 1);
	twi_write(SLAVE, &STATUS, 1, NULL);
	twi_read(SLAVE, 1, leer_status);
	twi_wait();
/*
	while (status==16){
		twi_write(SLAVE, &STATUS, 1, NULL);
		twi_read(SLAVE, 1, leer_status);
		twi_wait();
	}
*/
	


    // STATUS bit 0 is busyFlag
    busy_flag = lidar_status & 0x01;

    return busy_flag;
} 




//uint16_t lidar_v4_get_distance()
//{
    // 1. Trigger a range measurement.
 //   lidar_v4_take_range();

    // 2. Wait for busyFlag to indicate the device is idle.
    //waitForBusy();
//	lidar_v4_wait();

    // 3. Read new distance data from device registers
//	lidar_v4_distancia();
//    return lidar_v4_read_distance();

//}


