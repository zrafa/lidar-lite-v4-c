/*
 * main.c: little example program for testing garmin lidar lite v4
 * 
 * Rafael Ignacio Zurita (c) 2022 <rafa@fi.uncoma.edu.ar>
 */


#include <stddef.h>
#include <stdio.h>

#include "serial.h"
#include "twi.h"
#include "lidar-lite.h"

void main() {

	char msg[80];
	uint16_t distance;

	serial_init(115200);
	twi_init();		/* init i2c */
	lidar_v4_init();	/* init lidar with some specific values (check code) */

	while(1) {
		
		distance = lidar_v4_get_distance();
		sprintf(msg, "distance (cm): %i \n", distance);
		serial_put_str(msg);
	}
}
