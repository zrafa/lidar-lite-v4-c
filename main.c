#include <stddef.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "twi.h"
#include "lidar-lite.h"

void main() {

	char msg[80];
	uint16_t distance;

	serial_init();
	twi_init();  //Inicializa la interfaz i2c
	lidar_v4_init();  //Inicializa la interfaz i2c

	while(1) {
		
		distance = lidar_v4_get_distance();
		sprintf(msg, "distance (cm): %i", distance);
		serial_put_str(msg);
	}
}
