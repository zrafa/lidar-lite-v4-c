#include <stddef.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "twi.h"
#include "lidar-lite.h"

void main() {

	char msg[80];
	uint16_t newDistance;

	serial_init();
	twi_init();  //Inicializa la interfaz i2c
	lidar_v4_init();  //Inicializa la interfaz i2c

	sei();
	serial_put_str("test de prueba\n");

	while(1) {
/*

  		newDistance = lidar_v4_get_distance();
		serial_put_char(0x30+newDistance);
		serial_put_char('\n');
		sprintf(msg, "distancia (cm): %i \n", newDistance);
		serial_put_str(msg);
*/
  		// _delay_ms(20);  //Don't hammer too hard on the I2C bus
//  		newDistance = lidar_v4_get_distance();
		lidar_v4_temp();
	}
}
