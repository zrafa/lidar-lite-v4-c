
#ifndef LIDAR_LITE_H
#define LIDAR_LITE_H

void lidar_v4_temp();

int lidar_v4_init();
void lidar_v4_take_range();
uint16_t lidar_v4_read_distance();
void lidar_v4_wait();
uint8_t lidar_v4_get_busy_flag();
uint16_t lidar_v4_get_distance();

#endif  // LIDAR_LITE_H
