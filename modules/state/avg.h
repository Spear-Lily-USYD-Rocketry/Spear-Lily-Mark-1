#ifndef AVG_H
#define AVG_H

#include "boolean.h"
#include "peripheral_intf.h"

// #define BUFFER_MAX_SIZE 100
#define BUFFER_MAX_SIZE 3

// typedef struct _sensor_data
// {
// 	// imu => accelometer values
// 	int gyro_x;
// 	int gyro_y;
// 	int gyro_z;
// 	double accel_x;
// 	double accel_y;
// 	double accel_z;
// 	double mag_x;
// 	double mag_y;
// 	double mag_z;
// 	double baro_pressure;
// 	double baro_temperature;
//
// } sensor_data_t;

typedef struct {
	sensor_data_t data[BUFFER_MAX_SIZE];
	sensor_data_t sum;
	int size;
	int start;
} Avg;


bool initAvg(Avg * avg);

bool pushAvg(Avg * avg, sensor_data_t *data);

// bool popAvg(Avg * avg);

bool getAvg(Avg * avg, sensor_data_t * avg_data);

bool fullAvg(Avg * avg);

bool emptyAvg(Avg * avg);

#endif
