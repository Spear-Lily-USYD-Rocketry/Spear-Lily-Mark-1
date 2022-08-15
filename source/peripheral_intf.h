#ifndef PERIPHERAL_INTF_H_
#define PERIPHERAL_INTF_H_

#include <stdint.h>


typedef struct _calibration_data
{
	double accel_x_offset;
	double accel_y_offset;
	double accel_z_offset;
	double gyro_x_offset;
	float gyro_y_offset;
	float gyro_z_offset;
} calibration_data_t;

typedef struct _sensor_data
{

	int gyro_x;
	int gyro_y;
	int gyro_z;
	double accel_x;
	double accel_y;
	double accel_z;
	double mag_x;
	double mag_y;
	double mag_z;
	double baro_pressure;
	double baro_temperature;
	uint8_t state;
} sensor_data_t;

#endif /* PERIPHERAL_INTF_H_ */
