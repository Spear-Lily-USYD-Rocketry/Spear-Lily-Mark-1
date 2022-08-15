#include "assert.h"
#include "avg.h"

bool initAvg(Avg * avg){
	avg->size = 0; // initially nothing in this datastucture
	avg->start = 0; // location of first data to remove
	avg->sum.gyro_x           = 0; 
	avg->sum.gyro_y           = 0; 
	avg->sum.gyro_z           = 0; 
	avg->sum.accel_x          = 0; 
	avg->sum.accel_y          = 0; 
	avg->sum.accel_z          = 0; 
	avg->sum.mag_x            = 0; 
	avg->sum.mag_y            = 0; 
	avg->sum.mag_z            = 0; 
	avg->sum.baro_pressure    = 0; 
	avg->sum.baro_temperature = 0; 
	return false;
}

bool popAvg(Avg * avg){
	assert (!emptyAvg(avg));
	avg->sum.gyro_x           -= avg->data[avg->start].gyro_x          ;
	avg->sum.gyro_y           -= avg->data[avg->start].gyro_y          ;
	avg->sum.gyro_z           -= avg->data[avg->start].gyro_z          ;
	avg->sum.accel_x          -= avg->data[avg->start].accel_x         ;
	avg->sum.accel_y          -= avg->data[avg->start].accel_y         ;
	avg->sum.accel_z          -= avg->data[avg->start].accel_z         ;
	avg->sum.mag_x            -= avg->data[avg->start].mag_x           ;
	avg->sum.mag_y            -= avg->data[avg->start].mag_y           ;
	avg->sum.mag_z            -= avg->data[avg->start].mag_z           ;
	avg->sum.baro_pressure    -= avg->data[avg->start].baro_pressure   ;
	avg->sum.baro_temperature -= avg->data[avg->start].baro_temperature;
	// subtract the sum for the others here
	// your code...

	avg->start = (avg->start + 1) % BUFFER_MAX_SIZE;
	avg->size --;
	return false;
}

bool pushAvg(Avg * avg, sensor_data_t *data){
	bool status = false;
	if (fullAvg(avg)){ // if full
		status |= popAvg(avg);
	}
	avg->sum.gyro_x           += data->gyro_x          ;
	avg->sum.gyro_y           += data->gyro_y          ;
	avg->sum.gyro_z           += data->gyro_z          ;
	avg->sum.accel_x          += data->accel_x         ;
	avg->sum.accel_y          += data->accel_y         ;
	avg->sum.accel_z          += data->accel_z         ;
	avg->sum.mag_x            += data->mag_x           ;
	avg->sum.mag_y            += data->mag_y           ;
	avg->sum.mag_z            += data->mag_z           ;
	avg->sum.baro_pressure    += data->baro_pressure   ;
	avg->sum.baro_temperature += data->baro_temperature;

	int end = (avg->start + avg->size) % BUFFER_MAX_SIZE; // wrap it around

	avg->data[end].gyro_x           = data->gyro_x          ;
	avg->data[end].gyro_y           = data->gyro_y          ;
	avg->data[end].gyro_z           = data->gyro_z          ;
	avg->data[end].accel_x          = data->accel_x         ;
	avg->data[end].accel_y          = data->accel_y         ;
	avg->data[end].accel_z          = data->accel_z         ;
	avg->data[end].mag_x            = data->mag_x           ;
	avg->data[end].mag_y            = data->mag_y           ;
	avg->data[end].mag_z            = data->mag_z           ;
	avg->data[end].baro_pressure    = data->baro_pressure   ;
	avg->data[end].baro_temperature = data->baro_temperature;

	avg->size ++;
	return status;
}



bool getAvg(Avg * avg, sensor_data_t * avg_data){
	avg_data->gyro_x           = avg->sum.gyro_x           / avg->size;
	avg_data->gyro_y           = avg->sum.gyro_y           / avg->size;
	avg_data->gyro_z           = avg->sum.gyro_z           / avg->size;
	avg_data->accel_x          = avg->sum.accel_x          / avg->size;
	avg_data->accel_y          = avg->sum.accel_y          / avg->size;
	avg_data->accel_z          = avg->sum.accel_z          / avg->size;
	avg_data->mag_x            = avg->sum.mag_x            / avg->size;
	avg_data->mag_y            = avg->sum.mag_y            / avg->size;
	avg_data->mag_z            = avg->sum.mag_z            / avg->size;
	avg_data->baro_pressure    = avg->sum.baro_pressure    / avg->size;
	avg_data->baro_temperature = avg->sum.baro_temperature / avg->size;
	return false;
}

bool fullAvg(Avg * avg){
	return avg->size == BUFFER_MAX_SIZE; // same as the max size, then it's full
}

bool emptyAvg(Avg * avg){
	return avg->size == 0; // just check the size is 0
}
