#include "state.h"

#define DIFF_ACC 1 // this is the acceleration difference required for lift_off to coasting state transformation

double getAbsAcceleration(sensor_data_t *avg_g_sensor_data_handle){
	return sqrt(avg_g_sensor_data_handle->accel_x * avg_g_sensor_data_handle->accel_x +
			avg_g_sensor_data_handle->accel_y * avg_g_sensor_data_handle->accel_y + 
			avg_g_sensor_data_handle->accel_z * avg_g_sensor_data_handle->accel_z);
}

bool isStateChange(sensor_data_t *g_sensor_data_handle,sensor_data_t *avg_g_sensor_data_handle, State *state) {
	if (state->state == IDLE) { // 0
		// check the input condition
		// return avg_g_sensor_data_handle->accel_z < 10 &&
		// 	avg_g_sensor_data_handle->accel_z > 9;
		return true;
	} else if (state->state == INERT) { // 1
		// return avg_g_sensor_data_handle->accel_x < 10 &&
		// 	avg_g_sensor_data_handle->accel_x > 9;
		return true;
	} else if (state->state == FLIGHT) { // 2
		if(avg_g_sensor_data_handle->accel_x < 12 &&
			avg_g_sensor_data_handle->accel_x > 8){
			state->flight.last_altitude = getAltitude(avg_g_sensor_data_handle->baro_pressure,
					avg_g_sensor_data_handle->baro_temperature);
				return true;
		}
		return false;
	} else if (state->state == ARMED) { // 3

		//printf("isStateChange#if#if#if#if avg_g_sensor_data_handle->accel_x > 30: %d \n", avg_g_sensor_data_handle->accel_x > 30);// __AUTO_GENERATED_PRINT_VAR__
		//printf("isStateChange#if#if#if#if getAltitude(avg_g_sensor_data_handle->baro_pressure, avg_g_sensor_data_handle->baro_temperature): %lf \n", getAltitude(avg_g_sensor_data_handle->baro_pressure, avg_g_sensor_data_handle->baro_temperature));// __AUTO_GENERATED_PRINT_VAR__
		//printf("isStateChange#if#if#if#if state->flight.last_altitude: %lf \n", state->flight.last_altitude);// __AUTO_GENERATED_PRINT_VAR__

		return /* avg_g_sensor_data_handle->accel_x > 30 && */
			getAltitude(avg_g_sensor_data_handle->baro_pressure,
					avg_g_sensor_data_handle->baro_temperature) > (70 + state->flight.last_altitude);
		// the one second, 'jolt', condition to be added
	} else if (state->state == LIFT_OFF) { // 4 // +powered flight
		// calc the absolute accel values we need to cache the absolute acceleration of the last 100 points, sum?
		// in this state it accelerating until it runs out of propellant, and then it deaccelerates but still moving forward

		
		#ifdef PRESSURE_TESTING

			double avg_altitude = getAltitude(avg_g_sensor_data_handle->baro_pressure, avg_g_sensor_data_handle->baro_temperature);
			double current_altitude = getAltitude(g_sensor_data_handle->baro_pressure, g_sensor_data_handle->baro_temperature);

			if (avg_altitude < current_altitude)
				return false;
			else
				return true;

//			if (state->lift_off.baro_count > 3)
//				return true;
//			else
//				return false;

		#else
			double acc = getAbsAcceleration(avg_g_sensor_data_handle);
			if (state->lift_off.count == 0){
				state->lift_off.acc = acc;
			}
			else if (state->lift_off.count % BUFFER_MAX_SIZE == 0){
				if (state->lift_off.acc - acc > DIFF_ACC){
					return true;
				}
				else{
					state->lift_off.acc = acc;
				}
			}
			state->lift_off.count++;
			return false;
		#endif

		// the problem... (sliding window vs jumping window)
		// 1 2 3 4 5 8 8 8 | => change state 7 6 5 5 4 // what we want
		// 1 1 1 2 2 3 4 5 | 5 5 | 4 3 2 // what would probably happen (because been smoothed out in the sliding window algorithm)
  } else if (state->state == COASTING) { //5
		double acc = getAbsAcceleration(avg_g_sensor_data_handle);
		double altitude = getAltitude(avg_g_sensor_data_handle->baro_pressure, avg_g_sensor_data_handle->baro_temperature);

		#ifdef PRESSURE_TESTING
			//if > altitude lock
			if (state->coasting.count == 0){
				state->coasting.altitude = altitude;
			}
			if (state->coasting.count % BUFFER_MAX_SIZE == 0){
				if (state->coasting.altitude > altitude){ //minimum
					state->deployment.drogue_apogee_wait_count = 0;
					return true;
				}
				else{
					state->coasting.altitude = altitude;
				}
			}
			state->coasting.count++;
			return false;

		#else

			//if > altitude lock
			if (state->coasting.count == 0){
				state->coasting.acc = acc;
			}
			if (state->coasting.count % BUFFER_MAX_SIZE == 0){
				if (state->coasting.acc < acc){ //minimum
					state->deployment.drogue_apogee_wait_count = 0;
					return true;
				}
				else{
					state->coasting.acc = acc;
				}
			}
			state->coasting.count++;
			return false;

		#endif


  } else if (state->state == APOGEE) { //6

	  	//after 2 seconds change the DROGUE_DEPLOYMENT
	  //TOTAL_TASKS = 4
	  //immediate no wait time
		if ((int)state->deployment.drogue_apogee_wait_count * (10000*4) > 400000)
		{
			return true;
		}
		state->deployment.drogue_apogee_wait_count++;
		return false;

  } else if (state->state == DROGUE_DEPLOYMENT) { //7

		return state->deployment.drogue_fire_complete && (getAltitude(avg_g_sensor_data_handle->baro_pressure, avg_g_sensor_data_handle->baro_temperature) < 304.8); // 1000 ft

  } else if (state->state == MAIN_DEPLOYMENT) { //8
		return state->deployment.main_fire_complete;

  } else if (state->state == DESCENT) { //9
		// acceleration values should be approximately the same?

	  return (getAbsAcceleration(avg_g_sensor_data_handle) > 8 && getAbsAcceleration(avg_g_sensor_data_handle) < 12) ||  abs(getAltitude(avg_g_sensor_data_handle->baro_pressure, avg_g_sensor_data_handle->baro_temperature) - state->flight.last_altitude) < 40;

  } else if (state->state == TOUCHDOWN) {//10

	  return false; // return true so that we can terminate the program in the next call of changeState
  }
  return false;
}

bool changeState(sensor_data_t* g_sensor_data_handle, State *state){
	if (state->state == TOUCHDOWN){ //9
		return true;
	}
	state->state += 1;
	return false;
	// if (state->state == IDLE){
	// 	state->state = INERT;
	// 	return false;
	// }
}

bool performAction(sensor_data_t* g_sensor_data_handle, State *state){
	return false;
}

bool initState(State* state){
	state->state = IDLE;
	// state->flight.last_altitude;
	// state->lift_off.acc;
	state->lift_off.count = 0;
	state->lift_off.baro_count = 0;

	// state->coasting.acc;
	state->coasting.count = 0;
	state->coasting.altitude = 0;

	state->deployment.drogue_apogee_wait_count = 0;
	state->deployment.pyro_drogue_loop_count = 0;
	state->deployment.pyro_main_loop_count = 0;
	state->deployment.drogue_fire_complete = false;
	state->deployment.main_fire_complete = false;

	return false;
}

double getAltitude(double pressure, double temperature){
//	printf("getAltitude pressure: %lf \n", pressure);// __AUTO_GENERATED_PRINT_VAR__
//	printf("getAltitude temperature: %lf \n", temperature);// __AUTO_GENERATED_PRINT_VAR__
	// pressure: Pa
//	pressure = pressure * 0.01;
	// pressure: millibars?
	// temperature: degrees celcius
	return ((pow((1013.25 / pressure), 1 / 5.257) - 1.0) * (temperature + 273.15))/ 0.0065;
	//*altitude = ((pow((1013.25 / P), 1 / 5.257) - 1.0) * (T + 273.15)) / 0.0065;
}

