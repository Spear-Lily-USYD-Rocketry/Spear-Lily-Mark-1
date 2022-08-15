#ifndef STATE_H
#define STATE_H

#include "fsl_debug_console.h"

#include "boolean.h"
#include "peripheral_intf.h"
#include "avg.h"
#include <math.h>

enum _State {IDLE, INERT, FLIGHT, ARMED, LIFT_OFF, COASTING, APOGEE, DROGUE_DEPLOYMENT, MAIN_DEPLOYMENT, DESCENT, TOUCHDOWN};


typedef struct {
	double last_altitude;
} Flight;

typedef struct {
	int count;
	double acc;
	uint8_t baro_count;
	double altitude;
} LiftOff;

typedef struct{
	uint32_t drogue_apogee_wait_count;
	uint32_t pyro_main_loop_count;
	uint32_t pyro_drogue_loop_count;
	bool drogue_fire_complete;
	bool main_fire_complete;
} Deployment;

typedef LiftOff Coasting;

typedef struct{
	enum _State state;
	Flight flight;
	LiftOff lift_off;
	Coasting coasting;
	Deployment deployment;
} State;

bool isStateChange(sensor_data_t *g_sensor_data_handle, sensor_data_t* avg_g_sensor_data_handle, State *state);
bool changeState(sensor_data_t* g_sensor_data_handle, State *state);
bool performAction(sensor_data_t* g_sensor_data_handle, State *state);
bool initState(State* state);
double getAltitude(double pressure, double temperature);

#endif
