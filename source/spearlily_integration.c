/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    spearlily_integration.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "fsl_rtc.h"
/* TODO: insert other include files here. */
#include "fsl_pit.h"
#include "peripheral_intf.h"
#include "flash.h"
#include "xbee.h"
#include "baro.h"
#include "imu.h"
#include "pyro.h"
#include "state.h"

/* TODO: insert other definitions and declarations here. */
#define	SCHEDULAR_PIT_BASEADDR 	PIT
#define SCHEDULAR_PIT_CHANNEL 	kPIT_Chnl_0
#define PIT_HANDLER				PIT0_IRQHandler
#define PIT_IRQ_ID				PIT0_IRQn

#define PIT_SOURCE_CLOCK 		CLOCK_GetFreq(kCLOCK_BusClk)
#define TOTAL_TASKS 			4 //update this value as required

//global data variables
calibration_data_t *g_calibration_data;
sensor_data_t *g_sensor_data_handle;
sensor_data_t avg_g_sensor_data_handle; // average of last x data points
Avg sliding_avg; //sliding window data structure

flash_config_t *s_flashDriver;
uint8_t frame_counter=1; //telemetry frame counter

//flash configuration
int task_index = 0;
uint32_t data_write_base = 0x100000;
uint32_t meta_data_base = 0xFF000;
uint64_t data_counter = 0;
bool logging_enabled = false;

//pyro
uint32_t pyro_main_loop_count = 0;
uint32_t pyro_drogue_loop_count = 0;
bool fire_drogue = false;
bool fire_main = false;
State state;

void (*task_ptr[TOTAL_TASKS]) ();
uint64_t TASK_PERIOD = 10000U; //micro seconds

/**
 * Handler for time interrupts which drives the round robin schedular
 */
void PIT_HANDLER(void)
{

	#ifdef DEBUG_PRINT_ENABLED
    	PRINTF("\r\nTASK INDEX:%d", task_index);
	#endif
    	//PRINTF("\r\n Interrupt routine %d", PIT_GetCurrentTimerCount(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL));
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(SCHEDULAR_PIT_BASEADDR, SCHEDULAR_PIT_CHANNEL, kPIT_TimerFlag);

//    pitIsrFlag = true;
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();


    if (task_index == TOTAL_TASKS)
    	task_index = 0;

    ((void (*)(void)) task_ptr[task_index++])();
}


void IMU_DR_IRQ_HANDLER(void) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_INITPINS_IMU_DR_GPIO, 1U << BOARD_INITPINS_IMU_DR_PIN);
#else
	/* Clear external interrupt flag. */
	GPIO_PortClearInterruptFlags(BOARD_INITPINS_IMU_DR_GPIO, 1U << BOARD_INITPINS_IMU_DR_PIN);
#endif

	DisableIRQ(IMU_DR_IRQ);

	//registers for SPI debugging
//	int16_t prod_id = IMU_Read_Reg(PROD_ID);
//	int16_t diag_stat = IMU_Read_Reg(DIAG_STAT);


	int16_t x_acc_low = IMU_Read_Reg(X_ACCL_LOW);
	int16_t x_acc_high = IMU_Read_Reg(X_ACCL_OUT);
	int16_t y_acc_low = IMU_Read_Reg(Y_ACCL_LOW);
	int16_t y_acc_high = IMU_Read_Reg(Y_ACCL_OUT);
	int16_t z_acc_low = IMU_Read_Reg(Z_ACCL_LOW);
	int16_t z_acc_high = IMU_Read_Reg(Z_ACCL_OUT);

	int16_t x_gyro_low = IMU_Read_Reg(X_GYRO_LOW);
	int16_t x_gyro_high = IMU_Read_Reg(X_GYRO_OUT);
	int16_t y_gyro_low = IMU_Read_Reg(Y_GYRO_LOW);
	int16_t y_gyro_high = IMU_Read_Reg(Y_GYRO_OUT);
	int16_t z_gyro_low = IMU_Read_Reg(Z_GYRO_LOW);
	int16_t z_gyro_high = IMU_Read_Reg(Z_GYRO_OUT);


	float AXS = IMU_Accel_Scale((x_acc_high<<8) | (x_acc_low & 0xFF));
	float AYS = IMU_Accel_Scale((y_acc_high<<8) | (y_acc_low & 0xFF));
	float AZS = IMU_Accel_Scale((z_acc_high<<8) | (z_acc_low & 0xFF));

	float GXS = IMU_Accel_Scale((x_gyro_high<<8) | (x_gyro_low & 0xFF));
	float GYS = IMU_Accel_Scale((y_gyro_high<<8) | (y_gyro_low & 0xFF));
	float GZS = IMU_Accel_Scale((z_gyro_high<<8) | (z_gyro_low & 0xFF));

	g_sensor_data_handle->accel_x = (double) AXS;
	g_sensor_data_handle->accel_y = (double) AYS;
	g_sensor_data_handle->accel_z = (double) AZS;

	g_sensor_data_handle->gyro_x = (int) GXS; //correct this to double
	g_sensor_data_handle->gyro_y = (int) GYS;
	g_sensor_data_handle->gyro_z = (int) GZS;

	SDK_ISR_EXIT_BARRIER;

	//WDOG reset for IMU

}



void barometer_task()
{

	#ifdef DEBUG_PRINT_ENABLED
		PRINTF("\r\n Barometer Task");
	#endif

	double altitude_test = 0;
	barometer(&g_sensor_data_handle->baro_temperature, &g_sensor_data_handle->baro_pressure, &altitude_test);

	//WDOG reset
}

void imu_task()
{

	#ifdef DEBUG_PRINT_ENABLED
		PRINTF("\r\n IMU Task");
	#endif

	//Enable IRQ
	EnableIRQ(IMU_DR_IRQ);

	//->IMU_DR_IRQ_HANDLER
}

void telemetry_task()
{

	uint8_t frame_data[128] = {0};
	uint16_t packet_length = pack_frame(frame_data, &frame_counter, g_sensor_data_handle);
	transmit_frame(frame_data, packet_length);

	#ifdef DEBUG_PRINT_ENABLED
		PRINTF("\r\n Telemetry Task");
	#endif

	//WDOG reset
}


void pyro_task()
{

	#ifdef STATE_MACHINE_ENABLED

		//state logic
		pushAvg(&sliding_avg, g_sensor_data_handle);

		if (fullAvg(&sliding_avg))
		{
			getAvg(&sliding_avg, &avg_g_sensor_data_handle);

			if(isStateChange(g_sensor_data_handle, &avg_g_sensor_data_handle, &state))
			{
				state.state += 1;
				g_sensor_data_handle->state = (uint8_t) state.state;

			}
			else
			{
				if (state.state == ARMED)
				{
					logging_enabled = true;
				}
				else if (state.state == DROGUE_DEPLOYMENT)
				{
					//fire for 2 seconds
					if (state.deployment.pyro_drogue_loop_count * (10000*TOTAL_TASKS) < 2000000)
					{
						pyro_gpio_write(PYRO_FIRE_0_DROGUE, 1);
						pyro_gpio_write(PYRO_FIRE_2_DROGUE, 1);
						state.deployment.pyro_drogue_loop_count++;
					}
					else
					{
						state.deployment.drogue_fire_complete = true;
						pyro_gpio_write(PYRO_FIRE_0_DROGUE, 0);
						pyro_gpio_write(PYRO_FIRE_2_DROGUE, 0);
					}
					//stop firing

				}
				else if (state.state == MAIN_DEPLOYMENT)
				{
					//fire for 2 seconds
					if (state.deployment.pyro_main_loop_count * (10000*TOTAL_TASKS) < 2000000)
					{
						pyro_gpio_write(PYRO_FIRE_1_MAIN, 1);
						pyro_gpio_write(PYRO_FIRE_3_MAIN, 1);
						state.deployment.pyro_main_loop_count++;

					}
					else
					{
						state.deployment.main_fire_complete = true;
						pyro_gpio_write(PYRO_FIRE_1_MAIN, 0);
						pyro_gpio_write(PYRO_FIRE_3_MAIN, 0);
					}

				}
				else if (state.state == DESCENT)
				{
					//send only telemetry

				}
				else if (state.state == TOUCHDOWN)
				{
					logging_enabled = false;
					//send only telemetry - lower rate

				}



			}

		}

	#else

		if (fire_main == false)
		{
			//5 second startup
			//count * 10*4 = 5000
			if ((int)pyro_main_loop_count * (10000*TOTAL_TASKS) > 500000)
			{
				fire_main = true;
				pyro_main_loop_count = 0;
			}
		}
		else
		{
			//fire for 10 seconds => 2000000
			if ((int)pyro_main_loop_count * (10000*TOTAL_TASKS) > 2000000)
			{
				fire_main = false;
				pyro_main_loop_count = 0;
			}
		}
		pyro_main_loop_count++;

		if (fire_main){
			pyro_gpio_write(PYRO_FIRE_0_DROGUE, 1);
			pyro_gpio_write(PYRO_FIRE_2_DROGUE, 1);

			pyro_gpio_write(PYRO_FIRE_1_MAIN, 1);
			pyro_gpio_write(PYRO_FIRE_3_MAIN, 1);
		}
		else{
			pyro_gpio_write(PYRO_FIRE_0_DROGUE, 0);
			pyro_gpio_write(PYRO_FIRE_2_DROGUE, 0);

			pyro_gpio_write(PYRO_FIRE_1_MAIN, 0);
			pyro_gpio_write(PYRO_FIRE_3_MAIN, 0);
		}


	#endif



//	uint8_t is_pyro_batt_sense = read_batt_pyro_sense();

//	#ifdef DEBUG_PRINT_ENABLED
//		PRINTF("\r\n is_pyro_batt_enable: %d", is_pyro_batt_sense);
//	#endif

	//read pyro sense channels
//	uint8_t sense_ch_0_drogue = pyro_gpio_read(PYRO_SENSE_0_DROGUE);
//	uint8_t sense_ch_1_main = pyro_gpio_read(PYRO_SENSE_1_MAIN);
//	PRINTF("\r\n sense_ch_0: %d		%d", sense_ch_0_drogue, sense_ch_1_main);

//	if (sense_ch_0_drogue == 1) //&& state machine logic

}


void flash_memory_write_task()
{
	#ifdef DEBUG_PRINT_ENABLED
		PRINTF("\r\nFlash Task");
	#endif


	 if (logging_enabled == true)
	 {
		 write_data(s_flashDriver, meta_data_base, data_counter, g_sensor_data_handle);
//		 write_data(s_flashDriver, data_write_base, data_counter, g_sensor_data_handle);
//		 update_data_counter(s_flashDriver, meta_data_base, data_counter);
		 data_counter++;
	 }

}



/*
 * @brief   Application entry point.
 */
int main(void) {

	/* TASK INITIALISATION*/
	task_ptr[0] = barometer_task;
	task_ptr[1] = imu_task;
	task_ptr[2] = pyro_task;
	task_ptr[3] = telemetry_task;
//	task_ptr[4] = flash_memory_write_task;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    g_calibration_data = malloc(sizeof(calibration_data_t));
	g_sensor_data_handle = malloc(sizeof(sensor_data_t));
	s_flashDriver = malloc(sizeof(flash_config_t));

	//state machine initialisation
	initAvg(&sliding_avg);
	initState(&state);
	assert(emptyAvg(&sliding_avg));

	//flash
//	init_flash(s_flashDriver);
//	clear_flash(s_flashDriver, meta_data_base);


	imu_reset();
	barometer_reset();
	expandr_reset();
	pyro_enable();
	expandr_setup();

    //PIT config
    pit_config_t pitConfig;
    PIT_GetDefaultConfig(&pitConfig);
    /* Init pit module */
    PIT_Init(SCHEDULAR_PIT_BASEADDR, &pitConfig);

    /* Set timer period for channel 0 */
    PIT_SetTimerPeriod(SCHEDULAR_PIT_BASEADDR, SCHEDULAR_PIT_CHANNEL, USEC_TO_COUNT(TASK_PERIOD, PIT_SOURCE_CLOCK));

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(SCHEDULAR_PIT_BASEADDR, SCHEDULAR_PIT_CHANNEL, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(PIT_IRQ_ID);

    PIT_StartTimer(SCHEDULAR_PIT_BASEADDR, SCHEDULAR_PIT_CHANNEL);


    while(1){}
}
