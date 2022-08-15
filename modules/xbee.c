#include <stdint.h>
#include <string.h>
#include "peripheral_intf.h"
#include "fsl_uart.h"
#include "peripherals.h"

//TELEM-TX	26	PTA14	UART	UART0
//TELEM-RX	27	PTA15	UART	UART0

/*******************************************************************************
 * Telemetry functions
 ******************************************************************************/
/**
 * Temporary implementation of frame id increment
 * replace prev_frame_id proper data structure/pointer
 */
uint8_t xbee_next_frame_id( uint8_t *prev_frame_id)
{

   // frame_id ranges from 1 to 255; if incremented to 0, wrap to 1
   if (++(*prev_frame_id))
   {
      return 1;
   }

   return *prev_frame_id;
}

/**
 * Checksum calculation function
 */
uint8_t checksum(uint8_t *bytes, uint16_t length)
{
   uint16_t i;
   uint8_t checksum;
   uint8_t *p;

   checksum = 0xFF;
   for (p = (uint8_t *)bytes, i = length; i; ++p, --i)
   {
      checksum -= *p;
   }

   return checksum;
}


/*
 * Prepare frame for transmission
 * return - total frame length in bytes
 */
uint16_t pack_frame(uint8_t *frame, uint8_t *frame_counter, sensor_data_t *sensor_data)
{
	uint16_t content_length = 0;
	uint8_t checksum_part[100] = {0};

	//start bit
	uint8_t start = 0x7E;

	//frame type
	uint8_t frame_type = 0x10; //Transmit Request - 0x10
	memcpy(checksum_part, &frame_type, 1);
	content_length++;

	//frame id
	uint8_t frame_id = xbee_next_frame_id(frame_counter);
	memcpy(checksum_part+content_length, &frame_id, 1);
	content_length++;

	//broadcast address (destination MAC)
	uint8_t destination[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};
	memcpy(checksum_part+content_length, &destination, 8);
	content_length+=8;

	//reserved 16-bit - 0xFFFE
	uint8_t reserved[] = {0xFF, 0xFE};
	memcpy(checksum_part+content_length, &reserved, 2);
	content_length+=2;

	//radius 0x00 max hops
	uint8_t radius = {0x00};
	memcpy(checksum_part+content_length, &radius, 1);
	content_length++;

	//options byte
	uint8_t options = 0x00;
	memcpy(checksum_part+content_length, &options, 1);
	content_length++;

	// ** Sensor Data ** //

	//accel x,y,z
//	double accel_x = double_rand_range(0, 255);
	memcpy(checksum_part+content_length, &sensor_data->accel_x, sizeof(double));
	content_length+=sizeof(double);

//	double accel_y = double_rand_range(0, 255);
	memcpy(checksum_part+content_length, &sensor_data->accel_y, sizeof(double));
	content_length+=sizeof(double);

//	double accel_z = double_rand_range(0, 255);
	memcpy(checksum_part+content_length, &sensor_data->accel_z, sizeof(double));
	content_length+=sizeof(double);

	//baro
//	double baro_pressure = double_rand_range(0, 255);
	memcpy(checksum_part+content_length, &sensor_data->baro_pressure, sizeof(double));
	content_length+=sizeof(double);

//	double baro_temperature = double_rand_range(0, 255);
	memcpy(checksum_part+content_length, &sensor_data->baro_temperature, sizeof(double));
	content_length+=sizeof(double);

	//gyro x,y,z
//	int gyro_x = int_rand_range(0, 15);
	memcpy(checksum_part+content_length, &sensor_data->gyro_x, sizeof(int));
	content_length+=sizeof(int);

//	int gyro_y = int_rand_range(0, 15);
	memcpy(checksum_part+content_length, &sensor_data->gyro_y, sizeof(int));
	content_length+=sizeof(int);

//	int gyro_z = int_rand_range(0, 15);
	memcpy(checksum_part+content_length, &sensor_data->gyro_z, sizeof(int));
	content_length+=sizeof(int);

//	//mag x,y,z
////	double mag_x = double_rand_range(0, 255);
//	memcpy(checksum_part+content_length, &sensor_data->mag_x, sizeof(double));
//	content_length+=sizeof(double);
//
////	double mag_y = double_rand_range(0, 255);
//	memcpy(checksum_part+content_length, &sensor_data->mag_y, sizeof(double));
//	content_length+=sizeof(double);
//
////	double mag_z = double_rand_range(0, 255);
//	memcpy(checksum_part+content_length, &sensor_data->mag_z, sizeof(double));
//	content_length+=sizeof(double);

	memcpy(checksum_part+content_length, &sensor_data->state, sizeof(uint8_t));
	content_length+=sizeof(uint8_t);

	/*
	 * 	sizeof(double) = 8 bytes
	 * 	sizeof(int) = 4 bytes
	 */


	uint8_t checksum_byte = checksum(checksum_part, content_length);

	//length field (number of bytes between length and checksum field, excluding)
	uint8_t length_low  = content_length & 0xff;
	uint8_t length_high = content_length >> 8;

	//frame pack
//	*frame = malloc((content_length+4) * sizeof(uint8_t));
	memcpy(frame, &start, 1);
	memcpy((uint8_t *)frame+1, &length_high, 1);
	memcpy((uint8_t *)frame+2, &length_low, 1);
	memcpy((uint8_t *)frame+3, &checksum_part, content_length);
	memcpy((uint8_t *)frame+3+content_length, &checksum_byte, 1);

	return content_length+4;

}

void transmit_frame(uint8_t *frame_data, uint16_t packet_length)
{
	int field_pos=0;

	for (field_pos = 0; field_pos < packet_length; field_pos++)
	{
		UART_WriteBlocking(UART0_PERIPHERAL, (frame_data+field_pos), 1);

	}

}

