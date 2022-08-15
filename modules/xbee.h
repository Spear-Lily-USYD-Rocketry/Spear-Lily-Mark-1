
#ifndef XBEE_H_
#define XBEE_H_

uint8_t xbee_next_frame_id( uint8_t *prev_frame_id);
uint8_t checksum(uint8_t *bytes, uint16_t length);
uint16_t pack_frame(uint8_t *frame, uint8_t *frame_counter, sensor_data_t *sensor_data);
void transmit_frame(uint8_t *frame_data, uint16_t packet_length);

#endif /* XBEE_H_ */
