#ifndef FMS_H_
#define FMS_H_

#include <stdint.h>
#include <stdio.h>

#define HEADER_BYTES 6
#define TAIL_BYTES	 2

typedef enum{
	 FMS_RECEIVE_STATE_IDLE,
	 FMS_RECEIVE_STATE_PREAMBLE,
	 FMS_RECEIVE_STATE_LENGTH_H,
	 FMS_RECEIVE_STATE_LENGTH_L,
	 FMS_RECEIVE_STATE_SEQUENCE_H,
	 FMS_RECEIVE_STATE_SEQUENCE_L,
	 FMS_RECEIVE_STATE_CMD_H,
	 FMS_RECEIVE_STATE_CMD_L,
	 FMS_RECEIVE_STATE_DATA,
	 FMS_RECEIVE_STATE_CRC_H,
	 FMS_RECEIVE_STATE_CRC_L,
	 FMS_RECEIVE_STATE_TAIL_H,
	 FMS_RECEIVE_STATE_TAIL_L
}fms_state_t;

struct data_frame_struct_t{
	  uint16_t DataLength;
		uint16_t sequence;
	  uint16_t command;
	  uint16_t index;
	  uint16_t DataCRC;
	  uint16_t DataCRCActual;
	  uint8_t UserData[255];
};

void DataFrame_Handle(void);


#endif
