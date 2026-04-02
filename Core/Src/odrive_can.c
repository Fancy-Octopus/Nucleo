/*
 * odrive_can.c
 *
 *  Created on: Feb 9, 2025
 *      Author: strongp
 */

#include "main.h"
#include "string.h"
#include "odrive_can.h"
#include "lunautils.h"
#include "luna_wait.h"
#include "rover_controller.h"

/* (Added By Garrett G) Define for easy speed setting */
#define BASE_SPEED    12.5
#define DEPOSIT_SPEED   17.0

//CONFIG MACROS (Comment out to enable/disable)
//#define ODRV_PRINT_DEBUG

//MACROS
#define ODRV_FILTER_MASK 			0x7FF
#define TX_FIFO_MAX_MESSAGES 		3
#define MESSAGE_BUFFER_SIZE 		64
#define ODRV_CONN_TIMEOUT 			2000
#define MESSAGE_FLUSH_TIME 			30
#define REQ_INFO_TIME 				500

#define CMD_MASK					0x1F
#define CMD_HEARTBEAT 				0x01
#define CMD_ESTOP 					0x02
#define CMD_MOTOR_ERROR 			0x03
#define CMD_ENCODER_ERROR 			0x04
#define CMD_SENSORLESS_ERROR 		0x05
#define CMD_SET_NODE_ID 			0x06
#define CMD_AXIS_REQUESTED_STATE 	0x07
#define CMD_ENCODER_ESTIMATES 		0x09
#define CMD_GET_ENCODER_COUNT 		0x0A
#define CMD_SET_CONTROLLER_MODES	0x0B
#define CMD_SET_INPUT_POS 			0x0C
#define CMD_SET_INPUT_VEL 			0x0D
#define CMD_SET_INPUT_TORQUE 		0x0E
#define CMD_SET_LIMITS 				0x0F
#define CMD_START_ANTICOGGING		0x10
#define CMD_SET_TRAJ_VEL_LIMIT 		0x11
#define CMD_REBOOT					0x16
#define CMD_GET_POWER_INFO 			0x17
#define CMD_CLEAR_ERRORS			0x18
#define CMD_CONTROLLER_ERROR		0x1D

#define INPUT_MODE_VEL_RAMP 		0x02

#define CONTROL_MODE_VEL_CONTROL 	0x02
#define CONTROL_MODE_IDLE			0x01

#define AXIS_STATE_CL_CONTROL 		0x08
#define AXIS_STATE_IDLE				0x01

#define ODRV0_ID_AXIS_0 2
#define ODRV0_ID_AXIS_1 3

#define ODRV_ID_MASK			0x7E0
#define ODRV_ID_CREATE_ID(x,y) 		((x<<5)|y)
#define ODRV_ID_GET_NODE(x) 		((x&ODRV_ID_MASK)>>5)
#define ODRV_ID_GET_CMD(x) 		(x&CMD_MASK)

#ifdef ODRV_PRINT_DEBUG
#define ODRIVE_PRINTF printf
#else
#define ODRIVE_PRINTF(fmt, ...) ;
#endif

//TYPE DECLARATIONS

//	Axis Error 0 32bit
//	Axis Current State 4 8bit
//	Motor Error Flag 5
//	Encoder Error Flag 6
//	Controller Error Flag 7
//	Trajectory Done Flag 7.7
typedef struct{
  int axisId;
  float velEstimate;
  float posEstimate;
  uint32_t axisError;
  uint8_t axisCurrState;
  uint32_t motorError[2];
  uint32_t encoderError;
  uint8_t controllerError;
  uint8_t trajDone;
  schedule_t connStatus;
} odrive_axis_t;

typedef struct{
  odrive_axis_t axis0;
  odrive_axis_t axis1;
} odrive_t;

typedef struct{
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t data[32];
	uint8_t waiting;
} odrive_message_t;

//FUNCTION DECLARATIONS
int OdriveInit(FDCAN_HandleTypeDef* caninst);

//ODRIVE FUNCTIONS
int SetController(odrive_t* odrv,uint8_t axis);
int ResetController(odrive_t* odrv,uint8_t axis);
int SetSpeed(odrive_t* odrv,uint8_t axis,float vel);
int SetCLControl(odrive_t* odrv,uint8_t axis, uint8_t set2cl);
int ReceiveOdriveInfo(void);
int RequestMotorError(odrive_t* odrv,uint8_t axis);
int RequestControllerError(odrive_t* odrv,uint8_t axis);
int RequestEncoderError(odrive_t* odrv,uint8_t axis);
int ClearErrors(odrive_t* odrv,uint8_t axis);
int RebootOdrv(odrive_t* odrv);

//HARDWARE ABSTRACTION
odrive_axis_t* GetAxisStruct(int axisId);
uint8_t ReceiveRxMessage(uint8_t* data,uint8_t dataLen, uint32_t* id);
uint8_t SendTxMessage(uint32_t idField, uint8_t* data, uint8_t datalen, uint8_t remote);
void PrintCanMsg(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData, int rxDatalen);
void CheckCANHealth(void);
int FlushMessageBuffer(void);
void MessageBufferAdd(odrive_message_t* message);
odrive_axis_t* GetAxisStruct(int axisId);

//GLOBAL VARIABLES

//Pointer to the CAN handle used for CUBE HAL
FDCAN_HandleTypeDef* odrvCanHandle = NULL;

odrive_t odrvbl={0}, odrvbr={0}, odrvfl={0}, odrvfr={0}; //IDs assigned at runtime in OdriveInit

odrive_t* odrvs[] = {&odrvbl, &odrvbr, &odrvfl, &odrvfr, NULL};

odrive_message_t messageBuffer[MESSAGE_BUFFER_SIZE] = {0};

static schedule_t flushSchedule;

static schedule_t infoSchedule;

static int busoff = 0;

//FUNCTION DEFINITIONS
int OdriveInit(FDCAN_HandleTypeDef* caninst){
	FDCAN_FilterTypeDef sFilterConfig;
	odrvCanHandle = caninst;

	sFilterConfig.IdType       = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex  = 0U;
	sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1    = 0; //For Now ACCEPT EVERYTHING. We will check the ID in receive fx
	sFilterConfig.FilterID2    = 0;

	if (HAL_FDCAN_ConfigFilter(odrvCanHandle, &sFilterConfig) != HAL_OK){
		return 1;
	}

	//Configures the behavior of filters when the can ID doesn't match. We will reject everything.
	if (HAL_FDCAN_ConfigGlobalFilter(odrvCanHandle,FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
		return 1;
	}

	if (HAL_FDCAN_Start(odrvCanHandle) != HAL_OK){
		return 1;
	}

	/*Assign IDs
	 * O-Drive 1 (back left)
	 * 		Axis 0 -> Back left wheel
	 * 		Axis 1 -> Back bucket
	 * O-Drive 2 (back right)
	 * 		Axis 0 -> Back right wheel
	 * 		Axis 1 -> Middle right wheel
	 * O-Drive 3 (front left)
	 * 		Axis 0 -> Front left wheel
	 * 		Axis 1 -> Middle left wheel
	 * O-Drive 4 (front right)
	 * 		Axis 0 -> Front right wheel
	 * 		Axis 1 -> Front bucket
	 */
	odrvfl.axis0.axisId = 1;
	odrvfl.axis1.axisId = 2;
	odrvfr.axis0.axisId = 3;
	odrvfr.axis1.axisId = 4;
	odrvbl.axis0.axisId = 5;
	odrvbl.axis1.axisId = 6;
	odrvbr.axis0.axisId = 7;
	odrvbr.axis1.axisId = 8;
	SetScheduledTime(&odrvfl.axis0.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvfl.axis1.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvfr.axis0.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvfr.axis1.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvbl.axis0.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvbl.axis1.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvbr.axis0.connStatus,ODRV_CONN_TIMEOUT);
	SetScheduledTime(&odrvbr.axis1.connStatus,ODRV_CONN_TIMEOUT);

	SetScheduledTime(&flushSchedule,MESSAGE_FLUSH_TIME);
	SetScheduledTime(&infoSchedule,REQ_INFO_TIME);

	return 0;
}

int OdrivePoll(void){
	static rover_state_t state = ROVER_IDLE;

	CheckCANHealth();

	rover_state_t nextState = CurrentRoverState();

	if(busoff){
		if(!READ_BIT(odrvCanHandle->Instance->CCCR,FDCAN_CCCR_INIT)) busoff = 0;
		return 0;
	}

	ReceiveOdriveInfo();
	FlushMessageBuffer();

	//Next State Logic
	if(state != nextState){
		switch(nextState){
			case ROVER_READY:
				SetController(&odrvfr,0);
				SetController(&odrvfr,1);
				SetController(&odrvfl,0);
				SetController(&odrvfl,1);
				SetController(&odrvbr,0);
				SetController(&odrvbr,1);
				SetController(&odrvbl,0);
				SetController(&odrvbl,1);
			case ROVER_IDLE:
				SetCLControl(&odrvfr,0,0);
				SetCLControl(&odrvfr,1,0);
				SetCLControl(&odrvbr,1,0);
				SetCLControl(&odrvfl,1,0);
				SetCLControl(&odrvbl,0,0);
				SetCLControl(&odrvbl,1,0);


				SetCLControl(&odrvbr,0,0);
				SetCLControl(&odrvfl,0,0);
			break;
			case ROVER_FORWARD:
				SetSpeed(&odrvfr,0,BASE_SPEED);
				SetSpeed(&odrvfr,1,BASE_SPEED);
				SetSpeed(&odrvbr,1,BASE_SPEED);
				SetSpeed(&odrvfl,1,-BASE_SPEED);
				SetSpeed(&odrvbl,0,-BASE_SPEED);
				SetSpeed(&odrvbl,1,-BASE_SPEED);
				SetCLControl(&odrvfr,0,1);
				SetCLControl(&odrvfr,1,1);
				SetCLControl(&odrvbr,1,1);
				SetCLControl(&odrvfl,1,1);
				SetCLControl(&odrvbl,0,1);
				SetCLControl(&odrvbl,1,1);
			break;
			case ROVER_BACKWARD:
				SetSpeed(&odrvfr,0,-BASE_SPEED);
				SetSpeed(&odrvfr,1,-BASE_SPEED);
				SetSpeed(&odrvbr,1,-BASE_SPEED);
				SetSpeed(&odrvfl,1,BASE_SPEED);
				SetSpeed(&odrvbl,0,BASE_SPEED);
				SetSpeed(&odrvbl,1,BASE_SPEED);
				SetCLControl(&odrvfr,0,1);
				SetCLControl(&odrvfr,1,1);
				SetCLControl(&odrvbr,1,1);
				SetCLControl(&odrvfl,1,1);
				SetCLControl(&odrvbl,0,1);
				SetCLControl(&odrvbl,1,1);
			break;
			case ROVER_TURN_RIGHT:
				SetSpeed(&odrvfr,0,-BASE_SPEED);
				SetSpeed(&odrvfr,1,-BASE_SPEED);
				SetSpeed(&odrvbr,1,-BASE_SPEED);
				SetSpeed(&odrvfl,1,-BASE_SPEED);
				SetSpeed(&odrvbl,0,-BASE_SPEED);
				SetSpeed(&odrvbl,1,-BASE_SPEED);
				SetCLControl(&odrvfr,0,1);
				SetCLControl(&odrvfr,1,1);
				SetCLControl(&odrvbr,1,1);
				SetCLControl(&odrvfl,1,1);
				SetCLControl(&odrvbl,0,1);
				SetCLControl(&odrvbl,1,1);

			break;
			case ROVER_TURN_LEFT:
				SetSpeed(&odrvfr,0,BASE_SPEED);
				SetSpeed(&odrvfr,1,BASE_SPEED);
				SetSpeed(&odrvbr,1,BASE_SPEED);
				SetSpeed(&odrvfl,1,BASE_SPEED);
				SetSpeed(&odrvbl,0,BASE_SPEED);
				SetSpeed(&odrvbl,1,BASE_SPEED);
				SetCLControl(&odrvfr,0,1);
				SetCLControl(&odrvfr,1,1);
				SetCLControl(&odrvbr,1,1);
				SetCLControl(&odrvfl,1,1);
				SetCLControl(&odrvbl,0,1);
				SetCLControl(&odrvbl,1,1);

			break;
			case ROVER_DIG_FORWARD:
			case ROVER_DIG_BACKWARD:
				SetSpeed(&odrvbr,0,DEPOSIT_SPEED);
				SetCLControl(&odrvbr,0,1);
				SetSpeed(&odrvfl,0,DEPOSIT_SPEED);
				SetCLControl(&odrvfl,0,1);
			break;
			case ROVER_DEPOSIT_FORWARD:
				SetSpeed(&odrvfl,0,-DEPOSIT_SPEED);
				SetCLControl(&odrvfl,0,1);
			break;
			case ROVER_DEPOSIT_BACKWARD:
				SetSpeed(&odrvbr,0,-DEPOSIT_SPEED);
				SetCLControl(&odrvbr,0,1);
			break;
			case ROVER_ESTOP:
			for(int i=0;i<8;i++){
				if(odrvs[i]==NULL) break;
				RebootOdrv(odrvs[i]);
			}
			break;

			default:
			break;
		}
		state = nextState;
	}

	//State Logic
	static uint8_t swap = 0;
	switch(state){
		case ROVER_READY:
		case ROVER_IDLE:
			if(ScheduleReady(infoSchedule)){
				ResetSchedule(&infoSchedule);
				swap++;
				switch(swap%3){
					case 0:
						RequestEncoderError(&odrvfr,0);
						RequestEncoderError(&odrvfr,1);
						RequestEncoderError(&odrvbr,0);
						RequestEncoderError(&odrvbr,1);
						RequestEncoderError(&odrvfl,0);
						RequestEncoderError(&odrvfl,1);
						RequestEncoderError(&odrvbl,0);
						RequestEncoderError(&odrvbl,1);
					break;
					case 1:
						RequestControllerError(&odrvfr,0);
						RequestControllerError(&odrvfr,1);
						RequestControllerError(&odrvbr,0);
						RequestControllerError(&odrvbr,1);
						RequestControllerError(&odrvfl,0);
						RequestControllerError(&odrvfl,1);
						RequestControllerError(&odrvbl,0);
						RequestControllerError(&odrvbl,1);
					break;
					case 2:
						RequestMotorError(&odrvfr,0);
						RequestMotorError(&odrvfr,1);
						RequestMotorError(&odrvbr,0);
						RequestMotorError(&odrvbr,1);
						RequestMotorError(&odrvfl,0);
						RequestMotorError(&odrvfl,1);
						RequestMotorError(&odrvbl,0);
						RequestMotorError(&odrvbl,1);
					break;
					default:
				}
			}
		break;
		case ROVER_FORWARD:

		break;
		case ROVER_BACKWARD:

		break;
		case ROVER_TURN_RIGHT:

		break;
		case ROVER_TURN_LEFT:

		break;
		default:

		break;
	}

	return 0;
}




int SetController(odrive_t* odrv,uint8_t axis){
  uint8_t txData[8] = {};
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_SET_CONTROLLER_MODES);
  int message_tx[2] = {0};

  //txHeader.Identifier = ((2 << 5) | (0x00B));

  message_tx[0] = CONTROL_MODE_VEL_CONTROL;
  message_tx[1] = INPUT_MODE_VEL_RAMP;

  // Make bits into little endian format
  U32ToU8x4(message_tx[0], &txData[0]);
  U32ToU8x4(message_tx[1], &txData[4]);

  return SendTxMessage(id, txData, 8, false);
}




int ResetController(odrive_t* odrv,uint8_t axis){
  uint8_t txData[8] = {};
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_SET_CONTROLLER_MODES);
  int message_tx[2] = {0};

  //txHeader.Identifier = ((2 << 5) | (0x00B));

  message_tx[0] = CONTROL_MODE_IDLE;
  message_tx[1] = INPUT_MODE_VEL_RAMP;

  // Make bits into little endian format
  U32ToU8x4(message_tx[0], &txData[0]);
  U32ToU8x4(message_tx[1], &txData[4]);

  return SendTxMessage(id, txData, 8, false);
}





int SetCLControl(odrive_t* odrv,uint8_t axis, uint8_t set2cl){
  uint8_t txData[4] = {};
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_AXIS_REQUESTED_STATE);
  uint32_t message_tx;

  //txHeader.Identifier = ((2 << 5) | (0x00B));

  if(set2cl!=0){
	  message_tx = AXIS_STATE_CL_CONTROL;
  } else {
	  message_tx = AXIS_STATE_IDLE;
  }
  // Make bits into little endian format
  U32ToU8x4(message_tx, &txData[0]);
  return SendTxMessage(id, txData, 4, false);
}





int SetSpeed(odrive_t* odrv,uint8_t axis,float vel){
  uint8_t txData[8] = {};
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_SET_INPUT_VEL);
  float message_tx[2] = {0};

  message_tx[0] = vel; //velocity setting
  message_tx[1] = 0; //torque ff

  // Make bits into little endian format
  FloatToU8x4(message_tx[0], &txData[0]);
  FloatToU8x4(message_tx[1], &txData[4]);

  return SendTxMessage(id, txData, 8, false);
}




int RequestMotorError(odrive_t* odrv,uint8_t axis){
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_MOTOR_ERROR);
  uint8_t message_tx[4] = {0};
  return SendTxMessage(id, message_tx, 4, true);
}





int RequestControllerError(odrive_t* odrv,uint8_t axis){
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_CONTROLLER_ERROR);
  uint8_t message_tx[4] = {0};
  return SendTxMessage(id, message_tx, 4, true);
}




int RequestEncoderError(odrive_t* odrv,uint8_t axis){
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_ENCODER_ERROR);
  uint8_t message_tx[4] = {0};
  return SendTxMessage(id, message_tx, 4, true);
}




int ClearErrors(odrive_t* odrv,uint8_t axis){
  uint32_t nodeId = ((axis>0) ? odrv->axis1.axisId : odrv->axis0.axisId);
  uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_CLEAR_ERRORS);
  uint8_t message_tx[1] = {0};
  return SendTxMessage(id, message_tx, 0, false);
}




int RebootOdrv(odrive_t* odrv){
	uint32_t nodeId = (odrv->axis0.axisId);
	uint32_t id = ODRV_ID_CREATE_ID(nodeId,CMD_REBOOT);
	uint8_t message_tx[1] = {0};
	return SendTxMessage(id, message_tx, 0, false);
}




int ReceiveOdriveInfo(void){
  odrive_axis_t* axis;
  uint32_t id = 0;;
  uint8_t data[32];

  if(ReceiveRxMessage(data, 32, &id)!=1){
      //no message received
      return 0;
  }

  uint16_t nodeID = ODRV_ID_GET_NODE(id);
  uint16_t cmdID = ODRV_ID_GET_CMD(id);

  axis = GetAxisStruct(nodeID);
  if(axis == NULL){
      // No odrv struct contains this ID
      return 0;
  }

  ResetSchedule(&axis->connStatus);

  switch (cmdID) {
    case CMD_HEARTBEAT:
		ODRIVE_PRINTF("Heartbeat Received\r\n");
//		axis->axisError = U8x4ToU32(data);
//		axis->axisCurrState = *(data+4);
//		axis->motorError[0] = *(data+5);
//		axis->encoderError = *(data+6);
//		axis->controllerError = *(data+7)|0x01;
//		axis->trajDone = *(data+7)|0x80;
    break;

    case CMD_ENCODER_ESTIMATES:
		ODRIVE_PRINTF("\nGet ENCODER ESTIMATES Received\r\n");
		axis->posEstimate = U8x4ToFloat(data);
		axis->velEstimate = U8x4ToFloat(data+4);
		ODRIVE_PRINTF("Position: %f rev\r\n", (unsigned int)axis->posEstimate);
		ODRIVE_PRINTF("Velocity: %f revs/s\r\n", (unsigned int)axis->velEstimate);
    break;
    case CMD_MOTOR_ERROR:
		ODRIVE_PRINTF("\nMotor Error Status Received\r\n");
		axis->motorError[0] = U8x4ToU32(data);
		axis->motorError[1] = U8x4ToU32(data+4);
		ODRIVE_PRINTF("Error: %u \r\n", (unsigned int)axis->motorError);
    break;
    case CMD_ENCODER_ERROR:
		ODRIVE_PRINTF("\nEncoder Error Status Received\r\n");
		axis->encoderError = U8x4ToU32(data);
		ODRIVE_PRINTF("Error: %u \r\n", (unsigned int)axis->encoderError);
    break;
    case CMD_CONTROLLER_ERROR:
		ODRIVE_PRINTF("\nController Error Status Received\r\n");
		axis->controllerError = U8x4ToU32(data);
		ODRIVE_PRINTF("Error: %u \r\n", (unsigned int)axis->encoderError);
    break;

  }

  ODRIVE_PRINTF("nodeID: %X\n\r", nodeID);
  ODRIVE_PRINTF("cmdID: %X\n\r", cmdID);

  return 0;
}





odrive_axis_t* GetAxisStruct(int axisId){
  int i = 0;
  while(odrvs[i]!=NULL){
    if(odrvs[i]->axis0.axisId==axisId){
      return &(odrvs[i]->axis0);
    }
    if(odrvs[i]->axis1.axisId==axisId){
      return &(odrvs[i]->axis1);
    }
    i++;
  }
  return NULL;
}





uint8_t SendTxMessage(uint32_t idField, uint8_t* data, uint8_t datalen, uint8_t remote){
    FDCAN_TxHeaderTypeDef txHeader;

    if(odrvCanHandle==NULL){
	ODRIVE_PRINTF("CAN Handle NULL, O-Drive comms not initialized!");
	return 1;
    }

    txHeader.Identifier          = idField;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = (remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME);
    txHeader.DataLength          = datalen;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_STORE_TX_EVENTS;
    //txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0x52U;

    //TESTING MESSAGE BUFFER
    odrive_message_t message = {0};
    message.txHeader = txHeader;
    memcpy(&message.data,data,datalen);
    MessageBufferAdd(&message);
    return 1;
}




void MessageBufferAdd(odrive_message_t* message){
	for(int i=0;i<(MESSAGE_BUFFER_SIZE-1);i++){
		if(messageBuffer[i].waiting == 0){
			messageBuffer[i] = *message;
			messageBuffer[i].waiting = 0xAA;
			return;
		}
	}
}





int FlushMessageBuffer(void){
    HAL_StatusTypeDef err = HAL_OK;
	int bufIndex = 0;

	if(!ScheduleReady(flushSchedule)) return 0;
	ResetSchedule(&flushSchedule);

	for(bufIndex=0;bufIndex<(MESSAGE_BUFFER_SIZE-1);bufIndex++){
		if(messageBuffer[bufIndex].waiting != 0) break;
	}

	if(messageBuffer[bufIndex].waiting == 0) return 0; //return if no waiting message was found in the message buffer

	if(HAL_FDCAN_GetTxFifoFreeLevel(odrvCanHandle)<=0){
		return 0; //Message is not able to be sent, fifo full
	}
	else{
		//Add message to TX FIFO
		err = HAL_FDCAN_AddMessageToTxFifoQ(odrvCanHandle, &(messageBuffer[bufIndex].txHeader), messageBuffer[bufIndex].data);
		if (err != HAL_OK){
			return 0;
		}
		else{
			messageBuffer[bufIndex].waiting = 0; //message sent, no longer waiting
			return 1;
		}
	}
}





/**
  * @brief  Return Rx FIFO fill level.
  * @param  Data buffer address to load received data into
  * @param  Length of data that can be received into data buffer
  * @param  unsigned 32 bit integer pointer to store message ID
  * @retval 0 if no message, 1 if message received
  */
uint8_t ReceiveRxMessage(uint8_t* data,uint8_t dataLen, uint32_t* id){
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t recievedData[32] = {0};
  uint8_t ret = 0;

  if(HAL_FDCAN_GetRxFifoFillLevel(odrvCanHandle, FDCAN_RX_FIFO0)>0){
      if (HAL_FDCAN_GetRxMessage(odrvCanHandle, FDCAN_RX_FIFO0, &rxHeader, recievedData) != HAL_OK){
      	  //Rx error, need to add stuff here later
      }
      if(rxHeader.DataLength > dataLen){
	  //this would happen if you pass a buffer to small to store the data
      }
      else{
	  *id = rxHeader.Identifier;
	  memcpy(data,recievedData,dataLen);
	  //PrintCanMsg(&rxHeader, data, rxHeader.DataLength);
	  ret = 1;
      }
  }
  return ret;
}





void PrintCanMsg(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData, int rxDatalen){
  ODRIVE_PRINTF("Identifier -> %x\r\n",(unsigned int)rxHeader->Identifier);
  ODRIVE_PRINTF("IdType -> %x\r\n",(unsigned int)rxHeader->IdType);
  ODRIVE_PRINTF("DataLength -> %x\r\n",(unsigned int)rxHeader->DataLength);
#ifdef DETAILED_CAN_MSG_PRINT
  ODRIVE_PRINTF("ErrorStateIndicator -> %x\r\n",(unsigned int)rxHeader->ErrorStateIndicator);
  ODRIVE_PRINTF("FDFormat -> %x\r\n",(unsigned int)rxHeader->FDFormat);
  ODRIVE_PRINTF("IsFilterMatchingFrame -> %x\r\n",(unsigned int)rxHeader->IsFilterMatchingFrame);
  ODRIVE_PRINTF("FilterIndex -> %x\r\n",(unsigned int)rxHeader->FilterIndex);
  ODRIVE_PRINTF("RxFrameType -> %x\r\n",(unsigned int)rxHeader->RxFrameType);
  ODRIVE_PRINTF("RxTimestamp -> %x\r\n",(unsigned int)rxHeader->RxTimestamp);
#endif
  ODRIVE_PRINTF("Data -> [");
  for(int i=0;i<rxDatalen;i++){
      ODRIVE_PRINTF(" %x ", *(rxData+i));
  }
  ODRIVE_PRINTF("]\r\n\r\n");
}





void CheckCANHealth(void){
	FDCAN_ProtocolStatusTypeDef protocolStatus = {};
	FDCAN_ErrorCountersTypeDef errorCounters = {};


	HAL_FDCAN_GetProtocolStatus(odrvCanHandle, &protocolStatus);
	//if(protocolStatus.BusOff) printf("CAN BUS OFF\r\n");
	HAL_FDCAN_GetErrorCounters(odrvCanHandle, &errorCounters);

	if(protocolStatus.BusOff!=0){
		printf("BUSOFF TIME -> %u[ms]\r\n",(unsigned int)HAL_GetTick());
		CLEAR_BIT(odrvCanHandle->Instance->CCCR, FDCAN_CCCR_INIT);
		busoff = 1;
	}
	//if(errorCounters.TxErrorCnt==255) printf("TX ERROR MAX\r\n");
//	if (protocolStatus.BusOff) {
//		CLEAR_BIT(odrvCanHandle->Instance->CCCR, FDCAN_CCCR_INIT);
//	}
}





void PrintODriveConnStatus(void){
	FDCAN_ProtocolStatusTypeDef protocolStatus = {};
	FDCAN_ErrorCountersTypeDef errorCounters = {};
	odrive_axis_t* axis;
	HAL_FDCAN_GetProtocolStatus(odrvCanHandle, &protocolStatus);
	HAL_FDCAN_GetErrorCounters(odrvCanHandle, &errorCounters);
	printf("ODRIVE CONNECTION STATUS\r\n");
	printf("\tTX ERROR COUNT (TEC): %u\r\n",(unsigned int)errorCounters.TxErrorCnt);
	printf("\tRX ERROR COUNT (REC): %u\r\n\n",(unsigned int)errorCounters.RxErrorCnt);
	printf("\tODRV_FL_AXIS_0 -> %s\r\n", ScheduleReady(odrvfl.axis0.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_FL_AXIS_1 -> %s\r\n", ScheduleReady(odrvfl.axis1.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_FR_AXIS_0 -> %s\r\n", ScheduleReady(odrvfr.axis0.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_FR_AXIS_1 -> %s\r\n", ScheduleReady(odrvfr.axis1.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_BL_AXIS_0 -> %s\r\n", ScheduleReady(odrvbl.axis0.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_BL_AXIS_1 -> %s\r\n", ScheduleReady(odrvbl.axis1.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_BR_AXIS_0 -> %s\r\n", ScheduleReady(odrvbr.axis0.connStatus) ? "NOCONN":"GOOD");
	printf("\tODRV_BR_AXIS_1 -> %s\r\n", ScheduleReady(odrvbr.axis1.connStatus) ? "NOCONN":"GOOD");

	printf("ODRIVE ERRORS\r\n");
	axis = GetAxisStruct(2);
	printf("FL Wheel |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(3);
	printf("FR Wheel |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(6);
	printf("ML Wheel |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(4);
	printf("MR Wheel |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(5);
	printf("BL Wheel |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(8);
	printf("BR Wheel |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(1);
	printf("F DRUM |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
	axis = GetAxisStruct(7);
	printf("B DRUM |   M:%x|   A:%x|     C%x|\r\n",axis->motorError[0],axis->axisError,axis->controllerError);
}




//void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
//	FDCAN_ProtocolStatusTypeDef protocolStatus = {};
//	FDCAN_ErrorCountersTypeDef errorCounters = {};
//	if (hfdcan == odrvCanHandle) {
//        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
//			HAL_FDCAN_GetProtocolStatus(odrvCanHandle, &protocolStatus);
//			HAL_FDCAN_GetErrorCounters(odrvCanHandle, &errorCounters);
//			if(protocolStatus.BusOff!=0){
//				//printf("BUSOFF TIME -> %u[ms]\r\n",(unsigned int)HAL_GetTick());
//				CLEAR_BIT(odrvCanHandle->Instance->CCCR, FDCAN_CCCR_INIT);
//				busoff = 1;
//			}
//		}
//    }
//}





