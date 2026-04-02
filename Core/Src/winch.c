/*
 * winch.c
 *
 *  Created on: Mar 15, 2025
 *      Author: strongp
 */

/* NOTES
 * UART4 is the serial port for the roboclaw
 *
 *
 *
 *
 */
#include "main.h"
#include "stdint.h"
#include "rover_controller.h"

/* (Added by Garrett G) Define for quick changing winch speed (0 to 127) */
#define WINCH_SPEED   100

//MACROS
#define ROBOCLAW_ADDRESS 0x80

//TYPES
enum {
	M1FORWARD = 0,
	M1BACKWARD = 1,
	SETMINMB = 2,
	SETMAXMB = 3,
	M2FORWARD = 4,
	M2BACKWARD = 5,
	M17BIT = 6,
	M27BIT = 7,
	MIXEDFORWARD = 8,
	MIXEDBACKWARD = 9,
	MIXEDRIGHT = 10,
	MIXEDLEFT = 11,
	MIXEDFB = 12,
	MIXEDLR = 13,
	GETM1ENC = 16,
	GETM2ENC = 17,
	GETM1SPEED = 18,
	GETM2SPEED = 19,
	RESETENC = 20,
	GETVERSION = 21,
	SETM1ENCCOUNT = 22,
	SETM2ENCCOUNT = 23,
	GETMBATT = 24,
	GETLBATT = 25,
	SETMINLB = 26,
	SETMAXLB = 27,
	SETM1PID = 28,
	SETM2PID = 29,
	GETM1ISPEED = 30,
	GETM2ISPEED = 31,
	M1DUTY = 32,
	M2DUTY = 33,
	MIXEDDUTY = 34,
	M1SPEED = 35,
	M2SPEED = 36,
	MIXEDSPEED = 37,
	M1SPEEDACCEL = 38,
	M2SPEEDACCEL = 39,
	MIXEDSPEEDACCEL = 40,
	M1SPEEDDIST = 41,
	M2SPEEDDIST = 42,
	MIXEDSPEEDDIST = 43,
	M1SPEEDACCELDIST = 44,
	M2SPEEDACCELDIST = 45,
	MIXEDSPEEDACCELDIST = 46,
	GETBUFFERS = 47,
	GETPWMS = 48,
	GETCURRENTS = 49,
	MIXEDSPEED2ACCEL = 50,
	MIXEDSPEED2ACCELDIST = 51,
	M1DUTYACCEL = 52,
	M2DUTYACCEL = 53,
	MIXEDDUTYACCEL = 54,
	READM1PID = 55,
	READM2PID = 56,
	SETMAINVOLTAGES = 57,
	SETLOGICVOLTAGES = 58,
	GETMINMAXMAINVOLTAGES = 59,
	GETMINMAXLOGICVOLTAGES = 60,
	SETM1POSPID = 61,
	SETM2POSPID = 62,
	READM1POSPID = 63,
	READM2POSPID = 64,
	M1SPEEDACCELDECCELPOS = 65,
	M2SPEEDACCELDECCELPOS = 66,
	MIXEDSPEEDACCELDECCELPOS = 67,
	SETM1DEFAULTACCEL = 68,
	SETM2DEFAULTACCEL = 69,
	SETPINFUNCTIONS = 74,
	GETPINFUNCTIONS = 75,
	SETDEADBAND	= 76,
	GETDEADBAND	= 77,
	GETENCODERS = 78,
	GETISPEEDS = 79,
	RESTOREDEFAULTS = 80,
	GETTEMP = 82,
	GETTEMP2 = 83,	//Only valid on some models
	GETERROR = 90,
	GETENCODERMODE = 91,
	SETM1ENCODERMODE = 92,
	SETM2ENCODERMODE = 93,
	WRITENVM = 94,
	READNVM = 95,	//Reloads values from Flash into Ram
	SETCONFIG = 98,
	GETCONFIG = 99,
	SETM1MAXCURRENT = 133,
	SETM2MAXCURRENT = 134,
	GETM1MAXCURRENT = 135,
	GETM2MAXCURRENT = 136,
	SETPWMMODE = 148,
	GETPWMMODE = 149,
	FLAGBOOTLOADER = 255
};

//DECLARATIONS
static void SetMotor1(uint8_t speed, uint8_t dir);
static void SetMotor2(uint8_t speed);
static void RbclwCrcUpdate(uint8_t data,uint16_t* crc);
static void RbclwSendMessage(uint8_t addr,uint8_t command, uint8_t* data, uint8_t datalen);


//GLOBALS
UART_HandleTypeDef winchUart;

//DEFINITIONS
void WinchInit(void){
  GPIO_InitTypeDef gpio_init_structure;
  __HAL_RCC_UART4_CLK_ENABLE();

  /* Configure USART Tx as alternate function */
  gpio_init_structure.Pin       = GPIO_PIN_0;
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &gpio_init_structure);

  /* Configure USART Rx as alternate function */
  gpio_init_structure.Pin       = GPIO_PIN_11;
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Alternate = GPIO_AF6_UART4;
  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  winchUart.Instance                = UART4;
  winchUart.Init.BaudRate           = 115200;
  winchUart.Init.Mode               = UART_MODE_TX_RX;
  winchUart.Init.Parity             = COM_PARITY_NONE;
  winchUart.Init.WordLength         = COM_WORDLENGTH_8B;
  winchUart.Init.StopBits           = COM_STOPBITS_1;
  winchUart.Init.HwFlowCtl          = COM_HWCONTROL_NONE;
  winchUart.Init.OverSampling       = UART_OVERSAMPLING_8;
  winchUart.Init.ClockPrescaler     = UART_PRESCALER_DIV1;
  HAL_UART_Init(&winchUart);

  NVIC_SetPriority(UART_IT_RXNE, 0);
  NVIC_EnableIRQ(UART_IT_RXNE);
}

void WinchPoll(void){
	switch(CurrentRoverState()){
		case ROVER_WINCH_UP:
			SetMotor1(WINCH_SPEED,0);
			break;
		case ROVER_WINCH_DOWN:
			SetMotor1(WINCH_SPEED,1);
			break;
		default:
			SetMotor1(0,0);
			break;
	}
}

static void RbclwCrcUpdate(uint8_t data,uint16_t* crc){
  int i;
  *crc = *crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++){
    if (*crc & 0x8000){
      *crc = (*crc << 1) ^ 0x1021;
    }
    else{
      *crc <<= 1;
    }
  }
}


static void RbclwSendMessage(uint8_t addr,uint8_t command, uint8_t* data, uint8_t datalen){
    uint8_t i = 0;
    uint16_t crc = 0;
    uint8_t txbuffer[32];

    ///Calculate CRC///
    RbclwCrcUpdate(addr,&crc);
    RbclwCrcUpdate(command,&crc);
    for(int x=0;x<datalen;x++){
	RbclwCrcUpdate(*(data+x),&crc);
    }

    ///Construct Message///
    *(txbuffer+i++) = addr;
    *(txbuffer+i++) = command;
    for(int j=0;j<datalen;j++){
	*(txbuffer+i++) = *(data+j);
    }
    *(txbuffer+i++) = (uint8_t)((crc&0xFF00)>>8);
    *(txbuffer+i++) = (uint8_t)((crc&0x00FF)>>0);
//    *(dest+i++) = crc.crcbytes[1];
//    *(dest+i++) = crc.crcbytes[0];

    ///Send over UART///
    HAL_UART_Transmit(&winchUart,txbuffer,i,1);
    return;
}

static void SetMotor1(uint8_t speed,uint8_t dir){ // M1 FORWARD or BACKWARD: (0<x<64 backward, 0=64 stop, 64<x<127 forward)
  uint8_t data = speed;
  if(dir!=0){
	  RbclwSendMessage(ROBOCLAW_ADDRESS,M1FORWARD,&data,sizeof(data));
  } else {
	  RbclwSendMessage(ROBOCLAW_ADDRESS,M1BACKWARD,&data,sizeof(data));
  }
//  if (data < 64) {
//    data = data * 2;
//    RbclwSendMessage(ROBOCLAW_ADDRESS,M1BACKWARD,&data,sizeof(data)); // backward 0<data<64
//
//  }
//  else if (data > 64) {
//    data = (data - 64) * 2;
//    RbclwSendMessage(ROBOCLAW_ADDRESS,M1FORWARD,&data,sizeof(data)); // forward 64<data<127
//
//  }
//  else if (data == 64) {
//    data = 0;
//    RbclwSendMessage(ROBOCLAW_ADDRESS,M1FORWARD,&data,sizeof(data)); // stop data = 64
//  }
};

static void SetMotor2(uint8_t speed){ // M1 FORWARD or BACKWARD: (0<x<64 backward, 0=64 stop, 64<x<127 forward)
  uint8_t data = speed;
  if (data < 64) {
    data = data * 2;
    RbclwSendMessage(ROBOCLAW_ADDRESS,M2BACKWARD,&data,sizeof(data)); // backward 0<data<64

  }
  else if (data > 64) {
    data = (data - 64) * 2;
    RbclwSendMessage(ROBOCLAW_ADDRESS,M2FORWARD,&data,sizeof(data)); // forward 64<data<127

  }
  else if (data == 64) {
    data = 0;
    RbclwSendMessage(ROBOCLAW_ADDRESS,M2FORWARD,&data,sizeof(data)); // stop data = 64
  }
};
