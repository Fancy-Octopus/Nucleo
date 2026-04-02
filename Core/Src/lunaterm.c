/*
 * lunaterm.c
 *
 *  Created on: Mar 29, 2025
 *      Author: strongp
 */

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"
#include "main.h"

//PLACE INCLUDE OF OTHER MODULES NEEDED FOR CLI HERE//
#include "rover_controller.h"
#include "odrive_can.h"

//////////////////////////////////////////////////////

void ToggleUserLEDs(EmbeddedCli *cli, char *args, void *context);
void LunaTermSendChar(EmbeddedCli *embeddedCli, char c);
void PrintRoverState(EmbeddedCli *cli, char *args, void *context);
void WDReset(EmbeddedCli *cli, char *args, void *context);
void SetForwardState(EmbeddedCli *cli, char *args, void *context);
void SetReadyState(EmbeddedCli *cli, char *args, void *context);
void SetIdleState(EmbeddedCli *cli, char *args, void *context);
void SetWinchUp(EmbeddedCli *cli, char *args, void *context);
void SetWinchDown(EmbeddedCli *cli, char *args, void *context);
void SetDigState(EmbeddedCli *cli, char *args, void *context);
void AvgLoopTime(EmbeddedCli *cli, char *args, void *context);
void PrintOdriveStats(EmbeddedCli *cli, char *args, void *context);

EmbeddedCli *cli;

CliCommandBinding TogLed = {
    "tog-led",          // command name (spaces are not allowed)
    "toggle user led",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
    ToggleUserLEDs               // binding function
};

CliCommandBinding RoverState = {
    "get_state",          // command name (spaces are not allowed)
    "Get the current state set for the rover",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	PrintRoverState               // binding function
};

CliCommandBinding WatchdogReset = {
    "wd-rst",          // command name (spaces are not allowed)
    "Reset using IWDG",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	WDReset               // binding function
};

CliCommandBinding SetRovForward = {
    "set-fwd",          // command name (spaces are not allowed)
    "Set rover state to forward",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	SetForwardState               // binding function
};

CliCommandBinding WinchUp = {
    "winch-up",          // command name (spaces are not allowed)
    "Raise the winch",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	SetWinchUp               // binding function
};

CliCommandBinding WinchDown = {
    "winch-down",          // command name (spaces are not allowed)
    "Lower the winch",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	SetWinchDown               // binding function
};

CliCommandBinding SetRovRdy = {
    "set-rdy",          // command name (spaces are not allowed)
    "Set rover state to ready (cam up)",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	SetReadyState               // binding function
};

CliCommandBinding SetRovIdle = {
    "set-idle",          // command name (spaces are not allowed)
    "Set rover state to idle",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	SetIdleState               // binding function
};

CliCommandBinding SetDig = {
    "set-dig",          // command name (spaces are not allowed)
    "Set rover state to dig",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	SetDigState               // binding function
};

CliCommandBinding GetLoopTime = {
    "loop-time",          // command name (spaces are not allowed)
    "Get the average time of main loop execution (ms)",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	AvgLoopTime               // binding function
};

CliCommandBinding OdriveStats = {
    "stats-odrvs",          // command name (spaces are not allowed)
    "Print Odrive Stats",   // Optional help for a command (NULL for no help)
    0,              // flag whether to tokenize arguments (see below)
    NULL,            // optional pointer to any application context
	PrintOdriveStats               // binding function
};


void LunaTermInit(void){
	EmbeddedCliConfig cliConfig = *embeddedCliDefaultConfig();
	cliConfig.maxBindingCount = 20;
	cli = embeddedCliNew(&cliConfig);
	cli->writeChar = LunaTermSendChar;

	embeddedCliAddBinding(cli,TogLed);
	embeddedCliAddBinding(cli,RoverState);
	embeddedCliAddBinding(cli,WatchdogReset);
	embeddedCliAddBinding(cli,SetRovForward);
	embeddedCliAddBinding(cli,SetRovRdy);
	embeddedCliAddBinding(cli,GetLoopTime);
	embeddedCliAddBinding(cli,WinchUp);
	embeddedCliAddBinding(cli,WinchDown);
	embeddedCliAddBinding(cli,SetDig);
	embeddedCliAddBinding(cli,SetRovIdle);
	embeddedCliAddBinding(cli,OdriveStats);

	printf("\r\n");
}

void LunaTermPoll(void){
  static uint8_t byteIn;
  byteIn = 0;
  HAL_UART_Receive(&hcom_uart[COM1], &byteIn, 1, 5);
  if(byteIn!=0) embeddedCliReceiveChar(cli, byteIn);
  embeddedCliProcess(cli);
}

void LunaTermSendChar(EmbeddedCli *embeddedCli, char c){
  uint8_t data = c;
  HAL_UART_Transmit(&hcom_uart[COM1], &data, 1, 5);
}

void ToggleUserLEDs(EmbeddedCli *cli, char *args, void *context){
  BSP_LED_Toggle(LED_GREEN);
  BSP_LED_Toggle(LED_YELLOW);
  BSP_LED_Toggle(LED_RED);
}

void PrintRoverState(EmbeddedCli *cli, char *args, void *context){
	rover_state_t state = CurrentRoverState();
	switch(state){
		case ROVER_IDLE:
			printf("ROVER_IDLE\r\n");
			break;
		case ROVER_FORWARD:
			printf("ROVER_FORWARD\r\n");
			break;
		case ROVER_BACKWARD:
			printf("ROVER_BACKWARD\r\n");
			break;
		case ROVER_TURN_RIGHT:
			printf("ROVER_TURN_RIGHT\r\n");
			break;
		case ROVER_TURN_LEFT:
			printf("ROVER_TURN_LEFT\r\n");
			break;
		case ROVER_WINCH_DOWN:
			printf("ROVER_WINCH_DOWN\r\n");
			break;
		case ROVER_WINCH_UP:
			printf("ROVER_WINCH_UP\r\n");
			break;
		case ROVER_DIG_FORWARD:
			printf("ROVER_DIG_FORWARD\r\n");
			break;
		case ROVER_DEPOSIT_FORWARD:
			printf("ROVER_DEPOSIT_FORWARD\r\n");
			break;
		case ROVER_DIG_BACKWARD:
			printf("ROVER_DIG_BACKWARD\r\n");
			break;
		case ROVER_DEPOSIT_BACKWARD:
			printf("ROVER_DIG_BACKWARD\r\n");
			break;
		case ROVER_READY:
			printf("ROVER_READY\r\n");
			break;
		case ROVER_ESTOP:
			printf("ROVER ESTOP");
			break;
		default:
			printf("Uh, not a valid state\r\n");
			break;
	}
}

void WDReset(EmbeddedCli *cli, char *args, void *context){
	HAL_Delay(2000);
}

void SetForwardState(EmbeddedCli *cli, char *args, void *context){
	RequestRoverState(ROVER_FORWARD);
}

void SetReadyState(EmbeddedCli *cli, char *args, void *context){
	RequestRoverState(ROVER_READY);
}

void SetIdleState(EmbeddedCli *cli, char *args, void *context){
	RequestRoverState(ROVER_IDLE);
}

void SetDigState(EmbeddedCli *cli, char *args, void *context){
	RequestRoverState(ROVER_DIG_FORWARD);
}

void SetWinchUp(EmbeddedCli *cli, char *args, void *context){
	RequestRoverState(ROVER_WINCH_UP);
}

void SetWinchDown(EmbeddedCli *cli, char *args, void *context){
	RequestRoverState(ROVER_WINCH_DOWN);
}


void AvgLoopTime(EmbeddedCli *cli, char *args, void *context){
	printf("Average Loop Time -> %u\r\n",(unsigned int)GetAvgLoopTime());
}

void PrintOdriveStats(EmbeddedCli *cli, char *args, void *context){
	PrintODriveConnStatus();
}
