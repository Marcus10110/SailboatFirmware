/*
 * BmsSerial.h
 *
 *  Created on: Jul 20, 2013
 *      Author: Mark
 */

#ifndef BMSSERIAL_H_
#define BMSSERIAL_H_

#include "../BoatMotorController/BoatTypes.h"


#define CELL_CMD_GET_VOLTAGE  		0x40
#define CELL_CMD_GET_TEMP 			0x41
#define CELL_CMD_PING  				0x42
#define CELL_CMD_SET_CURRENT		0x43

#ifdef __MSP430F2272__

//MASTER BMS.
#define TX_UP_PIN			BIT1
#define TX_UP_PORT			P1OUT

#define RX_UP_PIN			BIT0
#define RX_UP_PORT			P1IN
#define RX_UP_SHIFT			0

//does not exist on master bms.
#define TX_DOWN_PIN			0
#define TX_DOWN_PORT		0

#define RX_DOWN_PIN			0
#define RX_DOWN_PORT		0
#define RX_DOWN_SHIFT		0

#else

//CELL BMS
#define TX_UP_PIN			BIT0
#define TX_UP_PORT			P3OUT

#define RX_UP_PIN			BIT2
#define RX_UP_PORT			P2IN
#define RX_UP_SHIFT			2

#define TX_DOWN_PIN			BIT2
#define TX_DOWN_PORT		P3OUT

#define RX_DOWN_PIN			BIT3
#define RX_DOWN_PORT		P3IN
#define RX_DOWN_SHIFT		3

#endif

//TODO: check if the transistors invert this. (both tx and rx, up and down)
#define SET_TX_UP		TX_UP_PORT |= TX_UP_PIN
#define CLEAR_TX_UP		TX_UP_PORT &= ~TX_UP_PIN

#ifdef __MSP430F2272__
#define SET_TX_DOWN
#define CLEAR_TX_DOWN
#else
#define SET_TX_DOWN		TX_DOWN_PORT |= TX_DOWN_PIN
#define CLEAR_TX_DOWN		TX_DOWN_PORT &= ~TX_DOWN_PIN
#endif



#define READ_RX_UP		(RX_UP_PORT & RX_UP_PIN) >> RX_UP_SHIFT

#ifdef __MSP430F2272__
#define READ_RX_DOWN	0
#else
#define READ_RX_DOWN	(RX_DOWN_PORT & RX_DOWN_PIN) >> RX_DOWN_SHIFT
#endif



#define MAX_TX_LENGTH	7


void BmsSerialInit();
void WriteDataBmsSerial(U8* data, U8 length, bool up);
bool StartReadData(bool up);
bool CheckOperationComplete();
void ForceIdle();
void TimerEventBmsSerial();





#endif /* BMSSERIAL_H_ */
