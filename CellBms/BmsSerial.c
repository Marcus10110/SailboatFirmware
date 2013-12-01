/*
 * BmsSerial.c
 *
 *  Created on: Jul 20, 2013
 *      Author: Mark
 */

#include <msp430.h>
#include "BmsSerial.h"

#ifdef __MSP430G2553__
#include "CellBmsDefines.h"
#endif
/*
 * Bms Serial:
 * All Cells are always listening for a new command.
 * When a Cell receives a command, it completely executes it before listening again.
 *
 *
 * Cell Commands:
 * Get Cell count:
 * BMS master xmits this command. it's a count, starting at zero.
 * Cell BMS receives the command, and retransmits it upstream. It starts a countdown timer for 1-3 second(s), depending on tx speed.
 * While waiting, it listens for a upstream response.
 * if a upstream response arrives, it transmits it downstream, without modification.
 * if the timer expires, it transmits the cell count down stream.
 *
 * Get Cell voltage.
 * The BMS master xmits a message up stream, with the cell ID.
 * Each cell decrements the cell ID as it transmits it up. It then listens upstream for a new command (with reset watchdog. on expire, go back to listening downstream.)
 * if the received cell ID is zero, then instead it will measure the cell voltage and transmit it down.
 *
 * Get Cell temp
 * Same as Cell Voltage.
 *
 * Set discharge current (or profile)
 * TBD.
 *
 *
 * Set LEDs (optional, could reflect operation)
 *
 *
 * protocol:
 *
 *
 */



//down means to the BMS master, up means to the end of the chain.
typedef enum  { IDLE, BMS_TX_UP, BMS_TX_DOWN, BMS_RX_UP, BMS_RX_DOWN } BmsSerialState;
extern bool BmsAbortWatchdog;

BmsSerialState CurrentState;

U8 Buffer[MAX_TX_LENGTH];
U8 BufferByteCount;

U8 CurrentByte;
U8 CurrentBit; //0 to 7, 0 = MSB??


U16 RxPacketSize;

void BmsSerialInit()
{
	SET_TX_UP;

	SET_TX_DOWN;
}

void WriteDataBmsSerial(U8* data, U8 length, bool up)
{
	//lets tx. low start bit, followed by MSB first, high stop bit.
	U8 i;

	if( length >=  MAX_TX_LENGTH)
		return;

	for( i = 0; i < length; ++i)
		Buffer[i+1] = data[i];
	BufferByteCount = length+1;
	Buffer[0] = length+1;


	CurrentByte = 0;
	CurrentBit = 0;

	if( up == TRUE)
		CurrentState = BMS_TX_UP;
	else
		CurrentState = BMS_TX_DOWN;
}

bool StartReadData(bool up)
{
	U8 i;
	for(i = 0; i < MAX_TX_LENGTH; ++i)
		Buffer[i] = 0;
	CurrentBit = 0; //skip first start bit, will be picked up by us.
	CurrentByte = 0;
	RxPacketSize = 0;


	//wait for a falling edge.
	U8 sample;
	if( up == TRUE )
	{
		while(1)
		{
			if( BmsAbortWatchdog == TRUE )
				return FALSE;
			sample = READ_RX_UP & 0x01;
			if( sample == 0x00)
				break;
		}
	}
	else
	{
		while(1)
		{
			if( BmsAbortWatchdog == TRUE )
				return FALSE;
			sample = READ_RX_DOWN & 0x01;
			if( sample == 0x00)
				break;
		}
	}
	//clear timer, just to give us a bit more time.
	TACTL |= TACLR;


	if( up == TRUE )
		CurrentState = BMS_RX_UP;
	else
		CurrentState = BMS_RX_DOWN;

//we may need to tune for up and down separately.
#ifndef __MSP430F2272__ //NOT defined. this is for the cell board.
	if( up == TRUE )
		TAR = 4000;//550;//CELL BMS, tune to 550. RX UP.
	else
		TAR = 4000;//650; //CELL BMS, tune to 650. RX down
	//RED_TOGGLE;
#else //this is the master board.
	TAR = 4000;//300; //set timer for half way. (plus a little less time to compensate) (timer limit is set to 800, 10 KHz) (8000, 1KHz)
#endif

	return TRUE;
}

bool CheckOperationComplete()
{
	if( CurrentState == IDLE )
		return TRUE;

	return FALSE;
}

void ForceIdle()
{
	CurrentState = IDLE;

}

void TimerEventBmsSerial()
{
	if( CurrentState == BMS_TX_UP || CurrentState == BMS_TX_DOWN )
	{
		if( CurrentByte == BufferByteCount )
		{
			//most def set bus back to idle (high)
			SET_TX_UP;
			SET_TX_DOWN;
			//TODO: DONE! inform end user of completed transaction.
			CurrentState = IDLE;
			return;
		}
		U8 bit_to_tx;

		if( CurrentBit == 0 )
		{
			bit_to_tx = 0;
		}
		else if( CurrentBit == 9)
		{
			bit_to_tx = 0x01;
		}
		else
		{
			bit_to_tx = Buffer[CurrentByte];
			bit_to_tx >>= (CurrentBit-1);
			bit_to_tx &= 0x01;
		}

		CurrentBit++;
		if( CurrentBit == 10 )
		{
			CurrentBit = 0;
			CurrentByte++;
		}

		//tx a single bit!
		if( CurrentState == BMS_TX_UP )
		{
			if( bit_to_tx == 0x00 )
				CLEAR_TX_UP;
			else
				SET_TX_UP;
		}
		else if( CurrentState == BMS_TX_DOWN )
		{
			if( bit_to_tx == 0x00 )
				CLEAR_TX_DOWN;
			else
				SET_TX_DOWN;
		}

	}
	else if( CurrentState == BMS_RX_UP || CurrentState == BMS_RX_DOWN)
	{
		U8 sample;

		if( CurrentState == BMS_RX_UP )
			sample = READ_RX_UP;
		else
			sample = READ_RX_DOWN;

#ifndef __MSP430F2272__ //NOT defined. this is for the cell board.
		//TX_DOWN_PORT ^= TX_DOWN_PIN; // for now, lets toggle the pin when we sample for timing analysis.
		//RED_TOGGLE;
#endif

		sample &= 0x01;

		if( sample != 0x00 )
			sample = 0x01;

		//idle case.
		if( CurrentByte == 0 && CurrentBit == 0 && sample == 0x01 )
			return;

		//rx 10 bits, 0-9.
		if( CurrentBit == 10 )
		{
			CurrentByte++;
			CurrentBit = 0;
		}

		if( CurrentByte == 1 && CurrentBit == 0)
		{
			RxPacketSize = Buffer[0];
		}

		if( (CurrentByte > 0) && (CurrentByte == RxPacketSize))
		{
			//TODO: exit the RX loop here!
			CurrentState = IDLE;
			return;
		}

		if( CurrentBit == 0 )
		{
			//start bit!
			if( sample!= 0x00 )
			{
				//it failed.
				CurrentState = IDLE;
				RxPacketSize = 0;
				return;
			}
		}
		else if( CurrentBit == 9 )
		{
			//stop bit!
			if( sample!= 0x01 )
			{
				//it failed.
				CurrentState = IDLE;
				RxPacketSize = 0;
				return;
			}
		}
		else
		{
			//data!
			if( sample != 0x00 )
			{
				//bit 1 = left shift 7.
				//bit 8 = left shift 0.
				Buffer[CurrentByte] |= 0x01 << (CurrentBit-1);
			}
		}

		CurrentBit++;
	}
}
