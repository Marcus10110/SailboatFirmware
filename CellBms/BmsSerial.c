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


#ifdef __MSP430F2272__
//set the timer direction to none.
#define STOP_TIMER_A()	TACTL &= ~MC_3
#define START_TIMER_A() TACTL |= MC_1 | TACLR

#else
#define STOP_TIMER_A() _NOP()
#define START_TIMER_A() _NOP()
#endif


//down means to the BMS master, up means to the end of the chain.
typedef enum  { IDLE, BMS_TX_UP, BMS_TX_DOWN, BMS_RX_UP, BMS_RX_DOWN, PENDING_RX } BmsSerialState;
extern bool BmsAbortWatchdog;
extern U16 WatchDogCounterS;

BmsSerialState CurrentState;

U8 Buffer[MAX_TX_LENGTH];
U8 BufferByteCount;

U8 CurrentByte;
U8 CurrentBit; //0 to 7, 0 = MSB??

bool IsrPending = FALSE;


U16 RxPacketSize;

void ReverseSerialDirection();

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

	START_TIMER_A(); //only does stuff on BMS Master
}

bool StartReadData(bool up)
{
	U8 i;
	for(i = 0; i < MAX_TX_LENGTH; ++i)
		Buffer[i] = 0;
	CurrentBit = 0; //skip first start bit, will be picked up by us.
	CurrentByte = 0;
	RxPacketSize = 0;


#ifdef __MSP430F2272__
	//use pin interrupt to sync timer.
	//P1OUT ^= BIT4; //indicate that we've started looking for the edge ISR. (it toggles in the ISR too.)
	IsrPending = TRUE;
	P1IFG = 0;
	_NOP();
	P1IE = RX_UP_PIN;
	_NOP();

	return TRUE;
	//the abort process must now be handled by the main timer. (TimerB)
	/*while(1)
	{
		if( BmsAbortWatchdog == TRUE )
		{
			//terminate the interrupt
			P1IE = 0;
			_NOP();
			CurrentState = IDLE; //we no longer start in idle for read operations on master.
			IsrPending = FALSE;
			return FALSE;
		}
		if( IsrPending == FALSE ) //detect that the P1 ISR has finished!
			return TRUE;
	}*/


#endif

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
			STOP_TIMER_A();
#ifdef __MSP430F2272__
			CurrentState = PENDING_RX;
			ReverseSerialDirection(); //clear out the watchdog, start a RX process. (state is still write UP)
#else
			CurrentState = IDLE;
#endif
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
		//P1OUT ^= BIT6;
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
			STOP_TIMER_A();
			return;
		}

		if( CurrentBit == 0 )
		{
			//start bit!
			if( sample!= 0x00 )
			{
				//it failed.
				CurrentState = IDLE;
				STOP_TIMER_A();
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
				STOP_TIMER_A();
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

void ReverseSerialDirection()
{
	//we need to clear this flag, otherwise the reset will never finish.
	while( WatchDogCounterS != 0 )
		WatchDogCounterS = 0;

	while( BmsAbortWatchdog != 0 )
		BmsAbortWatchdog = FALSE;

	WatchDogCounterS = 6; //set a 5 second timeout for this.
	StartReadData(TRUE);

}

void EdgeEventIsr()
{
	if( IsrPending == FALSE ) //debug check, this should not happen!
	 return;

	START_TIMER_A(); //starts the timer!
	IsrPending = FALSE;
	CurrentState = BMS_RX_UP;
	TAR = 4000; //pre-load half way!

	//kill the watchdog timer, it's not needed any more.
	while( WatchDogCounterS != 0 )
		WatchDogCounterS = 0;

	while( BmsAbortWatchdog != 0 )
		BmsAbortWatchdog = FALSE;

}

void RxWaitExpired()
{
	if( CurrentState != PENDING_RX ) //error condition, this should only happen while were waiting for an RX opperation.
		return;
	//(arrives on timer B ISR)

	//kill edge ISR
	P1IE = 0;
	_NOP();
	IsrPending = FALSE;

	//set state to idle.
	CurrentState = IDLE;

	//clean up the watchdog.
	//we need to clear this flag, otherwise the reset will never finish.
	while( WatchDogCounterS != 0 )
		WatchDogCounterS = 0;

	while( BmsAbortWatchdog != 0 )
		BmsAbortWatchdog = FALSE;

}
