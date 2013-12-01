#include <msp430.h> 
#include "../BoatMotorController/BoatTypes.h"
#include "../BoatMotorController/BoatPeripherals.h"
#include "BmsMasterDefines.h"
#include "../CellBms/BmsSerial.h"
/*
 * main.c
 */
//MSP430F2272


//5 relays.
//2 current sense inputs
//temp
//humidity
//tx/rx up
//3 LEDs
//buzzer

U16 RelayAdcValueNeeded = 0;
U16 RelayPreChargeTimeoutMs = 0;
U16 RelayPreChargeTimerMs = 0;
bool RelayPreChargeActive = FALSE;
U8 RelayId = 0;

U8 BmsTxData[6];
bool RelayStates[5];

extern U8 Buffer[MAX_TX_LENGTH];
extern U16 RxPacketSize;

U16 WatchDogCounterS = 0;
bool BmsAbortWatchdog = FALSE;

bool TxNextBmsMessage = FALSE;

U16 GetHumidity();
U16 GetTemp();
U16 GetPackVoltage();
bool SetRelay(U8 relay_id, bool on, bool use_precharge, U16 needed_adc_value, U16 timeout_ms);

int main(void) {
	//U16 adc_test = 0;
	//U16 adc_chan = 0;
	U8 i;
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    //setup clock for 16MHz.
    DCOCTL = CALDCO_8MHZ,
    BCSCTL1 = CALBC1_8MHZ;

    P2SEL &= ~RELAY_5; //switch 2.7 into a IO rather than XOUT

    P1DIR = YELLOW_BIT | GREEN_BIT | RED_BIT | TX_UP_PIN;
    P2DIR = PRE_CHARGE_4_BIT | PRE_CHARGE_5_BIT | RELAY_5;
    P3DIR = RELAY_1;
    P4DIR = PRE_CHARGE_1_BIT | PRE_CHARGE_2_BIT | PRE_CHARGE_3_BIT | RELAY_2 | RELAY_3 | RELAY_4;

    for(i = 0; i < 5; ++i)
    {
    	RelayStates[i] = FALSE;
    	//turn off relays.
    }
    RELAY1_OFF;
    RELAY2_OFF;
    RELAY3_OFF;
    RELAY4_OFF;
    RELAY5_OFF;
    PRE_CHARGE_1_OFF;
    PRE_CHARGE_2_OFF;
    PRE_CHARGE_3_OFF;
    PRE_CHARGE_4_OFF;
    PRE_CHARGE_5_OFF;



    BmsSerialInit();


    YELLOW_OFF;
    GREEN_OFF;
    RED_OFF;

    //YELLOW_ON;
    //GREEN_ON;
    //RED_ON;

    InitSerial();

    //init timer A for 1000hz  SMCLK is running at 8MHz.
    CCTL0 = CCIE;
    CCR0 = 8000;
    TACTL = TASSEL_2 + MC_1 + ID_0;  //TASSEL_2=SMCLK, MC_1=Up mode: the timer counts up to TACCR0, ID_3=div by 8


    InitAdc(ALL_ANALOG_INPUTS);
    InitSpi();

    __bis_SR_register( GIE );

    //adc_chan = INCH_3;
    //adc_test = ReadAnalogValue(adc_chan);

    //adc_chan = INCH_7;
    //adc_test = ReadAnalogValue(adc_chan);
    //WriteWordSerial(adc_test);

//---------------------------------------------------------------------------------------------------------

    //U8 len = 0;
    //U8 tx_data[4] = { 0x01, 0x0F, 0xAA, 0x55};

    while(1)
    {
    	if( TxNextBmsMessage == TRUE )
    	{
    		YELLOW_ON;
    		while(!CheckOperationComplete()); //wait for TX up process to finish.

    		//we need to clear this flag, otherwise the reset will never finish.
        	WatchDogCounterS = 0;
        	BmsAbortWatchdog = FALSE;
        	WatchDogCounterS = 0;
        	BmsAbortWatchdog = FALSE;

    		WatchDogCounterS = 6; //set a 5 second timeout for this.
    		StartReadData(TRUE);
    		while(!CheckOperationComplete());

    		TxNextBmsMessage = FALSE;
    		if( RxPacketSize == 0 )
    		{
    			//failure;
    			WriteByteSerial(PARAM_OFF);
    			YELLOW_OFF;
    			continue;
    		}
    		WriteBufferSerial( Buffer, RxPacketSize );
    		YELLOW_OFF;

    	}

    };
	
	return 0;
}


U16 GetHumidity()
{
	return ReadAnalogValue(INCH_7);
}

U16 GetTemp()
{
	return ReadAnalogValue(INCH_3);
}

U16 GetPackVoltage()
{
	return ReadAnalogValue(BATT_ANALOG);
}

U16 Ms = 0;
U32 Seconds = 0;
U16 Us = 0;
//U16 WatchDogCounter = 0;
// 5 seconds at 100Hz = 500
//#define WATCHDOG_LIMIT	500


#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{



	TimerEventBmsSerial();
	Us += 1000;

	if( Us == 1000)
	{
		Us = 0;
		Ms++;
	}

	if( (Ms > 0) && (Ms < 50))
		GREEN_ON;
	else
		GREEN_OFF;

	//GREEN_OFF;
	if( Ms == 1000 )
	{
		Ms = 0;
		Seconds++;
		if( WatchDogCounterS > 0 )
		{
			WatchDogCounterS -= 1;
			if(WatchDogCounterS == 0 )
			{

				BmsAbortWatchdog = TRUE;
			}
		}
	}


	return;

	//U16 RelayAdcValueNeeded = 0;
	//U16 RelayPreChargeTimeoutMs = 0; RelayPreChargeTimerMs
	//bool RelayPreChargeActive = FALSE;

	//TODO: replace pre-charge feature!
/*	if( RelayPreChargeActive == TRUE )
	{
		//perform relay precharge check and logic here. internal freq is 10ms (100Hz)
		U16 adv_value;

		switch( RelayId )
		{
		case 1:
			adv_value = ReadAnalogValue(PRE_CHARGE_1_ANALOG);
			break;
		case 2:
			adv_value = ReadAnalogValue(PRE_CHARGE_2_ANALOG);
			break;
		case 3:
			adv_value = ReadAnalogValue(PRE_CHARGE_3_ANALOG);
			break;
		case 4:
			adv_value = ReadAnalogValue(PRE_CHARGE_4_ANALOG);
			break;
		case 5:
			adv_value = ReadAnalogValue(PRE_CHARGE_5_ANALOG);
			break;
		default:
			RelayPreChargeActive = FALSE;
			WriteByteSerial(PARAM_OFF); //error case!
			goto TIMERA_RETURN;
		}

		if( adv_value >= RelayAdcValueNeeded )
		{
			switch( RelayId )
			{
			case 1:
				PRE_CHARGE_1_OFF;
				RELAY1_ON;
				break;
			case 2:
				PRE_CHARGE_2_OFF;
				RELAY2_ON;
				break;
			case 3:
				PRE_CHARGE_3_OFF;
				RELAY3_ON;
				break;
			case 4:
				PRE_CHARGE_4_OFF;
				RELAY4_ON;
				break;
			case 5:
				PRE_CHARGE_5_OFF;
				RELAY5_ON;
				break;
			}
			RelayPreChargeActive = FALSE;
			WriteByteSerial(PARAM_ON);
			goto TIMERA_RETURN;
		}
		else
		{
			RelayPreChargeTimerMs += 10;
			if( RelayPreChargeTimerMs >= RelayPreChargeTimeoutMs )
			{
				//oh crap, need to turn off pre-charge and return false.
				switch( RelayId )
				{
				case 1:
					PRE_CHARGE_1_OFF;
					break;
				case 2:
					PRE_CHARGE_2_OFF;
					break;
				case 3:
					PRE_CHARGE_3_OFF;
					break;
				case 4:
					PRE_CHARGE_4_OFF;
					break;
				case 5:
					PRE_CHARGE_5_OFF;
					break;
				}
				RelayPreChargeActive = FALSE;
				WriteByteSerial(PARAM_OFF);
				goto TIMERA_RETURN;
			}

		}

	}*/

	/*WatchDogCounter++;
	if( WatchDogCounter >= WATCHDOG_LIMIT )
	{
		//kill motor!
		//beep!
		YELLOW_ON;

	}*/

//TIMERA_RETURN:
	//GREEN_OFF;
}


bool CmdInProgress = FALSE;
U8 SerialByteIndex = 0;
U8 CmdRxBuffer[8];

/*
#define CMD_GET_CELL_VOLTAGE  		0x10 //param cell
#define CMD_GET_CELL_TEMP 			0x11 //param cell
#define CMD_SET_DISCHARGE_PROPS  	0x12 //param cell, TEMP params: current_H, current_L
#define CMD_GET_CELL_COUNT  		0x13
#define CMD_SET_RELAY 				0x14 //param relay, state
#define CMD_GET_MASTER_TEMP  		0x15
#define CMD_GET_MASTER_HUMIDITY  	0x16
 */

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	U8 value = UCA0RXBUF;
	U16 temporary = 0x0;

	CmdInProgress = TRUE;

	CmdRxBuffer[SerialByteIndex] = value;
	SerialByteIndex++;

	if( CmdRxBuffer[0] == CMD_GET_CELL_VOLTAGE && SerialByteIndex < 2 )
		return;
	if( CmdRxBuffer[0] == CMD_GET_CELL_TEMP && SerialByteIndex < 2 )
		return;
	if( CmdRxBuffer[0] == CMD_SET_RELAY && SerialByteIndex < 8 )
		return;
	if( CmdRxBuffer[0] == CMD_GET_CURRENT && SerialByteIndex < 2 )
		return;
	if( CmdRxBuffer[0] == CMD_SET_DISCHARGE_PROPS && SerialByteIndex < 4 )
		return;
	if( CmdRxBuffer[0] == CMD_PING_CELL && SerialByteIndex < 2 )
		return;
	if( CmdRxBuffer[0] == CMD_GET_RELAY && SerialByteIndex < 2 )
		return;

	CmdInProgress = FALSE;
	SerialByteIndex = 0;

	if( TxNextBmsMessage == TRUE )
	{
		//command in progress! we should NAK the PC for the new command.
		//for now let's ignore it and return.
		return;
	}

	switch(CmdRxBuffer[0])
	{
	case CMD_GET_CELL_VOLTAGE:
		BmsTxData[0] = CELL_CMD_GET_VOLTAGE;
		BmsTxData[1] = CmdRxBuffer[1]; //Cell ID
		WriteDataBmsSerial( BmsTxData, 2, TRUE );
		TxNextBmsMessage = TRUE;
		break;
	case CMD_GET_CELL_TEMP:
		BmsTxData[0] = CELL_CMD_GET_TEMP;
		BmsTxData[1] = CmdRxBuffer[1]; //Cell ID
		WriteDataBmsSerial( BmsTxData, 2, TRUE );
		TxNextBmsMessage = TRUE;
		break;
	case CMD_PING_CELL:
		BmsTxData[0] = CELL_CMD_PING;
		BmsTxData[1] = CmdRxBuffer[1]; //Cell ID
		WriteDataBmsSerial( BmsTxData, 2, TRUE );
		TxNextBmsMessage = TRUE;
		break;

	case CMD_SET_DISCHARGE_PROPS: //todo: updated this after testing with real command.
		BmsTxData[0] = CELL_CMD_SET_CURRENT;
		BmsTxData[1] = CmdRxBuffer[1]; //Cell ID
		BmsTxData[2] = CmdRxBuffer[2]; //Current H
		BmsTxData[3] = CmdRxBuffer[3]; //current L

		WriteDataBmsSerial( BmsTxData, 4, TRUE );
		TxNextBmsMessage = TRUE;

		break;

	case CMD_SET_RELAY:
		{
			U8 relay_index = CmdRxBuffer[1];
			bool relay_on, use_precharge;
			bool success;
			U16 timeout, adc_value; //ms byte first.

			if( relay_index > 5 )
				goto SET_RELAY_FAILED;

			if( CmdRxBuffer[2] == PARAM_ON )
				relay_on = TRUE;
			else if( CmdRxBuffer[2] == PARAM_OFF )
				relay_on = FALSE;
			else
				goto SET_RELAY_FAILED;

			if( CmdRxBuffer[3] == PARAM_ON )
				use_precharge = TRUE;
			else if( CmdRxBuffer[3] == PARAM_OFF )
				use_precharge = FALSE;
			else
				goto SET_RELAY_FAILED;

			timeout = (CmdRxBuffer[4] << 8) | CmdRxBuffer[5];
			adc_value = (CmdRxBuffer[6] << 8) | CmdRxBuffer[7];

			success = SetRelay( relay_index, relay_on, use_precharge, adc_value,  timeout );

			if( (success == TRUE ) && (use_precharge == TRUE) )
			{
				return; //in this case, we defer execution to the timer module.
			}
			if( success == TRUE )
				WriteByteSerial( PARAM_ON );
			else
				WriteByteSerial( PARAM_OFF );

			return;
SET_RELAY_FAILED:
			WriteByteSerial( PARAM_OFF );
			return;
		}

		break;

	case CMD_GET_MASTER_TEMP:
		temporary = GetTemp();
		WriteWordSerial(temporary);
		break;

	case CMD_GET_MASTER_HUMIDITY:
		temporary = GetHumidity();
		WriteWordSerial(temporary);

		break;

	case CMD_GET_CURRENT:
		temporary = ReadWordSpi(CmdRxBuffer[1]);
		WriteWordSerial(temporary);
		break;

	case CMD_GET_PACK_VOLTAGE:
		temporary = GetPackVoltage();
		WriteWordSerial(temporary);
		break;
	case CMD_GET_RELAY:
		{
			U8 relay_index = CmdRxBuffer[1];
			if( RelayStates[relay_index-1] == TRUE )
				WriteByteSerial( PARAM_ON );
			else
				WriteByteSerial( PARAM_OFF );
		}
		break;
	//default:

	}


}


bool SetRelay(U8 relay_id, bool on, bool use_precharge, U16 needed_adc_value, U16 timeout_ms)
{

	if( (on == TRUE) && (use_precharge == TRUE))
	{
		switch( relay_id )
		{
		case 1:
			PRE_CHARGE_1_ON;
			break;
		case 2:
			PRE_CHARGE_2_ON;
			break;
		case 3:
			PRE_CHARGE_3_ON;
			break;
		case 4:
			PRE_CHARGE_4_ON;
			break;
		case 5:
			PRE_CHARGE_5_ON;
			break;
		}

		RelayId = relay_id;

		RelayPreChargeTimerMs = 0;
		RelayPreChargeTimeoutMs = timeout_ms;
		RelayAdcValueNeeded = needed_adc_value;
		RelayPreChargeActive = TRUE;
		return TRUE; //this won't do anything.
	}

	RelayStates[relay_id-1] = on;


	switch(relay_id)
	{
	case 1:
		if( on == TRUE )
			RELAY1_ON;
		else
			RELAY1_OFF;
		break;
	case 2:
		if( on == TRUE )
			RELAY2_ON;
		else
			RELAY2_OFF;
		break;
	case 3:
		if( on == TRUE )
			RELAY3_ON;
		else
			RELAY3_OFF;
		break;
	case 4:
		if( on == TRUE )
			RELAY4_ON;
		else
			RELAY4_OFF;
		break;
	case 5:
		if( on == TRUE )
			RELAY5_ON;
		else
			RELAY5_OFF;
		break;
	}

	return TRUE;

}

#pragma vector=PORT1_VECTOR, PORT2_VECTOR, ADC10_VECTOR, USCIAB0TX_VECTOR, TIMERA1_VECTOR, WDT_VECTOR, TIMERB1_VECTOR, TIMERB0_VECTOR, NMI_VECTOR//, RESET_VECTOR
__interrupt void TRAP_ISR(void)
{
	GREEN_ON;
}

