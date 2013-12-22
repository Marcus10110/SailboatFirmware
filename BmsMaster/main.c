#include <msp430.h> 
#include "../BoatMotorController/BoatTypes.h"
#include "../BoatMotorController/BoatPeripherals.h"
#include "BmsMasterDefines.h"
#include "../CellBms/BmsSerial.h"
#include "../CellBms/BmsSharedDefines.h"
/*
 * main.c
 */
//MSP430F2272

typedef struct //about 83 bytes.
{
	U16 CellVoltages[CELL_COUNT];
	U16 CellTemperatures[ CELL_COUNT ];

	U16 PrechargeVoltages[RELAY_COUNT];
	U16 MasterCurrent;
	U16 ChargeCurrent;
	U16 PackVoltage;
	U16 MasterTemperature;
	U16 MasterHumidity;
	U16 ValidCellReadCount;
	U8	CriticalFlags;
} ReadDataStruct;

typedef struct //about 42 bytes.
{
	U16 CellCurrents[CELL_COUNT];
	bool	RelayStates[RELAY_COUNT];
	bool	RelayPrechargeStates[RELAY_COUNT];
	bool	DisableBuzzer;
	bool 	padding;
} WriteDataStruct;

ReadDataStruct ReadData;
WriteDataStruct WriteData;

U8 BmsTxData[6];

extern U8 Buffer[MAX_TX_LENGTH];
extern U16 RxPacketSize;

U16 WatchDogCounterS = 0;
bool BmsAbortWatchdog = FALSE;
U16 IdlePeriodCounterS = 0;
U16 PcIdlePeriodCounterS = 0;
U8 CriticalConditionFlags = 0;

void InitStructs( ReadDataStruct* read_data, WriteDataStruct* write_data );
void MainUpdate( ReadDataStruct* read_data, WriteDataStruct* write_data );
void LocalUpdate( ReadDataStruct* read_data, WriteDataStruct* write_data );
void CheckForCriticalConditions( ReadDataStruct* read_data );

U16 GetCellVoltage( U8 cell_id );
U16 GetCellTemperature( U8 cell_id );
void SetCellCurrent( U8 cell_id, U16 current );
U16 GetPrechargeVoltage( U8 relay_id );
U16 GetHumidity();
U16 GetTemp();
U16 GetPackVoltage();
U16 GetCurrent( U8 id );
void SetRelay( WriteDataStruct* write_data );
void SetPrecharge( WriteDataStruct* write_data );
void DelayMs(U16 ms);

int main(void) {
	//U16 adc_test = 0;
	//U16 adc_chan = 0;
	U8 i;
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    //setup clock for 16MHz.
    DCOCTL = CALDCO_8MHZ,
    BCSCTL1 = CALBC1_8MHZ;

    P2SEL &= ~RELAY_1; //switch 2.7 into a IO rather than XOUT
    P2SEL &= ~PRE_CHARGE_2_BIT; //2.6 switch to I/o not XIN

    P1DIR = YELLOW_BIT | GREEN_BIT | RED_BIT | TX_UP_PIN | BUZZ_BIT;
    P2DIR = PRE_CHARGE_2_BIT | PRE_CHARGE_1_BIT | RELAY_1;
    P3DIR = RELAY_5;
    P4DIR = PRE_CHARGE_5_BIT | PRE_CHARGE_4_BIT | PRE_CHARGE_3_BIT | RELAY_4 | RELAY_3 | RELAY_2;

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

    //setup RX pin interrupt.
    P1IES = RX_UP_PIN; //set bit for interrupt on high to low.
    P1IE = RX_UP_PIN;



    YELLOW_OFF;
    GREEN_OFF;
    RED_OFF;

    //YELLOW_ON;
    //GREEN_ON;
    //RED_ON;

    InitSerial();

    //init timer A for 1000hz  SMCLK is running at 8MHz.
    TACCTL0 = CCIE;
    TACCR0 = 8000;
    TACTL = TASSEL_2 + MC_0 + ID_0;  //TASSEL_2=SMCLK, MC_1=Up mode: the timer counts up to TACCR0, ID_3=div by 8. MC_0 = stop timer completely (and power it down)

    //init timer B for 1KHz, always on, used for cell serial watchdog and time.
    TBCCR0 = 8000;
    TBCTL = TBSSEL_2 + MC_1 + ID_0;
    TBCCTL0 = CCIE;



    InitAdc(ALL_ANALOG_INPUTS);
    InitSpi();

    InitStructs( &ReadData, &WriteData );

    __bis_SR_register( GIE );

    while(1)
    {
    	MainUpdate( &ReadData, &WriteData );
    };
	
	return 0;
}

void InitStructs( ReadDataStruct* read_data, WriteDataStruct* write_data )
{
	U16 i;
	for( i = 0; i < CELL_COUNT; ++i)
	{
		read_data->CellVoltages[i] = 0;
		read_data->CellTemperatures[i] = 0;
		write_data->CellCurrents[i] = 0;
	}

	for( i = 0; i < RELAY_COUNT; ++i)
	{
		write_data->RelayStates[i] = FALSE;
		write_data->RelayPrechargeStates[i] = FALSE;
	}
	read_data->ValidCellReadCount = 0;
	read_data->CriticalFlags = 0;

	write_data->DisableBuzzer = FALSE;

}

void MainUpdate( ReadDataStruct* read_data, WriteDataStruct* write_data )
{
	U16 i;
	volatile U16 tmp_cell_voltage, tmp_cell_temp;
	volatile U16 tmp_cell_current;
	volatile U16 valid_cell_count;


	if( PcIdlePeriodCounterS >= BMS_PC_SERIAL_TIMEOUT )
	{
		//CLEAR ALL CELL CURRENTS!!
		DISABLE_RX_INTERRUPT();
		for( i = 0; i < CELL_COUNT; ++i)
			write_data->CellCurrents[i] = 0;

		write_data->RelayStates[1] = FALSE;
		write_data->RelayPrechargeStates[1] = FALSE;

		ENABLE_RX_INTERRUPT();
	}



	valid_cell_count = 0;
	for( i = 0; i < CELL_COUNT; ++i)
	{
		LocalUpdate( read_data, write_data );
		//slow stuff.
		DelayMs(50);
		tmp_cell_voltage = GetCellVoltage(i);
		LocalUpdate( read_data, write_data );
		DelayMs(50);
		tmp_cell_temp = GetCellTemperature(i);

		if( tmp_cell_voltage != 0xFFFF && tmp_cell_temp != 0xFFFF )
		{
			valid_cell_count++;
		}
		else
		{
			//current cell is missing.
			CriticalConditionFlags |= CONDITION_CRITICAL_COMS_LOST;
		}

		DISABLE_RX_INTERRUPT();
		read_data->CriticalFlags = CriticalConditionFlags;
		read_data->CellVoltages[i] = tmp_cell_voltage;
		read_data->CellTemperatures[i] = tmp_cell_temp;
		tmp_cell_current = write_data->CellCurrents[i];
		if( valid_cell_count == CELL_COUNT )
			read_data->ValidCellReadCount++;
		ENABLE_RX_INTERRUPT();

		//write
		LocalUpdate( read_data, write_data );
		DelayMs(50);
		SetCellCurrent(i, tmp_cell_current);

		//check for errors.
		if( valid_cell_count == CELL_COUNT )
		{
			CriticalConditionFlags &= ~CONDITION_CRITICAL_COMS_LOST;


			CheckForCriticalConditions( read_data );


			while( IdlePeriodCounterS != 0 )
			{
				IdlePeriodCounterS = 0;
			}
		}
	}


}

void LocalUpdate( ReadDataStruct* read_data, WriteDataStruct* write_data )
{
	U16 i;
	volatile U16 tmp_master_current, tmp_charge_current, tmp_pack_voltage, tmp_master_temp, tmp_master_humidity;

	volatile U16 tmp_precharge_voltage;
	volatile bool tmp_relay_state;
	volatile bool tmp_relay_precharge_state;
	//this is where we read in all the sensors and stuff.

	//fast stuff.
	tmp_master_current = GetCurrent(1);
	tmp_charge_current = GetCurrent(2);
	tmp_pack_voltage = GetPackVoltage();
	tmp_master_temp = GetTemp();
	tmp_master_humidity = GetHumidity();

	DISABLE_RX_INTERRUPT();
	read_data->MasterCurrent = tmp_master_current;
	read_data->ChargeCurrent = tmp_charge_current;
	read_data->PackVoltage = tmp_pack_voltage;
	read_data->MasterTemperature = tmp_master_temp;
	read_data->MasterHumidity = tmp_master_humidity;
	ENABLE_RX_INTERRUPT();

	for( i = 0; i < RELAY_COUNT; ++i)
	{
		tmp_precharge_voltage = GetPrechargeVoltage(i);

		DISABLE_RX_INTERRUPT();
		read_data->PrechargeVoltages[i] = tmp_precharge_voltage;
		ENABLE_RX_INTERRUPT();
	}

	//write
	DISABLE_RX_INTERRUPT();
	SetRelay( write_data );
	SetPrecharge( write_data );
	ENABLE_RX_INTERRUPT();

}

void CheckForCriticalConditions( ReadDataStruct* read_data )
{
	//all read data is valid at this point, including cells.
	U16 i;

	CriticalConditionFlags &= ~CONDITION_CRITICAL_CELL_TEMP;
	CriticalConditionFlags &= ~CONDITION_CRITICAL_CELL_VOLTAGE;

	for( i = 0; i < CELL_COUNT; ++i)
	{
		if( ( read_data->CellTemperatures[i] > CELL_MAX_TEMPERATURE ) || ( read_data->CellTemperatures[i] < CELL_MIN_TEMPERATURE ) )
			CriticalConditionFlags |= CONDITION_CRITICAL_CELL_TEMP;


		if( ( read_data->CellVoltages[i] > CELL_MAX_VOLTAGE ) || ( read_data->CellVoltages[i] < CELL_MIN_VOLTAGE ) )
			CriticalConditionFlags |= CONDITION_CRITICAL_CELL_VOLTAGE;

	}


	if( read_data->MasterCurrent > MAX_MASTER_CURRENT )
		CriticalConditionFlags |= CONDITION_CRITICAL_MASTER_CURRENT;
	else
		CriticalConditionFlags &= ~CONDITION_CRITICAL_MASTER_CURRENT;

	if( read_data->ChargeCurrent > MAX_CHARGE_CURRENT )
		CriticalConditionFlags |= CONDITION_CRITICAL_CHARGE_CURRENT;
	else
		CriticalConditionFlags &= ~CONDITION_CRITICAL_CHARGE_CURRENT;

	if( ( read_data->PackVoltage > MAX_PACK_VOLTAGE ) || ( read_data->PackVoltage < MIN_PACK_VOLTAGE ) )
		CriticalConditionFlags |= CONDITION_CRITICAL_PACK_VOLTAGE;
	else
		CriticalConditionFlags &= ~CONDITION_CRITICAL_PACK_VOLTAGE;

	if( ( read_data->MasterTemperature > MASTER_MAX_TEMPERATURE ) || ( read_data->MasterTemperature < MASTER_MIN_TEMPERATURE ) )
		CriticalConditionFlags |= CONDITION_CRITICAL_TEMP;
	else
		CriticalConditionFlags &= ~CONDITION_CRITICAL_TEMP;

	if( IdlePeriodCounterS > BMS_MASTER_MAX_REFRESH_DELAY )
		CriticalConditionFlags |= CONDITION_CRITICAL_COMS_LOST;
	else
		CriticalConditionFlags &= ~CONDITION_CRITICAL_COMS_LOST;

}

U16 GetCellVoltage( U8 cell_id )
{
	U16 cell_voltage = 0;

	BmsTxData[0] = CELL_CMD_GET_VOLTAGE;
	BmsTxData[1] = cell_id; //Cell ID
	WriteDataBmsSerial( BmsTxData, 2, TRUE );
	YELLOW_ON;
	while(!CheckOperationComplete()); //wait for TX up process to finish.

	//we need to clear this flag, otherwise the reset will never finish.
	/*while( WatchDogCounterS != 0 )
		WatchDogCounterS = 0;

	while( BmsAbortWatchdog != 0 )
		BmsAbortWatchdog = FALSE;

	WatchDogCounterS = 6; //set a 5 second timeout for this.
	StartReadData(TRUE);
	while(!CheckOperationComplete());*/

	YELLOW_OFF;
	if( RxPacketSize == 0 )
	{
		//failure
		return 0xFFFF;
	}
	//4 bytes total, last 2 contain data. big endian.
	cell_voltage = (U16)Buffer[2] << 8;
	cell_voltage |= Buffer[3];

	return cell_voltage;
}

U16 GetCellTemperature( U8 cell_id )
{
	U16 cell_temperature = 0;

	BmsTxData[0] = CELL_CMD_GET_TEMP;
	BmsTxData[1] = cell_id; //Cell ID
	WriteDataBmsSerial( BmsTxData, 2, TRUE );

	YELLOW_ON;
	while(!CheckOperationComplete()); //wait for TX up process to finish.

	/*//we need to clear this flag, otherwise the reset will never finish.
	while( WatchDogCounterS != 0 )
		WatchDogCounterS = 0;

	while( BmsAbortWatchdog != 0 )
		BmsAbortWatchdog = FALSE;

	WatchDogCounterS = 6; //set a 5 second timeout for this.
	StartReadData(TRUE);
	while(!CheckOperationComplete());*/

	YELLOW_OFF;
	if( RxPacketSize == 0 )
	{
		//failure
		return 0xFFFF;
	}
	//4 bytes total, last 2 contain data. big endian.
	cell_temperature = (U16)Buffer[2] << 8;
	cell_temperature |= Buffer[3];

	return cell_temperature;
}

void SetCellCurrent( U8 cell_id, U16 current )
{
	BmsTxData[0] = CELL_CMD_SET_CURRENT;
	BmsTxData[1] = cell_id; //Cell ID
	BmsTxData[2] = current >> 8; //Current H
	BmsTxData[3] = (U8)current; //current L


	WriteDataBmsSerial( BmsTxData, 4, TRUE );

	YELLOW_ON;
	while(!CheckOperationComplete()); //wait for TX up process to finish.

	/*//we need to clear this flag, otherwise the reset will never finish.
	WatchDogCounterS = 0;
	BmsAbortWatchdog = FALSE;
	WatchDogCounterS = 0;
	BmsAbortWatchdog = FALSE;

	WatchDogCounterS = 6; //set a 5 second timeout for this.
	StartReadData(TRUE);
	while(!CheckOperationComplete());*/

	YELLOW_OFF;
}

U16 PrechargeAnalogInputs[RELAY_COUNT] = {PRE_CHARGE_1_ANALOG, PRE_CHARGE_2_ANALOG, PRE_CHARGE_3_ANALOG, PRE_CHARGE_4_ANALOG, PRE_CHARGE_5_ANALOG};

U16 GetPrechargeVoltage( U8 relay_id )
{
	U16 precharge_voltage = 0;

	precharge_voltage = ReadAnalogValue( PrechargeAnalogInputs[relay_id] );
	return precharge_voltage;
}

U16 GetHumidity()
{
	return ReadAnalogValue(HUMIDITY_ANALOG);
}

U16 GetTemp()
{
	return ReadAnalogValue(NTC_ANALOG);
}

U16 GetPackVoltage()
{
	return ReadAnalogValue(BATT_ANALOG);
}

U16 GetCurrent( U8 id )
{
	U16 current = 0;
	current = ReadWordSpi(id); //this function takes 1 or 2 right now.
	return current;
}

void SetRelay( WriteDataStruct* write_data )
{
	//if the pack is too low, you can't turn on master power.
	if( (write_data->RelayStates[0] == TRUE) && ( ReadData.PackVoltage >= MIN_PACK_VOLTAGE ) )
		RELAY1_ON;
	else
		RELAY1_OFF;

	if( write_data->RelayStates[1] == TRUE )
		RELAY2_ON;
	else
		RELAY2_OFF;

	if( write_data->RelayStates[2] == TRUE )
		RELAY3_ON;
	else
		RELAY3_OFF;

	if( write_data->RelayStates[3] == TRUE )
		RELAY4_ON;
	else
		RELAY4_OFF;

	if( write_data->RelayStates[4] == TRUE )
		RELAY5_ON;
	else
		RELAY5_OFF;
}

void SetPrecharge( WriteDataStruct* write_data )
{
	//if the pack is too low, you can't turn on master power.
	if( ( write_data->RelayPrechargeStates[0] == TRUE ) && ( ReadData.PackVoltage >= MIN_PACK_VOLTAGE ) )
		PRE_CHARGE_1_ON;
	else
		PRE_CHARGE_1_OFF;

	if( write_data->RelayPrechargeStates[1] == TRUE )
		PRE_CHARGE_2_ON;
	else
		PRE_CHARGE_2_OFF;

	if( write_data->RelayPrechargeStates[2] == TRUE )
		PRE_CHARGE_3_ON;
	else
		PRE_CHARGE_3_OFF;

	if( write_data->RelayPrechargeStates[3] == TRUE )
		PRE_CHARGE_4_ON;
	else
		PRE_CHARGE_4_OFF;

	if( write_data->RelayPrechargeStates[4] == TRUE )
		PRE_CHARGE_5_ON;
	else
		PRE_CHARGE_5_OFF;
}

void DelayMs(U16 ms)
{
	while(ms--)
	{
		__delay_cycles(8000);
	}

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

	DISABLE_RX_INTERRUPT();


	TimerEventBmsSerial();



	ENABLE_RX_INTERRUPT();
	return;

}

#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void)
{
	DISABLE_RX_INTERRUPT();
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


	if( CriticalConditionFlags != 0 )
	{
		RED_ON;
		if( ( (Seconds & 0x00000001) == 0) && ( WriteData.DisableBuzzer != TRUE ) ) //(use != true in case of data corruption)
		{
			BUZZ_TOGGLE;
		}
		else
		{
			BUZZ_OFF;
		}
	}
	else
	{
		RED_OFF;
	}



	//GREEN_OFF;
	if( Ms == 1000 )
	{
		Ms = 0;
		Seconds++;
		IdlePeriodCounterS++;
		PcIdlePeriodCounterS++;
		if( WatchDogCounterS > 0 )
		{
			WatchDogCounterS -= 1;
			if(WatchDogCounterS == 0 )
			{
				//the watchdog has expored. If we are in between a transmit and a RX, we need to abort it, kill the P1 IE, and set the state to idle.

				BmsAbortWatchdog = TRUE;
				RxWaitExpired();
			}
		}
	}
	ENABLE_RX_INTERRUPT();
}

U8 SerialRxBuffer[43];
U8 SerialRxPos = 0;

const U8 WriteDataSize = sizeof(WriteDataStruct);

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	U8 value = UCA0RXBUF;
	U8 i;
	U8* write_data_cast;
	//DISABLE_RX_INTERRUPT();
	_enable_interrupts();
	_nop();

	write_data_cast = (U8*)&WriteData;

	SerialRxBuffer[SerialRxPos] = value;

	SerialRxPos++;

	if( SerialRxPos >= WriteDataSize+1 )
	{
		SerialRxPos = 0;
		//done!
		//check checksum.
		if( SerialRxBuffer[WriteDataSize] != 0xDE )
		{
			goto exit_serial_isr;
		}
	}
	else
	{
		goto exit_serial_isr;
	}

	//load in the data!!
	for( i = 0; i < WriteDataSize; ++i)
		write_data_cast[i] = SerialRxBuffer[i];


	//tx back the data structure!

	WriteBufferSerial( (U8*)&ReadData, sizeof(ReadDataStruct) );

	while( PcIdlePeriodCounterS != 0)
		PcIdlePeriodCounterS = 0;

exit_serial_isr:
	//_disable_interrupts();
	_nop();

	//ENABLE_RX_INTERRUPT();

}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{

	EdgeEventIsr();

	P1IE = 0; //disable interrupt for ISR
	_NOP();
	P1IFG = 0; //clear IF flag too.
	_NOP();
}

#pragma vector=PORT2_VECTOR, ADC10_VECTOR, USCIAB0TX_VECTOR, TIMERA1_VECTOR, WDT_VECTOR, TIMERB1_VECTOR, NMI_VECTOR//, RESET_VECTOR
__interrupt void TRAP_ISR(void)
{
	GREEN_ON;
}

