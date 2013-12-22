#include <msp430.h> 
#include "../BoatMotorController/BoatTypes.h"
#include "../BoatMotorController/BoatPeripherals.h"
#include "CellBmsDefines.h"
#include "BmsSharedDefines.h"
#include "BmsSerial.h"
/*
 * main.c
 */

extern U8 Buffer[MAX_TX_LENGTH];
extern U16 RxPacketSize;


void SetDac( U16 value );
void InitDac();

U16 GetBattVoltage();
U16 GetTemperature();
void PassCommandUp( U8 length_up );
void ProcessCommand();
void InitAdcCell( U16 active_inputs );

U16 Ms = 0;
U16 Us = 0;
U16 WatchDogCounterS = 0;
U16 IdlePeriodCounterS = 0;
bool BmsAbortWatchdog = FALSE;
U8 tx_data[7] = {0x03, 0x01, 0x0F, 0xAA };
bool Discharging = FALSE;

U8 ConditionCritical = 0;
#define CONDITION_CRITICAL_VOLTAGE 1
#define CONDITION_CRITICAL_TEMP 2
#define CONDITION_CRITICAL_UNEXPECTED 4


int main(void)
{
	//WDTCTL = WDTPW | WDTCNTCL;	// clear WDT.  SMCLK as clock, divide by 32768.  244Hz.
	WDTCTL = WDTPW | WDTHOLD;


    DCOCTL = CALDCO_8MHZ,
    BCSCTL1 = CALBC1_8MHZ;

    //turn off XIN feature for GREEN led
    P2SEL &= ~BIT6;
    P2SEL2 &= ~BIT6;

    P2DIR |= YELLOW_BIT | GREEN_BIT | RED_BIT;
    P3DIR = TX_UP_PIN | TX_DOWN_PIN; //TX up& down, defined in BMS SERIAL.

    BmsSerialInit(); //just set both tx lines high.


    //init timer A for 10 khz  SMCLK is running at 8MHz.
    //no clock divider, 8MHz => 10KHz is 1/800. 8MHz => 1KHz is 1/8000
    CCTL0 = CCIE;
    CCR0 = 8000;
    TACTL = TASSEL_2 + MC_1 + ID_0;  //TASSEL_2=SMCLK, MC_1=Up mode: the timer counts up to TACCR0, ID_0=div by 1

    InitI2C( 0x60 );
    InitAdcCell( BIT0 | BIT1 ); //VBATT and NTC

    SetDac(0); //full scale range is 0 to 2.5V, 10 bit (1024)

    RED_OFF;
    YELLOW_OFF;
    GREEN_OFF;


    __bis_SR_register( GIE );


    //U16 batt_voltage = 0;
    //U16 temperature = 0;



    while(1)
    {
    	U8 rx_length = 0;
    	//U16 i;

    	//set these twice, just in case they are being modified by the timer.
    	while( WatchDogCounterS != 0 )
    		WatchDogCounterS = 0;

    	while( BmsAbortWatchdog != 0 )
    		BmsAbortWatchdog = FALSE;

    	//lets wait for a command from below.
    	while( rx_length == 0 )
    	{
			StartReadData(FALSE); //read from down.
			while( !CheckOperationComplete() );
			rx_length = RxPacketSize;
    	}

    	IdlePeriodCounterS = 0;
    	ProcessCommand();

    	/*tx_data[0] = CELL_CMD_PING;
    	tx_data[1] = 0;
    	WriteDataBmsSerial(tx_data,2,TRUE);
    	while(!CheckOperationComplete());
    	WatchDogCounterS = 3;
    	StartReadData(TRUE);
    	while(!CheckOperationComplete());
    	if( RxPacketSize != 0 )
    		RED_TOGGLE;

    	for( i = 0; i < 30000; ++i);*/

    }


	return 0;
}

void ProcessCommand()
{
	U8 cell_id_in = 0;
	//U8 i;
	U16 temperature;
	U16 batt_voltage;
	U16 dac_value;

	//possible errors: wrong length
	//no response from above.

	switch( Buffer[1] )
	{
	case CELL_CMD_PING:

		if( RxPacketSize != 3) //packet length, command, id.
			return;

		cell_id_in = Buffer[2];

		if( cell_id_in == 0 )
		{
			tx_data[0] = CELL_CMD_PING;
			tx_data[1] = 0x01; //PARAM_ON
			WriteDataBmsSerial(tx_data,2,FALSE);
			while( !CheckOperationComplete() );
			return;
		}

		cell_id_in--;

		tx_data[0] = CELL_CMD_PING;
		tx_data[1] = cell_id_in;
		PassCommandUp(2);
		break;

	case CELL_CMD_GET_TEMP:

		if( RxPacketSize != 3) //packet length, command, id.
			return;

		cell_id_in = Buffer[2];

		if( cell_id_in == 0 )
		{
			temperature = GetTemperature();

			if( ( temperature > CELL_MAX_TEMPERATURE ) || ( temperature < CELL_MIN_TEMPERATURE ) )  //30deg and 130deg
				ConditionCritical |= CONDITION_CRITICAL_TEMP;
			else
				ConditionCritical &= ~CONDITION_CRITICAL_TEMP;


			tx_data[0] = CELL_CMD_GET_TEMP;
			tx_data[1] = temperature >> 8;
			tx_data[2] = temperature & 0x00FF;

			WriteDataBmsSerial(tx_data,3,FALSE);
			while(!CheckOperationComplete());
			return;
		}

		cell_id_in--;

		tx_data[0] = CELL_CMD_GET_TEMP;
		tx_data[1] = cell_id_in;
		PassCommandUp(2);

		break;

	case CELL_CMD_GET_VOLTAGE:

		if( RxPacketSize != 3) //packet length, command, id.
			return;

		cell_id_in = Buffer[2];

		if( cell_id_in == 0 )
		{
			batt_voltage = GetBattVoltage();

			if( batt_voltage < 715 )  //about 2.8V
				ConditionCritical |= CONDITION_CRITICAL_VOLTAGE;
			else
				ConditionCritical &= ~CONDITION_CRITICAL_VOLTAGE;

			tx_data[0] = CELL_CMD_GET_VOLTAGE;
			tx_data[1] = batt_voltage >> 8;
			tx_data[2] = batt_voltage & 0x00FF;

			WriteDataBmsSerial(tx_data,3,FALSE);
			while(!CheckOperationComplete());
			return;
		}

		cell_id_in--;

		tx_data[0] = CELL_CMD_GET_VOLTAGE;
		tx_data[1] = cell_id_in;
		PassCommandUp(2);

		break;
	case CELL_CMD_SET_CURRENT:

		if( RxPacketSize != 5) //packet length, command, id, current_H, current_L
			return;

		cell_id_in = Buffer[2];

		if( cell_id_in == 0 )
		{
			dac_value = Buffer[3] << 8; //high
			dac_value |= Buffer[4]; //low

			if( dac_value != 0 )
				Discharging = TRUE;
			else
				Discharging = FALSE;

			SetDac( dac_value );
			//batt_voltage = GetBattVoltage();
			tx_data[0] = CELL_CMD_SET_CURRENT;
			tx_data[1] = 0x01; //PARAM ON

			WriteDataBmsSerial(tx_data,2,FALSE);
			while( !CheckOperationComplete() );

			return;
		}

		cell_id_in--;

		tx_data[0] = CELL_CMD_SET_CURRENT;
		tx_data[1] = cell_id_in;
		tx_data[2] = Buffer[3];
		tx_data[3] = Buffer[4];
		PassCommandUp(4);
		break;
	default:
	}
}

void PassCommandUp( U8 length_up )
{
	U8 i;
	WriteDataBmsSerial(tx_data,length_up,TRUE);
	while(!CheckOperationComplete());
	WatchDogCounterS = 5;
	StartReadData(TRUE);
	while(!CheckOperationComplete());

	if( RxPacketSize == 0 )
		return; //failure! no need to pass down, since we're a cell.

	for( i = 0; i < RxPacketSize-1; ++i)
		tx_data[i] = Buffer[i+1];

	WriteDataBmsSerial(tx_data,RxPacketSize-1,FALSE);
	while(!CheckOperationComplete());
}

U16 GetTemperature()
{
	return ReadAnalogValue(NTC_ANALOG);
}

U16 GetBattVoltage()
{
	return ReadAnalogValue(BATT_ANALOG);
}

//DAC is 10 bit.
void SetDac( U16 value )
{
	U8 buffer[2];

	if( value > 0x03FF )
		value = 0x03FF;

	value <<= 2;

	buffer[0] = value >> 8;
	buffer[1] = value;
	WriteMultipleI2c( 0x60, buffer, 2 );

}

void InitDac()
{
	WriteByteI2c( 0x60, 0x98 );
}

//special init function to use off-chip v-ref.
void InitAdcCell( U16 active_inputs )
{
	ADC10AE0 |= (active_inputs & 0x00FF);

	ADC10CTL0 = ADC10SHT_3 + ADC10ON + SREF_3 + REFBURST; 	//ADC10SHT_3=64 × ADC10CLKs, SREF_3=VR+ = buffered VEREF+ and VR- = VSS, reference in burst mode.

	ADC10CTL1 = INCH_1 + ADC10SSEL_3 + ADC10DIV_7; 			//INCH_1 is just a place holder here.
}


#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	//WDTCTL = WDTPW | WDTCNTCL;  //clear WDT.  Pretty crappy place to clear the WDT since everything else could be FUBAR and we could this timer could still get called.  Ideally we would catch a hang in the main loop.

	TimerEventBmsSerial();

	Us += 1000;

	if( Us == 1000)
	{
		Us = 0;
		Ms++;
	}

	if( (Ms > 0) && (Ms < 50))
	{
		GREEN_ON;
	}
	else
	{
		GREEN_OFF;
	}

	if( (Ms > 0) && (Ms < 500))
	{
		if( Discharging == TRUE )
			RED_ON;
	}
	else
	{
		RED_OFF;
	}



	if( IdlePeriodCounterS > CELL_MAX_REFESH_DELAY )
	{
		//stop charging.
		SetDac(0);
		Discharging = FALSE;

		YELLOW_ON;
	}
	else
	{
		YELLOW_OFF;
	}

	if( Ms >= 1000 )
	{
		Ms = 0;
		IdlePeriodCounterS++;

		if( WatchDogCounterS > 0 )
		{
			WatchDogCounterS -= 1;
			if(WatchDogCounterS == 0 )
			{
				BmsAbortWatchdog = TRUE;
			}
		}

	}

	if( ConditionCritical != 0 )
		RED_ON;
}

#pragma vector=PORT1_VECTOR, PORT2_VECTOR, ADC10_VECTOR, USCIAB0TX_VECTOR, NMI_VECTOR, WDT_VECTOR, COMPARATORA_VECTOR, TIMER0_A1_VECTOR, TIMER1_A0_VECTOR, TIMER1_A1_VECTOR, USCIAB0RX_VECTOR//, RESET_VECTOR
__interrupt void TRAP_ISR(void)
{
	ConditionCritical |= CONDITION_CRITICAL_UNEXPECTED;
	RED_ON;
}

