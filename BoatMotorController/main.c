#include <msp430.h> 
#include "BoatTypes.h"
#include "BoatPeripherals.h"
/*
 * main.c
 * NTC - (P1.0, A0)
 *
 * HUMIDITY p1.5 A5
 * Vref (1.2V) P1.4 A4
 *
 * SCL - p1.6 UCB0SCL
 * SDA - p1.7 UCB0SDA
 *
 * MCU RX - P1.1 UCA0RX
 * MCU TX P1.2 USC0TX
 *
 * ENC V p2.0
 * ENC U p2.1
 * ENC W p2.4
 *
 * BUZZ p2.6 - TA0.1
 *
 * YELLOW p3.1
 * GREEN p3.0
 * RED LED p1.3
 *
 * FWD 3.2
 * REV p2.2
 * KEY_IN p3.3
 *
 *
 * Commands:
 * Set Speed:
 * CMD_SET_SPEED HIGH_BYTE LOW_BYTE
 * CMD_SET_DIR DIR
 * CMD_GET_TEMP .... HIGH_BYTE LOW_BYTE
 * CMD_GET_HUMIDITY ...  HIGH_BYTE LOW_BYTE
 * CMD_GET_KEY ... KEY
 *
 */

#define YELLOW_BIT	BIT1
#define GREEN_BIT BIT0
#define RED_BIT BIT3

#define YELLOW_PORT P3OUT
#define GREEN_PORT P3OUT
#define RED_PORT P1OUT

#define YELLOW_ON	YELLOW_PORT &= ~YELLOW_BIT
#define GREEN_ON	GREEN_PORT &= ~GREEN_BIT
#define RED_ON	RED_PORT &= ~RED_BIT

#define YELLOW_OFF	YELLOW_PORT |= YELLOW_BIT
#define GREEN_OFF	GREEN_PORT |= GREEN_BIT
#define RED_OFF	RED_PORT |= RED_BIT

#define YELLOW_TOGGLE	GREEN_PORT ^= YELLOW_BIT
#define GREEN_TOGGLE	YELLOW_PORT ^= GREEN_BIT
#define RED_TOGGLE	RED_PORT ^= RED_BIT

#define FWD_PORT	P3OUT
#define REV_PORT	P2OUT

#define FWD_BIT		BIT2
#define REV_BIT		BIT2

#define FORWARD 0x01
#define REVERSE 0x00
#define NO_DIRECTION 0x02

#define SET_FWD	FWD_PORT |= FWD_BIT
#define CLEAR_FWD FWD_PORT &= ~FWD_BIT

#define SET_REV	REV_PORT |= REV_BIT
#define CLEAR_REV REV_PORT &= ~REV_BIT

#define KEY_PORT	P3OUT
#define KEY_BIT		BIT3
#define KEY_ON		P3OUT |= KEY_BIT
#define KEY_OFF		P3OUT &= ~KEY_BIT

#define BUZZ_PORT	P2OUT
#define BUZZ_BIT	BIT6
#define BUZZ_ON		BUZZ_PORT |= BUZZ_BIT
#define BUZZ_OFF	BUZZ_PORT &= ~BUZZ_BIT
#define BUZZ_TOGGLE BUZZ_PORT ^= BUZZ_BIT


U16 GetHumidity();
U16 GetTemp();
void SetDac( U16 value );
void InitDac();

bool Buzz = FALSE;

int main(void) {

    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    //setup clock for 16MHz.
    DCOCTL = CALDCO_16MHZ;
    BCSCTL1 = CALBC1_16MHZ;

    //setup LED pins, DIR pins, and key pin.
    P3DIR |= YELLOW_BIT | GREEN_BIT | FWD_BIT | KEY_PORT;
    P2DIR |= REV_BIT | BUZZ_BIT;
    P1DIR |= RED_BIT;

    P2SEL &= ~BIT6;
    P2SEL2 &= ~BIT6;
    P2SEL2 &= ~BIT7;

    YELLOW_OFF;
    GREEN_OFF;
    RED_OFF;

    KEY_OFF;
    //KEY_OFF;

    BUZZ_OFF;

	CLEAR_FWD;
	CLEAR_REV;


    InitSerial();
    InitI2C( 0x60 );

    //init timer A for 100hz  SMCLK is running at 16MHz.
    CCTL0 = CCIE;
    CCR0 = 20000;
    TACTL = TASSEL_2 + MC_1 + ID_3;  //TASSEL_2=SMCLK, MC_1=Up mode: the timer counts up to TACCR0, ID_3=div by 8

    __bis_SR_register( GIE );
    InitDac();
    InitAdc(BIT0 | BIT5 | BIT4);
    SetDac(0);



    while(1)
    {

    	/*unsigned int j = 0;
    	//int i;

    	j = GetTemp();

    	WriteByteSerial(j >> 8);
    	WriteByteSerial(j);


    	j = GetHumidity();

    	WriteByteSerial(j >> 8);
    	WriteByteSerial(j);

    	SetDac(i << 4);
    	YELLOW_TOGGLE;
    	i++;
    	for( j = 0; j < 60000; ++j );
    	for( j = 0; j < 60000; ++j );
    	for( j = 0; j < 60000; ++j );*/
    }

	return 0;
}

void SetDirection( U8 direction )
{

	//obviously active low, but since we're driving mosfets, we're inverted. 1 = on. (for us, active high)

	CLEAR_FWD;
	CLEAR_REV;


	if( direction == FORWARD )
	{
		SET_FWD;
		//YELLOW_ON;
	}
	else if( direction == REVERSE )
	{
		SET_REV;
		//YELLOW_ON;
	}
	else if( direction == NO_DIRECTION )
	{
		//YELLOW_OFF;
	}
}

U16 GetHumidity()
{
	return ReadAnalogValue(INCH_5);
}

U16 GetTemp()
{
	return ReadAnalogValue(INCH_0);
}


//DAC is 12 bit.
void SetDac( U16 value )
{
	U8 buffer[2];

	if( value > 0x0FFF )
		value = 0x0FFF;


	buffer[0] = value >> 8;
	buffer[1] = value;
	WriteMultipleI2c( 0x60, buffer, 2 );

}

void InitDac()
{
	WriteByteI2c( 0x60, 0x98 );
}

//timer A running at 100Hz.

U16 Ms = 0;
U32 Seconds = 0;
U16 WatchDogCounter = 0;
// 5 seconds at 100Hz = 500
#define WATCHDOG_LIMIT	500

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{

	Ms += 10;

	if( Buzz == TRUE )
	{
		BUZZ_TOGGLE;
	}

	if( Ms == 1000 )
	{
		Ms = 0;
		Seconds++;
	}

	if( Ms % 500 == 0)
	{
		RED_TOGGLE;
	}

	WatchDogCounter++;
	if( WatchDogCounter >= WATCHDOG_LIMIT )
	{
		//kill motor!
		//beep!
		SetDac( 0 );
		YELLOW_ON;
		//Buzz = TRUE;
	}
	else
	{
		//Buzz = FALSE;
		BUZZ_OFF;
	}

}

/*
* CMD_SET_SPEED HIGH_BYTE LOW_BYTE
* CMD_SET_DIR DIR
* CMD_GET_TEMP .... HIGH_BYTE LOW_BYTE
* CMD_GET_HUMIDITY ...  HIGH_BYTE LOW_BYTE
* CMD_GET_KEY ... KEY
*/
bool CmdInProgress = FALSE;
U8 SerialByteIndex = 0;
U8 CmdRxBuffer[4];

#define CMD_SET_SPEED	0x01 //length 3
#define CMD_SET_DIR		0x02 //length 2
#define CMD_GET_TEMP	0x03 //length 1
#define CMD_GET_HUMIDITY	0x04	 //length 1
#define CMD_SET_KEY		0x05	 //length 2
#define CMD_BEEP		0x06	//length 1
#define CMD_PING		0x07	//length 1
#define CMD_SET_LED		0x08	//length 4

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	U8 value;
	U16 temp;
	//RED_ON;
	value = UCA0RXBUF;
	CmdInProgress = TRUE;
	CmdRxBuffer[SerialByteIndex] = value;
	SerialByteIndex++;
	//WriteByteSerial(value);

	if( CmdRxBuffer[0] == CMD_SET_SPEED && SerialByteIndex < 3 )
		return;
	if( CmdRxBuffer[0] == CMD_SET_DIR && SerialByteIndex < 2 )
		return;
	if( CmdRxBuffer[0] == CMD_SET_LED && SerialByteIndex < 4 )
		return;
	if( CmdRxBuffer[0] == CMD_SET_KEY && SerialByteIndex < 2 )
		return;

	CmdInProgress = FALSE;
	SerialByteIndex = 0;

	//command finished!
	switch(CmdRxBuffer[0])
	{
	case CMD_SET_SPEED:
		WatchDogCounter = 0; //reset watchdog.
		if( (CmdRxBuffer[1] == 0 ) && (CmdRxBuffer[2] == 0) )
		{
			RED_OFF;
		}
		else
		{
			RED_ON;
		}
		SetDac( (U16)CmdRxBuffer[1] << 8 | CmdRxBuffer[2] );
		break;
	case CMD_SET_DIR:
		WatchDogCounter = 0; //reset watchdog.
		SetDirection( CmdRxBuffer[1] );
		break;
	case CMD_GET_TEMP:
		temp = GetTemp();
		WriteWordSerial(temp);
		break;
	case CMD_GET_HUMIDITY:
		temp = GetHumidity();
		WriteWordSerial(temp);
		break;
	case CMD_SET_KEY:
		WatchDogCounter = 0; //reset watchdog.
		if( CmdRxBuffer[1] == 0x55 )
		{
			KEY_ON;
			GREEN_ON;
		}
		else if( CmdRxBuffer[1] == 0x66)
		{
			KEY_OFF;
			GREEN_OFF;
		}
		break;
	case CMD_PING:
		WriteByteSerial(0xFF);
		WatchDogCounter = 0; //reset watchdog.
		YELLOW_OFF;
		break;
	case CMD_SET_LED:
		break;
	default:
		return;
	}


	return;
}
