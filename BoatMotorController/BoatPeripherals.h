/*
 * BoatPeripherals.h
 *
 *  Created on: May 13, 2013
 *      Author: Mark
 */

#ifndef BOATPERIPHERALS_H_
#define BOATPERIPHERALS_H_

#include "BoatTypes.h"

void InitAdc( U16 active_inputs );
U16 ReadAnalogValue( U16 input );
void InitI2C( U8 slave_address );
void WriteByteI2c( U8 device_address, U8 data );
void WriteMultipleI2c( U8 device_address, U8* data, U8 data_length );
void InitSerial();
void WriteByteSerial(U8 data);
void WriteWordSerial(U16 word);
void WriteBufferSerial( U8* data, U16 length );
void InitSpi();
U16 ReadWordSpi( U8 cs_pin );

#endif /* BOATPERIPHERALS_H_ */
