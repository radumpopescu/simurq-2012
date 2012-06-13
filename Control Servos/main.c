// This file is part of the MatrixPilot RollPitchYaw demo.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


// main program for testing the IMU.


#include "../libDCM/libDCM.h"


// Used for serial debug output
#include "stdio.h"
#include <string.h>

#define BYTECIR_TO_DEGREE 92160		// (360.0/256 * 2^16)

char debug_buffer[128] ;
int db_index = 0 ;
void send_debug_line( void ) ;
int servoOutputs(int value, int sMin, int sMax, int dMin, int dMax, int channel, boolean inverse);
void decodeString(void);
void copyStrings(void);


char serString[20];
char serFinal[20];
int serLen=0,finalLen=0;
int dataa[5];


int main (void)
{
	// Set up the libraries
	udb_init() ;
	dcm_init() ;
	
	udb_serial_set_rate(57600) ;
	
	LED_GREEN = LED_OFF ;
	
	// Start it up!
	udb_run() ;  // This never returns.
	
	return 0 ;
}

// Called every time we get gps data (1, 2, or 4 Hz, depending on GPS config)
void dcm_callback_gps_location_updated(void)
{
	// Blink GREEN led to show that the GPS is communicating
	udb_led_toggle(LED_GREEN) ;
	return ;
}


// Called every 1/2 second at high priority
void udb_background_callback_periodic(void)
{

	
	return ;
}


boolean tempI=true;
int cha=1;
// Called at 40 Hz, before sending servo pulses
void dcm_servo_callback_prepare_outputs(void)
{
	// if (udb_heartbeat_counter % 2 == 0)
	// {
		// udb_led_toggle(LED_GREEN);
		db_index = 0 ;
		decodeString();
		if (dataa[4]==1){// 
		
		struct relative2D matrix_accum ;
		long earth_pitch ;		// pitch in binary angles ( 0-255 is 360 degreres)
		long earth_roll ;		// roll of the plane with respect to earth frame
		long earth_yaw ;		// yaw with respect to earth frame
		
		//  Roll
		//  Earth Frame of Reference
		matrix_accum.x = rmat[8] ;
		matrix_accum.y = rmat[6] ;
		earth_roll = rect_to_polar(&matrix_accum) ;					// binary angle (0 - 256 = 360 degrees)
		earth_roll = (-earth_roll * BYTECIR_TO_DEGREE) >> 16 ;		// switch polarity, convert to -180 - 180 degrees
	
		//  Pitch
		//  Earth Frame of Reference
		//  Note that we are using the matrix_accum.x
		//  left over from previous rect_to_polar in this calculation.
		//  so this Pitch calculation must follow the Roll calculation
		matrix_accum.y = rmat[7] ;
		earth_pitch = rect_to_polar(&matrix_accum) ;				// binary angle (0 - 256 = 360 degrees)
		earth_pitch = (-earth_pitch * BYTECIR_TO_DEGREE) >> 16 ;	// switch polarity, convert to -180 - 180 degrees
		
		// Yaw
		// Earth Frame of Reference
		// Ardustation does not use yaw in degrees
		matrix_accum.x = rmat[4] ;
		matrix_accum.y = rmat[1] ;
		earth_yaw = rect_to_polar(&matrix_accum) ;				// binary angle (0 - 256 = 360 degrees)
		earth_yaw = (earth_yaw * BYTECIR_TO_DEGREE) >> 16 ;		// switch polarity, convert to -180 - 180 degrees
		
		int servo1=servoOutputs(earth_roll+180,0,360,2000,4000,1,false);
		int servo2=servoOutputs(earth_pitch+180,0,360,2000,4000,2,false);
		int servo3=servoOutputs(earth_yaw+180,0,360,2000,4000,3,false);
		sprintf( debug_buffer ,"%i - %i - %i\n", servo1,servo2,servo3);
		}
		/*
		Channel   LOCATION
		   1         S1
		   2         S2
		   3         S3
		   4         R3
		   5         R2
		   6         R1 
		*/
		
		
		else {
		
		int servo1=servoOutputs(dataa[0],0,180,2000,4000,1,false);
		int servo2=servoOutputs(dataa[1],0,180,2000,4000,2,false);
		int servo3=servoOutputs(dataa[2],0,180,2000,4000,3,false);
		int servo4=servoOutputs(dataa[3],0,100,2000,4000,4,false);
		
		sprintf( debug_buffer ,"%s\n%i - %i - %i - %i - %i\n", serFinal, servo1, servo2, servo3, servo4,dataa[4]);
		}
		 udb_serial_start_sending_data() ;
	// }
	if (udb_heartbeat_counter % 30 ==0)
		udb_led_toggle(LED_GREEN);
		if (udb_heartbeat_counter % 30 ==5)
		udb_led_toggle(LED_GREEN);
	if (udb_heartbeat_counter % 25 ==0)
		udb_led_toggle(LED_RED);
	
	
	return ;
 }

int oldServoValue[7];

int servoOutputs(int value, int sMin, int sMax, int dMin, int dMax, int channel, boolean inverse){
	if (inverse){
		value=sMin+sMax-value;
	}
	int output = (int)(dMin + (float)(value - sMin) * (float)(dMax - dMin) / (float)sMax);
	
	if (oldServoValue[channel]!=output){
		udb_pwOut[channel]=output;
		oldServoValue[channel]=output;
	}
	return output;
}


// Prepare a line of serial output and start it sending
void send_debug_line( void )
{
	db_index = 0 ;
	sprintf( debug_buffer , "lat: %li, long: %li, alt: %li\r\nrmat: %i, %i, %i, %i, %i, %i, %i, %i, %i\r\n" , 
		lat_gps.WW , long_gps.WW , alt_sl_gps.WW , 
		rmat[0] , rmat[1] , rmat[2] , 
		rmat[3] , rmat[4] , rmat[5] , 
		rmat[6] , rmat[7] , rmat[8]  ) ; 
	
	udb_serial_start_sending_data() ;
	
	return ;
}


// Return one character at a time, as requested.
// Requests will stop after we send back a -1 end-of-data marker.
int udb_serial_callback_get_byte_to_send(void)
{
	unsigned char c = debug_buffer[ db_index++ ] ;
	
	if (c == 0) return -1 ;
	
	return c ;
}

void copyStrings(void){
	int i;
	for (i=0;i<serLen;i++){
		serFinal[i]=serString[i];
	}
	finalLen=serLen;
}
char tempStr[10];
int tempLen=0;

void decodeString(void){
	int j,k;
	k=0;
	for(j=0; j<finalLen; j++){
		if(serFinal[j]==','){
			dataa[k]=0;
			j+=1;
			while (j<finalLen && serFinal[j]!=','){
					dataa[k]*=10;
					dataa[k]+=(int)serFinal[j]-48;
					j++;
			}
			k++;
			j--;
			}
	}	
}

// Don't respond to serial input
void udb_serial_callback_received_byte(char rxchar)
{
	db_index = 0 ;

	if (rxchar!='\n'){
		serString[serLen++]=rxchar;
	}
	else{
		serString[serLen++]='\n';
		serString[serLen++]='\0';
		copyStrings();
		serLen=0;
	}

	
	
	// sprintf( debug_buffer ,"%c\n",rxchar);
	
	 // udb_serial_start_sending_data() ;
	return ;
}


void udb_callback_radio_did_turn_off( void ) {}
