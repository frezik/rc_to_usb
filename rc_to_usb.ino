/*
Copyright (c) 2015,  Timm Murray
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of 
      conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of 
      conditions and the following disclaimer in the documentation and/or other materials 
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// RCArduino MultiChannel Loop back and servo ESC control for upto 8 RC channels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>
#include <Servo.h>
#include <EEPROM.h>
#include "UnoJoy.h"

#define DEBUG 0

// Assign your channel in pins
#define CHANNEL1_IN_PIN 2
#define CHANNEL2_IN_PIN 3
#define CHANNEL3_IN_PIN 4
#define CHANNEL4_IN_PIN 5
#define CHANNEL5_IN_PIN 6
#define CHANNEL6_IN_PIN 7
#define CHANNEL7_IN_PIN 8
#define CHANNEL8_IN_PIN 9

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
/*
Servo servoChannel1;
Servo servoChannel2;
Servo servoChannel3;
Servo servoChannel4;
Servo servoChannel5;
Servo servoChannel6;
Servo servoChannel7;
Servo servoChannel8;
*/

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define CHANNEL3_FLAG 4
#define CHANNEL4_FLAG 8
#define CHANNEL5_FLAG 16
#define CHANNEL6_FLAG 32
#define CHANNEL7_FLAG 64
#define CHANNEL8_FLAG 128

// Highbyte/Lowbyte addresses in EEPROM for stick calibration data
#define LEFT_X_MAX_HB_ADDR 0x00
#define LEFT_X_MAX_LB_ADDR 0x01
#define LEFT_X_MIN_HB_ADDR 0x02
#define LEFT_X_MIN_LB_ADDR 0x03
#define LEFT_Y_MAX_HB_ADDR 0x04
#define LEFT_Y_MAX_LB_ADDR 0x05
#define LEFT_Y_MIN_HB_ADDR 0x06
#define LEFT_Y_MIN_LB_ADDR 0x07
#define RIGHT_X_MAX_HB_ADDR 0x08
#define RIGHT_X_MAX_LB_ADDR 0x09
#define RIGHT_X_MIN_HB_ADDR 0x0A
#define RIGHT_X_MIN_LB_ADDR 0x0B
#define RIGHT_Y_MAX_HB_ADDR 0x0C
#define RIGHT_Y_MAX_LB_ADDR 0x0D
#define RIGHT_Y_MIN_HB_ADDR 0x0E
#define RIGHT_Y_MIN_LB_ADDR 0x0F

#define INPUT_CALIBRATION_PIN 10


// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unChannel1InShared;
volatile uint16_t unChannel2InShared;
volatile uint16_t unChannel3InShared;
volatile uint16_t unChannel4InShared;
volatile uint16_t unChannel5InShared;
volatile uint16_t unChannel6InShared;
volatile uint16_t unChannel7InShared;
volatile uint16_t unChannel8InShared;


uint16_t left_stick_x_max = 2016;
uint16_t left_stick_x_min = 996;
uint16_t left_stick_y_max = 2016;
uint16_t left_stick_y_min = 996;
uint16_t right_stick_x_max = 2016;
uint16_t right_stick_x_min = 996;
uint16_t right_stick_y_max = 2016;
uint16_t right_stick_y_min = 996;

uint16_t cal_left_stick_x_max = 0;
uint16_t cal_left_stick_x_min = pow( 2, 16 );
uint16_t cal_left_stick_y_max = 0;
uint16_t cal_left_stick_y_min = pow( 2, 16 );
uint16_t cal_right_stick_x_max = 0;
uint16_t cal_right_stick_x_min = pow( 2, 16 );
uint16_t cal_right_stick_y_max = 0;
uint16_t cal_right_stick_y_min = pow( 2, 16 );


int calibration_input_prev_state = LOW;


void setup()
{
  Serial.begin(115200);

  if( DEBUG ) Serial.println("multiChannels");

  pinMode( INPUT_CALIBRATION_PIN, INPUT_PULLUP );

  setupUnoJoy();
  read_calibration_data();

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(CHANNEL1_IN_PIN, calcChannel1,CHANGE);
  PCintPort::attachInterrupt(CHANNEL2_IN_PIN, calcChannel2,CHANGE);
  PCintPort::attachInterrupt(CHANNEL3_IN_PIN, calcChannel3,CHANGE);
  PCintPort::attachInterrupt(CHANNEL4_IN_PIN, calcChannel4,CHANGE);
  PCintPort::attachInterrupt(CHANNEL5_IN_PIN, calcChannel5,CHANGE);
  PCintPort::attachInterrupt(CHANNEL6_IN_PIN, calcChannel6,CHANGE);
  PCintPort::attachInterrupt(CHANNEL7_IN_PIN, calcChannel7,CHANGE);
  PCintPort::attachInterrupt(CHANNEL8_IN_PIN, calcChannel8,CHANGE);
}


void loop()
{
    int calibration_input_state = digitalRead( INPUT_CALIBRATION_PIN );

    if( calibration_input_state &&
        (calibration_input_state == calibration_input_prev_state)
    ) {
        if( DEBUG ) Serial.println( "Setting controller data" );
        set_rx_data_to_usb();
    }
    else if( (! calibration_input_state) &&
        (calibration_input_state == calibration_input_prev_state)
    ) {
        if( DEBUG ) Serial.println( "Calibrating data" );
        set_rx_data_to_calibration();
    }
    else if( (! calibration_input_state) &&
        (calibration_input_state != calibration_input_prev_state)
    ) {
        if( DEBUG ) Serial.println( "Starting calibration" );
        clear_calibration_vars();
    }
    else {
        if( DEBUG ) {
            Serial.println( "Done calibrating, saving data" );
            Serial.print( "Left X Max: " );
            Serial.println( cal_left_stick_x_max );
            Serial.print( "Left X Min: " );
            Serial.println( cal_left_stick_x_min );
            Serial.print( "Left Y Max: " );
            Serial.println( cal_left_stick_y_max );
            Serial.print( "Left Y Min: " );
            Serial.println( cal_left_stick_y_min );

            Serial.print( "Right X Max: " );
            Serial.println( cal_right_stick_x_max );
            Serial.print( "Right X Min: " );
            Serial.println( cal_right_stick_x_min );
            Serial.print( "Right Y Max: " );
            Serial.println( cal_right_stick_y_max );
            Serial.print( "Right Y Min: " );
            Serial.println( cal_right_stick_y_min );
        }
        left_stick_x_max = cal_left_stick_x_max;
        left_stick_x_min = cal_left_stick_x_min;
        left_stick_y_max = cal_left_stick_y_max;
        left_stick_y_min = cal_left_stick_y_min;
        right_stick_x_max = cal_right_stick_x_max;
        right_stick_x_min = cal_right_stick_x_min;
        right_stick_y_max = cal_right_stick_y_max;
        right_stick_y_min = cal_right_stick_y_min;
        write_calibration_data();
    }

    calibration_input_prev_state = calibration_input_state;
}


/* Yeah, set_rx_data_to_usb() and set_rx_data_to_calibration() have a lot of 
 * redundant code.  They needed to take the same data (which has to be copies 
 * of data set by the interrupts) and then do different things.  Copy-and-paste 
 * was the easiest way to get something that works.
 *
 * Things work fine as is, so there's no rush to change it.  If it needs to 
 * be improved, then start by spinning off the redundant code and then work 
 * from there.
 */

void set_rx_data_to_usb()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unChannel1In;
  static uint16_t unChannel2In;
  static uint16_t unChannel3In;
  static uint16_t unChannel4In;
  static uint16_t unChannel5In;
  static uint16_t unChannel6In;
  static uint16_t unChannel7In;
  static uint16_t unChannel8In;

  uint8_t bUpdateFlags = 0;
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
  
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
  

    if(bUpdateFlags & CHANNEL1_FLAG)
    {
      unChannel1In = unChannel1InShared;
    }
  
    if(bUpdateFlags & CHANNEL2_FLAG)
    {
      unChannel2In = unChannel2InShared;
    }
  
    if(bUpdateFlags & CHANNEL3_FLAG)
    {
      unChannel3In = unChannel3InShared;
    }

    if(bUpdateFlags & CHANNEL4_FLAG)
    {
      unChannel4In = unChannel4InShared;
    }
  
    if(bUpdateFlags & CHANNEL5_FLAG)
    {
      unChannel5In = unChannel5InShared;
    }
  
    if(bUpdateFlags & CHANNEL6_FLAG)
    {
      unChannel6In = unChannel6InShared;
    }
   
    if(bUpdateFlags & CHANNEL7_FLAG)
    {
      unChannel7In = unChannel7InShared;
    }
  
    if(bUpdateFlags & CHANNEL8_FLAG)
    {
      unChannel8In = unChannel8InShared;
    }
  
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
  
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  dataForController_t controllerData = getBlankDataForController();
  uint8_t channel_mapped;
 
  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop
  if(bUpdateFlags & CHANNEL1_FLAG)
  {
      channel_mapped = map( unChannel1In, left_stick_x_min, left_stick_x_max, 0, pow(2, 8) - 1 );
      if( DEBUG ) {
          Serial.println("");
          Serial.print(bUpdateFlags);
          Serial.print(",");
          Serial.print(unChannel1In);
          Serial.print("(");
          Serial.print(channel_mapped);
          Serial.print(")");
          Serial.print(",");
      }

      controllerData.leftStickX = channel_mapped;
  }
 
  if(bUpdateFlags & CHANNEL2_FLAG)
  {
      channel_mapped = map( unChannel2In, left_stick_y_min, left_stick_y_max, 0, pow(2, 8) - 1 );
      if( DEBUG ) {
          Serial.print(unChannel2In);
          Serial.print("(");
          Serial.print(channel_mapped);
          Serial.print(")");
          Serial.print(",");
      }

      controllerData.leftStickY = channel_mapped;
  }
 
  if(bUpdateFlags & CHANNEL3_FLAG)
  {
      channel_mapped = map( unChannel3In, right_stick_x_min, right_stick_x_max, 0, pow(2, 8) - 1 );
      if( DEBUG ) {
          Serial.print(unChannel3In);
          Serial.print("(");
          Serial.print(channel_mapped);
          Serial.print(")");
          Serial.print(",");
          Serial.print(",");
      }
      
      controllerData.rightStickX = channel_mapped;
  }
 
  if(bUpdateFlags & CHANNEL4_FLAG)
  {
      channel_mapped = map( unChannel4In, right_stick_y_min, right_stick_y_max, 0, pow(2, 8) - 1 );
      if( DEBUG ) {
          Serial.print(unChannel4In);
          Serial.print("(");
          Serial.print(channel_mapped);
          Serial.print(")");
          Serial.print(",");
          Serial.print(",");
      }

      controllerData.rightStickY = channel_mapped;
  }
 
  if(bUpdateFlags & CHANNEL5_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel5In);
          Serial.print(",");
      }
  }
 
  if(bUpdateFlags & CHANNEL6_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel6In);
          Serial.print(",");
      }
  }
 
  if(bUpdateFlags & CHANNEL7_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel7In);
          Serial.print(",");
      }
  }
 
  if(bUpdateFlags & CHANNEL8_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel8In);
          Serial.print(",");
      }
  }

  setControllerData( controllerData );
}

void set_rx_data_to_calibration()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unChannel1In;
  static uint16_t unChannel2In;
  static uint16_t unChannel3In;
  static uint16_t unChannel4In;
  static uint16_t unChannel5In;
  static uint16_t unChannel6In;
  static uint16_t unChannel7In;
  static uint16_t unChannel8In;

  uint8_t bUpdateFlags = 0;
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
  
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
  

    if(bUpdateFlags & CHANNEL1_FLAG)
    {
      unChannel1In = unChannel1InShared;
    }
  
    if(bUpdateFlags & CHANNEL2_FLAG)
    {
      unChannel2In = unChannel2InShared;
    }
  
    if(bUpdateFlags & CHANNEL3_FLAG)
    {
      unChannel3In = unChannel3InShared;
    }

    if(bUpdateFlags & CHANNEL4_FLAG)
    {
      unChannel4In = unChannel4InShared;
    }
  
    if(bUpdateFlags & CHANNEL5_FLAG)
    {
      unChannel5In = unChannel5InShared;
    }
  
    if(bUpdateFlags & CHANNEL6_FLAG)
    {
      unChannel6In = unChannel6InShared;
    }
   
    if(bUpdateFlags & CHANNEL7_FLAG)
    {
      unChannel7In = unChannel7InShared;
    }
  
    if(bUpdateFlags & CHANNEL8_FLAG)
    {
      unChannel8In = unChannel8InShared;
    }
  
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
  
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop
  if(bUpdateFlags & CHANNEL1_FLAG)
  {
      if( DEBUG ) {
          Serial.println("");
          Serial.print(bUpdateFlags);
          Serial.print(",");
          Serial.print(unChannel1In);
          Serial.print(",");
      }
      if( cal_left_stick_x_max < unChannel1In ) 
          cal_left_stick_x_max = unChannel1In;
      if( cal_left_stick_x_min > unChannel1In )
          cal_left_stick_x_min = unChannel1In;
  }
 
  if(bUpdateFlags & CHANNEL2_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel2In);
          Serial.print(",");
      }

      if( cal_left_stick_y_max < unChannel2In ) 
          cal_left_stick_y_max = unChannel2In;
      if( cal_left_stick_y_min > unChannel2In )
          cal_left_stick_y_min = unChannel2In;
  }
 
  if(bUpdateFlags & CHANNEL3_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel3In);
          Serial.print(",");
      }
      
      if( cal_right_stick_x_max < unChannel3In ) 
          cal_right_stick_x_max = unChannel3In;
      if( cal_right_stick_x_min > unChannel3In )
          cal_right_stick_x_min = unChannel3In;
  }
 
  if(bUpdateFlags & CHANNEL4_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel4In);
          Serial.print(",");
      }

      if( cal_right_stick_y_max < unChannel4In ) 
          cal_right_stick_y_max = unChannel4In;
      if( cal_right_stick_y_min > unChannel4In )
          cal_right_stick_y_min = unChannel4In;
  }
 
  if(bUpdateFlags & CHANNEL5_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel5In);
          Serial.print(",");
      }
  }
 
  if(bUpdateFlags & CHANNEL6_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel6In);
          Serial.print(",");
      }
  }
 
  if(bUpdateFlags & CHANNEL7_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel7In);
          Serial.print(",");
      }
  }
 
  if(bUpdateFlags & CHANNEL8_FLAG)
  {
      if( DEBUG ) {
          Serial.print(unChannel8In);
          Serial.print(",");
      }
  }
}


void calcChannel1()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState) // this is equivalent to digitalRead(CHANNEL1_IN_PIN) but about 10 times faster
  {
    ulStart = micros();
  }
  else
  {
    unChannel1InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL1_FLAG;
  }
}

void calcChannel2()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel2InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL2_FLAG;
  }
}

void calcChannel3()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel3InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL3_FLAG;
  }
}

void calcChannel4()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel4InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL4_FLAG;
  }
}

void calcChannel5()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel5InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL5_FLAG;
  }
}

void calcChannel6()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel6InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL6_FLAG;
  }
}

void calcChannel7()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel7InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL7_FLAG;
  }
}

void calcChannel8()
{
  static uint32_t ulStart;
 
  if(PCintPort::pinState)
  {
    ulStart = micros();
  }
  else
  {
    unChannel8InShared = (uint16_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL8_FLAG;
  }
}


void read_calibration_data()
{
    left_stick_x_max = (EEPROM.read(LEFT_X_MAX_HB_ADDR) << 8)
        | EEPROM.read(LEFT_X_MAX_LB_ADDR);
    left_stick_x_min = (EEPROM.read(LEFT_X_MIN_HB_ADDR) << 8)
        | EEPROM.read(LEFT_X_MIN_LB_ADDR);

    left_stick_y_max = (EEPROM.read(LEFT_Y_MAX_HB_ADDR) << 8)
        | EEPROM.read(LEFT_Y_MAX_LB_ADDR);
    left_stick_y_min = (EEPROM.read(LEFT_Y_MIN_HB_ADDR) << 8)
        | EEPROM.read(LEFT_Y_MIN_LB_ADDR);


    right_stick_x_max = (EEPROM.read(RIGHT_X_MAX_HB_ADDR) << 8)
        | EEPROM.read(RIGHT_X_MAX_LB_ADDR);
    right_stick_x_min = (EEPROM.read(RIGHT_X_MIN_HB_ADDR) << 8)
        | EEPROM.read(RIGHT_X_MIN_LB_ADDR);

    right_stick_y_max = (EEPROM.read(RIGHT_Y_MAX_HB_ADDR) << 8)
        | EEPROM.read(RIGHT_Y_MAX_LB_ADDR);
    right_stick_y_min = (EEPROM.read(RIGHT_Y_MIN_HB_ADDR) << 8)
        | EEPROM.read(LEFT_Y_MIN_LB_ADDR);
}

void write_calibration_data()
{
    EEPROM.write( LEFT_X_MAX_HB_ADDR, (byte) (left_stick_x_max >> 8) );
    EEPROM.write( LEFT_X_MAX_LB_ADDR, (byte) (left_stick_x_max & 0xFF) );
    EEPROM.write( LEFT_X_MIN_HB_ADDR, (byte) (left_stick_x_min >> 8) );
    EEPROM.write( LEFT_X_MIN_LB_ADDR, (byte) (left_stick_x_min & 0xFF) );

    EEPROM.write( LEFT_Y_MAX_HB_ADDR, (byte) (left_stick_y_max >> 8) );
    EEPROM.write( LEFT_Y_MAX_LB_ADDR, (byte) (left_stick_y_max & 0xFF) );
    EEPROM.write( LEFT_Y_MIN_HB_ADDR, (byte) (left_stick_y_min >> 8) );
    EEPROM.write( LEFT_Y_MIN_LB_ADDR, (byte) (left_stick_y_min & 0xFF) );


    EEPROM.write( RIGHT_X_MAX_HB_ADDR, (byte) (right_stick_x_max >> 8) );
    EEPROM.write( RIGHT_X_MAX_LB_ADDR, (byte) (right_stick_x_max & 0xFF) );
    EEPROM.write( RIGHT_X_MIN_HB_ADDR, (byte) (right_stick_x_min >> 8) );
    EEPROM.write( RIGHT_X_MIN_LB_ADDR, (byte) (right_stick_x_min & 0xFF) );

    EEPROM.write( RIGHT_Y_MAX_HB_ADDR, (byte) (right_stick_y_max >> 8) );
    EEPROM.write( RIGHT_Y_MAX_LB_ADDR, (byte) (right_stick_y_max & 0xFF) );
    EEPROM.write( RIGHT_Y_MIN_HB_ADDR, (byte) (right_stick_y_min >> 8) );
    EEPROM.write( RIGHT_Y_MIN_LB_ADDR, (byte) (right_stick_y_min & 0xFF) );
}

void clear_calibration_vars()
{
    cal_left_stick_x_max = 0;
    cal_left_stick_x_min = pow( 2, 16 );
    cal_left_stick_y_max = 0;
    cal_left_stick_y_min = pow( 2, 16 );
    cal_right_stick_x_max = 0;
    cal_right_stick_x_min = pow( 2, 16 );
    cal_right_stick_y_max = 0;
    cal_right_stick_y_min = pow( 2, 16 );
}

void update_calibration_vars( dataForController_t controllerData )
{
    if( controllerData.leftStickX > cal_left_stick_x_max )
        cal_left_stick_x_max = controllerData.leftStickX;
    if( controllerData.leftStickX < cal_left_stick_x_min )
        cal_left_stick_x_min = controllerData.leftStickX;

    if( controllerData.leftStickY > cal_left_stick_y_max )
        cal_left_stick_y_max = controllerData.leftStickY;
    if( controllerData.leftStickX < cal_left_stick_y_min )
        cal_left_stick_y_min = controllerData.leftStickY;


    if( controllerData.rightStickX > cal_right_stick_x_max )
        cal_right_stick_x_max = controllerData.rightStickX;
    if( controllerData.rightStickX < cal_right_stick_x_min )
        cal_right_stick_x_min = controllerData.rightStickX;

    if( controllerData.rightStickY > cal_right_stick_y_max )
        cal_right_stick_y_max = controllerData.rightStickY;
    if( controllerData.rightStickX < cal_right_stick_y_min )
        cal_right_stick_y_min = controllerData.rightStickY;
}
