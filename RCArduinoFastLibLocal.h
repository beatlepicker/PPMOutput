/*****************************************************************************************************************************
// RCArduinoChannels by DuaneB is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
//
// http://rcarduino.blogspot.com
//
*****************************************************************************************************************************/
#include <Arduino.h>

#if(0)
#define UNO_328 1 // 2016-02-14 TS: CHOOSE_BOARD
#else
#define ETHERMEGA_2560 1 // 2016-02-14 TS: CHOOSE_BOARD
#endif

//#define RC_CHANNEL_OUT_COUNT 4 // 2016-01-30 TS_SERVO_CHANNELS:
//#define RC_CHANNEL_OUT_COUNT 6 // 2016-01-30 TS_SERVO_CHANNELS: Added: 2 more channels.
#define RC_CHANNEL_OUT_COUNT 7 // 2016-02-13 TS_SERVO_CHANNELS: Added: Another 1 more channel.

#if(0)
#define RCARDUINO_PPM_PIN 9 // 2016-01-30 TS_PPM_OUTPUT: MODIFIED:
#else
#define RCARDUINO_PPM_PIN 0 // 2016-01-30 TS_PPM_OUTPUT:
#endif

#if(0)
#define TS_PPM_OUTPUT_PIN 13
#else
#define TS_PPM_OUTPUT_PIN 9
#endif

//#define RC_CHANNEL_IN_COUNT 3
#define RC_CHANNEL_IN_COUNT 6 // 2016-01-30 TS_SERVO_CHANNELS: MODIFIED: Increased by 3 
// two ticks per us, 3000 us * 2 ticks = 6000 minimum frame space
//#define MINIMUM_FRAME_SPACE 6000
#define MINIMUM_FRAME_SPACE 6000 // 2016-01-30 TS_SERVO_CHANNELS: MODIFIED:
#define MAXIMUM_PULSE_SPACE 5000

// 2016-01-25 TS: Minimum and Maximum servo pulse widths, you could change these, 
// Check the servo library and use that range if you prefer
#define RCARDUINO_SERIAL_SERVO_MIN 1000
#define RCARDUINO_SERIAL_SERVO_MAX 2000
#define RCARDUINO_SERIAL_SERVO_DEFAULT 1500

#define RC_CHANNELS_NOPORT 0
#define RC_CHANNELS_PORTB 1
#define RC_CHANNELS_PORTC 2
#define RC_CHANNELS_PORTD 3
#define RC_CHANNELS_NOPIN 255


// COMMENT OR UNCOMMENT THIS LINE TO ENABLE THE SECOND BANK OF SERVOS
//#define MORE_SERVOS_PLEASE 1

// the first bank of servos uses OC1A - this will disable PWM on digital pin 9 - a small price for 10 fast and smooth servos
// the second bank of servos uses OC1B - this will disable PWM on digital pin 10 - a small price for 10 more fast and smooth servos

// The library blindly pulses all ten servos one and after another
// If you change the RC_CHANNEL_OUT_COUNT to 4 servos, the library will pulse them more frequently than
// it can ten - 
// 2015-01-25 TS: 10 servos at 1500us = 15ms = 66Hz
// 4 Servos at 1500us = 6ms = 166Hz
// if you wanted to go even higher, run two servos on each timer
// 2 Servos at 1500us = 3ms = 333Hz
// 
// You might not want a high refresh rate though, so the setFrameSpace function is provided for you to 
// add a pause before the library begins its next run through the servos
// 2016-01-25 TS: for 50 hz, the pause should be to (20,000 - (RC_CHANNEL_OUT_COUNT * 2000))

#if defined (MORE_SERVOS_PLEASE)
#define RCARDUINO_MAX_SERVOS (RC_CHANNEL_OUT_COUNT*2)
#else
#define RCARDUINO_MAX_SERVOS (RC_CHANNEL_OUT_COUNT)
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CRCArduinoFastServos
//
// A class for generating signals in combination with a 4017 Counter
// 
// Output upto 10 Servo channels using just digital pins 9 and 12
// 9 generates the clock signal and must be connected to the clock pin of the 4017
// 12 generates the reset pulse and must be connected to the master reset pin of the 4017
//
// The class uses Timer1, as this prevents use with the servo library
// 2016-01-25 TS: The class uses pins 9 and 12
// The class does not adjust the servo frame to account for variations in pulse width,
// on the basis that many RC transmitters and receivers designed specifically to operate with servos
// 2016-01-25 TS: output signals between 50 and 100hz, this is the same range as the library
//
// 2016-01-25 TS: Use of an additional pin would provide for error detection, however using pin 12 to pulse master reset
// at the end of every frame means that the system is essentially self correcting
//
// Note
// This is a simplified derivative of the Arduino Servo Library created by Michael Margolis
// The simplification has been possible by moving some of the flexibility provided by the Servo library
// from software to hardware.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////

  
class CRCArduinoFastServos
{
public:
	static void setup();

	// configures timer1
	static void begin();

	// called by the timer interrupt service routine, see the cpp file for details.
	static void OCR1A_ISR();
	
#if defined(MORE_SERVOS_PLEASE)
	static void OCR1B_ISR();
#endif

	// called to set the pulse width for a specific channel, pulse widths are in microseconds - degrees are for wimps !
	static void attach(uint8_t nChannel,uint8_t nPin);
	static void writeMicroseconds(uint8_t nChannel,uint16_t nMicroseconds);
	static void setFrameSpaceA(uint8_t sChannel,uint16_t unMicroseconds);
	static void setFrameSpaceB(uint8_t sChannel,uint16_t unMicroseconds);
	
protected:
	class CPortPin
	{
		public:
			//uint8_t m_sPort;
			volatile unsigned char *m_pPort;
			uint8_t m_sPinMask;
			uint16_t m_unPulseWidth;
	};

	// this sets the value of the timer1 output compare register to a point in the future
	// based on the required pulse with for the current servo
	static void setOutputTimerForPulseDurationA() __attribute__((always_inline));
	
	
	static void setChannelPinLowA(uint8_t sChannel) __attribute__((always_inline));
	static void setCurrentChannelPinHighA();
		
		// Easy to optimise this, but lets keep it readable instead, its short enough.
	static volatile uint8_t*  getPortFromPin(uint8_t sPin) __attribute__((always_inline));
	static uint8_t getPortPinMaskFromPin(uint8_t sPin) __attribute__((always_inline));

	// Records the current output channel values in timer ticks
	// Manually set by calling writeChannel, the function adjusts from
	// user supplied micro seconds to timer ticks 
	volatile static CPortPin m_ChannelOutA[RC_CHANNEL_OUT_COUNT]; 
	// current output channel, used by the timer ISR to track which channel is being generated
	static uint8_t m_sCurrentOutputChannelA;
	
#if defined(MORE_SERVOS_PLEASE)
	// Optional channel B for servo number 10 to 19
	volatile static CPortPin m_ChannelOutB[RC_CHANNEL_OUT_COUNT]; 
	static uint8_t m_sCurrentOutputChannelB;
	static void setOutputTimerForPulseDurationB();
	
	static void setChannelPinLowB(uint8_t sChannel) __attribute__((always_inline));
	static void setCurrentChannelPinHighB() __attribute__((always_inline));
#endif

	// two helper functions to convert between timer values and microseconds
	static uint16_t ticksToMicroseconds(uint16_t unTicks) __attribute__((always_inline));
	static uint16_t microsecondsToTicks(uint16_t unMicroseconds) __attribute__((always_inline));
};


class CRCArduinoPPMChannels
{
public:
 static void begin();
 //static void INT0ISR() __attribute__((always_inline));
 static uint16_t getChannel(uint8_t nChannel);
 static uint8_t getSynchErrorCounter();
 static uint8_t getFrameCounter(); 

protected:
 static void forceResynch();

 static volatile uint16_t m_unChannelSignalIn[RC_CHANNEL_IN_COUNT];
 static uint8_t m_sCurrentInputChannel;

 static uint16_t m_unChannelRiseTime;
 static volatile uint8_t m_sOutOfSynchErrorCounter;
 static volatile uint8_t m_sFrameCounter;

#if(1)
// we could save a few micros by writting this directly in the signal handler rather than using attach interrupt
//void CRCArduinoPPMChannels::INT0ISR()
public:
static void INT0ISR() __attribute__((always_inline))
{
  // only ever called for rising edges, so no need to check the pin state
  
  // calculate the interval between this pulse and the last one we received which is recorded in m_unChannelRiseTime
  uint16_t ulInterval = TCNT1 - m_unChannelRiseTime;
  
  // if all of the channels have been received we should be expecting the frame space next, lets check it
  if(m_sCurrentInputChannel == RC_CHANNEL_IN_COUNT)
  {
  // we have received all the channels we wanted, this should be the frame space
  if(ulInterval < MINIMUM_FRAME_SPACE)
  {
   // it was not so we need to resynch
   forceResynch();
  }
  else
  {
    // it was the frame space, next interval will be channel 0
    m_sCurrentInputChannel = 0;
  }
  }
  else
  {
    // if we were expecting a channel, but found a space instead, we need to resynch
  if(ulInterval > MAXIMUM_PULSE_SPACE)
  {
    forceResynch();
  }
  else
  {
   // its a good signal, lets record it and move onto the next channel
   m_unChannelSignalIn[m_sCurrentInputChannel++] = ulInterval;
  }
  }
  // record the current time 
  m_unChannelRiseTime = TCNT1;  
} 
#endif
 
};




