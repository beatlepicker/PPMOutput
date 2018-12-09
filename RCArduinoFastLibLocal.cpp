


/*****************************************************************************************************************************
// RCArduinoSerialServos by DuaneB is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
//
// http://rcarduino.blogspot.com
//
*****************************************************************************************************************************/

#include "RCArduinoFastLibLocal.h"

/*----------------------------------------------------------------------------------------

This is essentially a derivative of the Arduino Servo Library created by Michael Margolis

As the technique is very similar to the Servo class, it can be useful to study in order
to understand the servo class.

What does the library do ? It uses a very inexpensive and common 4017 Counter IC
To generate pulses to independently drive up to 10 servos from two Arduino Pins

As previously mentioned, the library is based on the techniques used in the Arduino Servo
library created by Michael Margolis. This means that the library uses Timer1 and Timer1 output
compare register A.

2016-01-25 TS: OCR1A is linked to digital pin 9 and so we use digital pin 9 to generate the clock signal
for the 4017 counter.
2016-02-14 TS: Pin 12 is used as the reset pin. But Pin 12 is NOT toggled in this code?

2016-02-14 TS: RESOURCES: Timer1, and OCR1A (linked to D9 on UNO) Timer1 output compare register A.

*/

void CRCArduinoFastServos::setup()
{
 m_sCurrentOutputChannelA = 0;
 while(m_sCurrentOutputChannelA < RC_CHANNEL_OUT_COUNT)
 {
  m_ChannelOutA[m_sCurrentOutputChannelA].m_unPulseWidth = microsecondsToTicks(RCARDUINO_SERIAL_SERVO_MAX); 

#if defined (MORE_SERVOS_PLEASE)
  m_ChannelOutB[m_sCurrentOutputChannelA].m_unPulseWidth = microsecondsToTicks(RCARDUINO_SERIAL_SERVO_MAX); 
#endif

   m_sCurrentOutputChannelA++;
  }
}


// Timer1 Output Compare A interrupt service routine
// call out class member function OCR1A_ISR so that we can
// access out member variables
ISR(TIMER1_COMPA_vect)
{
	CRCArduinoFastServos::OCR1A_ISR();
}

void CRCArduinoFastServos::OCR1A_ISR()
{
static uint16_t unPulseState =  0;

	// If the channel number is >= 10, we need to reset the counter
	// and start again from zero.
	// to do this we pulse the reset pin of the counter
	// this sets output 0 of the counter high, effectivley
	// starting the first pulse of our first channel

//  PORTB &= 0xFD; // 2016-02-07 TS: Use bitwise-AND to ensure bit PORTB bit B1 is FALSE
  //PORTB^=2; // 2016-02-07 TS: Use bitwise-XOR to ensure bit PORTB bit B1 is TOGGLED

#if(0)  
  static uint16_t uiDelay = 0;
  for(uiDelay = 0; uiDelay < 100; uiDelay++) // 2016-02-13 TS: was < 600
  {
    static uint16_t uiDelay2 = 0; 
    uiDelay2++;
  }
#endif

//  PORTB |= 0x02; // 2016-02-07 TS: Use bitwise-OR to ensure bit PORTB bit B1 is TRUE
  
  
  if(m_sCurrentOutputChannelA >= RC_CHANNEL_OUT_COUNT)
  {
	// reset our current servo/output channel to 0
        m_sCurrentOutputChannelA = 0;
        unPulseState = 0;
#if(0)        
        for(uiDelay = 0; uiDelay < 13000; uiDelay++) // FRAME MARK delay
			  {
			    static uint16_t uiDelay2 = 0; 
			    uiDelay2++;
			  }
#endif
  }

	if (unPulseState == 0)
	{
		unPulseState = 1; // Toggle
		
#if defined(UNO_328)
		PORTB &= 0xFD; // 2016-02-07 TS: Use bitwise-AND to ensure bit PORTB bit B1 (D9) is FALSE
#endif		
		
#if defined(ETHERMEGA_2560)
		PORTB &= 0xDF; // 2016-02-14 TS: Use bitwise-AND to ensure bit PORTB bit B5 (D11) is FALSE
#endif
		
		OCR1A = TCNT1 + 400; // front porch low time in us
//		OCR1A = TCNT1 + 1000; // make it easy to see on scope
	}	
	else
	{
		unPulseState = 0; // Toggle
		
#if defined(UNO_328)
		PORTB |= 0x02; // 2016-02-07 TS: Use bitwise-OR to ensure bit PORTB bit B1 (D9) is TRUE
#endif		
		
#if defined(ETHERMEGA_2560)
		PORTB |= 0x20; // 2016-02-14 TS: Use bitwise-OR to ensure bit PORTB bit B5 (D11) is TRUE 
#endif
     
  	// set the duration of the output pulse
  	CRCArduinoFastServos::setOutputTimerForPulseDurationA();
  	// done with this channel so move on.
  	m_sCurrentOutputChannelA++;
  }
  
  // PORTB^=2; // 2016-02-07 TS: Use bitwise-XOR to ensure bit PORTB bit B1 is TOGGLED
}

void CRCArduinoFastServos::setChannelPinLowA(uint8_t sChannel)
{
	volatile CPortPin *pPortPin = m_ChannelOutA + sChannel;

	if(pPortPin->m_sPinMask)
	 *pPortPin->m_pPort ^= pPortPin->m_sPinMask;
}

void CRCArduinoFastServos::setCurrentChannelPinHighA()
{
	volatile CPortPin *pPortPin = m_ChannelOutA + m_sCurrentOutputChannelA;

	if(pPortPin->m_sPinMask)
	 *pPortPin->m_pPort |= pPortPin->m_sPinMask;
}

// After we set an output pin high, we need to set the timer to comeback for the end of the pulse 
void CRCArduinoFastServos::setOutputTimerForPulseDurationA()
{
  OCR1A = TCNT1 + m_ChannelOutA[m_sCurrentOutputChannelA].m_unPulseWidth; 
}

#if defined(MORE_SERVOS_PLEASE)
// Timer1 Output Compare B interrupt service routine
// call out class member function OCR1B_ISR so that we can
// access out member variables
ISR(TIMER1_COMPB_vect)
{
	CRCArduinoFastServos::OCR1B_ISR();
}

void CRCArduinoFastServos::OCR1B_ISR()
{
  PORTB|=4; // 2016-02-07 TS: Use bitwise-OR to ensure bit PORTB bit B2 is TRUE

  if(m_sCurrentOutputChannelB >= RC_CHANNEL_OUT_COUNT)
  {
	// reset our current servo/output channel to 0
    m_sCurrentOutputChannelB = 0;
  }

  // set the duration of the output pulse
  CRCArduinoFastServos::setOutputTimerForPulseDurationB();

  PORTB^=4; // 2016-02-07 TS: Use bitwise-XOR to ensure bit PORTB bit B2 is TOGGLED
  
  // done with this channel so move on.
  m_sCurrentOutputChannelB++;
}

void CRCArduinoFastServos::setChannelPinLowB(uint8_t sChannel)
{
	volatile CPortPin *pPortPin = m_ChannelOutB + sChannel;

	if(pPortPin->m_sPinMask)
	 *pPortPin->m_pPort ^= pPortPin->m_sPinMask;
}

void CRCArduinoFastServos::setCurrentChannelPinHighB()
{
	volatile CPortPin *pPortPin = m_ChannelOutB + m_sCurrentOutputChannelB;
	
	if(pPortPin->m_sPinMask)
	 *pPortPin->m_pPort |= pPortPin->m_sPinMask;
}



// After we set an output pin high, we need to set the timer to comeback for the end of the pulse 
void CRCArduinoFastServos::setOutputTimerForPulseDurationB()
{
  OCR1B = TCNT1 + m_ChannelOutB[m_sCurrentOutputChannelB].m_unPulseWidth; 
}
#endif

// updates a channel to a new value, the class will continue to pulse the channel
// with this value for the lifetime of the sketch or until writeChannel is called
// again to update the value
void CRCArduinoFastServos::writeMicroseconds(uint8_t nChannel,uint16_t unMicroseconds)
{
	// dont allow a write to a non existent channel
	if(nChannel > RCARDUINO_MAX_SERVOS)
		return;

  // constraint the value just in case
// 2016-02-13 TS: unconstrain:  unMicroseconds = constrain(unMicroseconds,RCARDUINO_SERIAL_SERVO_MIN,RCARDUINO_SERIAL_SERVO_MAX);

#if defined(MORE_SERVOS_PLEASE)
  if(nChannel >= RC_CHANNEL_OUT_COUNT)
  {
    unMicroseconds = microsecondsToTicks(unMicroseconds);
	unsigned char sChannel = nChannel-RC_CHANNEL_OUT_COUNT;
	// disable interrupts while we update the multi byte value output value
	uint8_t sreg = SREG;
	cli();
	  
	m_ChannelOutB[sChannel].m_unPulseWidth = unMicroseconds; 

	// enable interrupts
	SREG = sreg;
    return;
  }
#endif
  
  unMicroseconds = microsecondsToTicks(unMicroseconds);
  
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutA[nChannel].m_unPulseWidth = unMicroseconds; 

  // enable interrupts
  SREG = sreg;
}

uint16_t CRCArduinoFastServos::ticksToMicroseconds(uint16_t unTicks)
{
	return unTicks / 2;
}

uint16_t CRCArduinoFastServos::microsecondsToTicks(uint16_t unMicroseconds)
{
 return unMicroseconds * 2;
}

void CRCArduinoFastServos::attach(uint8_t sChannel,uint8_t sPin)
{
	if(sChannel >= RCARDUINO_MAX_SERVOS)
		return;

  #if defined(MORE_SERVOS_PLEASE)
  if(sChannel >= RC_CHANNEL_OUT_COUNT)
  {
	// disable interrupts while we update the multi byte value output value
	uint8_t sreg = SREG;
	cli();
	  
	m_ChannelOutB[sChannel-RC_CHANNEL_OUT_COUNT].m_unPulseWidth = microsecondsToTicks(RCARDUINO_SERIAL_SERVO_DEFAULT);
        m_ChannelOutB[sChannel-RC_CHANNEL_OUT_COUNT].m_pPort = getPortFromPin(sPin);
	m_ChannelOutB[sChannel-RC_CHANNEL_OUT_COUNT].m_sPinMask = getPortPinMaskFromPin(sPin);
 
	// enable interrupts
	SREG = sreg;
	pinMode(sPin,OUTPUT);
	return;
  }
  #endif
  
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutA[sChannel].m_unPulseWidth = microsecondsToTicks(RCARDUINO_SERIAL_SERVO_DEFAULT);
  m_ChannelOutA[sChannel].m_pPort = getPortFromPin(sPin);
  m_ChannelOutA[sChannel].m_sPinMask = getPortPinMaskFromPin(sPin);

  // enable interrupts
  SREG = sreg;
  
  pinMode(sPin,OUTPUT);
}

// this allows us to run different refresh frequencies on channel A and B
// for example servos at a 70Hz rate on A and ESCs at 250Hz on B
void CRCArduinoFastServos::setFrameSpaceA(uint8_t sChannel,uint16_t unMicroseconds)
{
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutA[sChannel].m_unPulseWidth = microsecondsToTicks(unMicroseconds);
  m_ChannelOutA[sChannel].m_pPort = 0;
  m_ChannelOutA[sChannel].m_sPinMask = 0;

  // enable interrupts
  SREG = sreg;
}

#if defined (MORE_SERVOS_PLEASE)
void CRCArduinoFastServos::setFrameSpaceB(uint8_t sChannel,uint16_t unMicroseconds)
{
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutB[sChannel].m_unPulseWidth = microsecondsToTicks(unMicroseconds);
  m_ChannelOutB[sChannel].m_pPort = 0;
  m_ChannelOutB[sChannel].m_sPinMask = 0;

  // enable interrupts
  SREG = sreg;
}
#endif

// Easy to optimise this, but lets keep it readable instead, its short enough.
volatile uint8_t* CRCArduinoFastServos::getPortFromPin(uint8_t sPin)
{
	volatile uint8_t* pPort = RC_CHANNELS_NOPORT;

#if(1) // 2016-02-14 TS:	
	if(sPin <= 7)
	{
		pPort = &PORTD;
	}
	else if(sPin <= 13)
	{
		pPort = &PORTB;
	}
	else if(sPin <= A5) // analog input pin 5
	{
		pPort = &PORTC;
	}
#else
//		pPort = &PORTD;  // 2016-02-14 aadst: AT Mega 328 (UNO)
		pPort = &PORTB;  // 2016-02-14 aadst: AT Mega 2560 (ETHER MEGA)
#endif
	
	return pPort;
}

// Easy to optimise this, but lets keep it readable instead, its short enough.
uint8_t CRCArduinoFastServos::getPortPinMaskFromPin(uint8_t sPin)
{
	uint8_t sPortPinMask = RC_CHANNELS_NOPIN;

#if(1) // 2016-02-14 TS:	
	if(sPin <= A5 && sPin != RCARDUINO_PPM_PIN)
	{
		if(sPin <= 7)
		{
			sPortPinMask = (1 << sPin);
		}
		else if(sPin <= 13)
		{
			sPin -= 8;
			sPortPinMask = (1 << sPin);
		}
		else if(sPin <= A5)
		{
			sPin -= A0;
			sPortPinMask = (1 << sPin);
		}
	}
#else
//		sPortPinMask = 1; // 2016-02-14 aadst: AT Mega 328 (UNO)
		sPortPinMask = 0x20; // 2016-02-14 aadst: AT Mega 2560 (ETHER MEGA)
#endif

	
	return sPortPinMask;
}	

void CRCArduinoFastServos::begin()
{
	TCNT1 = 0;              // clear the timer count   

	// Initilialise Timer1
	TCCR1A = 0;             // normal counting mode 
	TCCR1B = 2;     // set prescaler of 64 = 1 tick = 4us 

	// ENABLE TIMER1 OCR1A INTERRUPT to enabled the first bank (A) of ten servos
	TIFR1 |= _BV(OCF1A);     // clear any pending interrupts; 
	TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt  

#if defined(MORE_SERVOS_PLEASE)

	// ENABLE TIMER1 OCR1B INTERRUPT to enable the second bank (B) of 10 servos 
	TIFR1 |= _BV(OCF1B);     // clear any pending interrupts; 
	TIMSK1 |=  _BV(OCIE1B) ; // enable the output compare interrupt  

#endif

	OCR1A = TCNT1 + 4000; // Start in two milli seconds
	
	for(uint8_t sServo = 0;sServo<RC_CHANNEL_OUT_COUNT;sServo++)
	{
		// 2018-11-02 remove Serial.println(m_ChannelOutA[sServo].m_unPulseWidth);

#if defined(MORE_SERVOS_PLEASE)
		Serial.println(m_ChannelOutB[sServo].m_unPulseWidth);
#endif
		}
}

volatile CRCArduinoFastServos::CPortPin CRCArduinoFastServos::m_ChannelOutA[RC_CHANNEL_OUT_COUNT]; 
uint8_t CRCArduinoFastServos::m_sCurrentOutputChannelA;

#if defined(MORE_SERVOS_PLEASE)	
volatile CRCArduinoFastServos::CPortPin CRCArduinoFastServos::m_ChannelOutB[RC_CHANNEL_OUT_COUNT]; 
uint8_t CRCArduinoFastServos::m_sCurrentOutputChannelB;
#endif

volatile uint16_t CRCArduinoPPMChannels::m_unChannelSignalIn[RC_CHANNEL_IN_COUNT];
uint8_t CRCArduinoPPMChannels::m_sCurrentInputChannel = 0;

uint16_t CRCArduinoPPMChannels::m_unChannelRiseTime = 0;
uint8_t volatile CRCArduinoPPMChannels::m_sOutOfSynchErrorCounter = 0;

void CRCArduinoPPMChannels::begin()
{
 m_sOutOfSynchErrorCounter = 0;

 // enable the interrupt
 EICRA |= ((1 << ISC00)|(1<<ISC01));    // set INT0 to trigger on ANY logic change
 EIMSK |= (1 << INT0);     // Turns on INT0
}

#if(0) // 2016-01-30 TS: Moved to .h file to workaround compiler error.
// we could save a few micros by writting this directly in the signal handler rather than using attach interrupt
void CRCArduinoPPMChannels::INT0ISR()
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

// if we force a resynch we set the channel
void CRCArduinoPPMChannels::forceResynch() 
{ 
	m_sCurrentInputChannel = RC_CHANNEL_IN_COUNT;
	
	if(m_sOutOfSynchErrorCounter<255)
	 m_sOutOfSynchErrorCounter++; 
}

uint8_t CRCArduinoPPMChannels::getSynchErrorCounter() 
{ 
  uint8_t sErrors = m_sOutOfSynchErrorCounter;
  
  m_sOutOfSynchErrorCounter = 0;
  
  return sErrors;
}

uint16_t CRCArduinoPPMChannels::getChannel(uint8_t sChannel)
{
 uint16_t ulPulse;
 unsigned char sreg = SREG;
 
 cli();
 
 ulPulse = m_unChannelSignalIn[sChannel];
 m_unChannelSignalIn[sChannel] = 0;
  
 SREG = sreg;
 
 return ulPulse>>1;
}


