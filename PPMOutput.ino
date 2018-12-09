#include "RCArduinoFastLibLocal.h"


// include the library code:
#include <LiquidCrystal.h>

#define TS_20181209

// initialize the library with the numbers of the interface pins
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
//  TS 2015-12-31 - LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );
 LiquidCrystal lcd( 8, A5, 4, 5, 6, 7 ); // Jaycar 16x2 LCD


// 2016-01-25 TS: Assign servo indexes - create one more servo than you need if you want a framespace on the end - its recommended to have one
#define SERVO_CHANNEL_0 0
#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3 // 2016-01-30 TS_SERVO_CHANNELS: ADDED:
#define SERVO_CHANNEL_4 4 // 2016-01-30 TS_SERVO_CHANNELS: ADDED:
#define SERVO_CHANNEL_5 5 // 2016-01-30 TS_SERVO_CHANNELS: ADDED:
#define SERVO_CHANNEL_6 6 // 2016-02-13 TS_SERVO_CHANNELS: ADDED:
//#define SERVO_CHANNEL_7 7 // 2016-02-13 TS_SERVO_CHANNELS: ADDED:
#define SERVO_FRAME_SPACE 7 // 2016-01-30 TS_SERVO_CHANNELS: MODIFIED:
//#define SERVO_FRAME_SPACE 3 // 2016-01-30 TS_SERVO_CHANNELS:

// 2016-01-25 TS: Arduino Hardware Resources
//

// 2016-01-31 TS_PINS::

// 2016-01-30 TS_PPM_INPUT_INTERRUPT::
// 2016-01-30 TS_PPM_INPUT_INTERRUPT: D2: INPUT: From RCArduino blog - INTERRUPT 0 = DIGITAL PIN 2 = D2
// 2016-01-30 TS_PPM_INPUT_INTERRUPT: A simple approach for reading three RC Channels using pin change interrupts
//
// 2016-01-25 TS: D9: OUTPUT: OCR1A is linked to digital pin 9 and so we use digital pin 9 to generate the clock signal for the 4017 counter.
// 2016-01-25 TS: D12: OUTPUT: Pin 12 is used as the reset pin for 4017 Decade Counter.
//
// 2016-01-30 TS_PPM_OUTPUT::
// 2016-01-30 TS_PPM_OUTPUT: RCARDUINO_PPM_PIN is a reserved value that tells the library not to attach to a pin, the PPM PIN is fixed as digital pin 9


// 2016-01-30 TS_SERVO_CHANNELS::
//
// 2016-02-07 TS: 2016-02-07 TS: means using D9 to clock an external 4017 decoded decage counter for up to 10 servo outputs and reading PPM stream input also.
// 2016-02-07 TS: the 1st bank of up to 10 servos uses OC1A - this will disable PWM on digital pin 9 D9 - a small price for 10 fast and smooth servos
// 2016-02-07 TS: the 2nd bank of up to 10 servos uses OC1B - this will disable PWM on digital pin 10 D10 - a small price for 10 more fast and smooth servos
//
//
//

 // MultiChannels
//
// rcarduino.blogspot.com
//
// 2016-01-30 TS_PPM_INPUT_INTERRUPT: A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
// 

char bufferNew[20];
char bufferNew2[20];
char serialRead = 'Z';
int chIndexX = 0;
int chIndexY = 1;
unsigned long ulMillis = millis();


// NEED TO BUILD FOR ATmega2560 !!! include and instantion are NOT required e.g. #include <HardwareSerial.h> // Serial Serial1;

#define LCD_SCROLLING
#ifdef LCD_SCROLLING

#define CIRCBUF_ROWS 3
#define CIRCBUF_COLS 16

char lcdCircBuf[CIRCBUF_ROWS][CIRCBUF_COLS]; // [rows] [cols] . Use line [0] as bottom line. Remainder is circ buf.
static int lcdCircBufIndex = 1;
static int line = 0;
static int scrollLine = 0;
//const int chHeight = 17;
#endif

void setup()
{


  Serial.begin(9600);  // Arduino IDE Serial  -  D0 = Rx,  D1 = Tx
  Serial1.begin(9600); // Arduino IDE Serial1 - D18 = Rx, D19 = Tx
 
  Serial.println("hello from setup()"); 
  Serial1.println("hello from setup()");

  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //         1234567890123456 
  lcd.print("PPMOutput 181209"); // **VERSION 2018-12-09
  lcd.setCursor(chIndexX,chIndexY);
  //lcd.print("setup lcd bottom line");

  lcd.setCursor(0, 0);
  //lcd.print("                ");
  lcd.setCursor(0, 1);
  //lcd.print("                ");

  static unsigned long ulLoopCount = 0;
 
  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers 
  // dont change
 
#if defined(TS_20181209)
  // 2016-01-30 TS_PPM_OUTPUT:: RCARDUINO_PPM_PIN is a reserved value that tells the library not to attach to a pin, the PPM PIN is fixed as digital pin 9
  CRCArduinoFastServos::attach(SERVO_CHANNEL_0,RCARDUINO_PPM_PIN);
  CRCArduinoFastServos::attach(SERVO_CHANNEL_1,RCARDUINO_PPM_PIN);
  CRCArduinoFastServos::attach(SERVO_CHANNEL_2,RCARDUINO_PPM_PIN);
  CRCArduinoFastServos::attach(SERVO_CHANNEL_3,RCARDUINO_PPM_PIN); // 2016-01-30 TS: ADDED:
  CRCArduinoFastServos::attach(SERVO_CHANNEL_4,RCARDUINO_PPM_PIN); // 2016-01-30 TS: ADDED:
  CRCArduinoFastServos::attach(SERVO_CHANNEL_5,RCARDUINO_PPM_PIN); // 2016-01-30 TS: ADDED:  
  CRCArduinoFastServos::attach(SERVO_CHANNEL_6,RCARDUINO_PPM_PIN); // 2016-02-13 TS: ADDED:  
//  CRCArduinoFastServos::attach(SERVO_CHANNEL_7,RCARDUINO_PPM_PIN); // 2016-02-13 TS: ADDED:  
#endif

#if defined(UNO_328)
//  pinMode(TS_PPM_OUTPUT_PIN,OUTPUT); // 2016-01-30 TS_PPM_OUTPUT:
  pinMode(9,OUTPUT); // 2016-02-14 TS_PPM_OUTPUT: UNO 328 D9 PB1
#endif

#if defined(ETHERMEGA_2560)
  pinMode(11,OUTPUT); // 2016-02-14 TS_PPM_OUTPUT: ETHER MEGA 2560 D11 PB5
#endif
  
  // 2016-01-25 TS: lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  // dont change
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,7*2000); // 2016-01-30 TS_SERVO_CHANNELS: Don't change as advised by RCArduino above

  Serial.println("bye from setup()");

  // start the servos, start reading the PPM input 2016-01-25 TS: reading?
  CRCArduinoFastServos::begin();
//  CRCArduinoPPMChannels::begin();
  
#if(1)
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_0,1000); // THROTTLE
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_1,1500); // AILERON
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_2,1000); // ELEVATOR
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_3,1500); // RUDDER
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_4,1500); // GEAR
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_5,1500); // AUX1
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_6,12000); // 6 ? Use as FRAME MARK time of 12 msec
//CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_7,1000000); // 7 ?
#endif

}

void loop()
{
#define LOOPCOUNT_MIN 400 // 2018-11-02 for approx 180 degs of servo travel.
	//#define LOOPCOUNT_MIN 1000

	//#define LOOPCOUNT_MAX 1500
#define LOOPCOUNT_MAX 2100 // 2018-11-02 for approx 180 degs of servo travel.

	static unsigned long ulLoopCount = LOOPCOUNT_MIN;
	static unsigned long ulLoopCountMod = LOOPCOUNT_MIN;
	static unsigned long ulLoopCountDelta = 5; // This is alternated between pos and neg.
	static unsigned long ulLast_Servo_Channel;
	static unsigned long ulTemp;
	static uint16_t unDegrees = 0;

	ulMillis = millis();


	//                  0123456789012345 i.e. 16 characters per line on 2-line Arduino LCD
	sprintf(bufferNew, "Pulse uS %.4d", ulLoopCountMod);
	lcd.setCursor(0, 1);
	lcd.print(bufferNew); // Sat hello on LCD bottom line 

	ulLoopCountMod = (ulLoopCount % LOOPCOUNT_MAX);

	if (ulLoopCountMod == LOOPCOUNT_MIN)
	{
//		sprintf(bufferNew2, "%.8u", ulMillis); // This prints as 16 bit number
//		Serial.print(bufferNew2);
		Serial.print(ulMillis);
		Serial.print(":  ulLoopCountMod: ");
		Serial.println(bufferNew);
	}

  ulLoopCount += ulLoopCountDelta; // Use as cheap timestamp.


#if(0)
  if (Serial1.available() > 0)
  {
	  serialRead = Serial1.read();
	  if ((serialRead == (char)0x0d) || (serialRead == (char)0x0a))
	  {
		  Serial1.println();
		  lcd.println();
		  chIndexX = 0; // increment ch x = column
		  // chIndexY++; // increment ch y = row
	  }
	  else
	  {
	  sprintf(bufferNew, "%c", serialRead);
	  lcd.setCursor(chIndexX, chIndexY);
	  lcd.print(bufferNew);
	  Serial1.print(bufferNew);
	  chIndexX++;
	  //Serial1.println(bufferNew);
	  }

  }


#endif


#if defined(TS_20181209)
 	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_0	,ulLoopCountMod); // THROTTLE
	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_1, ulLoopCountMod); // AILERON
	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_2, ulLoopCountMod); // ELEVATOR
	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_3, ulLoopCountMod); // RUDDER
	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_4, ulLoopCountMod); // GEAR
	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_5, ulLoopCountMod); // AUX
	CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_6, ulLoopCountMod); // FRAME SPACE
	//delay(1); // 2018-11-02 THIS IS TOO SMALL. delay in millisec
	delay(10); // delay in millisec
#endif

#if(1)
  uint16_t unServo_Channel_0 =  1000;
  uint16_t unServo_Channel_1 =  1000;
  uint16_t unServo_Channel_2 =  1000;
  uint16_t unServo_Channel_3 =  1000;
  uint16_t unServo_Channel_4 =  1000;
  uint16_t unServo_Channel_5 =  1000;
  uint16_t unServo_Channel_6 =  1000;
//  uint16_t unServo_Channel_7 =  1000; // 2016-02-13 TS: Use end channel as frame mark
#else
  uint16_t unServo_Channel_0 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_0);
  uint16_t unServo_Channel_1 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_1);
  uint16_t unServo_Channel_2 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_2);
  uint16_t unServo_Channel_3 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_3); // 2016-01-30 TS: Added:
  uint16_t unServo_Channel_4 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_4); // 2016-01-30 TS: Added:
  uint16_t unServo_Channel_5 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_5); // 2016-01-30 TS: Added:
  uint16_t unServo_Channel_6 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_6); // 2016-02-13 TS: Added:
//  uint16_t unServo_Channel_7 =  CRCArduinoPPMChannels::getChannel(SERVO_CHANNEL_7); // 2016-02-13 TS: Added:
#endif

#if(0)
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_0,1000);
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_1,1100);
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_2,1200);    
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_3,1300);
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_4,1400);
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_5,1500);
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_6,1500);
  CRCArduinoFastServos::writeMicroseconds(SERVO_CHANNEL_7,1500);
#endif

  if(unServo_Channel_0)
  {
//    Serial.print("Servo_Channel_0: "); // 2016-01-30 TS_PRINT:    
//    Serial.println(unServo_Channel_0);
  }
   
 
  if(unServo_Channel_1)
  {
//    ulLast_Servo_Channel = ulMillis;
//    Serial.print("Servo_Channel_1: "); // 2016-01-30 TS_PRINT:    
//    Serial.println(unServo_Channel_1);
  }

  if(unServo_Channel_2)
  {
//    Serial.print("Servo_Channel_2: "); // 2016-01-30 TS_PRINT:    
//    Serial.println(unServo_Channel_2);
  }

  if(unServo_Channel_3)
  {
//    Serial.print("Servo_Channel_3: "); // 2016-01-30 TS_PRINT:    
//    Serial.println(unServo_Channel_3);
  }

    if(unServo_Channel_4)
  {
//    Serial.print("Servo_Channel_4: "); // 2016-01-30 TS_PRINT:    
//    Serial.println(unServo_Channel_4);
  }

  if(unServo_Channel_5)
  {
//    Serial.print("Servo_Channel_5: "); // 2016-01-30 TS_PRINT:    
//    Serial.println(unServo_Channel_5);
  }

  if(unServo_Channel_6)
  {
	  ulLast_Servo_Channel = ulMillis;
	  //    Serial.print("Servo_Channel_6: "); // 2016-02-13 TS_PRINT:    
//    Serial.println(unServo_Channel_6);
  }

#if(0)
  if(unServo_Channel_7)
  {
//    Serial.print("Servo_Channel_7: "); // 2016-02-13 TS_PRINT:    
//    Serial.println(unServo_Channel_7);
  }
#endif
  
  // check for missed signals or glitches, glitches first
  uint8_t sErrorCounter = CRCArduinoPPMChannels::getSynchErrorCounter();
  if(sErrorCounter)
  {
    Serial.print(ulLoopCount); // 2016-01-30 TS_PRINT:     
    Serial.print(":   ERR: "); // 2016-01-30 TS_PRINT:    
    Serial.println(sErrorCounter);
  }
  
  // if no throttle ? signal for > 200ms - should really use AUX, its the last channel in the frame
  if((ulMillis - ulLast_Servo_Channel) > 200)
  {
   Serial.println("No Signal");
  }

  if (ulLoopCount <= LOOPCOUNT_MIN)
  {
	  ulLoopCountDelta = 5;
	  delay(2000);
  }

  if (ulLoopCount >= LOOPCOUNT_MAX)
  {
	  ulLoopCountDelta = -5;
	  delay(2000);
  }



}

ISR(INT0_vect) {
 CRCArduinoPPMChannels::INT0ISR(); // Do all the real-time RC multiple servo pulsing here via RCArduino interrupt
}

void ts_PrintLn(char* dstString)
{
	if (lcdCircBufIndex >= CIRCBUF_ROWS)
		lcdCircBufIndex = 1;
	else
		lcdCircBufIndex++;

	strcpy(lcdCircBuf[0], dstString); // always use bottom line in index 0

	strcpy(lcdCircBuf[lcdCircBufIndex], lcdCircBuf[0]); // copy bottom line to current circular index

//	tft.fillScreen(ILI9341_BLACK);
//         0123456789012345
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0, 1);
	lcd.print("                ");

	for (scrollLine = 0; scrollLine < CIRCBUF_ROWS; scrollLine++)
	{
		//tft.setCursor(0, scrollLine * chHeight);
		//lcd.setCursor(0, scrollLine);
		lcd.setCursor(0, (1 - scrollLine));

		//lcd.println(lcdCircBuf[((scrollLine + lcdCircBufIndex) % (CIRCBUF_ROWS)) + 1]); // print from circular buffer
		lcd.print(lcdCircBuf[((scrollLine + lcdCircBufIndex) % (CIRCBUF_ROWS)) + 1]); // print from circular buffer
	}
}

