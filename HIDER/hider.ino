/* Based on Heltec Automation Ping Pong communication test example
 *
 * Function:
 * 1. Hide and seek game. This is the "Hider" code.
 * 
 * Description:
 * 1. Only hardware layer communicate, no LoRaWAN protocol support;
 * 2. Hider unit broadcasts location (lon/lat). The seeker units will determine the bearing and distance to the hider.

*/

//#define _DEBUG

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include "GPS_Air530Z.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
extern SSD1306Wire  display;
//SSD1306Wire  display(0x3c, 500000, SDA,SCL, GEOMETRY_128_64, GPIO10 ); // addr , freq , i2c group , resolution , rst


#define MYNAME "Hider"

/*
 * define LoraWan_RGB as 1 to use the RGB (multicolor addressable) led. Note this is set in the LoRaWan_App library, so we don't need to define it here
 * in this code we will set:
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB //was not previously set
#define LoraWan_RGB 0
#endif

//the LoRa radio frequency is specific to location. In North America use this:
#define RF_FREQUENCY                                868000000 // Hz

/*
LoRa (LOng RAnge) protocol is optimized for range despite noise and movement/doppler effects. See
https://www.thethingsnetwork.org/docs/lorawan/what-is-lorawan/#:~:text=LoRa%20is%20a%20wireless%20modulation,be%20received%20across%20great%20distances.
for details on how LoRa works
*/

/* 
the following define transmission and receive settings. Ultimately, the tradeoff 
on these settings is transmission "speed" (Really bandwidth, radio waves have a 
set speed... basically the speed of light) and range. So transmitting very long 
range sacrifices message size.
Some of these parameters are for the transmitter, some for the receiver.
Note that any given LoRa radio can be either or both a transmitter or receiver...
but if there is no central server coordinating then messages can collide.
LoRaWAN is the spec whereby a central server collects and distributes messages,
ensuring receipt.
Here we are just going to shout messages at set intervals, hoping they are received
but not performing any check of receipt (should work because only 1 radio will be
transmitting - everyone else is just listening)
*/

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000

//GPS module is the Air530Z. Declare an Air530ZClass object called "GPS" 
Air530ZClass GPS;


//This struct will hold info from the GPS. The nice thing about this is we can transmit a GPSinfo object without having to combine multiple variables
struct GPSinfo {
  double LAT; //8 bytes
  double LNG; //8 bytes
  int ALT; //4 bytes
  float HDOP; //4 bytes
};
//note this struct is 24 bytes

//this union allows conversion of a GPSinfo object to a series of 24 bytes
union msg2byte {
 struct GPSinfo GPSbits;
 byte bytebits[24];
}; 


//this is just a bunch of timer info.. so we update everything at the appropriate time
struct TIMERSTRUCT {
  uint32_t lastGPScheck;
  byte intGPS;
  uint32_t lastBATcheck;
  byte intBat;
  uint32_t lastSCREEN;
  byte intSCREEN;
  uint32_t lastMSG;
  byte intMSG;
};

TIMERSTRUCT TIMERS;
byte BATPCNT; //battery percent
int16_t Rssi,rxSize; 


//function declarations (in C++ these are not required, but functions not declared have to be defined before being used)
void drawScreen(void);
byte checkBAT(void);
bool checkTIMERS(uint32_t lastsec, byte interval);
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

//take a fractional number (like 3.1415) and return the decimal part (1415)
//useful for display purposes
//note that n specifies the number of significant digits (so fracPart(3.1415,2) returns 14)
int fracPart(double val, int n)
{
  val = abs(val);
  return (int)((val - (int)(val))*pow(10,n));
}

//the GPS is powered by the Vext (voltage external) pin of the MCU. This is hardwired (we can't alter this). So powering the Vext pin turns on the GPS.
//note that setting the Vext to low (0 volts) turns off the external voltage supply!
void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

//Powering off the Vext pin turns off the GPS. Note that Vext is off when we write the Vext pin high!
void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void setup() {
  #ifdef _DEBUG
	  Serial.begin(115200);
  #endif
  VextON();
  delay(10);

  TIMERS = {0,10,0,60,0,5,0,10}; //see Timerstruct to understand what each array element refers to

  //set up the LCD display
  display.init();
  display.clear();
  display.display();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32-16/2, "GPS initing...");
  display.display();

  //set up and turn on GPS
  GPS.setmode(MODE_GPS_GLONASS);
  GPS.begin();

  display.clear();
  display.display();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32-16/2, "Radio Init...");
  display.display(); 

    Rssi=0;

  //set up LoRa radio
  //these define the functions to execute on each event
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
 
  
  display.clear();
  display.drawString(64, 32-16/2, "Setup Done...");
  display.display(); 
}

//the lorawan unit we are using has nice battery options... including a dedicated battery voltage regulator, battery charger (when power is available on USB), and a battery voltage measure
// Li battery voltage is related to charge level, and this table provides a good estimate for li ion (but not LiFePO4, NiCad, alkaline, etc) 
byte checkBAT(void)
{
  uint16_t BatteryLev = getBatteryVoltage();
  if (BatteryLev >= 4150) {return BATPCNT = 100;} //battery lev is mV
  if (BatteryLev >= 4110) {return BATPCNT = 95;}
  if (BatteryLev >= 4080) {return BATPCNT = 90;}
  if (BatteryLev >= 4020) {return BATPCNT = 85;}
  if (BatteryLev >= 3980) {return BATPCNT = 80;}
  if (BatteryLev >= 3950) {return BATPCNT = 75;}
  if (BatteryLev >= 3910) {return BATPCNT = 70;}
  if (BatteryLev >= 3870) {return BATPCNT = 65;}
  if (BatteryLev >= 3850) {return BATPCNT = 60;}
  if (BatteryLev >= 3840) {return BATPCNT = 55;}
  if (BatteryLev >= 3820) {return BATPCNT = 50;}
  if (BatteryLev >= 3800) {return BATPCNT = 45;}
  if (BatteryLev >= 3790) {return BATPCNT = 40;}
  if (BatteryLev >= 3770) {return BATPCNT = 35;}
  if (BatteryLev >= 3750) {return BATPCNT = 30;}
  if (BatteryLev >= 3730) {return BATPCNT = 25;}
  if (BatteryLev >= 3710) {return BATPCNT = 20;}
  if (BatteryLev >= 3690) {return BATPCNT = 15;}
  if (BatteryLev >= 3610) {return BATPCNT = 10;}
  if (BatteryLev >= 3500) {return BATPCNT = 5;}
  return BATPCNT = 0;

}

//this function performs the timer checks
bool checkTIMERS(uint32_t lastsec, byte interval) {
  
  if (millis() > (lastsec+interval*1000) )  return true; //the MCU tracks the milliseconds it has been on, which can be accessed using the millis() function. This is a 32 bit unsigned int, so the millisecond counter will not reset for many days
  else {
    if (lastsec>(2^32-interval*1000) )  return true;

    return false;
  }

}

//in arduino the setup function runs once, and then the "loop" function runs repeatedly thereafter. So the program design should provide the code for the loop to appropriately execute everything we want.
//here loop basically checks all the timers, and if any run out then do that action. for example, if the lastGPScheck timer hits 0, check the the GPS location. The most important one... if the lastMSG Timer
//runs out, send a new message.
void loop()
{
  if (TIMERS.lastBATcheck == 0 || checkTIMERS(TIMERS.lastBATcheck,TIMERS.intBat)) {
    TIMERS.lastBATcheck = millis();
    checkBAT();
  }
  if (TIMERS.lastGPScheck == 0 || checkTIMERS(TIMERS.lastGPScheck,TIMERS.intGPS)) {
    TIMERS.lastGPScheck = millis();
    while (GPS.available() > 0)
    {
      GPS.encode(GPS.read());
    }
  }
  if (checkTIMERS(TIMERS.lastMSG,TIMERS.intMSG)) {
    TIMERS.lastMSG= millis();
  
    //transmit location
      turnOnRGB(COLOR_SEND,0);
      union msg2byte GPScoord;
      GPScoord.GPSbits.LAT = GPS.location.lat();
      GPScoord.GPSbits.LNG = GPS.location.lng();
      GPScoord.GPSbits.ALT = (int)GPS.altitude.feet();
      GPScoord.GPSbits.HDOP = (float)GPS.hdop.hdop(); //hdop = horizontal degree of precision, a measure of the GPS accuracy in horizontal plane. There is also a vdop, which we don't care about.

      Radio.Send( (uint8_t *)GPScoord.bytebits, sizeof(GPScoord) );  
      #ifdef _DEBUG
      Serial.printf("Xmit %f\n",GPScoord.GPSbits.HDOP);
      #endif
  }
  if (checkTIMERS(TIMERS.lastSCREEN,TIMERS.intSCREEN)) {
    TIMERS.lastSCREEN= millis();
    drawScreen();
  }
}

//this function draws stuff to our screen. Note that the screen is small... 128 pixels wide and 64 pixels tall. A single letter is about 6x8 pixels.
//the library used here has some limited graphics capabilities (rectangles, circles, lines), and very limited animation.
//This MCU is moderately fast, and could do some nice animations... but we are not going to do that.
void drawScreen(void)
{
    
  char str[30];
  byte index;

// these are variables for the battery level icon (a clear rectangle) with a filled rectangle representing the voltage "left"
  byte batW = 10;
  byte batH=15;
  byte batOUTMargin = 2;
  byte batINMargin=2;
  byte batlev_px = (batH*BATPCNT/100);
  
  display.clear();

  display.drawRect(128-batOUTMargin-(batW + 2*batINMargin), batOUTMargin, batW+(2*batINMargin), batH+(2*batINMargin));
  if (BATPCNT>0) {
    display.fillRect(128-batOUTMargin-(batW + 2*batINMargin) + batINMargin,batOUTMargin+ batINMargin+batH-batlev_px, batW, batlev_px);
  }
  index = sprintf(str,"%d",BATPCNT);
  str[index] = 0;  
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 2*batOUTMargin+ 2*batINMargin+batH, str);



//    if( GPS.location.age() < 1000 )
  index = sprintf(str,"sat: %d, alt: %dft",(int)GPS.satellites.value(),(int)GPS.altitude.feet());    
  str[index] = 0;  
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, str);

  index = sprintf(str,"lon:%d.%d",(int)GPS.location.lng(),fracPart(GPS.location.lng(),6));
  str[index] = 0;
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 10, str);
  index = sprintf(str,"lat:%d.%d",(int)GPS.location.lat(),fracPart(GPS.location.lat(),6));
  str[index] = 0;
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 25, str);

  byte localhour=GPS.time.hour();
  if (localhour<4) {localhour = localhour+20;}
  else {localhour = localhour-4;}
  index = sprintf(str,"GPS %02d:%02d:%02d, HDOP=%f",localhour,GPS.time.minute(),GPS.time.second(),GPS.hdop.hdop());
  str[index] = 0;
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 44, str);

  index = sprintf(str,"Sent: %dsec ago",(byte)((millis()-TIMERS.lastMSG)/1000));
  str[index] = 0;
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 54, str);
  display.display();
  
  index = sprintf(str,MYNAME);
  str[index] = 0;
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 54, str);
  display.display();
  
}

 //do this when transmission is done
void OnTxDone( void )
{
  #ifdef _DEBUG
  	Serial.println("TX done......");
	#endif
  
  //turn off the RGB LED, put radio to sleep
  turnOnRGB(0,0);
  Radio.Sleep( );
	
}

void OnTxTimeout( void )
{
  #ifdef _DEBUG
    Serial.println("TX Timeout......");
	#endif

    //same as txdone, but if tx timed out turn on the RGB LED to yellow (turnonRGB has 2 inputs, the first is color as a 32 bit value... actually three back
    //to back 8 byte values... which is only 24 bits but there is no type of that size. here I am providing the hex value for yellow). 
  	turnOnRGB(0x00ffff00,0);
    Radio.Sleep( );
}

//actually the hider never receives a message, but just in case...
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Rssi=rssi;
    rxSize=size;
//    memcpy(rxpacket, payload, size );
    turnOnRGB(COLOR_RECEIVED,0);
    Radio.Sleep( );

}
