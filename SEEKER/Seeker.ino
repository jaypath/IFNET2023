/* Heltec Automation Ping Pong communication test example
 *
 * Function:
 * 1. Hide and seek game. SEEKER code.
 * 
 * Description:
 * 1. Only hardware layer communicate, no LoRaWAN protocol support;
 * 2. This is the code for the SEEKER. The Seeker receives GPS location from the hider (as a LoRa message
 *    with GPS lon and lat), but does not transmit anything.

 See HIDER code for comments/details!   

*/

//#define _DEBUG


#define MYNAME "Seeker"

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include "GPS_Air530Z.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
#include <math.h>
extern SSD1306Wire  display;
//SSD1306Wire  display(0x3c, 500000, SDA,SCL, GEOMETRY_128_64, GPIO10 ); // addr , freq , i2c group , resolution , rst

/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

//LoRa settings should match across radios

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



#define BUFFER_SIZE                                 20 // Define the payload size here

//if GPS module is Air530Z, use this
Air530ZClass GPS;

bool lora_idle = true;

struct GPSinfo {
  double LAT;
  double LNG;
  int ALT;
  float HDOP;
};

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



union msg2byte {
 struct GPSinfo GPSbits;
 byte bytebits[24];
}; 

TIMERSTRUCT TIMERS;
byte BATPCNT;
int16_t Rssi,rxSize;
union msg2byte GPScoord;

void drawScreen(void);
byte checkBAT(void);
bool checkTIMERS(uint32_t lastsec, byte interval);
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
//double haversine(void); //this is already a function in the gps library!      

int fracPart(double val, int n)
{
  return (int)((val - (int)(val))*pow(10,n));
}

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

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

  TIMERS = {0,1,0,60,0,1,0,5};

  display.init();
  display.clear();
  display.display();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32-16/2, "GPS initing...");
  display.display();
  GPS.setmode(MODE_GPS_GLONASS);

  GPS.begin();

  display.clear();
  display.display();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32-16/2, "Radio Init...");
  display.display(); 

    Rssi=0;

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

byte checkBAT(void)
{
  uint16_t BatteryLev = getBatteryVoltage();
  if (BatteryLev >= 4150) {return BATPCNT = 100;}
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

bool checkTIMERS(uint32_t lastsec, byte interval) {
  
  if (millis() > (lastsec+interval*1000) )  return true;
  else {
    if (lastsec>(2^32-interval*1000) )  return true;

    return false;
  }

}

void drawScreen(void)
{
  char str[30];
  byte index;

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


//GPS
  float hdop = (float) GPS.hdop.hdop();
  char qual[6];
  if (hdop<=1) index=sprintf(qual,"Exc");
  if (hdop>1 && hdop<=2) index = sprintf(qual,"Exc");
  if (hdop>2 && hdop<=5) index = sprintf(qual,"Good");
  if (hdop>5 && hdop<=10) index = sprintf(qual,"Mod");
  if (hdop>10 && hdop<=20) index = sprintf(qual,"Fair");
  if (hdop>20) index = sprintf(qual,"Poor");
  qual[index] = 0;  
  byte localhour=GPS.time.hour();
  if (localhour<4) {localhour = localhour+20;}
  else {localhour = localhour-4;}

  index = sprintf(str,"%02d:%02d:%02d SIG=%s",localhour,GPS.time.minute(),GPS.time.second(),qual);
  str[index] = 0;  
//Serial.println(str);

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, str);

  index = sprintf(str,"Heading: %ddeg",(int)GPS.course.deg());
  str[index] = 0;  
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 10, str);

  double hiderDIST = TinyGPSPlus::distanceBetween(GPS.location.lat(),GPS.location.lng(),GPScoord.GPSbits.LAT,GPScoord.GPSbits.LNG);
  double hiderCOURSE = TinyGPSPlus::courseTo(GPS.location.lat(),GPS.location.lng(),GPScoord.GPSbits.LAT,GPScoord.GPSbits.LNG);
  

  index = sprintf(str,"Hider dist @ bearing");
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 20, str);
  display.display();

  if (abs(hiderDIST)>9000)   index = sprintf(str,"??' @ ??");
  else index = sprintf(str,"%d' @ %d",(int)(hiderDIST*3.28),(int)hiderCOURSE);
  str[index] = 0;
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 35, str);

  index = sprintf(str,"%ds ago",(byte)((millis()-TIMERS.lastMSG)/1000));
  str[index] = 0;
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 40, str);
  display.display();

uint16_t RMShdop = sqrt((sq(hdop) + sq(GPScoord.GPSbits.HDOP))/2);
  #ifdef _DEBUG
    Serial.printf("RMSHDOP=%d, hiderD=%.1f,hiderC=%.1f\n",RMShdop,hiderDIST*3.28,hiderCOURSE);
  #endif
  index = sprintf(str,"+/-%dft",(int)(RMShdop*6*3.3)); //RMS HDOP * 4m *3.3 ~ accuracy in feet. 
  str[index] = 0;
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 50, str);
  display.display();


  
  if (false) {  
    index = sprintf(str,MYNAME);
    str[index] = 0;
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 54, str);
    display.display();
  }
}

void loop()
{

  if(lora_idle)
  {
  	turnOffRGB();
    lora_idle = false;
    #ifdef _DEBUG
      Serial.println("into RX mode");
    #endif

    Radio.Rx(0);
  }

  if (TIMERS.lastBATcheck == 0 || checkTIMERS(TIMERS.lastBATcheck,TIMERS.intBat)) {
    TIMERS.lastBATcheck =  millis();
    checkBAT();
  }
  if (TIMERS.lastGPScheck == 0 || checkTIMERS(TIMERS.lastGPScheck,TIMERS.intGPS)) {
    TIMERS.lastGPScheck =  millis();
    while (GPS.available() > 0)
    {
      GPS.encode(GPS.read());
    }
  }
  if (checkTIMERS(TIMERS.lastSCREEN,TIMERS.intSCREEN)) {
    TIMERS.lastSCREEN= millis();
    drawScreen();
  }

}


//note that the SEEKER does not transmit. so this function won't be called
void OnTxDone( void )
{
  #ifdef _DEBUG
  	Serial.print("TX done......");
	#endif
  turnOnRGB(0,0);
}

//Again, SEEKER does not transmit. so this function won't be called
void OnTxTimeout( void )
{
    Radio.Sleep( );
    #ifdef _DEBUG
      Serial.print("TX Timeout......");  
    #endif
}

//SEEKERs get messages, which entails reading the message buffer from the LoRa radio chip (using memcpy)
//COLOR_RECEIVED is a #define or possibly a global const uint32... but I'm not certain where it is defined
// - it is not in the LoRawan_app.cpp library file...
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    memcpy(GPScoord.bytebits,payload,size);
    #ifdef _DEBUG
      Serial.printf("\r\nreceived packet with rssi %d , length %d. LAT=%f and LNG=%f and ALT=%d and HDOP=%f\r\n",rssi,size,GPScoord.GPSbits.LAT,GPScoord.GPSbits.LNG,GPScoord.GPSbits.ALT,GPScoord.GPSbits.HDOP);
    #endif
    turnOnRGB(COLOR_RECEIVED,0);
    Radio.Sleep( ); 

    lora_idle=true;
    TIMERS.lastMSG = millis();
}

/*
for reference only. Here I have copied in a function that calculates the distance between 2 GPS points
(essentially, calculate the arc distance between two points on a globe). This method is called the 
haversine distance... but this entire function is commented out because it turns out the GPS library already
implemented it's own haversine equation - so need to waste RAM on a duplicate. 
double haversine(void) {
double lat1;
double lat2;
double lon1;
double lon2;
double latR1;
double latR2;
double lonR1;
double lonR2;
double dlon;
double dlat;
double a;
double e;
double d;
double R = 6371.00;
double toDegrees = 57.295779;
double KMtoFT = 3280.84;

  lon1 = GPS.location.lng();
  lat1 = GPS.location.lat();

  lon2 = hiderLNG;
  lat2 = hiderLAT;

  lonR1 = lon1*(PI/180);
  lonR2 = lon2*(PI/180);
  latR1 = lat1*(PI/180);
  latR2 = lat2*(PI/180);
  
  //This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
  dlon = lonR2 - lonR1;
  dlat = latR2 - latR1;
  
  //This portion is the Haversine Formula for distance between two points. Returned value is in KM
  a = (sq(sin(dlat/2))) + cos(latR1) * cos(latR2) * (sq(sin(dlon/2)));

  d = R*2*asin(sqrt(a));

  Serial.printf("a = %f, Dist = %f",a, d);
  return d* KMtoFT;


//ugh... I hate trig. I think this is the same as above. But I can't think through this. Note that I already returned distance, so this would not execute
  e = 2 * atan2(sqrt(a), sqrt(1-a)) ;
  d = R * e;

  //convert to ft
  d = d * KMtoFT;

return d;

  Serial.println();
  Serial.print("Distance to destination(ft): ");
  //Serial.println(a);
  //Serial.println(e);
  Serial.println(d, 6);
  Serial.println();
  
  //This portion is the Haversine Formula calculates the required bearing between current location and destination. Returned value is in Degrees
  double x = cos(latR2)*sin(lonR2-lonR1); //calculate x
  
  Serial.print("X = ");
  Serial.println(x, 6);

  double y = cos(latR1)*sin(latR2)-sin(latR1)*cos(latR2)*cos(lonR2-lonR1); //calculate y

  Serial.print("Y = ");
  Serial.println(y, 6);
  float brRad = atan2(x, y); //return atan2 result for bearing. Result at this point is in Radians

  Serial.print("atan2(x, y) (Radians) = ");
  Serial.println(brRad, 6);

  float reqBear = toDegrees*brRad;
  Serial.print("Bearing: ");
  Serial.println(reqBear, 4);

}
*/
