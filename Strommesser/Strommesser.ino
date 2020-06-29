/**
    Akkustation I2C SLAVE Modul
    04.05.2016  A.Fischer     Erste Version
    23.05.2016  A.Fischer     Die I2C-Adresse wird per 74hc165 + I2CADDRESS bestimmt
    01.07.2016  A.Fischer     CMD_GETVERSION Implementiert
    10.07.2016  A.Fischer     CMD_LEDON und CMD_LEDOFF, Faecher sind initial ROT
    17.08.2016  A.Fischer     12 LEDs und DIMM auf 0.8
    19.08.2016  A.Fischer     Bei Aufruf von getState() wird die lokale LED angeschaltet, wenn ein Geraet zum Laden angeschlossen ist
    23.08.2016  A.Fischer 102 Addressierung hartverdrahtet. Abfrage des Moduls deaktiviert.
    21.11.2016  A.Fischer 103 Watchdog Timer
    13.12.2016  D.Munke   104 Abfrage auf 74HC595 deaktiviert
    15.12.2016  A.Fischer 200 Vom Master kommen 2 Bytes: Der Befehl und die Zieladresse.
                              In receiveEvent wird gecheckt, ob die Adress des Moduls mit der
                              Adresse des Befehls uebereinstimmt.
                              wdt_Reset() in loop() bei LASTCOMMAND == NO_DEFINE

    26.02.2017  A.Fischer 201 Erweiterung um die Befehle fuer Fingerprint.
    02.04.2017  A.Fischer 202 bei aufruf von Getstate wird ein an A2 angeschlossener Stromsensor abgefragt
    10.04.2017  A.Fischer 203 Neues Command: GETCURRENT liefert zu Diagnosezwecken den Wert des Stromsensors, dafuer die
                              neue Methode getCurrentSensorValue().
    10.08.2017  A.Fischer 204 Bugfixing bei getCurrent.
    27.08.2017  A.Fischer 210 Konsolidierung von CurrentSensor und bisherigem Zweig fuer die kleinen Faecher.
                              Der Unterschied macht sich nach Aussen im Blinken beim Anlegen der Betriebsspannung deutlich:
                              Handyfach blinkt 1x, Fach mit Currentsensor ("grosses Fach") blinkt 2x.
                              Loeschen der Fingerprint-Aufrufe.
    31.11.2017  A.Fischer 211 RS485
    01.03.2018  A.Fischer 005
    07.03.2018  A.Fischer 006 Anpassung in sendData(), weil es mit ByPassCaps auf der Platine zu Farming-Fehlern kam
    11.10.2018  A.Fischer 007 CMD_GETVERSION liefert 211 zurueck. Hier wird ein neues Muster bei gesperrten Faechern angezeigt.
                              Ausserdem ist ein Fach nach dem Anschalten nicht mehr ROT, sondern VIOLETT, damit man die einzelnen
                              Zustaende besser unterscheiden kann.

*/

/*
    Ein Fach, dass fuer einen Stromsensor konfiguriert wurde, kann AUCH den Zustand eine angeschlossenen USB-Geraetes erkennen.
    ABER: In dem Fall muss auf jeden Fall ein Sensor angeschlossen sein, weil die Schaltung sonst falsche Werte liefert.
*/




//*****************************************************************
/*
    FACH_FUER_HANDY 1: Kleines Fach
    FACH_FUER_HANDY 0: Grosses Fach mit angeschlossenem Stromsensor
*/
#define FACH_FUER_HANDY 0
byte I2CADDRESS = 10;
byte MASTERADDRESS= 2;
//*****************************************************************

//#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h>

#if(!FACH_FUER_HANDY)
#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance
#endif




#define VERSION             211
#define VOLTAGEMINIMUM      0x03  //----Der Wert, der fuer den Stromfluss steht, wenn kein HANDY angeschlossen ist
#define CURRENTMINIMUM      0.36   //Der Wert, der vom Stromsensor kommt, bis zu dem KEIN angeschlossenes Geraet erkannt wird
#define COLORDIMM           0.8

#define NO_DEFINE           0x00
#define CMD_GETVERSION      0x2E
#define CMD_GETSTATE        0x2F
#define CMD_PING            0x01
#define CMD_OPENDOOR        0x02
#define CMD_LEDON           0x03
#define CMD_LEDOFF          0x04
#define CMD_GETCURRENT      0x05
#define CMD_SETCOLOR_RED    0x11
#define CMD_SETCOLOR_GREEN  0x12
#define CMD_SETCOLOR_BLUE   0x13
#define CMD_QRY_PHONE0      0x14
#define CMD_QRY_PHONE1      0x15
#define CMD_SETCOLOR_SPERR  0x16
#define CMDTERMINATOR       0xFA



#define RETURNSTATE_PROCESSING 0x30
#define RETURNSTATE_BASELINE 0x40

#define RETURNSTATE_PHONE0CONNECT 0x01
#define RETURNSTATE_PHONE1CONNECT 0x02
#define RETURNSTATE_CURRENSENSORDETECT 0x04
#define RETURNSTATE_PING_RESPONSE 0x50

#define COLOR_RED   0x41
#define COLOR_GREEN 0x42
#define COLOR_BLUE  0x43

int ws2812b_DataPin          = 4;
int LED                      = 3;
int LOCK                     = 7;  //----Steuert das Relais fuer das Schloss
int HANDY0DETECT             = A0; //----Pin für angeschlossenes Handy
int HANDY1DETECT             = A1; //----Pin für angeschlossenes Handy
int CURRENTSENSOR            = A2; //----Pin fuer Stromsensor


#define SSerialRX        A4  //Serial Receive pin
#define SSerialTX        A5  //Serial Transmit pin

#define SSerialTxControl 2   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW


// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, ws2812b_DataPin, NEO_GRB + NEO_KHZ800);

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX

byte byLastCommand;
byte byReturn;

unsigned long previousMillis = 0;
const long interval = 1000; //milliseconds
unsigned int iTimeTicker = 0;
const unsigned int TIMETICKERMAX = 10; //seconds

uint8_t iPhone0Connected;
uint8_t iPhone1Connected;

void setup() {


  wdt_enable(WDTO_2S);     //2 s
  pinMode(LED, OUTPUT);

  pinMode( HANDY0DETECT, INPUT);
  pinMode( HANDY1DETECT, INPUT);
  pinMode( CURRENTSENSOR, INPUT);
  pinMode( LOCK, OUTPUT );

#if(!FACH_FUER_HANDY)
  emon1.current(CURRENTSENSOR, 111.1);             // Current: input pin, calibration.
#endif

  ////  I2CADDRESS += read_shift_regs();
  //Wire.begin( I2CADDRESS );        // join i2c bus
  //Wire.onRequest(requestEvent); // register event
  //Wire.onReceive(receiveEvent); // register event

  pinMode(SSerialTxControl, OUTPUT);
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver



  byLastCommand = NO_DEFINE;
  byReturn = NO_DEFINE;

  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  wdt_reset();

  //----Grosse Faecher blinken zweimal
#if(!FACH_FUER_HANDY)
  delay(400);
  digitalWrite(LED, HIGH);
  wdt_reset();
  delay(400);
  digitalWrite(LED, LOW);
  wdt_reset();
#endif

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  colorWipe(strip.Color(255, 0, 255), 50); // VIO
  wdt_reset();
  delay(500);
  colorWipe(strip.Color(0, 255, 0), 50); // GRE
  wdt_reset();


  
  // Start the software serial port, to another device
  RS485Serial.begin(9600);   // set the data rate
  RS485Serial.listen();

  Serial.begin( 9600 );
  Serial.print("Slave Device ready. Address "); Serial.print(I2CADDRESS, DEC); Serial.println("(dec)");

}



void sendData(byte pVal) {
  byte bTX[3] = {pVal, MASTERADDRESS, CMDTERMINATOR};
  Serial.write("sendData():");
  Serial.print(bTX[0], HEX); Serial.print(" "); Serial.print(bTX[1], HEX); Serial.print(" "); Serial.println(bTX[2], HEX);
  //----Delay hier, weil sonst ggf Frmaing-Fehler drohen
  delay(10);
  digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit
  //delay(20);
  RS485Serial.write(bTX, 3); // Send the byte back
  delay(20);
  digitalWrite(SSerialTxControl, RS485Receive);  // Disable RS485 Transmit
}


/**
   Gibt den aktuellen Zustand des Moduls zurueck
*/
/*
  void requestEvent() {
  //Serial.print("Slave.requestEvent() returning byReturn=");Serial.println(byReturn, HEX);
  //Wire.write( byReturn );
  }
*/

/*
  void x_receiveEvent(int howMany) {
  //  byte bRX[2];
  while (Wire.available()) { //Loop through all but the last
      Wire.read();
  //    bRX[0] = Wire.read();
  //    bRX[1] = Wire.read();
  //    byLastCommand = Wire.read(); // receive byte as a character
  }
  }
*/

/*
  void receiveEvent(int howMany) {
  byte bRX[2];
  uint8_t r = 0;
  while (1 <= Wire.available()) { //Loop through all but the last
    if (r < 2) {
      bRX[r] = Wire.read();
      r++;
    } else {
      Wire.read();  //Puffer leeren
    }
  }

  if (bRX[1] == I2CADDRESS) {
    //digitalWrite(LED, HIGH);
    //Serial.print(bRX[0], HEX);Serial.print(' ');Serial.print(bRX[1], HEX);Serial.println();
    byLastCommand = bRX[0];
  }
  }
*/

/**
   Im Wesentlichen warten wir hier darauf, dass wir einen Befehl bekommen
   und reagieren darauf.
*/

boolean bCmdComplete = false;
boolean bLED = true;

void loop() {

  uint8_t iVal = getCurrentSensorValue(); //queryPhone1();
  //Serial.print("val:");Serial.println(iVal, DEC);
  RS485Serial.print("val:");RS485Serial.println(iVal, DEC);
  delay(1000);
  digitalWrite(LED, bLED);
  bLED = !bLED;
/*
  uint8_t r = 0;
  byte bRX[3]= {0, 0, 0};
  bCmdComplete = false;
  byReturn = RETURNSTATE_PROCESSING;
  
  while( !bCmdComplete ){
    if (RS485Serial.available()) {
      //Serial.print(".");
      byte tmp = RS485Serial.read();
      if (tmp!=CMDTERMINATOR){
        if(r<2){
          bRX[r] = tmp;
          r++;
        }
      } else {
        //Serial.println("Terminator");
        bCmdComplete = true;
        r=0;
        //RS485Serial.read();  //Puffer leeren
      }
  
      if( bCmdComplete ){
        //Serial.print("Receive Finish:"); Serial.print(bRX[0], HEX); Serial.print(" "); Serial.print(bRX[1], HEX); Serial.print(" "); Serial.println(bRX[2], HEX);
        
        if (bRX[1] == I2CADDRESS) {
          byLastCommand = bRX[0];           
          wdt_reset();
        }
      }
      wdt_reset();
    }
    wdt_reset();
  }//while
  


  if (byLastCommand == NO_DEFINE) {
    //----nix...
    wdt_reset();
  } else if (byLastCommand == CMD_PING) {
    Serial.println("Received PING");
    byReturn = RETURNSTATE_PROCESSING;
    byReturn = RETURNSTATE_PING_RESPONSE;
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_GETCURRENT) {
    Serial.println("Received GETCURRENT");
    byReturn = RETURNSTATE_PROCESSING;
    byReturn = getCurrentSensorValue();
    byLastCommand = NO_DEFINE;


  } else if (byLastCommand == CMD_GETSTATE) {
    Serial.println("Received GETSTATE");
    byReturn = RETURNSTATE_PROCESSING;
    byReturn = getSystemState();
    if (byReturn == RETURNSTATE_BASELINE) {
      digitalWrite(LED, LOW);
    } else {
      digitalWrite(LED, HIGH);
    }
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_OPENDOOR) {
    Serial.println("Received OPENDOOR");
    byReturn = RETURNSTATE_PROCESSING;
    openDoor( 500 );
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_SETCOLOR_RED) {
    Serial.println("Received SETCOLOR_RED");
    byReturn = RETURNSTATE_PROCESSING;
    setColor( COLOR_RED );
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_SETCOLOR_GREEN) {
    Serial.println("Received SETCOLOR_GREEN");
    byReturn = RETURNSTATE_PROCESSING;
    setColor( COLOR_GREEN );
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_SETCOLOR_BLUE) {
    Serial.println("Received SETCOLOR_BLUE");
    byReturn = RETURNSTATE_PROCESSING;
    setColor( COLOR_BLUE );
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_SETCOLOR_SPERR) {
    Serial.println("Received SETCOLOR_SPERR");
    byReturn = RETURNSTATE_PROCESSING;
    setColor_Sperr();
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;
    

  } else if (byLastCommand == CMD_QRY_PHONE0) {
    Serial.println("Received QRY_PHONE0");
    byReturn = RETURNSTATE_PROCESSING;
    //    setColor( COLOR_BLUE );
    //delay(50);
    //    setColor( COLOR_GREEN );
    byReturn = queryPhone0();
    //    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_QRY_PHONE1) {
    Serial.println("Received QRY_PHONE1");
    byReturn = RETURNSTATE_PROCESSING;
    //    setColor( COLOR_BLUE );
    //delay(50);
    //    setColor( COLOR_GREEN );
    byReturn = queryPhone1();
    //    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_GETVERSION) {
    Serial.println("Received GETVERSION");
    byReturn = RETURNSTATE_PROCESSING;
    byReturn = VERSION;
    //    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_LEDON) {
    Serial.println("Received LEDON");
    byReturn = RETURNSTATE_PROCESSING;
    digitalWrite(LED, HIGH);
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;

  } else if (byLastCommand == CMD_LEDOFF) {
    Serial.println("Received LEDOFF");
    byReturn = RETURNSTATE_PROCESSING;
    digitalWrite(LED, LOW);
    byReturn = getSystemState();
    byLastCommand = NO_DEFINE;
  }

  if(byReturn != RETURNSTATE_PROCESSING){
    sendData(byReturn);
  }

*/
  wdt_reset();
}//loop


/**
   Gibt den Zustand des Moduls wieder.
   Ein Fach, dass fuer einen Stromsensor konfiguriert ist, kann AUCH ein angeschlossenes USB-Geraet erkennen.
   Es muss aber ein Stromsensor angeschlossen sein, weil die Schaltung sonst falsche Werte liefert.
*/
uint8_t getSystemState() {
#if(FACH_FUER_HANDY)
  return RETURNSTATE_BASELINE + getPhone0Connect() + getPhone1Connect();
#else
  return RETURNSTATE_BASELINE + getPhone0Connect() + getPhone1Connect() + getCurrentSensorConnect();
#endif

}

/**
   gibt zurueck, ob ein Telefon an Port 0 angeschlossen ist
*/
uint8_t getPhone0Connect() {

  uint8_t iReturn = 0x00;
  wdt_reset();
  return iReturn;
}

/**
   gibt zurueck, ob ein Telefon an Port 1 angeschlossen ist
   Dazu werden mehrere aufeinanderfolgende Messungen durchgefuehrt.
   Wenn alle OK sind, ist ein Geraet angeschlossen, ansonsten nicht.
*/
uint8_t getPhone1Connect() {
  uint8_t iReturn = 0x00;
  boolean bConnected = true;
  int iVal;

  for (int i = 0; i < 5; i++) {
    iVal = queryPhone1();
    if (iVal < VOLTAGEMINIMUM) {
      bConnected = false;
      break;
    } else {
      delay(100);
      wdt_reset();
    }
    wdt_reset();
  }

  if (bConnected) {
    iReturn = RETURNSTATE_PHONE1CONNECT;
  }
  return iReturn;
}

uint8_t getCurrentSensorConnect() {
  uint8_t iReturn = 0x00;

  if ( getCurrentSensorValue() > CURRENTMINIMUM ) {
    iReturn = RETURNSTATE_CURRENSENSORDETECT;
  }

  return iReturn;
}

uint8_t getCurrentSensorValue() {
  uint8_t iReturn = 0x00;
  double Irms;
#if(!FACH_FUER_HANDY)
  Irms = emon1.calcIrms(1480);
#else
  Irms = 0;
#endif
  Serial.print("getcurrentSensorValue:"); Serial.println(Irms, DEC);

  if ( Irms > CURRENTMINIMUM ) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  iReturn = map(Irms, .0, 100.0, 0, 100);

  return iReturn;
}

/**
   Kann aufgerufen werden per CMD_PHONE0.
   Ist eigentlich eine Debug-Funktion und sollte nicht produktiv genutzt werden
*/
uint8_t queryPhone0() {
  uint8_t iReturn = 0x00;
  int iVal = analogRead(HANDY0DETECT);
  int iTemp = map(iVal, 0, 1023, 0, 30000);
  iReturn = 30000 - iTemp;
  return iReturn;
}

uint8_t queryPhone1() {
  uint8_t iReturn = 0x00;
  int iVal = analogRead(HANDY1DETECT);
  //int iTemp = map(iVal, 0, 1023, 0, 4000);
  //iReturn = /*4001 -*/ iTemp;
  //iReturn = iVal;
  //return iReturn;
  return iVal;
}



/**
   Sendet ein Signal an das Relais fuer die Tuer.
   Mit dem Returnwert sind wir geruestet, um spaeter
   ggf. Schließkontakte verarbeiten zu koennen
*/
boolean openDoor(int pRelaisTime) {

  //Serial.print("openDoor()");
  digitalWrite( LOCK, HIGH );
  //Serial.print("..delayStart...");
  delay( pRelaisTime );
  //Serial.println("..delayEnde...");
  digitalWrite( LOCK, LOW );
  wdt_reset();
  //Serial.println("..done");
  return true;
}

/**
   setzt die Farbe der LEDs im Ladefach
*/
boolean setColor(byte pColor) {

  if ( pColor == COLOR_RED) {
    Serial.println("Setting Color to RED");
    colorWipe(strip.Color(255, 0, 0), 50); // Red
  } else if ( pColor == COLOR_GREEN) {
    Serial.println("Setting Color to GREEN");
    colorWipe(strip.Color(0, 255, 0), 50); // Green
  } else if ( pColor == COLOR_BLUE) {
    Serial.print("Setting Color to BLUE, Waiting 30 seconds...");
    colorWipe(strip.Color(0, 0, 255), 50); // Blue
  }
}


boolean setColor_Sperr(){
  Serial.println("Setting Color to SPERR");
  colorWipe(strip.Color(0, 0, 0), 10); // Red
  strip.setPixelColor(0, strip.Color(255, 0, 0) * COLORDIMM);
  strip.setPixelColor(strip.numPixels()-1, strip.Color(255, 0, 0) * COLORDIMM);
  strip.show();
  delay(10);
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c * COLORDIMM);
    strip.show();
    delay(wait);
  }
  wdt_reset();
}
