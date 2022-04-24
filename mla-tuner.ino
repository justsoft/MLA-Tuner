/*
  Created by John Zhong on April 24, 2022
  Thanks Wim der Kinderen for his modified version of ESP32-UART
*/

#include <stdlib.h>
#include <algorithm>
#include <sstream>
#include <string>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <Stepper.h>

// change this to the number of steps on your motor
#define STEPS 2048
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to:
// ULN2003 Motor Driver Pins
#define IN1 (19)
#define IN2 (18)
#define IN3 (5)
#define IN4 (17)

Stepper stepper(STEPS, IN1, IN3, IN2, IN4);

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
const int button = 0;      // button on PIN G0
const int readPin = 32;    // analog pin G32
const int LEDpin = 2;      // LED on pin G2
bool convert = false;
std::string rxString;

// UART service UUID data
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Connected");
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Disconnected");
      pServer->getAdvertising()->start();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      rxString = pCharacteristic->getValue();
      if (rxString.length() > 0)  {
        std::ostringstream ss;
        ss << "Received data:[" << rxString << "]";
        Serial.println( ss.str().c_str() );
        convert = true;      // flag to invoke convertControlpad routine
      }
    }
};

void sendText(const char *fmt, ...)
{
  if ( deviceConnected ) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsprintf( buf, fmt, args );
    pCharacteristic->setValue( buf );
    pCharacteristic->notify();
    va_end(args);
  }
}

void writeEEPROMLong( int addr, long int v )
{
  unsigned char * p = ( unsigned char * ) &v;
  for ( int i = 0; i < sizeof( v ); i++ ) {
    EEPROM.write(addr + i, p[i] );
  }
  EEPROM.commit();
}

long int readEEPROMLong( int addr ) {
  long int r;
  unsigned char * p = ( unsigned char * ) &r;
  for ( int i = 0; i < sizeof( r ); i++ ) {
    p[i] = EEPROM.read(addr + i);
  }
  return r;
}

#define MEMO_COUNT 4
#define EEPROM_SIZE ( sizeof(long) + MEMO_COUNT*sizeof(long) )
#define LONG_PRESSED_MS 500

long currentPosition = 0;
long marks[MEMO_COUNT];
long deltaSteps = 0;
int deltaDelay = 0;
int oneStep = 0;
int speed = 10;
unsigned long buttonPressed = 0;
typedef enum { kUp, kDown, kLeft, kRight, kF1, kF2, kF3, kF4 } KeyTypes;
KeyTypes key;
bool longPressed = false;

void SetMotorPINs( void ) {
  pinMode(IN1, OUTPUT );
  pinMode(IN2, OUTPUT );
  pinMode(IN3, OUTPUT );
  pinMode(IN4, OUTPUT );
}

void PowerOffMotor( void ) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(LEDpin, LOW);
}

#define MORSE_DOT_LEN 100
// double space between words
// space between letters
void ledMorse( const char *s ) {
  while ( *s ) {
    if (*s == '.') {
      digitalWrite(LEDpin, HIGH);
      delay(MORSE_DOT_LEN);
      digitalWrite(LEDpin, LOW);
    } else if (*s == '-' ) {
      digitalWrite(LEDpin, HIGH);
      delay(MORSE_DOT_LEN * 3);
      digitalWrite(LEDpin, LOW);
    } else if (*s == ' ' ) {
      if ( s[1] == ' ' ) {
        s++;
        delay( MORSE_DOT_LEN * 6 );
      } else {
        delay( MORSE_DOT_LEN * 2 );
      }
    }
    delay(MORSE_DOT_LEN);
    s++;
  }
}

//!!! Adruino compiler error
void keyPressed(int k) {
  key = ( KeyTypes ) k;
  buttonPressed = millis();
  longPressed = false;
}

void keyReleased() {
  if ( buttonPressed ) {
    longPressed = millis() - buttonPressed > LONG_PRESSED_MS;
    buttonPressed = 0;
  }
  Action( longPressed, true );
}


// ***************************** SETUP *******************************
void setup() {
  Serial.begin(115200);
  pinMode(LEDpin, OUTPUT);
  pinMode(button, INPUT);

  SetMotorPINs();
  stepper.setSpeed(10);
  PowerOffMotor();

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
    ledMorse( "... --- ...");
  } else {
    ledMorse( "--- -.-");
  }

  currentPosition = readEEPROMLong(0);
  Serial.print( "Loaded saved position: " );
  Serial.println( currentPosition );
  for ( size_t i = 0; i < MEMO_COUNT; i++ ) {
    marks[i] = readEEPROMLong(sizeof(unsigned long) * ( i + 1 ));
    Serial.print( "Loaded saved memo #" );
    Serial.print( i, 1 );
    Serial.print( ": " );
    Serial.println( marks[i] );
  }

  BLEDevice::init(std::string());
  BLEAddress bleAdr = BLEDevice::getAddress();
  std::string BLEName = "MLA Tuner #";
  BLEName += bleAdr.toString();
  BLEName.erase( std::remove( BLEName.begin(), BLEName.end(), ':'), BLEName.end());
  Serial.println(BLEName.c_str());
  BLEDevice::deinit();

  //delay(1000);

  BLEDevice::init(BLEName);
  BLEServer *pServer = BLEDevice::createServer(); // create BLE server
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start(); // start the service

  pServer->getAdvertising()->start(); // start advertising
  Serial.println("Waiting a client connection to notify...");
  Serial.println(" ");
}

// 0: Action( true, false ) --- still holding
// 1: Action( true, true )
// 2: Action( false, true )
void Action( bool isLongPressed, bool isReleased ) {
  oneStep = isLongPressed ? 0 : 1; // long press make non-stop moving
  switch (key) {
    case kUp:
      deltaDelay = 0;
      deltaSteps = -10;
      if ( isReleased && isLongPressed ) {
        deltaSteps = 0;
      }
      break;
    case kDown:
      deltaDelay = 0;
      deltaSteps = 10;
      if ( isReleased && isLongPressed ) {
        deltaSteps = 0;
      }
      break;
    case kLeft:
      deltaDelay = 20;
      deltaSteps = 1;
      if ( isReleased && isLongPressed ) {
        deltaSteps = 0;
      }
      break;
    case kRight:
      deltaDelay = 20;
      deltaSteps = -1;
      if ( isReleased && isLongPressed ) {
        deltaSteps = 0;
      }
      break;
    default:
      std::ostringstream ss;
      size_t i = key - kF1;
      if ( isLongPressed ) {
        if ( !isReleased ) {
          marks[i] = currentPosition;
          writeEEPROMLong( ( i + 1 )*sizeof(long), currentPosition );
          sendText( "Pos: %ld saved to M%d\n", currentPosition, i + 1 );
        }
      } else {
        oneStep = 1;
        deltaDelay = 0;
        long x = marks[i];
        deltaSteps = x - currentPosition;
        sendText( "Recall M%d, pos: %ld\n", i + 1, x );
      }
  }
  if ( deltaSteps ) {
    digitalWrite(LEDpin, HIGH);
  } else {
    PowerOffMotor();
  }
}

unsigned long lastUpd = 0;
long lastPosition = 0;

void loop() {
  if (deviceConnected) {
    if (convert) convertControlpad();
    unsigned long ms = millis();
    if ( lastPosition != currentPosition && ms - lastUpd > 1000 ) {
      lastUpd = ms;
      lastPosition = currentPosition;
      writeEEPROMLong( 0, lastPosition );
      sendText( "Pos: %ld\n", currentPosition );
    }

    if ( buttonPressed ) {
      if ( millis() - buttonPressed > LONG_PRESSED_MS ) {
        longPressed = true;
        buttonPressed = 0;
        Action( true, false );
      }
    }

    if ( deltaSteps ) {
      if ( deltaSteps > 0 ) {
        stepper.step( 1 );
        deltaSteps -= oneStep;
        currentPosition++;
      } else {
        stepper.step( -1 );
        deltaSteps += oneStep;
        currentPosition--;
      }
      if ( deltaDelay ) delay( deltaDelay );
      if ( deltaSteps == 0 ) {
        delay( 50 );
        PowerOffMotor();
      }
    } else {
      delay(50);
    }
  } else {
    delay(100);
  }
}


// ************************* CONVERT CONTROLPAD CODE ************************
void convertControlpad() {
  convert = false;
  Serial.print("      ");
  if (rxString == "!B11:") {
    Serial.println("********** Start Action 1");
    keyPressed(kF1);
  }
  else if (rxString == "!B10;") {
    Serial.println("********** Stop Action 1");
    keyReleased();
  }
  else if (rxString == "!B219") {
    keyPressed(kF2);
    Serial.println("********** Start Action 2");
  }
  else if (rxString == "!B20:") {
    Serial.println("********** Stop Action 2");
    keyReleased();
  }
  else if (rxString == "!B318") {
    Serial.println("********** Start Action 3");
    keyPressed(kF3);

  }
  else if (rxString == "!B309") {
    Serial.println("********** Stop Action 3");
    keyReleased();
  }
  else if (rxString == "!B417") {
    Serial.println("********** Start Action 4");
    keyPressed(kF4);
  }
  else if (rxString == "!B408") {
    Serial.println("********** Stop Action 4");
    keyReleased();
  }

  else if (rxString == "!B516") {
    Serial.println("********** Start Action UP");
    keyPressed( kUp );
  }
  else if (rxString == "!B507") {
    Serial.println("********** Stop Action UP");
    keyReleased();
  }

  else if (rxString == "!B615") {
    Serial.println("********** Start Action DOWN");
    keyPressed( kDown );
  }
  else if (rxString == "!B606") {
    Serial.println("********** Stop Action DOWN");
    keyReleased();
  }

  else if (rxString == "!B714") {
    Serial.println("********** Start Action LEFT");
    keyPressed( kLeft );
  }
  else if (rxString == "!B705") {
    Serial.println("********** Stop Action LEFT");
    keyReleased();
  }

  else if (rxString == "!B813") {
    Serial.println("********** Start Action RIGHT");
    keyPressed( kRight );
  }
  else if (rxString == "!B804") {
    Serial.println("********** Stop Action RIGHT");
    keyReleased();
  } else if (rxString.rfind("speed:", 0 ) == 0 ) {
    Serial.print("********** Set speed");
    int v = strtol( rxString.c_str() + 6, NULL, 10 );

    stepper.setSpeed( v );
    sendText( "Set speed to: %d\n", v );
  } else if ( rxString.rfind("reset", 0 ) == 0 ) {
    sendText( "Reset the position %ld to 0\n", currentPosition );
    currentPosition = 0;
  }

  rxString = "";
}
