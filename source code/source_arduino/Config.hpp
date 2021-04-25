#ifndef __CONFIG__
#define __CONFIG__

//#define __DEBUG__
//#define __SOFTWARE_SERIAL__
//

#define USE_SPEKTRUM_RX_INPUT
//#define USE_GOBLE_INPUT

#ifdef __SOFTWARE_SERIAL__
#include <SoftwareSerial.h>
#endif

#ifdef USE_GOBLE_INPUT
#ifdef __SOFTWARE_SERIAL__
#define BT_RX A0
#define BT_TX A1
extern SoftwareSerial BlueTooth;
#else
#define BlueTooth1 Serial
#endif
#endif // USE_GOBLE_INPUT

#ifdef USE_SPEKTRUM_RX_INPUT
#ifdef __SOFTWARE_SERIAL__
#define INPUT_PIN A0
#define OUTPUT_PIN A1
extern SoftwareSerial SerialRX;
#else
#define serialRx Serial
#endif
#endif // USE_GOBLE_INPUT


#define Console Serial

#define BUZZER_PIN A2
#define SERVO_CAL_PIN A6
#define GOBLE_BAUD_RATE 115200

#endif //__CONFIG_
