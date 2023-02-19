#ifndef __CONFIG__
#define __CONFIG__

//#define TRACEPRINT
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

#define USE_SPOT_NANO
#define ROBOT_NAME SPOT_NANO

#define BUZZER_PIN A2
#define SERVO_CAL_PIN A6
#define GOBLE_BAUD_RATE 115200

// Psysical dimensions in mm
#define LEG_LEN 70
#define LEG_DIST 120
#define LEG_WIDTH_TOP 47
#define LEG_WIDTH_BOT 120

#define SERVO_MIN_PULSE  600 //   0 degrees
#define SERVO_MID_PULSE 1500 //  90
#define SERVO_MAX_PULSE 2350 // 180

#define SERVO_MIN_PULSE_ABS  580 // Absolute minimum, mechanical stop (ES09A/MA).
#define SERVO_MAX_PULSE_ABS 2380 // Absolute maximum, mechanical stop.

#endif //__CONFIG_
