#ifndef __CONFIG__
#define __CONFIG__

//
//#define __DEBUG__
//#define __SOFTWARE_SERIAL__
//

#ifdef __SOFTWARE_SERIAL__
#include <SoftwareSerial.h>
#define BT_RX A0
#define BT_TX A1
#define Console Serial
extern SoftwareSerial BlueTooth;
#else
#define Console Serial
#define BlueTooth1 Serial
#endif
//

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

#define SERVO_MIN_PULSE 1000
#define SERVO_MAX_PULSE 2000

#endif //__CONFIG_
