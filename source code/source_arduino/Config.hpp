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

#define BUZZER_PIN A2
#define SERVO_CAL_PIN A6
#define GOBLE_BAUD_RATE 115200

#endif //__CONFIG_
