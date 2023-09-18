/*
  This code is modification of https://github.com/maestrakos/quad by Alexandros Petkos
  for https://www.instructables.com/Ez-Arduino-12-DOF-Quadruped-Robot-Robot-Dog-Lassie/
  
  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/

#include "Config.hpp"
#include "Buzzer.hpp"
#include "Hardware.hpp"
#include "Kinematics.hpp"

#ifdef USE_GOBLE_INPUT
#include "GoBLE.hpp"

#ifdef __SOFTWARE_SERIAL__
SoftwareSerial BlueTooth(BT_RX, BT_TX);
_GoBLE<SoftwareSerial, HardwareSerial> Goble(BlueTooth, Console);
#else
_GoBLE<HardwareSerial, HardwareSerial> Goble(BlueTooth1, Console);
#endif
#endif


#ifdef USE_SPEKTRUM_RX_INPUT
#include "spektrum.h"

#ifdef __SOFTWARE_SERIAL__
SoftwareSerial serialRx(INPUT_PIN, OUTPUT_PIN); // RX, TX
#endif

unsigned long lastSerialRxTime = 0;
unsigned char spekFrame[SPEK_FRAME_SIZE];
unsigned char spekFramePosition = 0;
bool rcFrameComplete = false;
long spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];
unsigned long currentTime;

#endif // USE_SPEKTRUM_RX_INPUT

//
Buzzer buzzer(BUZZER_PIN);
Hardware hardware;
Kinematics kinematics(hardware);
float vo = kinematics.vrt_offset, ho = kinematics.hrz_offset;

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0;   //> indicates the direction of rotation
float height = 0; //> indicates the leg extension

int state = 0;        //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
float _period = 4.0;  //> indicates the number of steps every second

int8_t joystickLX = 0;
int8_t joystickLY = 0;
int8_t joystickRX = 0;
int8_t joystickRY = 0;

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body

unsigned long duration;
//
//int sample_sum, sample_num = 10,
//                sample_index;
//float freq;

//: handle input parameters
float stick_min = 6.f;
float lx, ly, rx, ry;

void setup()
{
  hardware.init_hardware();
#ifdef USE_GOBLE_INPUT
  Goble.begin(GOBLE_BAUD_RATE);
#endif

#ifdef USE_SPEKTRUM_RX_INPUT
#ifdef __SOFTWARE_SERIAL__
  pinMode(INPUT_PIN, INPUT);
#endif
  serialRx.begin(SPEKTRUM_BAUDRATE);
#endif

#ifdef __DEBUG__
  Console.begin(115200);
  Console.println("in debugging mode");
#endif
  calibrateServoCenter();
  //
#ifdef __DEBUG__
  Console.println("started");
#endif
#ifdef TRACEPRINT
  Console.println("started");
#endif
  buzzer.beepShort();
}

void loop()
{
  //hardware.testGPIOservos(); return;
  duration = millis();
  hardware.handle_hardware();
  kinematics.handle_kinematics(state, _direction, turn, height, _period);
  //
  const long timeInterval = 60000;
  static long previousDuration = 0;
  if ((duration - previousDuration) > timeInterval) {
    hardware.detach(); // turn off servos while not moving for 1 min
    joystickLX = 0; joystickLY = 0;
    joystickRX = 0; joystickRY = 0;
    previousDuration = duration;
    return;
  }

  //: test mode -  while A6 connects to 5V again;  walk in trot gait; stop after 1 min
  if (analogRead(SERVO_CAL_PIN) > 1000) {
    joystickLY = 64;
    //joystickRX = 127;
    state = 1;
    handle_input();
  }
  else {
#ifdef USE_GOBLE_INPUT
  //
  if (Goble.available()) {
    previousDuration = duration;
    switch (state) {
      case 1:
        joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
        break;
      case 0:
      case 2:
        joystickRY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
        break;
      case 3:
        joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickLX = map(Goble.readJoystickX(), 0, 255, 127, -128);
        break;
      default:
        joystickLX = 0; joystickLY = 0;
        joystickRX = 0; joystickRY = 0;
    }

    if (Goble.readSwitchUp() == PRESSED) {
      state = 0;
      buzzer.beepShort();
    } else if (Goble.readSwitchDown() == PRESSED) {
      state = 2;
      buzzer.beepShort();
    } else if (Goble.readSwitchLeft() == PRESSED) {
      state = 3;
      buzzer.beepShort();
    } else if (Goble.readSwitchRight() == PRESSED) {
      state = 1;
      buzzer.beepShort();
    }

    if (Goble.readSwitchMid() == PRESSED) {
      buzzer.beepShort();
    } else if (Goble.readSwitchSelect() == PRESSED) {
      buzzer.beepShort();
    } else if (Goble.readSwitchAction() == PRESSED) {
      buzzer.beepShort();
    } else if (Goble.readSwitchStart() == PRESSED) {
      buzzer.beepShort();
      hardware.detach();
      joystickLX = 0; joystickLY = 0;
      joystickRX = 0; joystickRY = 0;
      previousDuration = duration;
    }
  }
  //
  handle_input();
  //
#endif // USE_GOBLE_INPUT

#ifdef USE_SPEKTRUM_RX_INPUT

  currentTime = millis();

  if (currentTime - lastSerialRxTime > SPEKTRUM_NEEDED_FRAME_INTERVAL)
  {
    spekFramePosition = 0;
    rcFrameComplete = false;
  }

  while (spekFramePosition < SPEK_FRAME_SIZE && serialRx.available() > 0)
  {
    lastSerialRxTime = currentTime;

    unsigned char c = serialRx.read();
    spekFrame[spekFramePosition++] = c;
    if (spekFramePosition < SPEK_FRAME_SIZE)
    {
      rcFrameComplete = false;
    }
    else
    {
      spekFramePosition = 0;
      rcFrameComplete = true;
    }
  }

  if (rcFrameComplete)
  {
    previousDuration = duration;
    rcFrameComplete = false;

    // Get the RC control channel inputs
    for (int b = 3; b < SPEK_FRAME_SIZE; b += 2)
    {
      const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);

      if (spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT)
      {
        spekChannelData[spekChannel] = ((spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
      }
    }

    int stateChannel = AUX1;
    unsigned int stateSwitch = map(spekChannelData[stateChannel], 0, 2047, 988, 2012 );
    if ((stateSwitch < 1100) && (state != 0)) // RM TX16S 6pos switch (MM output) ~= 988
    {
      state = 0;
      buzzer.beepShort();
    }
    else if ((stateSwitch > 1100) && (stateSwitch < 1350) && (state != 1)) // 1193
    {
      state = 1;
      buzzer.beepShort();
    }
    else if ((stateSwitch > 1350) && (stateSwitch < 1550) && (state != 2)) // 1398
    {
      state = 2;
      buzzer.beepShort();
    }
    else if ((stateSwitch > 1550) && (stateSwitch < 1700) && (state != 3)) // 1602
    {
      state = 3;
      buzzer.beepShort();
    }
    else if ((stateSwitch > 1700) && (stateSwitch < 1800) && (state != 4)) // 1807
    {
      state = 4;
      buzzer.beepShort();
    }
    else if ((stateSwitch > 1800) && (stateSwitch < 2100) && (state != 5)) // 2012
    {
      state = 5;
      buzzer.beepShort();
      calibrateServoCenter();
    }

//    Console.println("spekChannelData[stateChannel]/2: " + String(spekChannelData[stateChannel]>>1) + ", stateSwitch: " + String(stateSwitch) + ", state: " + String(state));

    joystickLY = map(spekChannelData[THRO], 2047, 0, 127, -128); // DSMx 11 bit 2048 mode -> 8 bit mode
    joystickLX = map(spekChannelData[RUDD], 2047, 0, 127, -128);
    joystickRY = map(spekChannelData[ELEV], 2047, 0, 127, -128);
    joystickRX = map(spekChannelData[AILE], 2047, 0, 127, -128);
    handle_input();
  }
#endif // USE_SPEKTRUM_RX_INPUT
  } // Test mode
#ifdef __DEBUG__
  if (Console.available())
    handle_serial();
#endif
}

void calibrateServoCenter()
{
  while (analogRead(SERVO_CAL_PIN) > 1000 || state == 5)
  {
    // servo calibration mode - while A6 connects to 5V, all servos in 90° for servo arm adjustment.
    for (int leg = 0; leg < 4; leg++)
    {
//      _sRotation = {0,65,90}; // Close to a Cal position,weird numbers...???
      _sRotation = {0,45,90}; // Idle position
      hardware.set_leg(leg, _sRotation);

//      for (int joint = 0; joint < 4; joint++)
//      {
//        hardware.set_servo(leg,joint, SERVO_MID_PULSE);
//      }
    }
  delay(2000);
  }
}

inline void aborted()
{
  Console.println("Program aborted!");
  buzzer.beepError();
  while (1)
    ;
}


void handle_input()
{
  lx = Kinematics::inter(lx, joystickLX / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick
  ly = Kinematics::inter(ly, joystickLY / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick
  rx = Kinematics::inter(rx, joystickRX / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick
  ry = Kinematics::inter(ry, joystickRY / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick
//  Console.println("joystickLX: " + String(joystickLX) + ", lx: " + String(lx));
//  Console.println("joystickLY: " + String(joystickLY) + ", ly: " + String(ly));
//  Console.println("joystickRX: " + String(joystickRX) + ", rx: " + String(rx));
//  Console.println("joystickRY: " + String(joystickRY) + ", ry: " + String(ry));
  if (abs(lx) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float x0 = lx - stick_min * Kinematics::sign(lx); //> subtracts the deadzone
    if (state == 1)
    {
      _direction.y = 0; //x0 / 10.f;
    }
    else if (state != 4)
    {
      _direction.y = x0 / 2;
    }
  }
  else
    _direction.y = 0;

  if (abs(ly) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float y0 = ly - stick_min * Kinematics::sign(ly); //> subtracts the deadzone
    if (state == 1)
    {
      _direction.x = y0 / 10.f;
      if (y0 > 0)
        kinematics.vrt_offset = Kinematics::inter(kinematics.vrt_offset, vo - 6.f, 2.f);
      else
        kinematics.vrt_offset = Kinematics::inter(kinematics.vrt_offset, vo + 3.f, 2.f);
    }
    else if (state != 4)
    {
      _direction.x = y0 / 2;
      kinematics.vrt_offset = vo;
    }
  }
  else
  {
    _direction.x = 0;
    kinematics.vrt_offset = vo;
  };

  if (abs(rx) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float x1 = rx - stick_min * Kinematics::sign(rx); //> subtracts the deadzone
    if (state == 1)
      turn = x1 / 16.f;
    else if (state != 4)
      turn = x1;
  }
  else
    turn = 0;

  if (abs(ry) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float y1 = ry - stick_min * Kinematics::sign(ry); //> subtracts the deadzone
    height = y1;
  }
  else
    height = 0;
}

#ifdef __DEBUG__
// !! make sure you have enabled Newline or Carriage return
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input
void handle_serial()
{
  //: reads and stores the serial data
  int i = 0;
  float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Console.available())
  {
    char c = Console.read();
    if (c == 13 || c == 32 || c == '\n')
    {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    }
    else
      s_buff += c;
  }

  if (_mode == 0)
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)
    if (state == 4)
    {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

#define properties 0
void commands_exe(float val1, float val2, float val3)
{
  //: propertios 0 is used to calibrate the joints
  if (properties == 0)
  {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Console.print("- leg ");
    Console.print(leg);
    Console.print(" joint ");
    Console.print(joint);
    Console.print(" set to ");
    Console.print(servo);
    Console.print(".\n");

    hardware.set_servo(leg, joint, servo);
  }
  //: propertios 1 is used for small adjustments to balance the weight
  else if (properties == 1)
  {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Console.print("- leg ");
    Console.print(leg);
    Console.print(" null ");
    Console.print(empty);
    Console.print(" set to ");
    Console.print(ammount);
    Console.print(".\n");

    kinematics.base_offset[leg] = ammount;
  }
}
#endif //__DEBUG__
