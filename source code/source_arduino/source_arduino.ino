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
#include "GoBLE.hpp"

#ifdef USE_GOBLE_INPUT
#ifdef __SOFTWARE_SERIAL__
SoftwareSerial BlueTooth(BT_RX, BT_TX);
_GoBLE<SoftwareSerial, HardwareSerial> Goble(BlueTooth, Console);
#else
_GoBLE<HardwareSerial, HardwareSerial> Goble(BlueTooth1, Console);
#endif
#endif


#ifdef USE_SPEKTRUM_RX_INPUT

#ifdef __SOFTWARE_SERIAL__
SoftwareSerial serialRx(INPUT_PIN, OUTPUT_PIN); // RX, TX
#endif

// Spektrum channel order
#define THRO 0
#define AILE 1
#define ELEV 2
#define RUDD 3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate, not at 11ms.
#define AUX6 10
#define AUX7 11

// Only support DSMx SERIALRX_SPEKTRUM2048:
#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEK_FRAME_SIZE                      16
#define SPEK_CHAN_SHIFT                       3
#define SPEK_CHAN_MASK                     0x07
#define SPEKTRUM_NEEDED_FRAME_INTERVAL        5
#define SPEKTRUM_BAUDRATE                115200

// Ugly globals....
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
float _period = 10.0; //> indicates the number of steps every second

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
  serialRx.begin(SPEKTRUM_BAUDRATE);
#endif

#ifdef __DEBUG__
  Console.begin(115200);
  Console.println("in debugging mode");
#endif
  //: servo calibration mode - while A6 connects to 5V, all servos in 90° for servo arm adjustment.
  while (analogRead(SERVO_CAL_PIN) > 1000)  delay(1000);

  //
#ifdef __DEBUG__
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
    joystickLY = 127;
    //joystickRX = 127;
    state = 1;
    goto __handle_input;
  }

#ifdef USE_GOBLE_INPUT
  //
  if (Goble.available()) {
    previousDuration = duration;
    hardware.attach(); // turn on servos if they are off
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
#endif // USE_GOBLE_INPUT

#ifdef USE_SPEKTRUM_RX_INPUT
  static unsigned long lastSerialRxTime = 0;
  static unsigned char spekFrame[SPEK_FRAME_SIZE];
  static unsigned char spekFramePosition = 0;
  static bool rcFrameComplete = false;

  unsigned long spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];

  if (currentTime - lastSerialRxTime > SPEKTRUM_NEEDED_FRAME_INTERVAL)
  {
    spekFramePosition = 0;
  }

  if (spekFramePosition < SPEK_FRAME_SIZE && serialRx.available())
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
      rcFrameComplete = true;
    }
  }

  if (rcFrameComplete)
  {
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

    switch (state)
    {
      case 1:
        joystickLY = map(spekChannelData[THRO], 2047, 0, 127, -128); // DSMx 11 bit 2048 mode -> 8 bit mode 
        joystickRX = map(spekChannelData[AILE], 2047, 0, 127, -128);
        break;
      case 0:
      case 2:
        joystickRY = map(spekChannelData[ELEV], 2047, 0, 127, -128);
        joystickRX = map(spekChannelData[AILE], 2047, 0, 127, -128);
        break;
      case 3:
        joystickLY = map(spekChannelData[THRO], 2047, 0, 127, -128);
        joystickLX = map(spekChannelData[RUDD], 0, 2047, 127, -128);
        break;
      default:
        joystickLX = 0; joystickLY = 0;
        joystickRX = 0; joystickRY = 0;
    }
  }
#endif // USE_SPEKTRUM_RX_INPUT

__handle_input:
  handle_input();
  //
#ifdef __DEBUG__
  if (Console.available())
    handle_serial();
#endif
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
  //Console.println("joystickLY: " + String(joystickLY) + ", ly: " + String(ly));
  //Console.println("joystickRX: " + String(joystickRX) + ", rx: " + String(rx));
  //Console.println("joystickRY: " + String(joystickRY) + ", ry: " + String(ry));
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
