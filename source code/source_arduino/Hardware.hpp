#ifndef __HARDWARE__
#define __HARDWARE__
#include "Config.hpp"

#include "datatypes.h"
#include "GPIOservo.hpp"
/*
  #include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
*/



class Hardware {
  private:
    /*
      ==============================
      HARDWARE - SERVO PARAMETERS
      ==============================
    */
    GPIOservo servo[4][3] = {
      {GPIOservo(12), GPIOservo(13), GPIOservo(5)}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
      {GPIOservo(2), GPIOservo(3), GPIOservo(4)},   // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
      {GPIOservo(9), GPIOservo(8), GPIOservo(6)},   // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
      {GPIOservo(10), GPIOservo(11), GPIOservo(7)}  // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
    };

    const int pulse_min = SERVO_MIN_PULSE;
    const int pulse_max = SERVO_MAX_PULSE;

    // ## pulse_max offset to adjust servo angle
    // ## tune value if legs are not balance equally.
    int s_offset_max[4][3] = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0}
    };

    // ## pulse center offset to subtrim servo center angle (us)
    int s_offset_center[4][3] = {
      // Good Subtrim using A6 Calibration mode
      {  40,    0, -50}, // RB Shoulder Elbow Wrist
      {-110,    0, -90}, // RF
      {  90,  -70, -70}, // LF
      {  30,  -40,  40}  // LB
     };

    const int s_optinv[4][3] = {
      {1, 0, 1}, // ## {dir, dir, dir}
      {1, 0, 1}, // ## {dir, dir, dir}
      {0, 1, 0}, // ## {dir, dir, dir}
      {0, 1, 0}  // ## {dir, dir, dir}
    };

    const int d_constraint_min[3] { -70, 20, 40}; // ## {deg, deg, deg}
    const int d_constraint_max[3] {70, 110, 150}; // ## {deg, deg, deg}

    const int right_back_leg = 0;
    const int right_front_leg = 1;
    const int left_front_leg = 2;
    const int left_back_leg = 3;
    //
    boolean attached = false;
    const int legs[4] = {right_back_leg, left_back_leg, right_front_leg, left_front_leg};
  public:
    Hardware() {}

    /*
      ::: SETUP :::
    */
    void init_hardware()
    {
      attach();
    }

    void attach() {
      if (attached) return;
      for (int joint = 0; joint < 3; joint++) {
        for (int i = 0; i < 4; i++) {
          servo[legs[i]][joint].attach(pulse_min, pulse_max + s_offset_max[legs[i]][joint], s_offset_center[legs[i]][joint]);
          delay(30);
        }
      }
      attached = true;
    }

    void detach() {
      if (!attached)  return;
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].detach();
        }
      }
      attached = false;
    }


    void set_leg(int leg, datatypes::Rotator rot)
    {
      set_joint(leg, 0, rot.yaw);
      set_joint(leg, 1, rot.pitch);
      set_joint(leg, 2, rot.roll);
      //Console.println("");
    }

    void set_servo(int leg, int joint, float pulse)
    {
      servo[leg][joint].writeMicroseconds(pulse);
    }

  private:
    void set_joint(int leg, int joint, float deg)
    {
      int _min = pulse_min;
      int _max = pulse_max;
      int _inv = s_optinv[leg][joint];
      int _minC = d_constraint_min[joint];
      int _maxC = d_constraint_max[joint];

      if (deg < _minC)
        deg = _minC;
      else if (deg > _maxC)
        deg = _maxC;

      float pulse;
      if (_inv == 0) {
        pulse = map(deg, _minC, _maxC, _min, _max);
      } else if (_inv == 1) {
        pulse = map(deg, _minC, _maxC, _max, _min);
      }
      servo[leg][joint].writeMicroseconds(pulse);
}

    /*
      == == == == == == == == == == == == == == ==
      HARDWARE - MPU VARIABLES
      == == == == == == == == == == == == == == ==
      /

      //uint16_t packetSize;    // expected DMP packet size (default 42 bytes)
      //uint8_t fifoBuffer[64]; // FIFO storage buffer

      //Quaternion q;           // [w, x, y, z]         quaternion container
      //float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
      //VectorFloat gravity;    // [x, y, z]            gravity vector

      MPU6050 mpu;

      /*
      ::: [Gyroscope/Accelerometer Sensor] FUNCTIONS :::
    */

    void update_mpu_data()
    {
      /*if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        _sRotation = {ypr[0] * 80 / M_PI,
                      -ypr[1] * 80 / M_PI,
                      ypr[2] * 80 / M_PI
                     };

        /*if (DEBUG == 0) {
          Console.print(ypr[0] * 60 / M_PI);
          Console.print("/");
          Console.print(-ypr[1] * 60 / M_PI);
          Console.print("/");
          Console.println(ypr[2] * 60 / M_PI);
          }
        }*/
    }
  public:
    void handle_hardware()
    {
      //update_mpu_data();
    }


    void init_mpu()
    {
      /*mpu.initialize();
        uint8_t dmp_s = mpu.dmpInitialize();

        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);

        if (dmp_s == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
        if (DEBUG == 1)
          Console.println(":ERROR[" + String(dmp_s) + "]");
        while (1); /*:pause_sketch:
        }*/
    }
#if defined(__DEBUG__)
    void testGPIOservo() {
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].sweep();
        }
      }
    }
#endif
};
#endif
