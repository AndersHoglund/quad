#ifndef __GPIO_SERVO__
#define __GPIO_SERVO__
#include <Servo.h>

class GPIOservo {
  private:

  public:
    GPIOservo(): angle(90), rev(false), attached(false), pinIndex(-1) {

    }
    GPIOservo(int pinIndex ): GPIOservo() {
      this->pinIndex = pinIndex;
#if defined(ESP32)
      this->attach(); // calling in constructor not working in AVR mcu
#endif
    }

    GPIOservo(int pinIndex, int min, int max, int trim ): GPIOservo(pinIndex) {
      this->gpioServoMin = min;
      this->gpioServoMax = max;
      this->gpioServoCenterTrim = trim;
    }

    bool attach(uint8_t pinIndex) {
      this->pinIndex = pinIndex;
      return attach();
    }

    bool attach(int minPulse, int maxPulse, int centerTrim ) {
      this->gpioServoMin = minPulse;
      this->gpioServoMax = maxPulse;
      this->gpioServoCenterTrim = centerTrim;
      return attach();
    }

    bool attach(uint8_t pinIndex, int minPulse, int maxPulse, int centerTrim ) {
      this->pinIndex = pinIndex;
      this->gpioServoMin = minPulse;
      this->gpioServoMax = maxPulse;
      this->gpioServoCenterTrim = centerTrim;
      return attach();
    }

    bool attach() {
      if (this->pinIndex < 0) return false;
      if (attached) {
        gpioServo.detach();
      }
      gpioServo.attach(this->pinIndex, gpioServoMin, gpioServoMax);
      attached = true;
      write(angle);
      prevTime = millis();
      return attached;
    }

    void detach() {
      if (attached) {
        gpioServo.detach();
        attached=false;
      }
    }

    void write(int angle) {
      if (attached)
      {
        int angleTrim = map(1500 + this-> , gpioServoMin, gpioServoMax, 0, 180)- 90;
        gpioServo.write(angle + angleTrim);
        this->angle = angle;
      }
    }

    void writeMicroseconds(long us) {
      if (attached) 
      {
        gpioServo.writeMicroseconds(us + this->gpioServoCenterTrim);
      }
    }

    int getAngle() {
      return angle;
    }

    bool move(int targetAngle) {
      if (angle == targetAngle)  {
        return true;
      }
      currentTime = millis();
      if (currentTime - prevTime >= angleTimeGap) {
        long diffTime = currentTime - prevTime;
        int diffAngle = diffTime / angleTimeGap;
        prevTime = currentTime;
        this->write(angle);
        if (targetAngle < angle) {
          angle -= diffAngle;
          if (angle < 0) angle = 0;
        } else {
          angle += diffAngle;
          if (angle > 180) angle = 180;
        }
      }
      return false;
    }

    void sweep() {
      currentTime = millis();
      if (currentTime - prevTime >= angleTimeGap) {
        long diffTime = currentTime - prevTime;
        int diffAngle = diffTime / angleTimeGap;
        prevTime = currentTime;
        this->write(angle);
        if (rev) {
          angle -= diffAngle;
          if (angle < 0) {
            rev = false;
            angle = 0;
          }
        } else {
          angle += diffAngle;
          if (angle > 180) {
            rev = true;
            angle = 180;
          }
        } //rev
      }
    }

  private:
    int pinIndex;
    int gpioServoCenterTrim = 0;
    int gpioServoMin = 550;
    int gpioServoMax = 2550;
    //const int gpioServoMax = 2350;
    Servo gpioServo;
    //
    const short angleTimeGap = 5;
    int angle;
    bool rev;
    long prevTime, currentTime;
    //
    bool attached;
};
#endif
