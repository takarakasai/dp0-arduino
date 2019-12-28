
#include "dyne.hpp"

#include <stdint.h>
#include <Wire.h>

#define USE_VELOCITY_CONTROL_MODE

namespace dyne {
  // I2C address
  constexpr uint8_t kLLegAddr = 0x31;
  constexpr uint8_t kRLegAddr = 0x32;

  // I2C command requires 3[bytes]
  // [CMD] [VALUE(upper byte)] [VALUE(lower byte)]
  constexpr uint8_t kVelCmd = 0x10;
  constexpr uint8_t kPosCmd = 0x20;
  constexpr uint8_t kPwmCmd = 0x30;

#if defined(USE_VELOCITY_CONTROL_MODE)
  // 1.0[m/sec] ==> [round/minutes]
  constexpr float kWheelRadius = 0.06; // [m]
  constexpr float kDistPerRound = 2.0 * 3.1415 * kWheelRadius; // 2*pi*r [m]
  constexpr float kVel2RPS = 15.0 * 60.0 / kDistPerRound;
#else
  // 1.0[m/sec] ==> 80 (PWM value);
  constexpr float kVel2PWM = 80 / 1.0;
#endif

#if defined(USE_VELOCITY_CONTROL_MODE)
 static void SendMotorVelCommand(uint8_t addr, int16_t vel) {
    Wire.beginTransmission(addr);

    if (vel < -2000) {
      vel = -2000;
    } else if (vel > +2000) {
      vel = +2000;
    }

    uint8_t data[3] = {
      kVelCmd,
      (uint8_t)(vel >> 8),
      (uint8_t)(vel)
    };
    Wire.write(data, 3);

    Wire.endTransmission();
  }
#else
  static void SendMotorPwmCommand(uint8_t addr, int16_t vel) {
    Wire.beginTransmission(addr);

    if (vel < -80) {
      vel = -80;
    } else if (vel > +80) {
      vel = +80;
    }

    uint8_t data[3] = {
      kPwmCmd,
      (uint8_t)(vel >> 8),
      (uint8_t)(vel)
    };
    Wire.write(data, 3);

    Wire.endTransmission();
  }
#endif

  void Stop(void) {
#if defined(USE_VELOCITY_CONTROL_MODE)
    SendMotorVelCommand(kLLegAddr, 0);
    SendMotorVelCommand(kRLegAddr, 0);
#else
    SendMotorPwmCommand(kLLegAddr, 0);
    SendMotorPwmCommand(kRLegAddr, 0);
#endif
  }

  /*
   * in rot [rad/s]
   * out PWM value / Vel [rpm]
   */
  int16_t Linear2Motor(float vel) {
#if defined(USE_VELOCITY_CONTROL_MODE)
    return kVel2RPS * vel;
#else
    return kVel2PWM * vel;
#endif
  }

  /*
   * in rot [rad/s]
   * out PWM value
   */
  int16_t Rotary2Motor(float rot) {
    constexpr float kRadius = 0.1; // [m]
#if defined(USE_VELOCITY_CONTROL_MODE)
    return kVel2RPS * (kRadius * rot);
#else
    // 0.5[m/sec] ==> 80 (PWM value);
    return kVel2PWM * (kRadius * rot);
#endif
  }

  constexpr bool IsStop(float vel_x, float rot_z) {
    // FIXME: rot_z condition
    return (-0.01 < vel_x && vel_x < 0.01) &&
           (-0.1 < rot_z && rot_z < +0.1);
  }

  // void TwistCallback(const geometry_msgs::Twist& msg) {
    //// msg.linear.x/y/z [m/sec]
    //// msg.angular.x/y/z [rad/sec]
    // float vel_x = msg.linear.x;
    // float rot_z = msg.angular.z;
  void UpdateRequest(const float vel_x, const float rot_z) {
//    if(IsStop(vel_x, rot_z)) {
//      Stop();
//    } else {
      int16_t pwm_lin = Linear2Motor(vel_x);
      int16_t pwm_rot = Rotary2Motor(rot_z);
      // right (0x32)
      //  back : [0xFFDF,0xFFB0] := [-21,-80]
      //  front: [0x0020,0x005A] := [+32,+90]
      //                         -dead    0     +dead
      //   saturate |     back     | stop | stop  |    front     | saturate
      //      -80   | val + offset |  0   |  0    | val + offset |   +80
      int16_t left  = pwm_lin - pwm_rot;
      int16_t right = pwm_lin + pwm_rot;

#if defined(USE_VELOCITY_CONTROL_MODE)
      SendMotorVelCommand(kLLegAddr, +left);
      SendMotorVelCommand(kRLegAddr, -right);
#else
      if (left > 0) {
        if (left < 4) {
          left = 0;
        } else if (left < 30) {
          left += 30;
        }
      } else {
        if (left > -4) {
        } else if (left > -30) {
          left -= 30;
        }
      }
      if (right > 0) {
        if (right < 4) {
          right = 0;
        } else if (right < 30) {
          right += 30;
        }
      } else {
        if (right > -4) {
          right = 0;
        } else if (right > -35) {
          right -= 35;
        }
      }
      SendMotorPwmCommand(kLLegAddr, -left);
      SendMotorPwmCommand(kRLegAddr, -right);
//    }
#endif
  }
}
