/* Header for ESP32MotorControl
 *
 * Copyright (C) 2018  Joao Lopes
 * https://github.com/JoaoLopesF/ESP32MotorControl
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This header file describes the public API for SerialDebug.
 *
 */

#ifndef ESP32MotorControl_H
#define ESP32MotorControl_H

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "esp_attr.h"
#include "esp_system.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_log.h"

//////// Defines
#define LEDC_TIMER              LEDC_TIMER_1 // Timer do LEDC utilizado
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE // Modo de velocidade do LEDC
#define PWM_A_PIN               LEDC_CHANNEL_2 // Canal do LEDC utilizado
#define PWM_B_PIN               LEDC_CHANNEL_3 // Canal do LEDC utilizado
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Resolução do PWM
#define LEDC_FREQUENCY          2000 // Frequência em Hertz do sinal PWM


//////// Class

class ESP32MotorControl {
public:
  // Fields
  bool mMotorForward[2] = {true, true};

  // Methods:
  void setSTBY(uint8_t _gpioSTBY);

  void attachMotors(uint8_t _gpioAIN1, uint8_t _gpioAIN2, uint8_t _gpioPWMA,
                    uint8_t _gpioBIN1, uint8_t _gpioBIN2, uint8_t _gpioPWMB);

  void motorSpeed(uint8_t motor, float speed);
  void motorFullForward(uint8_t motor);
  void motorForward(uint8_t motor);
  void motorFullReverse(uint8_t motor);
  void motorReverse(uint8_t motor);
  void motorStop(uint8_t motor);

  void motorsStop();

  void handle();

  uint8_t getMotorSpeed(uint8_t motor);
  bool isMotorForward(uint8_t motor);
  bool isMotorStopped(uint8_t motor);

private:
  uint8_t gpioSTBY = 0, gpioAIN1 = 0, gpioAIN2 = 0, gpioPWMA = 0, gpioBIN1 = 0,
          gpioBIN2 = 0, gpioPWMB = 0;

  // Fields:

  bool mMotorAttached[2] = {false, false};

  enum MotorState {
    MOTOR_STOPPED,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
  };

  MotorState mMotorState[2] = {MOTOR_STOPPED, MOTOR_STOPPED};

  // Methods

  bool isMotorValid(uint8_t motor);
  void InitMotorPWM(gpio_num_t pin, ledc_channel_t channel);
  void PwmWrite(ledc_channel_t channel, int pwm);
};

#endif // ESP32MotorControl_H