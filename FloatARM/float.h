/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Mateusz Przybyla
 */

#ifndef FLOAT_H
#define FLOAT_H

#include <stdint.h>
#include "pwm.h"

/* 
 * struct MotorsData
 *
 * The following structure carries incoming data bytes
 * The structure is packed because we want to keep the
 * incoming bytes one after another. This allows us to 
 * use struct pointer directly on a memory adress.
 * The size of the structure is 16 bytes long.
 */
struct MotorsData {
  uint16_t header;
  float    left_motor_force;
  float    right_motor_force;
  float    center_motor_power;
  uint16_t crc;
} __attribute__((__packed__));
typedef struct MotorsData MotorsData;

/* 
 * enum MotorsPwmChannels
 *
 * The enum holding predefined values of channels used for PWM generation
 * by Arduino Due used by Float.
 */
typedef enum {MOTOR_LEFT = 3, MOTOR_RIGHT = 2, MOTOR_CENTER = 6} MotorsPwmChannels;

/*
 * enum MotorsPwmOffsets
 *
 * The enum holding number of samples for each of the motors that needs to be
 * set in PWM in order to synchronize with ESC but not start the motor.
 */
typedef enum {MOTOR_LEFT_OFFSET = 6462, MOTOR_RIGHT_OFFSET = 6462, MOTOR_CENTER_OFFSET = 6462} MotorsPwmOffsets;

/*
 * enum MotorsPwmScales
 *
 * The enum holding the ratio (value from MotorsData) / (number of samples for PWM). 
 * It allows to calculate the appropriate duty cycle for PWM in order to generate
 * desired motor motion.
 */
typedef enum {MOTOR_LEFT_SCALE = 3000, MOTOR_RIGHT_SCALE = 3000, MOTOR_CENTER_SCALE = 3000} MotorsPwmScales;

/*
 * setMotors(MotorsData* motors_data)
 *
 * Set appropriate PWM signals to comply with desired values 
 * given by motors_data.
 */
void setMotors(const MotorsData* motors_data);

/* 
 * stopMotors()
 *
 * Turn off all motors.
 */
void stopMotors();

#endif // FLOAT_H