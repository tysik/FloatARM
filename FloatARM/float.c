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

#include "float.h"

void setMotors(const MotorsData* motors_data) {
  uint16_t left = MOTOR_LEFT_OFFSET + (uint16_t) (motors_data->left_motor_force * MOTOR_LEFT_SCALE);
  uint16_t right = MOTOR_RIGHT_OFFSET + (uint16_t) (motors_data->right_motor_force * MOTOR_RIGHT_SCALE);
  uint16_t center = MOTOR_CENTER_OFFSET + (uint16_t) (motors_data->center_motor_power * MOTOR_CENTER_SCALE);

  pwmSetDuty(MOTOR_LEFT, left);
  pwmSetDuty(MOTOR_RIGHT, right);
  pwmSetDuty(MOTOR_CENTER, center);
}

void stopMotors() {
  pwmSetDuty(MOTOR_LEFT, MOTOR_LEFT_OFFSET);
  pwmSetDuty(MOTOR_RIGHT, MOTOR_RIGHT_OFFSET);
  pwmSetDuty(MOTOR_CENTER, MOTOR_CENTER_OFFSET);
}