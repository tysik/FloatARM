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

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "sam.h"

/*
 * pwmInit(uint16_t motor_right, uint16_t motor_left, uint16_t motor_center)
 *
 * Initialize PWM peripheral for three channels used by Float. The PWM frequency 
 * is 50 Hz with resolution of 64615 samples per period. 
 * The motor channels are provided by motor_xxx argument. Float uses:
 * PORTC.PIN6 for right motor (PWML2 in peripheral B mode, pin 38 on Arduino Due)
 * PORTC.PIN8 for left motor (PWML3 in peripheral B mode, pin 40 on Arduino Due)
 * PORTC.PIN23 for middle motor (PWML6 in peripheral B mode, pin 7 on Arduino Due)
 */
void pwmInit(uint16_t motor_right, uint16_t motor_left, uint16_t motor_center);

/* 
 * pwmSetPeriod(uint16_t channel, uint16_t period)
 *
 * Set a new period for the PWM assigned at provided channel.
 * The period must be provided in samples (maximal allowed value
 * is 2^16 - 1 = 65535 samples). Note that changing the period 
 * also changes the frequency of PWMs execution.
 */
void pwmSetPeriod(uint16_t channel, uint16_t period);

/* 
 * pwmSetDuty(uint16_t channel, uint16_t duty)
 *
 * Set a new duty cycle for the PWM assigned at provided channel.
 * The duty cycle must be provided in samples (whole period is 64615
 * samples) and not in percentage of whole period.
 */
void pwmSetDuty(uint16_t channel, uint16_t duty);

#endif // PWM_H