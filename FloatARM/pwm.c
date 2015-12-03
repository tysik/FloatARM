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

#include "pwm.h"

void pwmSetPeriod(PwmChannels channel, uint16_t period) {
  // If the channel is disabled write period to the period register
  // If the channel is enabled write period to the update period register
  if ((PWM->PWM_SR & (1 << channel)) == 0)
    PWM->PWM_CH_NUM[channel].PWM_CPRD = period;
  else
    PWM->PWM_CH_NUM[channel].PWM_CPRDUPD = period;
}

void pwmSetDuty(PwmChannels channel, uint16_t duty) {
  // If the channel is disabled write duty to the duty register
  // If the channel is enabled write duty to the update duty register
  if ((PWM->PWM_SR & (1 << channel)) == 0)
    PWM->PWM_CH_NUM[channel].PWM_CDTY = duty;
  else
    PWM->PWM_CH_NUM[channel].PWM_CDTYUPD = duty;
}

void pwmInit() {
  // PORTC.PIN6 for right motor (PWML2 in peripheral B mode, pin 38 on Arduino Due)
  // PORTC.PIN8 for left motor (PWML3 in peripheral B mode, pin 40 on Arduino Due)
  // PORTC.PIN23 for middle motor (PWML6 in peripheral B mode, pin 7 on Arduino Due)

  // Enable the clock for PORTC line
  PMC->PMC_PCER0 |= 1 << ID_PIOC;

  // Disable PIO Control on PC{6,8,23} and set up for Peripheral B
  PIOC->PIO_PDR  |= PIO_PC6 | PIO_PC8 | PIO_PC23;
  PIOC->PIO_ABSR |= PIO_PC6 | PIO_PC8 | PIO_PC23;

  // Enable output on PC{6,8,23}
  PIOC->PIO_OER  |= PIO_PC6 | PIO_PC8 | PIO_PC23;

  // Enable pull-up on pin
  PIOC->PIO_PUDR |= PIO_PC6 | PIO_PC8 | PIO_PC23;

  // Enable the PWM clock (ID_PWM = 36)
  PMC->PMC_PCER1 |= 1 << (ID_PWM - 32);

  // Configure clock A
  // Calculating the frequency MCK / (PREA x DIVA)
  // freq = 84000000 / (2 X 13) = 3230769.23077 Hz
  PWM->PWM_CLK = 13 | (1 << 8);

  // Channel Mode Register - Use clock A as configured in PWM_CLK
  PWM->PWM_CH_NUM[MOTOR_RIGHT].PWM_CMR  |= PWM_CMR_CPRE_CLKA;
  PWM->PWM_CH_NUM[MOTOR_LEFT].PWM_CMR   |= PWM_CMR_CPRE_CLKA;
  PWM->PWM_CH_NUM[MOTOR_MIDDLE].PWM_CMR |= PWM_CMR_CPRE_CLKA;
      
  // Final frequency = 3230769.23077 / 64615 = 50.0002 Hz
  pwmSetPeriod(MOTOR_RIGHT, 64615);
  pwmSetPeriod(MOTOR_LEFT, 64615);
  pwmSetPeriod(MOTOR_MIDDLE, 64615);
      
  // Duty cycle = ratio of Period to Duty (Duty / Period) * 100 = 10%
  pwmSetDuty(MOTOR_RIGHT, 6462);
  pwmSetDuty(MOTOR_LEFT, 6462);
  pwmSetDuty(MOTOR_MIDDLE, 6462);
      
  // Enable channels
  PWM->PWM_ENA |= (1 << MOTOR_RIGHT) | (1 << MOTOR_LEFT) | (1 << MOTOR_MIDDLE);
}