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

 #include "timer.h"

void timerInit() {
  // Enable TC0 (27 is TC0)
  PMC->PMC_PCER0 = 1 << 27;
  
  // Disable TC clock
  TC0->TC_CHANNEL->TC_CCR = TC_CCR_CLKDIS;
  
  // Disable interrupts
  TC0->TC_CHANNEL->TC_IDR = 0xFFFFFFFF;
  
  // Clear status register
  TC0->TC_CHANNEL->TC_SR;
  
  // Set Mode
  TC0->TC_CHANNEL->TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5;
  
  // Compare Value
  TC0->TC_CHANNEL[0].TC_RC = 18000; // 24 = 1 ms; 18000 = 0.750 s
  
  // Configure and enable interrupt on RC compare
  NVIC_EnableIRQ((IRQn_Type) ID_TC0);
  TC0->TC_CHANNEL->TC_IER = TC_IER_CPCS;
  
  // Reset counter (SWTRG) and enable counter clock (CLKEN)
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void runHeartBeat() {
  // Enable IO on Arduino Due LED (L)
  PIOB->PIO_PER = PIO_PB27;
  
  // Set to output
  PIOB->PIO_OER = PIO_PB27;
  
  // Disable pull-up
  PIOB->PIO_PUDR = PIO_PB27;  
}

void TC0_Handler() {
  static int flag = 0;
  
  // Read status of TC0
  TC0->TC_CHANNEL->TC_SR;
  
  if (flag) 
    PIOB->PIO_SODR = PIO_PB27; 
  else 
    PIOB->PIO_CODR = PIO_PB27; 
  
  flag = !flag;
}