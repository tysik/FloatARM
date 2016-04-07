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
  // Pin B.27 (Arduino LED)
  // Turn on clock for line B
  PMC->PMC_PCER0 |= ID_PIOB;

  // Disable PIO and select peripheral B
  PIOB->PIO_PDR |= PIO_PB27;
  PIOB->PIO_ABSR |= PIO_PB27;

  // Enable pull-up
  PIOB->PIO_PUDR |= PIO_PB27;

  // We need to enable clock for Timer 0
  PMC->PMC_PCER0 |= ID_TC0;
    
  // Channel Mode Register 0
  TC0->TC_CHANNEL[0].TC_CMR =
  TC_CMR_WAVE |                   //Wave mode
  TC_CMR_TCCLKS_TIMER_CLOCK2 |    //MCK / 8
  TC_CMR_WAVSEL_UP |              //UP mode without automatic trigger on RC Compare
  TC_CMR_ACPA_TOGGLE |            //RA Compare Effect on TIOA (Toggle)
  TC_CMR_ACPC_TOGGLE |            //RC Compare Effect on TIOA (Toggle)
  TC_CMR_CPCTRG;                  //Compare RC Trigger: trigger when counter value matches the RC value
    
  //Set compare value in Register A and C
  TC0->TC_CHANNEL[0].TC_RA = 1000000;         //RA controls the duty cycle
  TC0->TC_CHANNEL[0].TC_RC = 10000000;         //RC controls the frequency
    
  //Enables the clock a software trigger is performed: the counter is reset and the clock is started
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}