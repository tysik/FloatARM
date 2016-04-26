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

#include "uart.h"

void uartInit() {
  // Disable interrupts on Rx and Tx
  PIOA->PIO_IDR |= PIO_PA8A_URXD | PIO_PA9A_UTXD;
  
  // Disable the PIO of the Rx and Tx pins so that the peripheral controller can use them
  PIOA->PIO_PDR |= PIO_PA8A_URXD | PIO_PA9A_UTXD;
  
  // Set peripheral AB select register Rx and Tx pins to 0 (Peripheral A function)
  PIOA->PIO_ABSR &= ~(PIO_PA8A_URXD | PIO_PA9A_UTXD);
  
  // Enable the pull up on the Rx and Tx pin
  PIOA->PIO_PUER |= PIO_PA8A_URXD | PIO_PA9A_UTXD;
  
  // Enable the peripheral uart controller
  PMC->PMC_PCER0 |= 1 << ID_UART;
  
  // Reset and disable receiver & transmitter
  UART->UART_CR |= UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
  
  // Set the baudrate to 115200
  UART->UART_BRGR = 45; // 84000000 / 16 * x = BaudRate (write x into UART_BRGR)
  
  // No Parity
  UART->UART_MR |= UART_MR_PAR_NO;
  
  // Disable PDC channel requests
  UART->UART_PTCR |= UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  
  // Disable / Enable interrupts on end of receive
  UART->UART_IDR = 0xFFFFFFFF;
  NVIC_EnableIRQ((IRQn_Type) ID_UART);
  UART->UART_IER |= UART_IER_RXRDY;
  
  // Enable receiver and trasmitter
  UART->UART_CR |= UART_CR_RXEN | UART_CR_TXEN;
}

void uartPutChar(uint8_t c) {
  // Wait for the transmitter to be ready
  while (!(UART->UART_SR & UART_SR_TXRDY));
  
  // Send the character
  UART->UART_THR = c;
}

uint8_t uartGetChar() {
  // Wait for the receiver to be ready
  while (!(UART->UART_SR & UART_SR_RXRDY));

  // Read the character
  return (uint8_t) UART->UART_RHR;
}

uint16_t uartCRC(uint8_t* data, uint32_t length) {
  uint16_t crc = 0;
  uint8_t aux[2] = {0, 0};

  for (int i = 0; i < length; ++i) {
    aux[1] = aux[0];
    aux[0] = data[i];

    if (crc & 0x8000) {
      crc = (crc & 0x7fff) * 2;
      crc ^= 0x8005;
    }
    else 
      crc *= 2;

    crc ^= aux[0] + aux[1] * 256;
  }

  return crc;
}

/*
 * UART_Handler()
 *
 * Callback function used by UART interrupt.
 */
void UART_Handler() {
  static uint8_t idx = 0;             // Index of received bytes
  static MotorsData* motors_data;     // Holder for received data
  static uint8_t rx_buffer[sizeof(MotorsData)];

  // Check if the interrupt source is receive ready
  if (UART->UART_IMR & UART_IMR_RXRDY) {
    // Fill buffer with incoming byte
    rx_buffer[idx] = uartGetChar();
    
    // Restart listening when one of the header bytes is incorrect
    if (idx == 0 && rx_buffer[idx] != 0xAA) {
      idx = 0;
      return;
    }
    else if (idx == 1 && rx_buffer[idx] != 0xBB) {
      idx = 0;
      return;
    }
        
    // Check if the buffer is full
    if (idx >= sizeof(MotorsData) - 1) {
      // Convert raw data
      motors_data = (MotorsData*) rx_buffer;

      // Check CRC (12 data bytes; +2 to omit header)
      if (motors_data->crc == uartCRC(rx_buffer + 2, 12)) 
        setMotors(motors_data);
      else 
        stopMotors();

      // Restart listening
      idx = 0;
      return;
    }
    
    idx++;
   }
}
