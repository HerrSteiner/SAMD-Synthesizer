/*
SAMD21 FM drone synthesizer
Copyright (C) 2023 Malte Steiner

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "wavetablesFixedPoint.h"

#define SAMPLE_RATE 48000
#define WAVE_TABLE_SIZE 8192

const uint32_t maxAnalogIn = 4095;
const uint32_t shiftfactor = 1024;

struct OscData {
  // everything phase related
  uint32_t inc = 0;
  uint32_t phase_accumulator = 0;

  // the rest
  uint32_t waveform1 = maxAnalogIn;
  uint32_t waveform2 = 0;

  uint32_t crossFMint = 0;
  uint32_t crossFM = 0;
  uint32_t volume = 0;
};

volatile struct OscData osc1, osc2;
const unsigned int wave_max = WAVE_TABLE_SIZE - 1;
const unsigned int wave_max_shifted = WAVE_TABLE_SIZE * shiftfactor;
uint32_t sample2 = 0;

// ======================================================
// SETUP STUFF
// ======================================================

void setup() {
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);

  tcConfigure(SAMPLE_RATE);
}

void tcConfigure(int sampleRate) {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |  // Connect GCLK0 at 48MHz as a clock source for TC4 and TC5
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;

  NVIC_SetPriority(TC4_IRQn, 0);  // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);       // Connect the TC4 timer to the Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;     // Enable compare overflow (OVF) interrupt
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_WAVEGEN_MFRQ;  // Set-up TC4 timer for Match Frequency mode (MFRQ)
  TC4->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / sampleRate - 1);
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;  // Enable timer TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
}

// ======================================================
// AUDIO GENERATION INTERRUPT
// ======================================================

void TC4_Handler() {

  uint32_t value1, value2;
  uint32_t tableIndex;

  uint32_t sample1 = 0.f;
  uint32_t accumulator;

  // -------- osc 1 ---------------------
  accumulator = osc1.phase_accumulator;
  accumulator += osc1.inc;

  if (accumulator >= wave_max_shifted) {
    accumulator -= wave_max_shifted;
  }

  accumulator += (((sample2 * osc2.crossFM) / maxAnalogIn) << 10);
  if (accumulator >= wave_max_shifted) {
    accumulator -= wave_max_shifted;
  }


  tableIndex = accumulator >> 10;
  value1 = sinetable[tableIndex];
  value2 = sine2table[tableIndex];

  osc1.phase_accumulator = accumulator;
  sample1 = (((value1 * osc1.waveform1) >> 6) + ((value2 * osc1.waveform2) >> 6)) / maxAnalogIn;

  // -------- osc 2 ---------------------
  accumulator = osc2.phase_accumulator;
  accumulator += osc2.inc;

  if (accumulator >= wave_max_shifted) {
    accumulator -= wave_max_shifted;
  }

  accumulator += ((sample1 * osc1.crossFM) / maxAnalogIn) << 10;
  if (accumulator >= wave_max_shifted) {
    accumulator -= wave_max_shifted;
  }

  tableIndex = accumulator >> 10;
  value1 = sinetable[tableIndex];
  value2 = sawtable[tableIndex];

  osc2.phase_accumulator = accumulator;
  sample2 = (((value1 * osc2.waveform1) >> 6) + ((value2 * osc2.waveform2) >> 6)) / maxAnalogIn;

 // -------- output ---------------------
  DAC->DATA.reg = ((sample1 * osc1.volume + sample2 * osc2.volume) / maxAnalogIn) >> 1;

  TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
}

// ======================================================
// MAIN LOOP
// ======================================================

uint16_t counter = 0;

void loop() {
  counter++;

  if (counter > 1000) {  // we not so often sample the inputs but I don't want to use delay() which might interact with the interrupt
    // ===================================================
    // reading the inputs
    // ===================================================

    osc1.inc = (((analogRead(A1) << 12) + (analogRead(A4) << 13)) / SAMPLE_RATE) << 10;

    uint32_t waveform = analogRead(A2);
    osc1.waveform2 = waveform;
    osc1.waveform1 = maxAnalogIn - waveform;

    osc1.crossFM = analogRead(A3);
    osc1.volume = analogRead(5);

    osc2.inc = (((analogRead(A6) << 12) + (analogRead(A9) << 13)) / SAMPLE_RATE) << 10;  // <<13 is the same as * WAVE_TABLE_SIZE

    waveform = analogRead(A7);
    osc2.waveform2 = waveform;
    osc2.waveform1 = maxAnalogIn - waveform;

    osc2.crossFM = analogRead(A8);
    osc2.volume = analogRead(10);

    counter = 0;
  }
}