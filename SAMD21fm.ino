#include <Arduino.h>
#include "wavetables.h"

#define SAMPLE_RATE 96000.f
#define WAVE_TABLE_SIZE 8192

struct OscData {
  // everything phase related
  float inc = 0.f;
  float phase_accumulator = 0.f;
  
  // the rest
  float waveform1 = 1;
  float waveform2 = 0;

  uint16_t crossFMint = 0;
  float crossFM = 0.f;
  float volume = 0.f;
};

volatile struct OscData osc1, osc2;
const unsigned int wave_max = WAVE_TABLE_SIZE - 1;
float sample2 = 0.f;

// ======================================================
// SETUP STUFF
// ======================================================

void setup() {
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);

  tcConfigure((int)SAMPLE_RATE);
}

void tcConfigure(int sampleRate) {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |  // Connect GCLK0 at 48MHz as a clock source for TC4 and TC5
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;

  NVIC_SetPriority(TC4_IRQn, 0);  // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);       // Connect the TC4 timer to the Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;     // Enable compare overflow (OVF) interrupt
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_WAVEGEN_MFRQ;  // Set-up TC4 timer for Match Frequency mode (MFRQ)
  TC4->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
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
  
    float value1, value2;
    uint16_t tableIndex;
    
    float sample1 = 0.f;
    float accumulator;
    
    // -------- osc 1 ---------------------
    accumulator = osc1.phase_accumulator;    
    accumulator += osc1.inc;
    
    if (osc2.crossFMint > 2) {
      accumulator += (sample2 * osc2.crossFM);
    }
    if (accumulator > wave_max) {
      accumulator -= wave_max;
    }

    tableIndex = accumulator; // implicite conversion
    value1 = sintable[tableIndex];
    value2 = sintable2[tableIndex];
    
    osc1.phase_accumulator = accumulator;
    sample1 = (value1 * osc1.waveform1 + value2 * osc1.waveform2);

    // -------- osc 2 ---------------------
    accumulator = osc2.phase_accumulator;    
    accumulator += osc2.inc;
    
    if (osc1.crossFMint > 2) {
      accumulator += (sample1 * osc1.crossFM);
    }

    if (accumulator > wave_max) {
      accumulator -= wave_max;
    }

    tableIndex = accumulator;
    value1 = sintable[tableIndex];  
    value2 = sintable2[tableIndex];
    
    osc2.phase_accumulator = accumulator;
    sample2 = (value1 * osc2.waveform1 + value2 * osc2.waveform2);
     
    DAC->DATA.reg =  (sample1*osc1.volume + sample2*osc2.volume);

    TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
}

// ======================================================
// MAIN LOOP
// ======================================================

uint16_t counter = 0;
const float incFactor = WAVE_TABLE_SIZE / SAMPLE_RATE;

void loop() {
  counter++;

  if (counter > 100) { // we not so often sample the inputs but I don't want to use delay() which might interact with the interrupt
    // ===================================================
    // reading the inputs
    // ===================================================

    uint16_t frequency = analogRead(A1)*2;// + analogRead(A4);// A4 = analog feedback
    osc1.inc = frequency * incFactor;

    float waveform = analogRead(A2) / 4095.f;
    osc1.waveform2 = waveform;
    osc1.waveform1 = 1.f - waveform;

    uint16_t cross = analogRead(A3);
    osc1.crossFMint = cross;
    osc1.crossFM = cross / 2046.f;
    osc1.volume = analogRead(5) / 8190.f;

    
    frequency = analogRead(A6)*2;// + analogRead(A9);// A9 = analog feedback
    osc2.inc = frequency * incFactor;
    
    waveform = analogRead(A7) / 4095.f;
    osc2.waveform2 = waveform;
    osc2.waveform1 = 1.f - waveform;

    cross = analogRead(A8);
    osc2.crossFMint = cross;
    osc2.crossFM = cross / 2046.f;
    osc2.volume = analogRead(10) / 8190.f;
  
    counter = 0;
  }
}