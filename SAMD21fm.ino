#include <Arduino.h>
//#include <TimerTC3.h>
//#define TIMER TimerTc3
#include "wavetables.h"

#define SAMPLE_RATE 32000.f
#define WAVE_TABLE_SIZE 4096

struct OscData {
  // everything phase related
  //float frequency = 0.f;
  float inc = 0.f;
  float phase_accumulator = 0.f;
  uint16_t phaseInt = 0;

  // the rest
  uint16_t waveform1 = 4095;
  uint16_t waveform2 = 0;

  uint16_t crossFMint = 0;
  float crossFM = 0.f;
  float volume = 0.f;
};

volatile struct OscData osc1, osc2;
volatile bool ready = false;  // Ready flag
volatile float stepSize;
volatile float x = 0.0f;
const unsigned int wave_max = WAVE_TABLE_SIZE - 1;
float sample2 = 0.f;
//float current_freq = 16.35;
//float mod_freq = 16;

/// THE WAVE TABLES, 1024 + 1 wrap around
//volatile int16_t sine[WAVE_TABLE_SIZE];

// ======================================================
// SETUP STUFF
// ======================================================



void setup() {

  //noInterrupts();
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);

  //TIMER.initialize((long)round(1000000 / SAMPLE_RATE));
  //TIMER.attachInterrupt(audioISR);
  //interrupts();
  /*
  for (uint16_t i = 0; i < WAVE_TABLE_SIZE; i++)  // Calculate the sine table with 1024 entries
  {
    sine[i] = (int16_t)(sinf(2 * PI * (float)i / WAVE_TABLE_SIZE) * 512.0f);  // + 512.0f);    // 10-bit resolution with 0V offset
  }*/
  tcConfigure((int)SAMPLE_RATE);
}

// ======================================================
// AUDIO GENERATION INTERRUPT
// ======================================================

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

void TC4_Handler() {
  
  signed int value1, value2;
    uint16_t indexBase;
    float indexFract;
    float modSample1 = 0.f;
    float modSample2 = 0.f;

    float sample1 = 0.f;
    
    float accumulator;
    
    // -------- osc 1 ---------------------
    accumulator = osc1.phase_accumulator;    
    accumulator += osc1.inc;
    
    if (osc2.crossFMint > 2) {
      accumulator += (sample2 * osc2.crossFM) / 1023.f;
    }
    if (accumulator > wave_max) {
      accumulator -= wave_max;
    }

    indexBase = accumulator;
    //uint16_t i = indexBase + 1;
    //uint16_t mod = i % WAVE_TABLE_SIZE;
    //indexFract = accumulator - indexBase;
    uint16_t v1 = sintable[indexBase];
    //uint16_t v2 = sintable[mod];
    float wave1 = osc1.waveform1 / 4095.f;
    float wave2 = osc1.waveform2 / 4095.f;
    
    uint16_t sample = v1;

    v1 = sintable2[indexBase];
    //v2 = sintable2[mod];
    uint16_t sampleT = v1;

    osc1.phase_accumulator = accumulator;
    sample1 = (sample * wave1 + sampleT * wave2);

    // -------- osc 2 ---------------------
    accumulator = osc2.phase_accumulator;    
    accumulator += osc2.inc;
    
    if (osc1.crossFMint > 2) {
      accumulator += (sample1 * osc1.crossFM) / 1023.f;
    }

    if (accumulator > wave_max) {
      accumulator -= wave_max;
    }

    indexBase = accumulator;
    //i = indexBase + 1;
    //mod = i % WAVE_TABLE_SIZE;
    //indexFract = accumulator - indexBase;
    v1 = sintable[indexBase];
    //v2 = sintable[mod];
    wave1 = osc2.waveform1 / 4095.f;
    wave2 = osc2.waveform2 / 4095.f;
    
    sample = v1;

    v1 = sintable2[indexBase];
    //v2 = sintable2[mod];
    sampleT = v1;

    osc2.phase_accumulator = accumulator;
    sample2 = (sample * wave1 + sampleT * wave2);
     
    //analogWrite(A0, sample);
    DAC->DATA.reg =  (sample1*osc1.volume + sample2*osc2.volume) * 0.5f;

    TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
  //ready = true;                      // Set the ready flag
}

// 32 bits
//float phase_accumulator = 0.f;
//int mainPhaseInt = 0;
//float phase_incr = 1.f;
//float mod_accumulator = 0.f;
//float mod_incr = 1.f;
//int fmInt = 0;
//float fm = 0.f;
float modSample1 = 0.f;
float modSample2 = 0.f;

float sample1 = 0.f;
//float sample2 = 0.f;

//float waveform1 = 1.f;
//float waveform2 = 0.f;
//float cross = 0.f;


float mapf(long x, long in_min, long in_max, long out_min, long out_max) {
  long first = (x - in_min) * (out_max - out_min);
  long second = (in_max - in_min) + out_min;
  return first / (float)second;
}

// ======================================================
// MAIN LOOP
// ======================================================

//float inc = 0.f;
//float modinc = 0.f;

uint16_t counter = 0;
const float incFactor = WAVE_TABLE_SIZE / SAMPLE_RATE;

void loop() {
  counter++;

  if (ready) {

    /*
    uint16_t x0 = x;                                          // Determine the lower wavetable index
    uint16_t x1 = x + 1;                                      // Determine the upper wavetable index  
    int16_t y0 = sine[x0];                                    // Read the lower wavetable value
    int16_t y1 = sine[x1 % WAVE_TABLE_SIZE];                        // Read the upper wavetable value
    int16_t y = y0 * (x1 - x) + y1 * (x - x0);                // Use linear interpolation: y = y0 * (x1 - x) + y1 * (x - x0) / (x1 - x0);
    x += stepSize;                                            // Increment the step count
    x = x > 1024.0f ? x - 1024.0f : x;                        // Adjust step count for overflow
    
    //y = y * volume / 1023;                                    // Adjust for volume
    y += 512;                                                 // Add +1.65V DC offset
    y = constrain(y, 0, 1023);                                // Constrain the output limits
    DAC->DATA.reg = y;                                        // Update the DAC
    ready = false;                                            // Clear the ready flag
    */
    // ===================================================
    // process audio
    // ===================================================
    signed int value1, value2;
    uint16_t indexBase;
    float indexFract;
    float modSample1 = 0.f;
    float modSample2 = 0.f;

    float sample1 = 0.f;
    float sample2 = 0.f;
    float accumulator;
    /*
    osc2.phase_accumulator += osc2.inc;
    if (osc2.phase_accumulator > wave_max) {
      osc2.phase_accumulator -= wave_max;
    }
    //osc2.phaseInt = round(osc2.phase_accumulator);

    indexBase = osc2.phase_accumulator;
    uint16_t i = indexBase + 1;
    indexFract = osc2.phase_accumulator - indexBase;
    value1 = sintable[indexBase];
    value2 = sintable[indexBase + 1];
    modSample1 = value1 + ((value2 - value1) * indexFract);

    value1 = sintable2[indexBase];
    value2 = sintable2[indexBase + 1];
    modSample2 = value1 + ((value2 - value1) * indexFract);

    modSample1 = modSample1 * osc2.waveform1 + modSample2 * osc2.waveform2;


    if (osc2.crossFMint > 0) {
      osc1.phase_accumulator += (modSample1 * osc2.crossFM) / 1023.f;
    }
    */
    accumulator = osc1.phase_accumulator;    
    accumulator += osc1.inc;
    if (accumulator > wave_max) {
      accumulator -= wave_max;
    }

    //osc1.phaseInt = round(osc1.phase_accumulator);
    indexBase = accumulator;
    uint16_t i = indexBase + 1;
    uint16_t mod = i % WAVE_TABLE_SIZE;
    indexFract = accumulator - indexBase;
    uint16_t v1 = sintable[indexBase];
    uint16_t v2 = sintable[mod];
    float wave1 = osc1.waveform1 / 4095.f;
    float wave2 = osc1.waveform2 / 4095.f;
    //uint16_t iMinusA = i - accumulator;
    //uint16_t aMinusI = accumulator - indexBase;    
    //sample1 = value1 * (1.f - indexFract) + value2 * indexFract;
    //sample1 = value1 + ((value2 - value1) * indexFract);

    //value1 = sintable2[indexBase];
    //value2 = sintable2[indexBase + 1];
    //sample2 = value1 + ((value2 - value1) * indexFract);

    //sample1 = sample1 * osc1.waveform1 + (sample2 * osc1.waveform2);
    uint16_t sample = v1 * (i - accumulator) + v2 * (accumulator - indexBase);  //= round((sample1 * osc1.volume + modSample1 * osc2.volume) * 0.4f);

    v1 = sintable2[indexBase];
    v2 = sintable2[mod];
    uint16_t sampleT = v1 * (i - accumulator) + v2 * (accumulator - indexBase);

    osc1.phase_accumulator = accumulator;

    //analogWrite(A0, sample);
    DAC->DATA.reg = (sample * wave1 + sampleT * wave2)*osc1.volume ;
    /*
    if (osc1.crossFMint > 0) {
      osc2.phase_accumulator += (sample1 * osc1.crossFM) / 1023.f;
    }*/
    
    ready = false;    
  } 
  else if (counter > 100) {
    // ===================================================
    // reading the inputs
    // ===================================================

    //osc1.inc = analogRead(A1) * WAVE_TABLE_SIZE / SAMPLE_RATE;
    uint16_t frequency = analogRead(A1) + analogRead(A4);  //mapf(analogRead(A1), 0, 4095, 20, 2200); //+ mapf(analogRead(A4), 0, 4095, 0, 2600);  // A4 = analog feedback
    osc1.inc = frequency * incFactor;

    uint16_t waveform = analogRead(A2);
    osc1.waveform2 = waveform;
    osc1.waveform1 = 4095 - waveform;

    uint16_t cross = analogRead(A3);
    osc1.crossFMint = cross;
    osc1.crossFM = cross / 2.f;
    osc1.volume = analogRead(5) / 4095.f;

    
    frequency = analogRead(A6) + analogRead(A9);  // + analogRead(9); //mapf( analogRead(A9) , 0, 4095, 0, 2600); // A9 = analog feedback
    osc2.inc = frequency * incFactor;
    
    waveform = analogRead(A7);
    osc2.waveform2 = waveform;
    osc2.waveform1 = 4095 - waveform;

    cross = analogRead(A8);
    osc2.crossFMint = cross;
    osc2.crossFM = cross / 2.f;
    osc2.volume = analogRead(10) / 4095.f;

  
    counter = 0;
  }
}