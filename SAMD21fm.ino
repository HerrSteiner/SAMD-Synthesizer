#include <Arduino.h>
#include <TimerTC3.h>
#define TIMER TimerTc3

#define SAMPLE_RATE (16000.f)
#define WAVE_TABLE_SIZE 1024

struct OscData {
  // everything phase related
  float frequency = 0.f;
  float inc = 0.f;
  float phase_accumulator = 0.f;
  int phaseInt = 0;

  // the rest
  float waveform1 = 1.f;
  float waveform2 = 0.f;

  int crossFMint = 0;
  float crossFM = 0.f;
  float volume = 0.f;
};

volatile struct OscData osc1, osc2;

//float current_freq = 16.35;
//float mod_freq = 16;

/// THE WAVE TABLES, 1024 + 1 wrap around
const int16_t sintable[1025] = { 512, 515, 518, 521, 524, 527, 530, 533, 537, 540, 543, 546, 549, 552, 555, 559, 562, 565, 568, 571, 574, 577, 580, 583, 587, 590, 593, 596, 599, 602, 605, 608, 611, 614, 617, 621, 624, 627, 630, 633, 636, 639, 642, 645, 648, 651, 654, 657, 660, 663, 666, 669, 672, 675, 678, 681, 684, 687, 690, 693, 696, 699, 701, 704, 707, 710, 713, 716, 719, 722, 725, 727, 730, 733, 736, 739, 741, 744, 747, 750, 753, 755, 758, 761, 764, 766, 769, 772, 774, 777, 780, 782, 785, 788, 790, 793, 796, 798, 801, 803, 806, 809, 811, 814, 816, 819, 821, 824, 826, 829, 831, 834, 836, 838, 841, 843, 846, 848, 850, 853, 855, 857, 860, 862, 864, 866, 869, 871, 873, 875, 878, 880, 882, 884, 886, 888, 890, 893, 895, 897, 899, 901, 903, 905, 907, 909, 911, 913, 915, 917, 919, 920, 922, 924, 926, 928, 930, 931, 933, 935, 937, 939, 940, 942, 944, 945, 947, 949, 950, 952, 953, 955, 957, 958, 960, 961, 963, 964, 966, 967, 968, 970, 971, 973, 974, 975, 977, 978, 979, 980, 982, 983, 984, 985, 986, 988, 989, 990, 991, 992, 993, 994, 995, 996, 997, 998, 999, 1000, 1001, 1002, 1003, 1004, 1004, 1005, 1006, 1007, 1008, 1008, 1009, 1010, 1011, 1011, 1012, 1013, 1013, 1014, 1014, 1015, 1015, 1016, 1017, 1017, 1017, 1018, 1018, 1019, 1019, 1020, 1020, 1020, 1021, 1021, 1021, 1021, 1022, 1022, 1022, 1022, 1022, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1022, 1022, 1022, 1022, 1022, 1021, 1021, 1021, 1021, 1020, 1020, 1020, 1019, 1019, 1018, 1018, 1017, 1017, 1017, 1016, 1015, 1015, 1014, 1014, 1013, 1013, 1012, 1011, 1011, 1010, 1009, 1008, 1008, 1007, 1006, 1005, 1004, 1004, 1003, 1002, 1001, 1000, 999, 998, 997, 996, 995, 994, 993, 992, 991, 990, 989, 988, 986, 985, 984, 983, 982, 980, 979, 978, 977, 975, 974, 973, 971, 970, 968, 967, 966, 964, 963, 961, 960, 958, 957, 955, 953, 952, 950, 949, 947, 945, 944, 942, 940, 939, 937, 935, 933, 931, 930, 928, 926, 924, 922, 920, 919, 917, 915, 913, 911, 909, 907, 905, 903, 901, 899, 897, 895, 893, 890, 888, 886, 884, 882, 880, 878, 875, 873, 871, 869, 866, 864, 862, 860, 857, 855, 853, 850, 848, 846, 843, 841, 838, 836, 834, 831, 829, 826, 824, 821, 819, 816, 814, 811, 809, 806, 803, 801, 798, 796, 793, 790, 788, 785, 782, 780, 777, 774, 772, 769, 766, 764, 761, 758, 755, 753, 750, 747, 744, 741, 739, 736, 733, 730, 727, 725, 722, 719, 716, 713, 710, 707, 704, 701, 699, 696, 693, 690, 687, 684, 681, 678, 675, 672, 669, 666, 663, 660, 657, 654, 651, 648, 645, 642, 639, 636, 633, 630, 627, 624, 621, 617, 614, 611, 608, 605, 602, 599, 596, 593, 590, 587, 583, 580, 577, 574, 571, 568, 565, 562, 559, 555, 552, 549, 546, 543, 540, 537, 533, 530, 527, 524, 521, 518, 515, 512, 508, 505, 502, 499, 496, 493, 490, 486, 483, 480, 477, 474, 471, 468, 464, 461, 458, 455, 452, 449, 446, 443, 440, 436, 433, 430, 427, 424, 421, 418, 415, 412, 409, 406, 402, 399, 396, 393, 390, 387, 384, 381, 378, 375, 372, 369, 366, 363, 360, 357, 354, 351, 348, 345, 342, 339, 336, 333, 330, 327, 324, 322, 319, 316, 313, 310, 307, 304, 301, 298, 296, 293, 290, 287, 284, 282, 279, 276, 273, 270, 268, 265, 262, 259, 257, 254, 251, 249, 246, 243, 241, 238, 235, 233, 230, 227, 225, 222, 220, 217, 214, 212, 209, 207, 204, 202, 199, 197, 194, 192, 189, 187, 185, 182, 180, 177, 175, 173, 170, 168, 166, 163, 161, 159, 157, 154, 152, 150, 148, 145, 143, 141, 139, 137, 135, 133, 130, 128, 126, 124, 122, 120, 118, 116, 114, 112, 110, 108, 106, 104, 103, 101, 99, 97, 95, 93, 92, 90, 88, 86, 84, 83, 81, 79, 78, 76, 74, 73, 71, 70, 68, 66, 65, 63, 62, 60, 59, 57, 56, 55, 53, 52, 50, 49, 48, 46, 45, 44, 43, 41, 40, 39, 38, 37, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 19, 18, 17, 16, 15, 15, 14, 13, 12, 12, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 19, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40, 41, 43, 44, 45, 46, 48, 49, 50, 52, 53, 55, 56, 57, 59, 60, 62, 63, 65, 66, 68, 70, 71, 73, 74, 76, 78, 79, 81, 83, 84, 86, 88, 90, 92, 93, 95, 97, 99, 101, 103, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 133, 135, 137, 139, 141, 143, 145, 148, 150, 152, 154, 157, 159, 161, 163, 166, 168, 170, 173, 175, 177, 180, 182, 185, 187, 189, 192, 194, 197, 199, 202, 204, 207, 209, 212, 214, 217, 220, 222, 225, 227, 230, 233, 235, 238, 241, 243, 246, 249, 251, 254, 257, 259, 262, 265, 268, 270, 273, 276, 279, 282, 284, 287, 290, 293, 296, 298, 301, 304, 307, 310, 313, 316, 319, 322, 324, 327, 330, 333, 336, 339, 342, 345, 348, 351, 354, 357, 360, 363, 366, 369, 372, 375, 378, 381, 384, 387, 390, 393, 396, 399, 402, 406, 409, 412, 415, 418, 421, 424, 427, 430, 433, 436, 440, 443, 446, 449, 452, 455, 458, 461, 464, 468, 471, 474, 477, 480, 483, 486, 490, 493, 496, 499, 502, 505, 508, 512 };
const int16_t sintable2[1025] = { 512, 518, 524, 530, 537, 543, 549, 555, 562, 568, 574, 580, 586, 593, 599, 605, 611, 617, 623, 629, 635, 641, 647, 653, 659, 665, 671, 676, 682, 688, 694, 699, 705, 710, 716, 721, 727, 732, 737, 743, 748, 753, 758, 763, 768, 773, 778, 783, 788, 792, 797, 801, 806, 810, 815, 819, 823, 828, 832, 836, 840, 844, 847, 851, 855, 858, 862, 865, 869, 872, 875, 879, 882, 885, 888, 890, 893, 896, 898, 901, 903, 906, 908, 910, 912, 914, 916, 918, 920, 922, 923, 925, 926, 927, 929, 930, 931, 932, 933, 934, 935, 935, 936, 936, 937, 937, 937, 938, 938, 938, 938, 937, 937, 937, 937, 936, 936, 935, 934, 933, 933, 932, 931, 930, 928, 927, 926, 925, 923, 922, 920, 918, 917, 915, 913, 911, 909, 907, 905, 903, 900, 898, 896, 893, 891, 888, 886, 883, 880, 878, 875, 872, 869, 866, 863, 860, 857, 854, 851, 847, 844, 841, 837, 834, 831, 827, 824, 820, 816, 813, 809, 806, 802, 798, 794, 791, 787, 783, 779, 775, 772, 768, 764, 760, 756, 752, 748, 744, 740, 736, 732, 728, 724, 720, 716, 712, 708, 704, 700, 697, 693, 689, 685, 681, 677, 673, 669, 665, 661, 657, 654, 650, 646, 642, 638, 635, 631, 627, 624, 620, 616, 613, 609, 606, 602, 599, 595, 592, 589, 585, 582, 579, 576, 572, 569, 566, 563, 560, 557, 554, 551, 548, 546, 543, 540, 538, 535, 532, 530, 527, 525, 523, 520, 518, 516, 514, 512, 509, 507, 505, 504, 502, 500, 498, 496, 495, 493, 492, 490, 489, 487, 486, 485, 484, 482, 481, 480, 479, 478, 477, 477, 476, 475, 474, 474, 473, 473, 472, 472, 471, 471, 471, 471, 470, 470, 470, 470, 470, 470, 470, 471, 471, 471, 471, 472, 472, 473, 473, 474, 474, 475, 476, 476, 477, 478, 479, 479, 480, 481, 482, 483, 484, 485, 486, 488, 489, 490, 491, 492, 494, 495, 496, 498, 499, 500, 502, 503, 505, 506, 508, 509, 511, 513, 514, 516, 517, 519, 521, 522, 524, 526, 527, 529, 531, 532, 534, 536, 538, 539, 541, 543, 544, 546, 548, 550, 551, 553, 555, 556, 558, 560, 562, 563, 565, 566, 568, 570, 571, 573, 574, 576, 578, 579, 581, 582, 584, 585, 586, 588, 589, 591, 592, 593, 595, 596, 597, 598, 599, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 610, 611, 612, 613, 613, 614, 615, 615, 616, 616, 617, 617, 618, 618, 619, 619, 619, 619, 619, 620, 620, 620, 620, 620, 620, 620, 620, 619, 619, 619, 619, 618, 618, 618, 617, 617, 616, 616, 615, 614, 614, 613, 612, 611, 611, 610, 609, 608, 607, 606, 605, 604, 603, 602, 601, 599, 598, 597, 595, 594, 593, 591, 590, 589, 587, 586, 584, 582, 581, 579, 578, 576, 574, 573, 571, 569, 567, 565, 564, 562, 560, 558, 556, 554, 552, 550, 548, 546, 544, 542, 540, 538, 536, 534, 532, 530, 528, 526, 524, 522, 520, 518, 516, 514, 512, 509, 507, 505, 503, 501, 499, 497, 495, 493, 491, 489, 487, 485, 483, 481, 479, 477, 475, 473, 471, 469, 467, 465, 463, 461, 459, 458, 456, 454, 452, 450, 449, 447, 445, 444, 442, 441, 439, 437, 436, 434, 433, 432, 430, 429, 428, 426, 425, 424, 422, 421, 420, 419, 418, 417, 416, 415, 414, 413, 412, 412, 411, 410, 409, 409, 408, 407, 407, 406, 406, 405, 405, 405, 404, 404, 404, 404, 403, 403, 403, 403, 403, 403, 403, 403, 404, 404, 404, 404, 404, 405, 405, 406, 406, 407, 407, 408, 408, 409, 410, 410, 411, 412, 413, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 424, 425, 426, 427, 428, 430, 431, 432, 434, 435, 437, 438, 439, 441, 442, 444, 445, 447, 449, 450, 452, 453, 455, 457, 458, 460, 461, 463, 465, 467, 468, 470, 472, 473, 475, 477, 479, 480, 482, 484, 485, 487, 489, 491, 492, 494, 496, 497, 499, 501, 502, 504, 506, 507, 509, 510, 512, 514, 515, 517, 518, 520, 521, 523, 524, 525, 527, 528, 529, 531, 532, 533, 534, 535, 537, 538, 539, 540, 541, 542, 543, 544, 544, 545, 546, 547, 547, 548, 549, 549, 550, 550, 551, 551, 552, 552, 552, 552, 553, 553, 553, 553, 553, 553, 553, 552, 552, 552, 552, 551, 551, 550, 550, 549, 549, 548, 547, 546, 546, 545, 544, 543, 542, 541, 539, 538, 537, 536, 534, 533, 531, 530, 528, 527, 525, 523, 521, 519, 518, 516, 514, 512, 509, 507, 505, 503, 500, 498, 496, 493, 491, 488, 485, 483, 480, 477, 475, 472, 469, 466, 463, 460, 457, 454, 451, 447, 444, 441, 438, 434, 431, 428, 424, 421, 417, 414, 410, 407, 403, 399, 396, 392, 388, 385, 381, 377, 373, 369, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 248, 244, 240, 236, 232, 229, 225, 221, 217, 214, 210, 207, 203, 199, 196, 192, 189, 186, 182, 179, 176, 172, 169, 166, 163, 160, 157, 154, 151, 148, 145, 143, 140, 137, 135, 132, 130, 127, 125, 123, 120, 118, 116, 114, 112, 110, 108, 106, 105, 103, 101, 100, 98, 97, 96, 95, 93, 92, 91, 90, 90, 89, 88, 87, 87, 86, 86, 86, 86, 85, 85, 85, 85, 86, 86, 86, 87, 87, 88, 88, 89, 90, 91, 92, 93, 94, 96, 97, 98, 100, 101, 103, 105, 107, 109, 111, 113, 115, 117, 120, 122, 125, 127, 130, 133, 135, 138, 141, 144, 148, 151, 154, 158, 161, 165, 168, 172, 176, 179, 183, 187, 191, 195, 200, 204, 208, 213, 217, 222, 226, 231, 235, 240, 245, 250, 255, 260, 265, 270, 275, 280, 286, 291, 296, 302, 307, 313, 318, 324, 329, 335, 341, 347, 352, 358, 364, 370, 376, 382, 388, 394, 400, 406, 412, 418, 424, 430, 437, 443, 449, 455, 461, 468, 474, 480, 486, 493, 499, 505, 512 };

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
  tcConfigure((int)SAMPLE_RATE);
}

// ======================================================
// AUDIO GENERATION INTERRUPT
// ======================================================

 void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

/*
void TC5_Handler (void)
{
  analogWrite(A0, wavSamples[sIndex]);
  sIndex++;
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}*/

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
float sample2 = 0.f;
const unsigned int wave_max = WAVE_TABLE_SIZE - 1;
//float waveform1 = 1.f;
//float waveform2 = 0.f;
//float cross = 0.f;


//void audioISR() {
void TC5_Handler (void) {
  signed int value1, value2;
  int indexBase;
  float indexFract;
  float modSample1 = 0.f;
  float modSample2 = 0.f;

  float sample1 = 0.f;
  float sample2 = 0.f;

  osc2.phase_accumulator += osc2.inc;
  if (osc2.phase_accumulator > wave_max) {
    osc2.phase_accumulator -= wave_max;
  }
  //osc2.phaseInt = round(osc2.phase_accumulator);

  indexBase = floor(osc2.phase_accumulator);
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

  osc1.phase_accumulator += osc1.inc;
  if (osc1.phase_accumulator > wave_max) {
    osc1.phase_accumulator -= wave_max;
  }

  //osc1.phaseInt = round(osc1.phase_accumulator);
  indexBase = floor(osc1.phase_accumulator);
  indexFract = (osc1.phase_accumulator - indexBase);
  value1 = sintable[indexBase];
  value2 = sintable[indexBase + 1];
  //sample1 = value1 * (1.f - indexFract) + value2 * indexFract;
  sample1 = value1 + ((value2 - value1) * indexFract);

  value1 = sintable2[indexBase];
  value2 = sintable2[indexBase + 1];
  sample2 = value1 + ((value2 - value1) * indexFract);

  sample1 = sample1 * osc1.waveform1 + (sample2 * osc1.waveform2);
  int16_t sample = round((sample1 * osc1.volume + modSample1 * osc2.volume) * 0.4f);
  
  //analogWrite(A0, sample);
  DAC->DATA.reg = sample;  
  if (osc1.crossFMint > 0) {
    osc2.phase_accumulator += (sample1 * osc1.crossFM) / 1023.f;
  }
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  tcStartCounter();
}

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



void loop() {
  osc1.frequency = mapf(analogRead(A1), 0, 4095, 20, 2200) + mapf(analogRead(A4), 0, 4095, 0, 2600);  // A4 = analog feedback

  osc1.waveform2 = analogRead(A2) / 4095.f;
  osc1.waveform1 = 1.f - osc1.waveform2;

  osc1.crossFMint = analogRead(A3);
  osc1.crossFM = osc1.crossFMint / 8.f;
  osc1.volume = analogRead(5) / 4095.f;

  osc1.inc = (osc1.frequency * 1024.f) / SAMPLE_RATE;


  osc2.frequency = mapf(analogRead(A6), 0, 4095, 20, 2200);  // + analogRead(9); //mapf( analogRead(A9) , 0, 4095, 0, 2600); // A9 = analog feedback
  osc2.waveform2 = analogRead(A7) / 4095.f;
  osc2.waveform1 = 1.f - osc2.waveform2;
  osc2.crossFMint = analogRead(A8);
  osc2.crossFM = osc2.crossFMint / 8.f;
  osc2.volume = analogRead(10) / 4095.f;

  osc2.inc = (osc2.frequency * 1024.f) / SAMPLE_RATE;
  
  //delay(1);
}