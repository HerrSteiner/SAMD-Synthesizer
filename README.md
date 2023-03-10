# SAMD-Synthesizer
sound experiments with the XIAO SAMD chip

Exploration of what can be done in software on that microcontroller, basically a software synthesizer running on a microcontroller which gives other possibilities as being on an allpurpose desktop computer.
The outcome is a noise / drone synthesizer which I am using for my musicprojects such as TMS, Notstandskomitee and Elektronengehirn (https://block4label.bandcamp.com).

The SAMD21 has one 10bit DAC on pin A0 which I used for sound, its connected through a simple CR highpass filter and a 10k ohm pot as analog volume control to output.
Pins A1 - A10 are used as ADCs and connected with 10k pots which are fed from the 3.3V reference with the exception of A4 and A9 those input comes from A0 to create an audio feedback on the Frequency section.
The pots control two oscillators with each the parameters frequency, waveform (oscillator blends from sinus to triple sinus wavetable, osc2 from sinus to saw), cross FM towards the other oscillator, audio feedback to FM (it doesn't blow up like feedback in audio path, it just create interesting instabilities) and volume.

A video of the prototype in action can be seen at https://youtu.be/kiY89RPKTYg

Its a quirky noise device with lot of interesting oddities, cross bleed etc which reminds on qualities of the EMS Synthi without me wanting to recreate one in this case. 
The exciting thing is that this box doesn't contain much electronics, basically a 6€ microcontroller running the whole show.

It is programmed in the Arduino IDE with the appropriate board library installed, but it uses no further libraries because I wanted to have control and insight of what is going on. The SAMD21 is an ARM M0 device and has no floating point unit which means floating point calculations cost a lot of cycles. I started first with a version with reduced floating point operations which is still in the repository. I then ported to fixed point calculation and avoid so floats at all which improves a lot the performance. This version is to be preferred.
