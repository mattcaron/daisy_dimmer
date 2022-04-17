# PWM to Constant Signal controller

This project is the solution to my car stereo problem. To wit:

Given a car which outputs a 12V PWM signal on the dimmer line (orange wire) to
the radio and has no illumination line, with a duty cycle between 19% and 93%
(give or take) make a circuit to output constant 12V (ideal, but anything above
5V should work) to the radio which has an illumination wire (orange wire with
white stripe).

I tried to do this with an RC circuit, but at a 15% duty cycle couldn't get
above 2V in simulation. A friend suggested I use a MOSFET, but I couldn't find
one which would be fully on at 2V and not blow up with a 12V gate to drain
differential when the lights were off. My friend then suggested a gate driver
circuit, but at that point, soldering all those discrete components was going to
make for a physically large package. So, in this approach I use a 6-20V to 5V
buck converter which I feed to the ESP8266 module's 5V input to power it. The
dimmer signal gets fed in via a 12V to 3.3V level shifter. I use the 3.3V
regulated output from the ES8266 board to drive the low voltage side and the 12V
input from the car to drive the HV side and it should all work just fine.

I'll attempt to remember to update this once it's all done and working, but I'm
waiting on parts.

## Acknowledgements

Thanks to Benjamin von Deschwanden for his
[ESP32 Signal Generator](https://github.com/vdeschwb/esp32-signal-generator)
project. It was handy to not have to do that work myself to make a test
fixture. Thanks to him for sharing.
