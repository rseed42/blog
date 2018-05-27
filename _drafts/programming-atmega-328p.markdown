---
layout: post
title: Programming Atmega 328p
---

# Connection
* Reset
* MISO
* MOISE
* SCLK
* VCC
* GND

See below how to connect the pins to the raspberry. All lines have a 1k resistor between the raspberry and the arduino.

# Programmer
Install avrdude
```
sudo apt-get install -y avrdude
```
List all supported microcontrollers
```
avrdude -c avrisp
```
The Atmega 328 is called `m328`.

Copy the avrdude config file, since we need to define a new programmer:
```
cp /etc/avrdude.conf ~/avrdude_gpio.conf
```
Add the following configuration to `avrdude_gpio.conf` (at the end of the file):
```
#------------------------------------------------------------
# Raspberry Programmer
#------------------------------------------------------------
programmer
  id    = "pi_1";
  desc  = "Use the Linux sysfs interface to bitbang GPIO lines";
  type  = "linuxgpio";
  reset = 12;
  sck   = 24;
  mosi  = 23;
  miso  = 18;
;
```

We can now check the connection to the microcontroller with the following command:
```
sudo avrdude -p m328 -C ~/avrdude_gpio.conf -c pi_1 -v
```
If the connection is ok, then we are ready to program it.

```
sudo avrdude -p m328 -C avrdude_gpio.conf -c pi_1 -v -U flash:w:build-pro-ATmega328/led.hex:i
```
Calculate the fuses:
http://www.engbedded.com/fusecalc/

Set up the fuses for an 8 MHz external oscillator:
```
avrdude ... -U lfuse:w:0xfe:m
```

The full command ist:

```
sudo avrdude -p m328 -C avrdude_gpio.conf -c pi_1 -v -U lfuse:w:0xfe:m -U flash:w:build-pro-ATmega328/led.hex:i
```

NEW: Using the linuxspi programmer

Connect directly to the MISO/MOSI/etc. corresponding pins.
Change the commands
Fuses

