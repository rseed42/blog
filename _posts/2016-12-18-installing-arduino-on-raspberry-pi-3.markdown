---
layout: post
title: Installing Arduino on Raspberry Pi 3
date: '2016-12-18 18:15:25'
tags:
- raspberry
- arduino
---

#### Overview
This guide will show how to install the latest Arduino IDE and supporting packages to enable the programming of Arduino devices from the Raspberry Pi 3.

You will need a USB to Serial FTDI converter in order to program the Arduino devices and to receive the output of the serial console when testing them.

The complete setup allows us to develop and program Arduino devices on the command line, without any IDE. We are going to use up-to-date versions of the software, so most of the packages will have to be fetched from Github and installed locally.

#### Preliminary Steps

First, let's organize the locally installed packages on the Raspberry. I like to use a directory called "local" for this purpose. Since there are no other users, it makes no sense to install anything in system directories. We will place them in ~/local:

```
mkdir ~/local
cd local
```

#### Arduino IDE

Because of some compatibility issues, I prefer to install an up-to-date version of the Arduino IDE. The official Debian package is just too old and not compatible anymore with newer libraries.

Download to the website https://www.arduino.cc/en/Main/Software. Make sure that you select the Linux ARM (experimental) version. The AMD64 architecture doesn't work on the Raspberry. The current version is 
```
ARDUINO 1.6.13
```
Download the package and move it to the Raspberry with something like:
```
scp arduino-1.6.13-linuxarm.tar.xz raspi:local
```
The package can be decompressed inside the ~/local directory using:
```
cd ~/local
tar -xf arduino-1.6.13-linuxarm.tar.xz
```
Now we have the Arduino IDE installed under
```
~/local/arduino-1.6.13
```
It contains libraries, board description files, etc.

We can not compile the source files (Arduino sketches) with a plain old Make file, because there are many dependencies and hidden parts from the developer. Therefore, we need Arduino.mk which enables us to work on the command line. It lives on Github, so we just need to clone it to the ~/local directory:

```
vpetkov@raspi:~$ git clone https://github.com/sudar/Arduino-Makefile ~/local/arduino_mk
Cloning into '/home/vpetkov/local/arduino_mk'...
remote: Counting objects: 2483, done.
remote: Total 2483 (delta 0), reused 0 (delta 0), pack-reused 2483
Receiving objects: 100% (2483/2483), 1.48 MiB | 522.00 KiB/s, done.
Resolving deltas: 100% (1097/1097), done.
Checking connectivity... done.
```
Now we are set up to make a first test.

#### Test Arduino Installation
First, create a directory that contains the code:
```
mkdir ~/test
```
We need a Makefile and an *.ino file. Let's create these:
```
cd ~/test
touch Makefile
touch Test.ino
```
Now, please keep in mind that the programming test requires the following devices:

1. A USB to Serial FTDI adapter
2. Arduino Pro Mini 3.3v

The Makefile is tailored for this particular device. If you have another, the hardware-specific parts have to be updated.

Makefile:
```
BOARD_TAG=pro328
MCU=atmega328
MONITOR_PORT=/dev/ttyUSB0
ARDUINO_DIR = /home/$(shell whoami)/local/arduino-1.6.13
ARDMK_DIR = /home/$(shell whoami)/local/arduino_mk
USER_LIB_PATH=/home/$(shell whoami)/local/lib
BOARD_TAG = pro
BOARD_SUB = ATmega328
MCU = atmega328p
F_CPU=8000000L
AVRDUDE_ARD_BAUDRATE = 57600
include $(ARDMK_DIR)/Arduino.mk
```

The source code is also very simple:

Test.ino:
```
void setup(){
  Serial.begin(9600);
  Serial.println("Arduino on Raspberry Pi Test");
}
void loop(){
  delay(2000);
  Serial.println("Still there");
  Serial.print("\n");
}
```

Simply compile with "make". All tools are contained in the Arduino IDE and all configuration in the Makefile:

```
make
```

The program should now be compiled successfully.

When you connect the FTDI converter, make sure that it appears as /tty/USB0. This can be checked with dmesg:

```
vpetkov@raspi:~/test$ dmesg | tail
[ 8864.644767] usb 1-1.4: Product: CP2102 USB to UART Bridge Controller
[ 8864.644780] usb 1-1.4: Manufacturer: Silicon Labs
[ 8864.644792] usb 1-1.4: SerialNumber: 0001
[ 8865.808457] usbcore: registered new interface driver usbserial
[ 8865.808562] usbcore: registered new interface driver usbserial_generic
[ 8865.808649] usbserial: USB Serial support registered for generic
[ 8865.812262] usbcore: registered new interface driver cp210x
[ 8865.812395] usbserial: USB Serial support registered for cp210x
[ 8865.812559] cp210x 1-1.4:1.0: cp210x converter detected
[ 8865.813036] usb 1-1.4: cp210x converter now attached to ttyUSB0
```
We can now program the Arduino. Make sure that you have correctly connected the RX/TX and the reset pins, as well as the Vcc/Ground. You need the 100nF capacitor on the reset line, for it to work correctly.

Please, also make sure that the Python python-serial package is installed, since it is used by Arduino.mk:

```
make upload
```
If sucessful, you should see something like:
```
/home/vpetkov/local/arduino-1.6.13/hardware/tools/avr/bin/avrdude -q -V -p atmega328p -C /home/vpetkov/local/arduino-1.6.13/hardware/tools/avr/etc/avrdude.conf -D -c arduino -b 57600 -P /dev/ttyUSB0 \
		-U flash:w:build-pro-ATmega328/test.hex:i

avrdude: AVR device initialized and ready to accept instructions
avrdude: Device signature = 0x1e950f (probably m328p)
avrdude: reading input file "build-pro-ATmega328/test.hex"
avrdude: writing flash (1668 bytes):
avrdude: 1668 bytes of flash written

avrdude: safemode: Fuses OK (E:00, H:00, L:00)

avrdude done.  Thank you.

make[1]: Leaving directory '/home/vpetkov/test'
```

Now we can test the message that the Arduino sends to us. I like to use a python command to monitor the serial port:

```
python -m serial.tools.miniterm -p /dev/ttyUSB0 -b 9600
```

You should see period messages like:

```
Still there
```

Press Ctrl and ] to exit.

#### User Libraries

You might have noticed the line 

```
USER_LIB_PATH=/home/$(shell whoami)/local/lib
```

This is where I put my user libraries. For instance, my temperature/humidity sensor uses the DHT library:

```
git clone https://github.com/adafruit/DHT-sensor-library ~/local/lib/DHT
```
Also, I use MySensors for the wireless network:

```
git clone https://github.com/mysensors/MySensors ~/local/mysensors
```
