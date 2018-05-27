---
layout: post
title: Sensor Network Gateway and Controller
---

#### Hardware Setup




#### Software

Build the raspberry gateway

```
cd ~/local/mysensors
```
Configure the daemon with all bells and whistles
```
./configure --my-gateway=ethernet --my-transport=nrf24 --my-rf24-irq-pin=15 --my-leds-err-pin=12 --my-leds-rx-pin=16 --my-leds-tx-pin=18 --my-leds-blinking-inverse
```
Build
```
make
```
Install
```
sudo make install
```
Add it to the boot sequence
```
sudo systemctl enable mysgw.service
```
Start with
```
sudo systemctl start mysgw.service
```
Check its status
```
vpetkov@raspi:~/local/mysensors$ systemctl status mysgw.service
● mysgw.service - MySensors Gateway daemon
   Loaded: loaded (/etc/systemd/system/mysgw.service; enabled)
   Active: active (running) since Sun 2016-12-18 18:19:14 UTC; 24s ago
 Main PID: 4674 (mysgw)
   CGroup: /system.slice/mysgw.service
           └─4674 /usr/local/bin/mysgw
```

Check that it has opened the port. I observed that it fails to do that if the radio is not properly connected.

```
vpetkov@raspi:~$ netstat -nt
Active Internet connections (w/o servers)
Proto Recv-Q Send-Q Local Address           Foreign Address         State      
tcp        0    156 192.168.0.100:22        192.168.0.29:54483      ESTABLISHED
tcp        0      0 127.0.0.1:5003          127.0.0.1:57986         ESTABLISHED
tcp        0      0 192.168.0.100:22        192.168.0.29:54327      ESTABLISHED
tcp        0      0 127.0.0.1:57986         127.0.0.1:5003          ESTABLISHED
```

The gateway has port 5003 open, so it is doing fine.

#### Controller

Create a data subdirectory

```
cd ~/senet/controller
mkdir data
```

Start tmux

```
tmux -S controller
```

Start the controller

```
./controller.py
```