---
layout: post
title: Setting Raspberry Pi GPIO Pins on Startup
---

Check firmware version first:

```
vpetkov@raspres:~$ vcgencmd version
Nov 25 2016 16:03:50 
Copyright (c) 2012 Broadcom
version 48a26a2ae46c497139b3d5a9c8d15485c7b3bfbc (clean) (release)
```

Setting the GPIO pins on boot is supported from July 15th 2014.

The boot-time GPIO configuration is created as a Device Tree source (.dts) file and then compiled into a binary Device Tree blob (.dtb) file.

Install the Device Tree compiler:

```
sudo apt-get install device-tree-compiler
```

Check:

```
vpetkov@raspres:~$ which dtc
/usr/bin/dtc
```

