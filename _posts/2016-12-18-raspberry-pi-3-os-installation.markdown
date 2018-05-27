---
layout: post
title: Raspberry Pi 3 OS Installation
date: '2016-12-18 16:20:36'
tags:
- raspberry
---

#### Overview
Raspberry Pi 3 doesn't have a built-in hard disk drive, it uses a micro-SD card instead. This card is usually not sold together with the Raspberry, so it has to be bought and installed separately, unless you have a kit that provides that already.

I would recommend to buy an SD card with at least 8 GB of memory. Due to the low price point I am using a 32 GB card.

I decided to go with Raspbian Lite because I'd like to have a minimal system for my sensor network controller. Since it is not going to have a monitor attached to it, most of the software bloat in alternative distributions is useless here. However, this means that the only (comfortable) way to access the device is via SSH. SSH is disabled by default in the latest version (2016-11-25), which means that some configuration tweaks will be necessary before booting up with the SD card.

#### Installing the image on the SD Card
First, we need to find the device name of the SD Card. The fdisk command shows us detailed information about disk drives and partitions:
```
venelin@clevo ~ $ sudo fdisk -l

Disk /dev/sda: 232.9 GiB, 250059350016 bytes, 488397168 sectors
...
Disk /dev/mmcblk0: 29.7 GiB, 31914983424 bytes, 62333952 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0xe47ffcac

Device         Boot Start      End  Sectors  Size Id Type
/dev/mmcblk0p1       2048 62332927 62330880 29.7G  b W95 FAT32

```
You can see that the device name of the SD card is /dev/mmcblk0 and it has one partition /dev/mmcblk0p1. We will copy the disk image directly to the disk drive /dev/mmcblk0. 

First we need to unmount the existing partition if it is mounted:
```
venelin@clevo ~ $ sudo umount /dev/mmcblk0p1
```
Now we can byte-copy the image with the dd command:
```
venelin@clevo ~/Downloads $ sudo dd bs=4M if=2016-11-25-raspbian-jessie-lite.img of=/dev/mmcblk0
331+1 records in
331+1 records out
1390411776 bytes (1.4 GB) copied, 57.5949 s, 24.1 MB/s
```
Let's also flush any unwritten data to prepare the SD card for removal:
```
sudo sync
```
We can check what the disk image now contains:

```
venelin@clevo ~/Downloads $ sudo fdisk -l

Disk /dev/sda: 232.9 GiB, 250059350016 bytes, 488397168 sectors
...

Disk /dev/mmcblk0: 29.7 GiB, 31914983424 bytes, 62333952 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x244b8248

Device         Boot  Start     End Sectors  Size Id Type
/dev/mmcblk0p1        8192  137215  129024   63M  c W95 FAT32 (LBA)
/dev/mmcblk0p2      137216 2715647 2578432  1.2G 83 Linux
```
The Raspbian image contains two partitions:

1. A FAT32 boot partition
2. The ext4 root partition of the OS

We need to modify some configuration files before we move the SD Card to the Raspberry device. For clarity I created manually the local directories where they could be mounted:
```
venelin@clevo ~/Downloads $ sudo mkdir -p /media/venelin/raspi/boot
venelin@clevo ~/Downloads $ sudo mkdir -p /media/venelin/raspi/root
```
Then mounted them with:
```
venelin@clevo ~/Downloads $ sudo mount /dev/mmcblk0p1 -t vfat -o uid=venelin /media/venelin/raspi/boot/
venelin@clevo /media/venelin/raspi $ sudo mount /dev/mmcblk0p2 /media/venelin/raspi/root/
```

#### SSH Activation
According to the latest Raspbian Lite changelog

http://downloads.raspberrypi.org/raspbian/release_notes.txt

SSH is now disabled by default. I don't plan on using a monitor to access the Raspberry, therefore SSH must be enabled. This is fairly easy, we need only to create an empty file in the boot partition:

```
venelin@clevo ~ $ touch /media/venelin/raspi/boot/ssh
```

#### Assigning a Static IP Address (Obsolete)

Note: This has worked for Raspbian Jessie, but the latest Raspbian Stretch has changed some of the networking configuration, and the below 
guide hasn't been tested with that yet!

In my home network I like to have a dedicated IP for any server machine. This is mostly caused by lack of flexibility of my router software in regard to DHCP. It is at least possible to configure the range for dynamic IPs, so I have restricted them to a certain range. Anything above that can be safely assigned a static IP address.

In the latest versions of Raspbian it is best to configure a static IP via the DHCP configuration file

```
/etc/dhcpcd.conf
```

The static IP configuration is added to the end of the file:
```
interface eth0

static ip_address=192.168.0.100/24
static routers=192.168.0.1
static domain_name_servers=192.168.0.1
```

In this case my router has an IP 192.168.0.1.

#### DHCP Configuration For Wifi (Updated for Raspbian Stretch)

In addition to enabling ssh via creating an empty file in the boot partition (see above), we also need to put the wifi credentials in the boot partition as well. Allegedly, *wpa_supplicant.conf* has to be fully specified, so we first copy the existing */etc/wpa_supplicant/wpa_supplicant.conf* to */boot/wpa_supplicant.conf*. Then, we can encode the wifi password:
```
wpa_passphrase "wifi ssid" "wifi password"

network={
	ssid="wifi ssid"
	#psk="wifi password"
	psk=7e74dd066afb9ee688dbc30cd223e2299e3429c7bc8081ed717ae2ef370ce4e1
}
```
The full configuration file looks as follows:
```
country=DE
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
network={
	ssid="wifi ssid"
	psk=7e74dd066afb9ee688dbc30cd223e2299e3429c7bc8081ed717ae2ef370ce4e1
}
```
After the system boots for the first time, it moves the configuration file to its standard location at

```
/etc/wpa_supplicant/wpa_supplicant.conf
```

#### Prepare the SD Card for Removal

The mounted partitions should be unmounted before the SD card is removed:
```
venelin@clevo ~ $ sudo umount /dev/mmcblk0p1 
venelin@clevo ~ $ sudo umount /dev/mmcblk0p2
```
