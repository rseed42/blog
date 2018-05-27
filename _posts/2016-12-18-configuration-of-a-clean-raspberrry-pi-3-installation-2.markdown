---
layout: post
title: Configuration of a Clean Raspberrry Pi 3 Installation
date: '2016-12-18 16:21:07'
tags:
- raspberry
- arduino
---

#### Booting Up

We can now install the SD card and make some additional configurations to allows us to start using the Raspberry. I am assuming that it will be connected via LAN. This is always the safest option. Wifi can be setup later.

1. The SD card is put faced down in the Raspberry. There should be only one way to do that.
2. Power up the Raspberry.

You should see the green LED blinking. If only the read one is on, then there might be some problem with the image on the SD card. It is also a good idea to check on the LAN port to see that the TX/RX lights are also active.

#### Connecting to the Raspberry
In your home router, look for some menu like

```
DHCP Client Devices
```
In my case, the entry looks like this:
```
raspberrypi	B8:27:EB:42:C4:23	192.168.0.100	Ethernet(100Mbps)	N/A	*** STATIC IP ADDRESS **
```
If you have configured a static IP, this check might not be necessary, but I find it useful anyway. We can also ping it to check the the connectivity is OK:
```
venelin@clevo ~ $ ping -c4 192.168.0.100
PING 192.168.0.100 (192.168.0.100) 56(84) bytes of data.
64 bytes from 192.168.0.100: icmp_seq=1 ttl=64 time=4.12 ms
64 bytes from 192.168.0.100: icmp_seq=2 ttl=64 time=1.15 ms
64 bytes from 192.168.0.100: icmp_seq=3 ttl=64 time=1.34 ms
64 bytes from 192.168.0.100: icmp_seq=4 ttl=64 time=1.07 ms

--- 192.168.0.100 ping statistics ---
4 packets transmitted, 4 received, 0% packet loss, time 3004ms
rtt min/avg/max/mdev = 1.075/1.924/4.120/1.271 ms
```
With the last check we can see if the SSH port is available:
```
venelin@clevo ~ $ telnet 192.168.0.100 22
Trying 192.168.0.100...
Connected to 192.168.0.100.
Escape character is '^]'.
SSH-2.0-OpenSSH_6.7p1 Raspbian-5+deb8u3
^]
Protocol mismatch.
Connection closed by foreign host.
```
We can now connect to the Raspberry. Raspbian is shipped with a default sudo-enabled user called "pi":

* Default Raspbian user: *pi*
* Default Raspbian user password: *raspbian* (*raspberry* for Stretch)

Let's connect with that user and change the credentials:

```
venelin@clevo ~ $ ssh pi@192.168.0.100
The authenticity of host '192.168.0.100 (192.168.0.100)' can't be established.
ECDSA key fingerprint is bb:61:88:d9:2c:58:a5:57:10:3f:f8:ce:46:cc:0b:98.
Are you sure you want to continue connecting (yes/no)? yes
Warning: Permanently added '192.168.0.100' (ECDSA) to the list of known hosts.
pi@192.168.0.100's password: 

The programs included with the Debian GNU/Linux system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
permitted by applicable law.

SSH is enabled and the default password for the 'pi' user has not been changed.
This is a security risk - please login as the 'pi' user and type 'passwd' to set a new password.
```

Since pi has sudo rights, we need to immediately change this password.

```
pi@raspberrypi:~ $ passwd
Changing password for pi.
(current) UNIX password: 
Enter new UNIX password: 
Retype new UNIX password: 
passwd: password updated successfully
```

#### Changing the Host Name
Shorter names are always better for frequent tasks, therefore I like to name my Raspberry "raspi". However, be careful with these steps or you might lose connectivity and you might need to start all over again.

First, check the name of the system:

```
pi@raspberrypi:~ $ hostname
raspberrypi
```
Let's change that:
```
pi@raspberrypi:~ $ sudo hostname -b raspi
pi@raspberrypi:~ $ hostname
raspi
```
This is not the end of it, however. The original name is still contained in several configuration files. We can easily find them with a recursive grep:
```
pi@raspberrypi:~ $ sudo grep -lr "raspberrypi" /etc/*
sudo: unable to resolve host raspi
/etc/apt/sources.list.d/raspi.list
/etc/hostname
/etc/hosts
/etc/ssh/ssh_host_dsa_key.pub
/etc/ssh/ssh_host_key
/etc/ssh/ssh_host_ed25519_key.pub
/etc/ssh/ssh_host_key.pub
/etc/ssh/ssh_host_rsa_key.pub
/etc/ssh/ssh_host_ecdsa_key.pub
```
Particularly worrisome are some SSH configuration files. If we blindly go about it, SSH will not work and we are locked out. First, however, we need to change the network name. Replace the original name in /etc/hosts with "raspi" and reboot the system:

```
nano /etc/hosts
```
Reboot:
```
sudo reboot
```

You can see that the network host name is now changed if you try to ping it from within the system:

```
pi@raspberrypi:~ $ ping -c2 raspi
PING raspi (127.0.1.1) 56(84) bytes of data.
64 bytes from raspi (127.0.1.1): icmp_seq=1 ttl=64 time=0.109 ms
64 bytes from raspi (127.0.1.1): icmp_seq=2 ttl=64 time=0.057 ms

--- raspi ping statistics ---
2 packets transmitted, 2 received, 0% packet loss, time 999ms
rtt min/avg/max/mdev = 0.057/0.083/0.109/0.026 ms
```

Despite the change of the network host name, you will notice that the Linux host name is still the old one. Change it in the hostname file:
```
nano /etc/hostname
```

Now we need to perform a delicate change on SSH. Please, make sure that you don't disconnect from SSH until this is done.

We have seen that many SSH certificates are tied to the original name of the system. We need to delete and regenerate these. 

Remove the old SSH certificates:
```
pi@raspi:~ $ sudo rm /etc/ssh/ssh_host_*
```
Immediately reconfigure the SSH server to regenerate the certificates:
```
pi@raspi:~ $ sudo dpkg-reconfigure openssh-server
Creating SSH2 RSA key; this may take some time ...
2048 6a:3a:d8:1d:c7:51:61:ca:a4:f5:bc:f0:9b:b0:fd:73 /etc/ssh/ssh_host_rsa_key.pub (RSA)
Creating SSH2 DSA key; this may take some time ...
1024 82:7a:cb:1d:65:63:5b:96:64:b8:83:be:a5:42:b8:b1 /etc/ssh/ssh_host_dsa_key.pub (DSA)
Creating SSH2 ECDSA key; this may take some time ...
256 b8:04:9d:e0:6e:52:cc:d2:f4:52:c9:ad:76:5a:e4:35 /etc/ssh/ssh_host_ecdsa_key.pub (ECDSA)
Creating SSH2 ED25519 key; this may take some time ...
256 e1:2f:19:b7:b2:d8:14:87:63:b9:e1:26:5c:97:58:03 /etc/ssh/ssh_host_ed25519_key.pub (ED25519)
```
Restart SSH:
```
pi@raspi:~ $ sudo service ssh restart
```

reboot the system:
```
pi@raspi:~ $ sudo reboot
```
If you try to connect now, you will notice that there is an error recognizing the SSH fingerprint. I usually fix this by removing the last entry in the known_hosts of my laptop's SSH directory:
```
venelin@clevo ~ $ emacs ~/.ssh/known_hosts
```
After connecting
```
venelin@clevo ~ $ ssh pi@192.168.0.100
The authenticity of host '192.168.0.100 (192.168.0.100)' can't be established.
ECDSA key fingerprint is 73:b4:f5:c4:7a:c7:65:83:5b:6e:b4:3d:02:d4:78:f3.
Are you sure you want to continue connecting (yes/no)? yes
Warning: Permanently added '192.168.0.100' (ECDSA) to the list of known hosts.
pi@192.168.0.100's password: 

The programs included with the Debian GNU/Linux system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
permitted by applicable law.
Last login: Sun Dec 18 15:12:17 2016 from 192.168.0.29
```
you will notice that the system is now properly named:
```
pi@raspi:~ $ hostname
raspi
```
This should do it.

#### Adding a Custom User
The first thing I like to do on the new system is to add my own user 

```
pi@raspberrypi:~ $ sudo useradd -G sudo vpetkov
```

Set the password for the new user
```
pi@raspberrypi:~ $ sudo passwd vpetkov
Enter new UNIX password: 
Retype new UNIX password: 
passwd: password updated successfully
```

We can see that user "pi" is added by default to many additional group that enable access to some hardware features:
```
pi@raspberrypi:~ $ id
uid=1000(pi) gid=1000(pi) groups=1000(pi),4(adm),20(dialout),24(cdrom),27(sudo),29(audio),44(video),46(plugdev),60(games),100(users),101(input),108(netdev),997(gpio),998(i2c),999(spi)
```
Let's add the newly created user to these groups

```
sudo usermod -a -G adm vpetkov
sudo usermod -a -G dialout vpetkov
sudo usermod -a -G audio vpetkov
sudo usermod -a -G video vpetkov
sudo usermod -a -G plugdev vpetkov
sudo usermod -a -G input vpetkov
sudo usermod -a -G netdev vpetkov
sudo usermod -a -G gpio vpetkov
sudo usermod -a -G i2c vpetkov
sudo usermod -a -G spi vpetkov
```
We should have more or less set up our custom user by now.

```
pi@raspberrypi:~ $ sudo su vpetkov
vpetkov@raspberrypi:/home/pi$ id
uid=1001(vpetkov) gid=1001(vpetkov) groups=1001(vpetkov),4(adm),20(dialout),27(sudo),29(audio),44(video),46(plugdev),101(input),108(netdev),997(gpio),998(i2c),999(spi)
```
We can not reconnect with the user we want to use:
```
venelin@clevo ~ $ ssh vpetkov@192.168.0.100
vpetkov@192.168.0.100's password: 

The programs included with the Debian GNU/Linux system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
permitted by applicable law.
Could not chdir to home directory /home/vpetkov: No such file or directory
```
Notice that the home directory is missing. Let's rectify that:
```
vpetkov@raspi:/$ sudo mkdir /home/vpetkov

We trust you have received the usual lecture from the local System
Administrator. It usually boils down to these three things:

    #1) Respect the privacy of others.
    #2) Think before you type.
    #3) With great power comes great responsibility.

[sudo] password for vpetkov: 
```
Change ownership:
```
vpetkov@raspi:/$ sudo chown vpetkov:vpetkov /home/vpetkov
```
When we reconnect, our current working directory is now the proper use home directory.

#### Simplifying SSH Access
Before we start using the Raspberry system regularly, let's make that as easy as possible. First, we need to modify our SSH client config, so we don't need to type the full IP address all the time (alternatively, this can also be done with a DHCP configuration for the name of the raspberry if your router allows it).

If don't have an SSH client configuration file, create one in your SSH directory:
```
venelin@clevo ~ $ touch ~/.ssh/config
```
Inside it we only need to specify an alias for the IP and which user we want to login as:

```
nano ~/.ssh/config
```
My file contains this example configuration, based on the tutorial:
```
Host raspi
    HostName 192.168.0.100
    User vpetkov
```
Now it is sufficient to type
```
venelin@clevo ~ $ ssh raspi
```
to connect to the Raspberry.

#### Setting up SSH Keys
Using a password to connect to remote systems is not only more cumbersome, but also more risky, compared to using SSH keys. Let's set up the keys. 

First, generate the SSH key with:
```
venelin@clevo ~ $ ssh-keygen -t rsa
Generating public/private rsa key pair.
Enter file in which to save the key (/home/venelin/.ssh/id_rsa): /home/venelin/.ssh/id_rsa_raspi3
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/venelin/.ssh/id_rsa_raspi3.
Your public key has been saved in /home/venelin/.ssh/id_rsa_raspi3.pub.
The key fingerprint is:
ca:ae:42:45:34:0a:7d:fb:3c:c0:ff:f3:c2:13:a3:bc venelin@clevo
The key's randomart image is:
+---[RSA 2048]----+
|.. .o            |
| ...o.           |
|  .+ .           |
|    =            |
|   . =  S        |
|  .  .=.o        |
| .   .o= o       |
|  .  .o *        |
|   ...E. =.      |
+-----------------+
```
Notice that several files are created in your ~/.ssh directory:
```
venelin@clevo ~/.ssh $ ls -l
total 32
-rw-r--r-- 1 venelin venelin   55 Dec 18 16:42 config
...
-rw------- 1 venelin venelin 1766 Dec 18 16:46 id_rsa_raspi3
-rw-r--r-- 1 venelin venelin  395 Dec 18 16:46 id_rsa_raspi3.pub
```
We need to copy the public key id_rsa_raspi3.pub to the Raspberry account. Fortunately, there is an easy command to do that and we don't even need to worry about chmod-ing the authorized_keys file anymore:
```
venelin@clevo ~ $ ssh-copy-id raspi
/usr/bin/ssh-copy-id: INFO: attempting to log in with the new key(s), to filter out any that are already installed
/usr/bin/ssh-copy-id: INFO: 3 key(s) remain to be installed -- if you are prompted now it is to install the new keys
vpetkov@192.168.0.100's password: 

Number of key(s) added: 3

Now try logging into the machine, with:   "ssh 'raspi'"
and check to make sure that only the key(s) you wanted were added.
```
Notice that all public keys are copied over. If you need a more targeted approach, remove the rest or copy the file manually.

Now connecting to the Raspberry is as easy as typing 
```
venelin@clevo ~ $ ssh raspi

The programs included with the Debian GNU/Linux system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
permitted by applicable law.
Last login: Sun Dec 18 15:51:37 2016 from 192.168.0.29
vpetkov@raspi:~$ 
```


