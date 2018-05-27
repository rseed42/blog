---
layout: post
title: Compile MySensors Raspberry Pi Gateway
---

We need to enable the LED status controls as well as the mqtt broker.

Serial Gateway:
```
./configure --my-transport=nrf24 --my-gateway=serial --my-serial-port=/dev/ttyAMA0 --my-leds-err-pin=12 --my-leds-rx-pin=16 --my-leds-tx-pin=18
```

MQTT Gateway:

```
./configure --my-transport=nrf24 --my-gateway=serial --my-serial-port=/dev/ttyAMA0 --my-leds-err-pin=12 --my-leds-rx-pin=16 --my-leds-tx-pin=18
```

```
make
```

```
sudo make install
```

```
sudo systemctl enable mysgw.service
```

Output in syslog

```
tail -f /var/log/syslog
```
