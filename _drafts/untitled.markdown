---
layout: post
title: Install Docker on Raspbian Stretch
---

```
sudo apt-get install \
     apt-transport-https \
     ca-certificates \
     curl \
     gnupg2 \
     software-properties-common
```

```
curl -fsSL https://download.docker.com/linux/$(. /etc/os-release; echo "$ID")/gpg | sudo apt-key add -
```

```
sudo apt-key fingerprint 0EBFCD88
```

```
echo "deb [arch=armhf] https://download.docker.com/linux/$(. /etc/os-release; echo "$ID") \
     $(lsb_release -cs) stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list
```

```
sudo apt-get update
```

```
sudo apt-get install docker-ce
```

```
sudo usermod -a -G docker vpetkov
```
```
sudo docker run armhf/hello-world
```
