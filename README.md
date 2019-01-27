# 123Tune-plus-Simulator
123Tune+ ignition client software Simulator , reverse engineered, to write your own client software instead using the provided closed source client


# Introduction

The 123* series are electronic ignitions which replace the old contact point based ignition, with a electronic version.
The 123Tune+ is manageble thrugh a bluetooth client
Unfortunately the 123Tune+ ignition and client software is close source. 
The 123Tune+ is missing an importand feature, switchable curves which was available in the 123Tune which is managed through USB and a curve switch cable.

So I reverse engineered the used bluetooth protocol, to made it fully accepted by the (IOS) client.

# Release

## Initial
Worked on reverse engineering around 2017/2018, and released working demo
about 4 months of contacting vendor.

## 2019 bluez 5.5/123Client protocol changes
While porting this code to ESP32, i noticed the new released 123tune client
software behaves slightly different. Probably all within margin of the specs, which are
closed source. Ported this code to bluez 5.5 as well (not much has changged)
Pushed this version in Jan 2019

# issues with the 123Tune+

* Depends on platform (android/IOS), which might completely forgotten in 40 years.
* Software is closed source, and in combination with above remarks, worrisome for long run
* Easy curve switching feature is missing (which is available in regular 123Tune), felt the need to implement that myself.
* It has a major security flaw, described under BUGS. I refuse to use it, and so should you. Or do you not care if someone takes controll over your ignition?

# intended uses

* Write your own (open) client 
* implement missing features like missing curve switch
* implement missing performance tuning options
* Intentions for integrating with microcontrollers like ESP32.
* make sure your client is future proof, as your oldtimer might become another 40 years older, you want to make sure client software option is available long after Iphones and Android has been forgotten.

# BUGS

* Of course there are no bugs in my software, but i found some while reverse engineering.
* Most importand: found a security/design issue, which prevents me to put it in use because it is quite easy for anyone to take controll of your ignition. informed/contacted info@123ignitionshop.com but no firmware update is to be expected soon (if ever).

# availability

The software is available as is, and just as i left it, including lost of comments, and debug.

# How to use

Note: Tested/developped under Linux (ubuntu 16.04LTS + 18.04LTS) )
Used a general availailable BLE 4.1/USB module and the Thinkpad T430
internal bluetooth (4.0)

From that start with a bluetooth bluez clone:
git clone https://git.kernel.org/pub/scm/bluetooth/bluez.git
or just download http://www.kernel.org/pub/linux/bluetooth/bluez-5.50.tar.xz

replace bluez/tools/btgatt-server.c with file provided in this repo, compile it, and you're almost ready to go.

To get it a acepted by the 123Tune+ client (IOS) some additional bluetooth tuning needs to be done, just faking the key properties, and your 123Tuneclient should see and accept it. (All functions available) .

Commands below should set it up correctly, and ready to go.
```

With cli setup al shown below no sytem setting adjustment in  /etc/bluetooth/main.conf
are not required, as you set name etc with commands below.
/etc/init.d/bluetooth start (if not already started)

btmgmt -i hci0 power off
btmgmt -i hci0 le on
btmgmt -i hci0 connectable on
btmgmt -i hci0 name "123\\TUNE+"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on

hciconfig noauth

hcitool -i hci0 cmd 0x08 0x0008 18 02 01 1a  02 0a 04  11 07 79 60 22 a0 be af c0 bd de 48 79 62 f1 84 2b da  00 00 00 00 00 00 00
hcitool -i hci0 cmd 0x08 0x0009 08 07 ff 85 00 ff 01 F0 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

# Start userspace gatt server 
bluez/tools/btgatt-server -1 
```

# Future development

## 2018
I intended to write an open source client, but started to analyse the client/server protocol first, and write a simulator for the 123Tune hardware device.
With that, writing and testing client software would be quite easy.
However, I ran into a major security flaw, which requires a new firmware into the 123Tune+ device, which isn't available.
I refuse to start driving with  an easy remote exploitable ignition system. With that I lost my interrest to write the client, and i did NOT build the 123Tune+ in, altough it cost about 400 euro's!

Tested the Simulator agains 123Tune software on IOS begin 2018, and I noticed an update came for the client. didn't test that
at that time.

## 2019
Just for fun I'm started working on an ESP32 version, but then I
discovered the protocol is slightly different used by new released 123Tune
client software. So I fixed that first in btgatt-server.c, available now.

working on Arduino ESP32 port of the simulator, but I've additional ideas for portable usage.

