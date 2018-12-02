# 123Tune-plus-Simulator
123Tune+ ignition client software Simulator , reverse engineered, to write your own client software instead using the provided closed source client


# Introduction

The 123* series are electronic ignitions which replace the old contact point based ignition, with a electronic version.
The 123Tune+ is manageble thrugh a bluetooth client
Unfortunately the 123Tune+ ignition and client software is close source. 
The 123Tune+ is missing an importand feature, switchable curves which was available in the 123Tune which is managed through USB and a curve switch cable.

So I reverse engineered the used bluetooth protocol, to made it fully accepted by the (IOS) client.

# issues with the 123Tune+

* Depends on platform (android/IOS), which might completely forgotten in 40 years.
* Easy curve switching feature is missing, feel the need to implement that myself.
* It has a major security flaw, described under BUGS. I refuse to use it, and so should you. or do don't care if someone takes controll over your ignition?

# intended uses

* Write your own (open) client 
* implement missing features like missing curve switch
* implement missing performance tuning options
* make sure your client is future proof, as your oldtimer might be 40 years of older, you want to make sure client software option is available long after Iphones and Android has been forgotten.

# BUGS

* Of course there are no bugs in my software, but i found some while reverse engineering.
* Most importand: found a security/design issue, which prevents me to put it in use because it is quite easy for anyone to take controll of your ignition. informed/contacted info@123ignitionshop.com but no firmware update is to be expected soon (if ever).

# availability

The software is available as is, and just as i left it.

# How to use

Note: Tested/developped only under Linux (ubuntu)
Used a general availailable BLE/USB module.

From that start with a bluetooth bluez clone:
git clone https://git.kernel.org/pub/scm/bluetooth/bluez.git

replace bluez/tools/btgatt-server.c with this file, compile it, and you're almost ready to go.

To get it a cepted by the 123Tune+ client (IOS) some additional bluetooth tuning needs to be done, just faking the key properties, and your 123Tuneclient should see and accept it. (All functions available) .

I didn't test it recently but commands below shouold set it up ready to go. (notice, the duplicate probably can be left out)
```

adjust  /etc/bluetooth/main.conf with proper name, is most likely not required, as you set name with commands below.
/etc/init.d/bluetooth restart

btmgmt -i hci0 power off
btmgmt -i hci0 le on
btmgmt -i hci0 connectable on
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on

hciconfig noauth
btmgmt -i hci0 name "123\\TUNE+"
btmgmt -i hci0 connectable on
btmgmt -i hci0 advertising on

btmgmt -i hci0 power off
btmgmt -i hci0 power on
btmgmt -i hci0 advertising on

# manufactorer data van 123tune+ MUST match to get accepted, so give what it wants
hcitool -i hci0 cmd 0x08 0x0008 18 02 01 1a  02 0a 04  11 07 79 60 22 a0 be af c0 bd de 48 79 62 f1 84 2b da  00 00 00 00 00 00 00
hcitool -i hci0 cmd 0x08 0x0009 08 07 ff 85 00 ff 01 F0 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

```

# Future development

I intended to write an open source client, but started to analyse the client/server protocol first, and write a simulator for the 123Tune hardware device.
With that, writing and testing client software would be quite easy.
However, I ran into a major security flaw, which requires a new firmware into the 123Tune+ device, which isn't available.
I refuse to start driving with  an easy remote exploitable ignition system. With that I lost my interrest to write the client, and i did NOT build the 123Tune+ in, altough it cost about 400 euro's!

Tested the Simulator agains 123Tune software on IOS begin 2018, and I noticed an update came for the client. didn't test that, but i expect no issues if you want to start using to write your client. 
