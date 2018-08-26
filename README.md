# 123Tune-plus-Simulator
123Tune+ ignition client software Simulator , reverse engineered, to write your own client software instead using the provided closed source client


# Introduction

The 123* series are electronic ignitions which replace the old contact point based ignition, with a electronic version.
The 123Tune+ is manageble thrugh a bluetooth client
Unfortunately the 123Tune+ ignition and client software is close source. 
The 123Tune+ is missing an importand feature, switchable curves which was available in the 123Tune which is managed through USB and a curve switch cable.

So I reverse engineered the used bluetooth protocol, to made it fully accepted by the (IOS) client.

# inteneded uses

* Write your onw (open) client
* implement missing features like missing curve switch
* implement missing performance tuning options
* make sure your client is future proof, as your oldtimer might be 40 years of older, you want to make sure client software option is available long after Iphones and Android has been forgotten.

# BUGS

Of course there are no bugs in my software, but i found some while reverse engineering.
Most importand: found a  security/design issue, which prevents me to put it in use because it is quite easy for anyone to take controll of your ignition.
