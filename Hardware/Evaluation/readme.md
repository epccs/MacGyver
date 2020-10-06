# Description

This shows the setup and methods used for evaluation of MacGyver.

# Table of References


# Table Of Contents:

1. ^1 TWI0



## ^1 TWI0

The twi0_mc is from Microchip and was done for an m4809; I had to change a few things (baud equation, and a register or two) to get it working; it is not multi-master. The twi0_bsd started with both m328pb and m324pb and attempted to do multi-master, but two masters talking will damage each other's data. The older TWI hardware on a 328pb has a single peripheral that will generate interrupts for both master and slave, but the newer xmega core TWI has two peripherals, one for the master and the other for the slave. I think that will allow a master to talk to a slave on the same device. Unfortunately, this does not fix my problem with two masters talking, but if each master has a slave, perhaps I can pass a talker token around.
