### Multiwii 2.3 fork

This repo is a fork of the MW2.3 source with my changes in it, most notably an automatic altitude hold algorithm for the BARO/ACC.

Newer changes in commit history.

#### initial changes

config.h

* added #MSPRXONLY

RX.cpp

* in computeRC added #MSPRXONLY

Protocol.cpp

* added s_struct_w_no_reply on line 241
* changed MSP_SET_RAW_RC to use s_struct_w_no_reply to save radio latency
* changed MSP_DEBUG to use 8 uint_16t instead of 4

IMU.cpp

* implemented new automatic alt hold algorithm

Multiwii.cpp/.h

* changed debug[4] to debug[8]
