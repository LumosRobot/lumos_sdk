#!/bin/bash

sudo ifconfig $1 multicast
# Delete the default loopback multicast route first so that LCM sockets
# bind to the physical interface rather than lo.
sudo ip route del 224.0.0.0/4 dev lo 2>/dev/null || true
sudo ip route add 224.0.0.0/4 dev $1
