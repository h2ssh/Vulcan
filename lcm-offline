#!/usr/bin/env bash

if [ $# == 1 ]; then
    dev=$1;
else
    dev='lo';
fi

echo "setting $dev to be multicast device"

sudo ifconfig $dev multicast
sudo route add -net 239.255.0.0/16 dev $dev
