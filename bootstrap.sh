#!/bin/bash

meson setup --wipe --prefix=$PWD --werror --warnlevel=3 --buildtype=debugoptimized build
