#!/bin/bash

rm -rf build/* bin/*
meson setup --prefix=$PWD --werror --warnlevel=3 --buildtype=debugoptimized --pkg-config-path=external/pkgconfig,external/lib/pkgconfig $@ build
