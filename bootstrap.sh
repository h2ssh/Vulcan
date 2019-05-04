#!/bin/bash

scons lcm=1 # (generate legacy LCM messages)
cp external/gnuplot-iostream/gnuplot-iostream.h build/include/
