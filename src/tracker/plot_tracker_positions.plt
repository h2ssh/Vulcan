#!/usr/bin/gnuplot --persist

data = 'tracker_detected_positions.dat'
set zeroaxis linetype 3 linewidth 1 linecolor rgb "black"

plot data using 1:2 title 'Position'

pause -1
