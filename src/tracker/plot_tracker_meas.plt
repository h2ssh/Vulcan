#!/usr/bin/gnuplot --persist

data = 'tracker_meas_'.ARG1.'.dat'

set multiplot layout 3,2 columnsfirst
set zeroaxis linetype 3 linewidth 1 linecolor rgb "black"

plot data using 2 with lines title 'x-vel', \
    data using 3 with lines title 'variance'

plot data using 4 with lines title 'y-vel', \
    data using 5 with lines title 'variance'

plot data using 6 with lines title 'Vel Magnitude'

plot data using 7 with lines title 'x-accel', \
    data using 8 with lines title 'variance'

plot data using 9 with lines title 'y-accel', \
    data using 10 with lines title 'variance'

plot data using 11 with lines title 'Accel Magnitude'

unset multiplot

pause -1
