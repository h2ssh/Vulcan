#!/usr/bin/gnuplot --persist

values='values_'
gradient='gradients_'
maxima='maxima_'
deriv='deriv_'
start='start_'
end='end_'
ext='.dat'

set autoscale
plot 0.0
replot values.ARG1.ext using 1:2 with lines title "Value"
replot gradient.ARG1.ext using 1:2 with lines title "1st Deriv"
replot maxima.ARG1.ext using 1:2 with impulses title "Maxima"
replot deriv.ARG1.ext  using 1:2 with lines title "2nd Deriv"
#replot start.ARG1.ext using 1:2 with lines title "Start Maxima"
#replot end.ARG1.ext using 1:2 with impulses title "End Maxima"

pause -1

