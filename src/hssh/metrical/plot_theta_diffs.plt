set autoscale
plot 'theta.log' using 1 with lines title "IMU Velocity"
replot 'theta.log' using 2 with lines title "Encoders"
replot 'theta.log' using 3 with lines title "IMU Orientation"
