#!/usr/bin/gnuplot --persist

set style line 1 linecolor rgb '0x87ceeb' linewidth 2

set style line 2 linecolor rgb '0xff6347' linewidth 2

set style line 3 linecolor rgb '0x6a5acd' linewidth 2

set style line 4 linecolor rgb '0xda70d6' linewidth 2

set style line 5 linecolor rgb '0x32cd32' linewidth 2

set title 'Intention estimates for an agent moving through an intersection'

set ylabel 'Probability'
set yrange [-0.1:1.1]

set xlabel 'Time Step'

set for [i=1:5] linetype i

title(n) = sprintf("%d", n)

#plot ARG1 using :1 with lines ls 1 title '1', '' using :2 with lines ls 2 title '2', '' using :3 with lines ls 3 title '3', \
#	'' using :4 with lines ls 4 title '4', '' using :5 with lines ls 5 title '5'

plot for [n=1:5] ARG1 u :n w lines ls n title title(n)
