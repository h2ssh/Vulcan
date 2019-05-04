set ylabel 'Update Time (milliseconds)'
set xlabel 'Topological Event'
set key top left

plot ARG1 using 1:5 with lines lt 5 lc rgb "#87ceeb" lw 2 title 'Exhaustive Search', \
     ARG2 using 1:5 with lines lt 7 lc rgb "#ff6347" lw 2 title 'Lazy Evaluation'
